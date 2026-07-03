// protocol.cpp - UART-Befehlskanal: lesen, parsen, dispatchen, quittieren.
#include "protocol.h"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "config.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "protocol";

static constexpr int MAX_LINE     = 128;
static constexpr int MAX_TOKENS   = 3;   // ACHSE:BEFEHL:WERT
static constexpr int MAX_RESPONSE = 256;

// ---------------------------------------------------------------------------
//  Machine-Hilfen
// ---------------------------------------------------------------------------
void Machine::stopAll() {
    if (tr) tr->forceStop();
    if (fr) fr->forceStop();
    if (fv) fv->forceStop();
    if (tn) tn->forceStop();
}

Axis* Machine::byName(const char* code) {
    if (strcmp(code, "TR") == 0) return tr;
    if (strcmp(code, "FR") == 0) return fr;
    if (strcmp(code, "FV") == 0) return fv;
    if (strcmp(code, "TN") == 0) return tn;
    return nullptr;
}

// ---------------------------------------------------------------------------
//  String-Hilfen
// ---------------------------------------------------------------------------
static void str_upper(char* s) {
    for (; *s; ++s) *s = static_cast<char>(toupper(static_cast<unsigned char>(*s)));
}

// Tokenisiert 'line' an ':' in bis zu MAX_TOKENS Teile (in-place).
static int tokenize(char* line, char* tok[MAX_TOKENS]) {
    int n = 0;
    char* p = line;
    while (n < MAX_TOKENS && p != nullptr && *p != '\0') {
        tok[n++] = p;
        char* sep = strchr(p, ':');
        if (sep == nullptr) break;
        *sep = '\0';
        p = sep + 1;
    }
    return n;
}

// true, wenn der Richtungs-Wert "positiv/rechts" bedeutet.
static bool is_positive_dir(const char* v) {
    return strcmp(v, "R") == 0 || strcmp(v, "RIGHT") == 0 ||
           strcmp(v, "1") == 0 || strcmp(v, "+") == 0 ||
           strcmp(v, "CW") == 0;
}

// ---------------------------------------------------------------------------
//  Befehlsverarbeitung -> schreibt genau eine Antwortzeile (ohne '\n') in resp
// ---------------------------------------------------------------------------
static void handle_axis_rotation(Axis* ax, const char* cmd, const char* val,
                                 int ntok, char* resp) {
    if (strcmp(cmd, "START") == 0) {
        if (ax->start()) strcpy(resp, "OK");
        else             strcpy(resp, "ERR:NO_SPEED");
    } else if (strcmp(cmd, "STOP") == 0) {
        ax->stop();
        strcpy(resp, "OK");
    } else if (strcmp(cmd, "SPEED") == 0) {
        if (ntok < 3) { strcpy(resp, "ERR:MISSING_VALUE"); return; }
        long v = strtol(val, nullptr, 10);
        if (v < 0 || v > 255) { strcpy(resp, "ERR:BAD_VALUE"); return; }
        ax->setSpeed255(static_cast<uint8_t>(v));
        strcpy(resp, "OK");
    } else if (strcmp(cmd, "DIR") == 0) {
        if (ntok < 3) { strcpy(resp, "ERR:MISSING_VALUE"); return; }
        ax->setDirection(is_positive_dir(val));
        strcpy(resp, "OK");
    } else {
        strcpy(resp, "ERR:UNKNOWN_CMD");
    }
}

static void handle_axis_position(Axis* ax, const char* cmd, const char* val,
                                 int ntok, char* resp) {
    // Vorwaerts-Aliasse: FWD/VOR/TILT/UP ; Rueckwaerts: BACK/ZUR/LOWER/DOWN
    if (strcmp(cmd, "FWD") == 0 || strcmp(cmd, "VOR") == 0 ||
        strcmp(cmd, "TILT") == 0 || strcmp(cmd, "UP") == 0) {
        if (ax->jog(true)) strcpy(resp, "OK");
        else               strcpy(resp, "ERR:AT_LIMIT");
    } else if (strcmp(cmd, "BACK") == 0 || strcmp(cmd, "ZUR") == 0 ||
               strcmp(cmd, "LOWER") == 0 || strcmp(cmd, "DOWN") == 0) {
        if (ax->jog(false)) strcpy(resp, "OK");
        else                strcpy(resp, "ERR:AT_LIMIT");
    } else if (strcmp(cmd, "HOME") == 0) {
        if (ax->home()) strcpy(resp, "OK");
        else            strcpy(resp, "ERR:HOME_FAILED");
    } else if (strcmp(cmd, "STOP") == 0) {
        ax->stop();
        strcpy(resp, "OK");
    } else if (strcmp(cmd, "SPEED") == 0) {
        if (ntok < 3) { strcpy(resp, "ERR:MISSING_VALUE"); return; }
        long v = strtol(val, nullptr, 10);
        if (v < 0 || v > 255) { strcpy(resp, "ERR:BAD_VALUE"); return; }
        ax->setPositionSpeed255(static_cast<uint8_t>(v));
        strcpy(resp, "OK");
    } else if (strcmp(cmd, "MOVE") == 0 || strcmp(cmd, "GOTO") == 0) {
        if (ntok < 3) { strcpy(resp, "ERR:MISSING_VALUE"); return; }
        float target = strtof(val, nullptr);
        // Mit Winkelsensor (TN): geregelte, behutsame Fahrt auf den Sollwinkel.
        // Ohne Sensor (FV): Direktfahrt ueber den Schrittzaehler.
        if (ax->hasAngleSensor()) {
            if (ax->startMoveToAngle(target))      strcpy(resp, "OK");
            else if (!ax->sensorMagnetOk())        strcpy(resp, "ERR:NO_MAGNET");
            else                                   strcpy(resp, "ERR:BUSY");
        } else {
            if (ax->moveToUnit(target)) strcpy(resp, "OK");
            else                        strcpy(resp, "ERR:BUSY");
        }
    } else if (strcmp(cmd, "ANGLE") == 0) {
        // Aktuellen Ist-Winkel zurueckmelden (nur mit Sensor).
        if (!ax->hasAngleSensor()) { strcpy(resp, "ERR:NO_SENSOR"); return; }
        snprintf(resp, MAX_RESPONSE, "ANGLE:%.2f", ax->sensorAngleDeg());
    } else if (strcmp(cmd, "AZERO") == 0) {
        // Aktuelle Position als 0 Grad festlegen (Nullpunkt-Kalibrierung).
        if (!ax->hasAngleSensor()) { strcpy(resp, "ERR:NO_SENSOR"); return; }
        ax->setAngleZero();
        strcpy(resp, "OK");
    } else if (strcmp(cmd, "CAL") == 0) {
        if (ntok < 3) { strcpy(resp, "ERR:MISSING_VALUE"); return; }
        float spu = strtof(val, nullptr);
        if (spu <= 0.0f) { strcpy(resp, "ERR:BAD_VALUE"); return; }
        ax->setCalibration(spu);
        strcpy(resp, "OK");
    } else {
        strcpy(resp, "ERR:UNKNOWN_CMD");
    }
}

static void build_status(Machine* m, char* resp) {
    int hfv = m->fv ? static_cast<int>(m->fv->homing()) : 0;
    int htn = m->tn ? static_cast<int>(m->tn->homing()) : 0;
    // TN-Winkelsensor: gemessener Ist-Winkel (TNsens), Magnet-OK (TNmag),
    // Closed-Loop-Zustand (TNcl: 0=Idle 1=Moving 2=Done 3=Error).
    bool  tn_has  = m->tn && m->tn->hasAngleSensor();
    float tn_ang  = tn_has ? m->tn->sensorAngleDeg() : 0.0f;
    int   tn_mag  = (tn_has && m->tn->sensorMagnetOk()) ? 1 : 0;
    int   tn_cl   = m->tn ? static_cast<int>(m->tn->angleState()) : 0;
    snprintf(resp, MAX_RESPONSE,
        "STATUS:CONN=1,EN=%d,LIGHT=%d,"
        "TR=%d,FR=%d,"
        "FV=%.2f,FVrun=%d,TN=%.2f,TNrun=%d,"
        "FVmin=%d,FVmax=%d,TNmin=%d,TNmax=%d,"
        "HOMEfv=%d,HOMEtn=%d,"
        "TNsens=%.2f,TNmag=%d,TNcl=%d,"
        "SENStn=%d,SENSfwd=%d,SENSback=%d",
        cfg::drivers_enabled() ? 1 : 0, cfg::light_on() ? 1 : 0,
        (m->tr && m->tr->isRunning()) ? 1 : 0,
        (m->fr && m->fr->isRunning()) ? 1 : 0,
        m->fv ? m->fv->positionUnit() : 0.0f,
        (m->fv && m->fv->isRunning()) ? 1 : 0,
        m->tn ? m->tn->positionUnit() : 0.0f,
        (m->tn && m->tn->isRunning()) ? 1 : 0,
        (m->fv && m->fv->limitMin()) ? 1 : 0,
        (m->fv && m->fv->limitMax()) ? 1 : 0,
        (m->tn && m->tn->limitMin()) ? 1 : 0,
        (m->tn && m->tn->limitMax()) ? 1 : 0,
        hfv, htn,
        tn_ang, tn_mag, tn_cl,
        cfg::sensor_tn_drive_allowed() ? 1 : 0,
        cfg::sensor_fv_fwd_allowed()   ? 1 : 0,
        cfg::sensor_fv_back_allowed()  ? 1 : 0);
}

static void process_line(Machine* m, char* line, char* resp) {
    // Trim + Grossschreibung (Protokoll ist case-insensitiv).
    str_upper(line);

    char* tok[MAX_TOKENS] = {nullptr, nullptr, nullptr};
    int ntok = tokenize(line, tok);
    if (ntok == 0) { strcpy(resp, "ERR:EMPTY"); return; }

    const char* t0 = tok[0];

    // --- Globale Ein-Wort-Befehle ---
    if (ntok == 1) {
        if (strcmp(t0, "PING") == 0)        { strcpy(resp, "OK"); return; }
        if (strcmp(t0, "STOP") == 0)        { m->stopAll(); strcpy(resp, "OK"); return; }
        if (strcmp(t0, "STATUS") == 0)      { build_status(m, resp); return; }
        // Logische Lichtschranken-Zustaende fuer LabVIEW: je 1 = erlaubt, 0 = stop.
        // Reihenfolge: TN-Gatter, FV vorwaerts, FV rueckwaerts.
        if (strcmp(t0, "SENS?") == 0) {
            snprintf(resp, MAX_RESPONSE, "SENS:%d,%d,%d",
                     cfg::sensor_tn_drive_allowed() ? 1 : 0,
                     cfg::sensor_fv_fwd_allowed()   ? 1 : 0,
                     cfg::sensor_fv_back_allowed()  ? 1 : 0);
            return;
        }
        // ENABLE/DISABLE sind manuelle Overrides; im Normalbetrieb regelt das
        // Auto-Enable von FastAccelStepper den gemeinsamen EN-Pin selbst.
        if (strcmp(t0, "ENABLE") == 0)      { cfg::set_drivers_enabled(true);  strcpy(resp, "OK"); return; }
        if (strcmp(t0, "DISABLE") == 0)     { m->stopAll(); cfg::set_drivers_enabled(false); strcpy(resp, "OK"); return; }
        strcpy(resp, "ERR:UNKNOWN_CMD");
        return;
    }

    // --- Beleuchtung ---
    if (strcmp(t0, "LIGHT") == 0) {
        if (strcmp(tok[1], "ON") == 0)       { cfg::set_light(true);  strcpy(resp, "OK"); }
        else if (strcmp(tok[1], "OFF") == 0) { cfg::set_light(false); strcpy(resp, "OK"); }
        else                                 strcpy(resp, "ERR:BAD_VALUE");
        return;
    }

    // --- Achsbefehle ---
    Axis* ax = m->byName(t0);
    if (ax == nullptr) { strcpy(resp, "ERR:UNKNOWN_AXIS"); return; }

    const char* cmd = tok[1];
    const char* val = (ntok >= 3) ? tok[2] : "";
    if (ax->mode() == AxisMode::Rotation)
        handle_axis_rotation(ax, cmd, val, ntok, resp);
    else
        handle_axis_position(ax, cmd, val, ntok, resp);
}

// ---------------------------------------------------------------------------
//  UART-Task
// ---------------------------------------------------------------------------
static void uart_task(void* arg) {
    Machine* m = static_cast<Machine*>(arg);
    char line[MAX_LINE];
    int  len = 0;
    char resp[MAX_RESPONSE];
    uint8_t byte;

    ESP_LOGI(TAG, "UART-Befehlskanal aktiv (UART%d, %d Baud)",
             cfg::UART_PORT, cfg::UART_BAUD);

    for (;;) {
        int n = uart_read_bytes(cfg::UART_PORT, &byte, 1, pdMS_TO_TICKS(20));
        if (n <= 0) continue;

        if (byte == '\r') continue;             // CR ignorieren
        if (byte == cfg::LINE_TERM) {           // Zeilenende -> verarbeiten
            line[len] = '\0';
            if (len > 0) {
                process_line(m, line, resp);
            } else {
                strcpy(resp, "ERR:EMPTY");
            }
            // Genau EINE Antwortzeile mit '\n' zurueck (deterministisch fuer VISA Read).
            int rlen = strlen(resp);
            resp[rlen]     = cfg::LINE_TERM;
            resp[rlen + 1] = '\0';
            uart_write_bytes(cfg::UART_PORT, resp, rlen + 1);
            len = 0;
        } else if (len < MAX_LINE - 1) {
            line[len++] = static_cast<char>(byte);
        } else {
            // Zeile zu lang -> verwerfen und Fehler melden.
            len = 0;
            const char* err = "ERR:LINE_TOO_LONG\n";
            uart_write_bytes(cfg::UART_PORT, err, strlen(err));
        }
    }
}

void protocol_start(Machine* machine) {
    uart_config_t uart_cfg = {};
    uart_cfg.baud_rate = cfg::UART_BAUD;
    uart_cfg.data_bits = UART_DATA_8_BITS;
    uart_cfg.parity    = UART_PARITY_DISABLE;
    uart_cfg.stop_bits = UART_STOP_BITS_1;
    uart_cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_cfg.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_driver_install(cfg::UART_PORT, cfg::UART_RX_BUF,
                                        cfg::UART_TX_BUF, 0, nullptr, 0));
    ESP_ERROR_CHECK(uart_param_config(cfg::UART_PORT, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(cfg::UART_PORT, cfg::UART_TX_PIN,
                                 cfg::UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    xTaskCreate(uart_task, "uart_task", 4096, machine, 10, nullptr);
}
