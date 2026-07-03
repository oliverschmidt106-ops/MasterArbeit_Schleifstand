// fv.cpp - Zustandsmaschine des Faservorschubs: Homing, Park & Resume.
//
// Ablauf der zweistufigen Referenzfahrt an der Min-Schranke (GPIO 11):
//   SeekFast : Eilgang rueckwaerts bis Schranke LOW (Auto-Stopp in Axis::update)
//   Release  : Freifahren vorwaerts um FV_HOME_RELEASE_MM (Schranke wieder HIGH)
//   SeekSlow : langsame zweite Anfahrt bis Schranke LOW -> Zaehler = 0
//   Offset   : Vorfahren auf FV_HOME_OFFSET_MM -> Zustand HOMED
//
// Nebenlaeufigkeit: Die Befehls-Funktionen laufen im uart_task, fv_tick() im
// control_task (gleiches Muster wie der restliche Achscode: kurze, atomare
// Zustandsuebergaenge, keine Locks). fv_tick() MUSS nach Axis::update()
// aufgerufen werden, damit der Endlagen-Auto-Stopp bereits gegriffen hat.
#include "fv.h"

#include <cmath>
#include <cstdlib>

#include "axis.h"
#include "config.h"
#include "esp_log.h"

static const char* TAG = "fv";

// Interne Ablauf-Phasen (feiner als die Protokoll-Zustaende in FvState).
enum class Phase {
    None,
    SeekFast, Release, SeekSlow, Offset,   // Referenzfahrt
    ParkMove,                               // Fahrt zur Parkposition
    ResumePre, ResumeFinal,                 // Rueckkehr mit Backlash-Kompensation
    UserMove,                               // FV_MOVE / Legacy-MOVE
};

static Axis*            s_axis  = nullptr;
static volatile FvState s_state = FvState::NotHomed;
static Phase            s_phase = Phase::None;

static int32_t s_saved_pos   = 0;      // gespeicherte Arbeitsposition (steps)
static bool    s_saved_valid = false;  // niemals ueber Neustart hinweg gueltig
static int32_t s_target      = 0;      // Ziel der aktiven Zielfahrt (steps)
static int32_t s_phase_start = 0;      // Startposition der Homing-Phase (Wegbudget)

// Toleranz fuer "Ziel erreicht" (steps). Eine regulaer beendete moveTo-Fahrt
// steht exakt auf dem Ziel; die Toleranz faengt nur Rundungen ab.
static constexpr int32_t TARGET_TOL_STEPS = 2;

// steps/mm zur Laufzeit von der Achse holen (FV:CAL kann sie aendern!).
static float   spm()                { return s_axis->stepsPerUnit(); }
static int32_t mm_to_steps(float m) { return static_cast<int32_t>(lroundf(m * spm())); }

// ---------------------------------------------------------------------------
//  Fehlerbehandlung: jeder Abbruch fuehrt fail-safe nach NOT_HOMED
// ---------------------------------------------------------------------------
static void fail(const char* why) {
    if (s_axis != nullptr) s_axis->forceStop();
    s_phase       = Phase::None;
    s_state       = FvState::NotHomed;
    s_saved_valid = false;
    ESP_LOGW(TAG, "FV -> NOT_HOMED (%s)", why);
}

// ---------------------------------------------------------------------------
//  Phasen-Starts der Referenzfahrt
// ---------------------------------------------------------------------------
static bool start_seek(bool fast) {
    s_phase_start = s_axis->positionSteps();
    if (!s_axis->runAtHz(/*forward=*/false, fast ? cfg::FV_HOME_SPEED_FAST_HZ
                                                 : cfg::FV_HOME_SPEED_SLOW_HZ))
        return false;
    s_phase = fast ? Phase::SeekFast : Phase::SeekSlow;
    return true;
}

static bool start_release() {
    s_phase_start = s_axis->positionSteps();
    s_target      = s_phase_start + mm_to_steps(cfg::FV_HOME_RELEASE_MM);
    if (!s_axis->moveToStepsAtHz(s_target, cfg::FV_HOME_SPEED_FAST_HZ)) return false;
    s_phase = Phase::Release;
    return true;
}

static bool start_offset() {
    s_target = mm_to_steps(cfg::FV_HOME_OFFSET_MM);
    if (!s_axis->moveToStepsAtHz(s_target, cfg::FV_HOME_SPEED_FAST_HZ)) return false;
    s_phase = Phase::Offset;
    return true;
}

// ---------------------------------------------------------------------------
//  Ticks
// ---------------------------------------------------------------------------
static void homing_tick() {
    const int32_t pos    = s_axis->positionSteps();
    const int32_t budget = mm_to_steps(cfg::FV_HOME_MAX_TRAVEL_MM);

    switch (s_phase) {
    case Phase::SeekFast:
    case Phase::SeekSlow:
        // Falsche Schranke bei Rueckwaertsfahrt -> generischer Abbruch
        // (deckt auch Kabelbruch ab: beide Kanaele fallen auf LOW).
        if (!cfg::sensor_fv_fwd_allowed()) { fail("falsche Schranke (FWD)"); return; }
        if (labs(pos - s_phase_start) > budget) { fail("HOME_TIMEOUT Suchfahrt"); return; }
        if (!cfg::sensor_fv_back_allowed()) {
            // Schranke erreicht -> Axis::update() hat bereits gestoppt.
            if (s_axis->isRunning()) return;
            if (s_phase == Phase::SeekFast) {
                if (!start_release()) fail("Freifahren nicht startbar");
            } else {
                // Ausloesepunkt bei Langsamfahrt = Referenz 0.
                s_axis->setCurrentPositionSteps(0);
                if (!start_offset()) fail("Offset-Fahrt nicht startbar");
            }
            return;
        }
        // Schranke frei, aber Motor steht -> externer STOP o.ae.
        if (!s_axis->isRunning()) fail("Suchfahrt unterbrochen");
        break;

    case Phase::Release:
        if (labs(pos - s_phase_start) > budget) { fail("HOME_TIMEOUT Freifahren"); return; }
        if (s_axis->isRunning()) return;
        if (labs(pos - s_target) > TARGET_TOL_STEPS) { fail("Freifahren unterbrochen"); return; }
        if (!cfg::sensor_fv_back_allowed()) { fail("Schranke nach Freifahren belegt"); return; }
        if (!start_seek(/*fast=*/false)) fail("Langsam-Anfahrt nicht startbar");
        break;

    case Phase::Offset:
        if (s_axis->isRunning()) return;
        if (labs(pos - s_target) > TARGET_TOL_STEPS) { fail("Offset-Fahrt unterbrochen"); return; }
        s_phase = Phase::None;
        s_state = FvState::Homed;
        ESP_LOGI(TAG, "Referenzfahrt ok, Arbeitspunkt %.3f mm", (double)fv_position_mm());
        break;

    default:
        // HOMING ohne aktive Phase darf nicht vorkommen.
        fail("inkonsistenter Homing-Zustand");
        break;
    }
}

static void position_tick() {
    // Jede ausgeloeste Schranke waehrend einer Positionierfahrt (auch Jog)
    // invalidiert die Referenz: encoderlos ist nicht pruefbar, ob Schritte
    // verloren gingen -> konservativ NOT_HOMED.
    const bool limit  = !cfg::sensor_fv_fwd_allowed() || !cfg::sensor_fv_back_allowed();
    const bool moving = s_axis->isRunning();
    if (limit && (moving || s_phase != Phase::None)) {
        fail("Schranke waehrend Positionierfahrt");
        return;
    }
    if (s_phase == Phase::None || moving) return;

    // Aktive Zielfahrt ist beendet -> auswerten.
    const bool reached =
        labs(s_axis->positionSteps() - s_target) <= TARGET_TOL_STEPS;

    switch (s_phase) {
    case Phase::ParkMove:
        // Bei Unterbrechung (STOP) bleibt PARKED: saved_position ist weiter
        // gueltig, FV_RESUME funktioniert von jeder Zwischenposition.
        s_phase = Phase::None;
        if (!reached) ESP_LOGW(TAG, "Parkfahrt unterbrochen");
        break;

    case Phase::ResumePre:
        if (!reached) { s_phase = Phase::None; ESP_LOGW(TAG, "Resume unterbrochen"); break; }
        // Endanfahrt: immer aus Richtung FV_APPROACH_DIR auf saved_position.
        s_target = s_saved_pos;
        if (!s_axis->moveToStepsAtHz(s_target, s_axis->positionSpeedHz())) {
            s_phase = Phase::None;
            ESP_LOGW(TAG, "Resume-Endanfahrt nicht startbar");
            break;
        }
        s_phase = Phase::ResumeFinal;
        break;

    case Phase::ResumeFinal:
        s_phase = Phase::None;
        if (reached) {
            s_state       = FvState::Homed;
            s_saved_valid = false;
            ESP_LOGI(TAG, "Resume ok (%.3f mm)", (double)fv_position_mm());
        } else {
            ESP_LOGW(TAG, "Resume-Endanfahrt unterbrochen");  // bleibt PARKED
        }
        break;

    case Phase::UserMove:
        s_phase = Phase::None;   // STOP mittendrin ist ok, Zaehler bleibt gueltig
        break;

    default:
        s_phase = Phase::None;
        break;
    }
}

// ---------------------------------------------------------------------------
//  Oeffentliche API
// ---------------------------------------------------------------------------
void fv_init(Axis* axis) {
    s_axis        = axis;
    s_state       = FvState::NotHomed;   // Neustart: Referenz immer ungueltig
    s_phase       = Phase::None;
    s_saved_valid = false;
}

void fv_tick() {
    if (s_axis == nullptr) return;
    switch (s_state) {
    case FvState::Homing:                    homing_tick();   break;
    case FvState::Homed: case FvState::Parked: position_tick(); break;
    default: break;   // NOT_HOMED: nur Hardware-Gating (Axis::update)
    }
}

FvState fv_state() { return s_state; }

const char* fv_state_str() {
    switch (s_state) {
    case FvState::Homing: return "HOMING";
    case FvState::Homed:  return "HOMED";
    case FvState::Parked: return "PARKED";
    default:              return "NOT_HOMED";
    }
}

float fv_position_mm() {
    if (s_axis == nullptr) return 0.0f;
    return static_cast<float>(s_axis->positionSteps()) / spm();
}

const char* fv_cmd_home() {
    if (s_axis == nullptr) return "ERR:NO_AXIS";
    if (s_state == FvState::Homing) return "ERR:BUSY";
    s_axis->forceStop();                 // laufende Bewegung sicher beenden
    s_saved_valid = false;
    // Startet die Achse bereits in der Schranke, direkt mit Freifahren
    // beginnen (kein Deadlock, Fahrt aus der Schranke ist immer erlaubt).
    // Erst die Phase starten, DANN den Zustand umschalten: fv_tick() laeuft
    // in der control_task und darf nie HOMING ohne aktive Phase sehen.
    const bool ok = cfg::sensor_fv_back_allowed() ? start_seek(/*fast=*/true)
                                                  : start_release();
    if (!ok) { fail("Referenzfahrt nicht startbar"); return "ERR:HOME_FAILED"; }
    s_state = FvState::Homing;
    return "OK";
}

const char* fv_cmd_park() {
    if (s_axis == nullptr) return "ERR:NO_AXIS";
    if (s_state == FvState::Homing) return "ERR:BUSY";
    if (s_state == FvState::Parked) return "ERR:PARKED";
    if (s_state != FvState::Homed)  return "ERR:NOT_HOMED";
    if (s_phase != Phase::None || s_axis->isRunning()) return "ERR:BUSY";
    s_saved_pos   = s_axis->positionSteps();
    s_saved_valid = true;
    s_target      = mm_to_steps(cfg::FV_PARK_POS_MM);
    if (!s_axis->moveToStepsAtHz(s_target, s_axis->positionSpeedHz())) {
        s_saved_valid = false;
        return "ERR:BLOCKED";
    }
    s_phase = Phase::ParkMove;
    s_state = FvState::Parked;
    return "OK";
}

const char* fv_cmd_resume() {
    if (s_axis == nullptr) return "ERR:NO_AXIS";
    if (s_state == FvState::Homing) return "ERR:BUSY";
    if (s_state != FvState::Parked) return "ERR:NOT_PARKED";
    if (s_phase != Phase::None || s_axis->isRunning()) return "ERR:BUSY";
    if (!s_saved_valid) return "ERR:NOT_HOMED";
    // Vorposition: Ziel um FV_BACKLASH_MM entgegen der Anfahrrichtung
    // unterfahren, damit die Endanfahrt definiert aus FV_APPROACH_DIR kommt.
    int32_t pre = s_saved_pos
                  - cfg::FV_APPROACH_DIR * mm_to_steps(cfg::FV_BACKLASH_MM);
    if (pre < 0) pre = 0;                       // Softlimit unten
    s_target = pre;
    if (!s_axis->moveToStepsAtHz(s_target, s_axis->positionSpeedHz()))
        return "ERR:BLOCKED";
    s_phase = Phase::ResumePre;
    return "OK";
}

// Gemeinsamer Kern fuer relative (FV_MOVE) und absolute (Legacy) Zielfahrten.
static const char* start_user_move(int32_t target_steps) {
    if (s_state == FvState::Homing) return "ERR:BUSY";
    if (s_state == FvState::Parked) return "ERR:PARKED";
    if (s_state != FvState::Homed)  return "ERR:NOT_HOMED";
    if (s_phase != Phase::None || s_axis->isRunning()) return "ERR:BUSY";
    if (target_steps < 0 || target_steps > mm_to_steps(cfg::FV_MAX_TRAVEL_MM))
        return "ERR:LIMIT";
    s_target = target_steps;
    if (!s_axis->moveToStepsAtHz(s_target, s_axis->positionSpeedHz()))
        return "ERR:BLOCKED";
    s_phase = Phase::UserMove;
    return "OK";
}

const char* fv_cmd_move_rel(float d_mm) {
    if (s_axis == nullptr) return "ERR:NO_AXIS";
    return start_user_move(s_axis->positionSteps() + mm_to_steps(d_mm));
}

const char* fv_cmd_move_abs(float mm) {
    if (s_axis == nullptr) return "ERR:NO_AXIS";
    return start_user_move(mm_to_steps(mm));
}

const char* fv_cmd_jog(bool forward) {
    if (s_axis == nullptr) return "ERR:NO_AXIS";
    if (s_state == FvState::Homing) return "ERR:BUSY";
    if (s_state == FvState::Parked) return "ERR:PARKED";
    // In NOT_HOMED (Freifahren/Einrichten) und HOMED erlaubt. Achtung: Jog
    // kennt keine Softlimits; loest eine Schranke aus, faellt der Zustand
    // in HOMED konservativ auf NOT_HOMED zurueck (siehe position_tick).
    return s_axis->jog(forward) ? "OK" : "ERR:AT_LIMIT";
}

void fv_invalidate() {
    // Nach Laufzeit-Kalibrierung (FV:CAL): alle Positionsbezuege ungueltig.
    s_phase       = Phase::None;
    s_state       = FvState::NotHomed;
    s_saved_valid = false;
}
