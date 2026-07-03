// config.cpp - Implementierung der GPIO-Initialisierung und Hilfsfunktionen.
#include "config.h"

#include <cstddef>

namespace cfg {

// Interne Zustaende (gespiegelt, damit Abfragen ohne GPIO-Read moeglich sind).
// Hinweis: EN wird NICHT mehr gespiegelt, da FastAccelStepper den gemeinsamen
// Pin via Auto-Enable selbst treibt -> drivers_enabled() liest den realen Pegel.
static bool s_light_on = false;

// ---------------------------------------------------------------------------
//  Entprellung der Lichtschranken (Mehrheitsfilter)
// ---------------------------------------------------------------------------
// Ein neuer Pegel wird erst als 'stable' uebernommen, wenn er
// SENSOR_VOTE_SAMPLES Abtastungen in Folge gleich ist. Abtastung durch
// limits_poll() (limit_task, alle SENSOR_SAMPLE_MS). Ein einzelner verrauschter
// Read (Motor-EMI) setzt nur den Kandidaten-Zaehler zurueck und loest keinen
// Zustandswechsel/Fehl-Stop aus.
struct LimitFilter {
    gpio_num_t   pin;
    volatile int stable;     // entprellter Pegel (von der Lese-API gelesen)
    int          candidate;  // aktuell gezaehlter abweichender Pegel
    int          votes;      // Anzahl gleicher Lesungen fuer 'candidate'
};

// Genau die drei real verdrahteten Sensoren. Start fail-safe auf "stop" (LOW):
// es wird nichts freigegeben, bevor der Filter einen stabilen HIGH-Pegel sieht.
static LimitFilter s_limits[] = {
    { PIN_TN_GATE,     SENSOR_STOP_LEVEL, SENSOR_STOP_LEVEL, 0 },
    { PIN_FV_LIM_FWD,  SENSOR_STOP_LEVEL, SENSOR_STOP_LEVEL, 0 },
    { PIN_FV_LIM_BACK, SENSOR_STOP_LEVEL, SENSOR_STOP_LEVEL, 0 },
};
static constexpr size_t LIMIT_COUNT = sizeof(s_limits) / sizeof(s_limits[0]);

static LimitFilter* find_limit(gpio_num_t pin) {
    for (size_t i = 0; i < LIMIT_COUNT; ++i) {
        if (s_limits[i].pin == pin) return &s_limits[i];
    }
    return nullptr;
}

void init_gpios() {
    // --- Ausgaenge: EN und Beleuchtung ---
    // EN als INPUT_OUTPUT, damit der Pegel ruecklesbar ist (STATUS-Feld). Spaeter
    // uebernimmt FastAccelStepper den Pin via Auto-Enable (ebenfalls INPUT_OUTPUT).
    gpio_config_t out_cfg = {};
    out_cfg.mode         = GPIO_MODE_INPUT_OUTPUT;
    out_cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
    out_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    out_cfg.intr_type    = GPIO_INTR_DISABLE;
    out_cfg.pin_bit_mask = (1ULL << PIN_EN) | (1ULL << PIN_LIGHT);
    gpio_config(&out_cfg);

    // Sicherer Grundzustand: Treiber AUS (EN inaktiv), Licht AUS.
    gpio_set_level(PIN_EN, ENABLE_ACTIVE_LOW ? 1 : 0);
    gpio_set_level(PIN_LIGHT, 0);
    s_light_on = false;

    // --- Eingaenge: Lichtschranken OHNE internen Pull ---
    // Den Pegel definiert ausschliesslich der externe Spannungsteiler 1k8/3k3.
    // Ein interner Pull-up wuerde den 3,3k-Zweig verfaelschen, ein interner
    // Pull-down den HIGH-Pegel absenken -> beide deaktiviert.
    gpio_config_t in_cfg = {};
    in_cfg.mode         = GPIO_MODE_INPUT;
    in_cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
    in_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    in_cfg.intr_type    = GPIO_INTR_DISABLE;
    in_cfg.pin_bit_mask = (1ULL << PIN_TN_GATE) | (1ULL << PIN_FV_LIM_FWD) |
                          (1ULL << PIN_FV_LIM_BACK);
    gpio_config(&in_cfg);

    // Filter bleibt fail-safe auf SENSOR_STOP_LEVEL vorbelegt (siehe s_limits);
    // limits_poll() promotet einen Kanal erst nach SENSOR_VOTE_SAMPLES stabilen
    // HIGH-Lesungen auf "erlaubt". So bewegt sich beim Boot nichts ungeprueft.
}

void limits_poll() {
    for (size_t i = 0; i < LIMIT_COUNT; ++i) {
        LimitFilter& f = s_limits[i];
        int raw = gpio_get_level(f.pin);
        if (raw == f.stable) {
            f.candidate = raw;   // Pegel bestaetigt den stabilen Zustand -> Reset
            f.votes     = 0;
        } else if (raw == f.candidate) {
            if (++f.votes >= SENSOR_VOTE_SAMPLES) {
                f.stable = raw;  // lange genug konstant abweichend -> uebernehmen
                f.votes  = 0;
            }
        } else {
            f.candidate = raw;   // neuer abweichender Pegel -> Zaehlung neu starten
            f.votes     = 1;
        }
    }
}

void set_drivers_enabled(bool enabled) {
    // Manueller Override (Protokoll ENABLE/DISABLE). Auto-Enable von
    // FastAccelStepper kann diesen Zustand im Betrieb wieder ueberschreiben.
    gpio_set_level(PIN_EN, (enabled == ENABLE_ACTIVE_LOW) ? 0 : 1);
}

bool drivers_enabled() {
    // Realen Pegel am gemeinsamen EN-Pin lesen (Pin ist INPUT_OUTPUT). So bleibt
    // das STATUS-Feld korrekt, auch wenn Auto-Enable den Pin selbst schaltet.
    return (gpio_get_level(PIN_EN) == 0) == ENABLE_ACTIVE_LOW;
}

void set_light(bool on) {
    gpio_set_level(PIN_LIGHT, on ? 1 : 0);
    s_light_on = on;
}

bool light_on() { return s_light_on; }

// Entprellten Pegel eines Sensorpins lesen; unbekannt/PIN_NONE -> fail-safe STOP.
static int sensor_level(gpio_num_t pin) {
    const LimitFilter* f = find_limit(pin);
    return (f != nullptr) ? f->stable : SENSOR_STOP_LEVEL;
}

// Logische Lese-API: erlaubt == entprellter Pegel HIGH (SENSOR_GO_LEVEL).
bool sensor_tn_drive_allowed() { return sensor_level(PIN_TN_GATE)     == SENSOR_GO_LEVEL; }
bool sensor_fv_fwd_allowed()   { return sensor_level(PIN_FV_LIM_FWD)  == SENSOR_GO_LEVEL; }
bool sensor_fv_back_allowed()  { return sensor_level(PIN_FV_LIM_BACK) == SENSOR_GO_LEVEL; }

bool limit_triggered(gpio_num_t pin) {
    if (pin == PIN_NONE) return false;
    const LimitFilter* f = find_limit(pin);
    if (f == nullptr) return false;            // unbekannter Pin -> nicht gesperrt
    // Neue Konvention: gesperrt, wenn der entprellte Pegel LOW ist (stop).
    return f->stable == SENSOR_STOP_LEVEL;
}

}  // namespace cfg
