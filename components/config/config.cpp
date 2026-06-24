// config.cpp - Implementierung der GPIO-Initialisierung und Hilfsfunktionen.
#include "config.h"

#include <cstddef>

#include "esp_timer.h"

namespace cfg {

// Interne Zustaende (gespiegelt, damit Abfragen ohne GPIO-Read moeglich sind).
// Hinweis: EN wird NICHT mehr gespiegelt, da FastAccelStepper den gemeinsamen
// Pin via Auto-Enable selbst treibt -> drivers_enabled() liest den realen Pegel.
static bool s_light_on = false;

// ---------------------------------------------------------------------------
//  Entprellung der Lichtschranken
// ---------------------------------------------------------------------------
// Zeitbasierter Filter: ein gelesener Pegel wird erst als 'stable' uebernommen,
// wenn er LIMIT_DEBOUNCE_MS lang unveraendert ist. Abtastung durch limits_poll()
// (limit_task, alle LIMIT_SAMPLE_MS). Das unterdrueckt kurze Motor-EMI-Glitches,
// die sonst einen Fehl-Stop ausloesen wuerden.
struct LimitFilter {
    gpio_num_t   pin;
    volatile int stable;          // entprellter Pegel (von limit_triggered gelesen)
    int          last_raw;        // zuletzt gelesener Rohpegel
    uint32_t     stable_since_ms; // seit wann last_raw unveraendert ist
};

static inline uint32_t now_ms() {
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

// Genau die drei real verdrahteten Sensoren. Start fail-safe auf "ausgeloest".
static LimitFilter s_limits[] = {
    { PIN_TN_GATE,     LIMIT_ACTIVE_LEVEL, LIMIT_ACTIVE_LEVEL, 0 },
    { PIN_FV_LIM_FWD,  LIMIT_ACTIVE_LEVEL, LIMIT_ACTIVE_LEVEL, 0 },
    { PIN_FV_LIM_BACK, LIMIT_ACTIVE_LEVEL, LIMIT_ACTIVE_LEVEL, 0 },
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

    // --- Eingaenge: Lichtschranken (active-high) ---
    // Der externe 4,7k Pull-up nach 3V3 liefert den definierten HIGH-/Fail-safe-
    // Pegel und dominiert den Pegel. Der interne Pull-Up bleibt zusaetzlich aktiv
    // (redundanter Fail-safe; ~45k parallel zum 4,7k -> vernachlaessigbar).
    gpio_config_t in_cfg = {};
    in_cfg.mode         = GPIO_MODE_INPUT;
    in_cfg.pull_up_en   = GPIO_PULLUP_ENABLE;
    in_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    in_cfg.intr_type    = GPIO_INTR_DISABLE;
    in_cfg.pin_bit_mask = (1ULL << PIN_TN_GATE) | (1ULL << PIN_FV_LIM_FWD) |
                          (1ULL << PIN_FV_LIM_BACK);
    gpio_config(&in_cfg);

    // Entprell-Filter mit dem aktuellen Ist-Pegel vorbelegen (vermeidet einen
    // unechten Wechsel direkt nach dem Boot).
    uint32_t t = now_ms();
    for (size_t i = 0; i < LIMIT_COUNT; ++i) {
        int lvl = gpio_get_level(s_limits[i].pin);
        s_limits[i].stable          = lvl;
        s_limits[i].last_raw        = lvl;
        s_limits[i].stable_since_ms = t;
    }
}

void limits_poll() {
    uint32_t t = now_ms();
    for (size_t i = 0; i < LIMIT_COUNT; ++i) {
        int raw = gpio_get_level(s_limits[i].pin);
        if (raw != s_limits[i].last_raw) {
            s_limits[i].last_raw        = raw;       // neuer Pegel -> Entprell-Timer neu
            s_limits[i].stable_since_ms = t;
        } else if (raw != s_limits[i].stable &&
                   (t - s_limits[i].stable_since_ms) >= LIMIT_DEBOUNCE_MS) {
            s_limits[i].stable = raw;                // lange genug stabil -> uebernehmen
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

bool limit_triggered(gpio_num_t pin) {
    if (pin == PIN_NONE) return false;
    const LimitFilter* f = find_limit(pin);
    if (f == nullptr) return false;            // unbekannter Pin -> nicht ausgeloest
    // active-high: ausgeloest, wenn der entprellte Pegel HIGH ist.
    return f->stable == LIMIT_ACTIVE_LEVEL;
}

}  // namespace cfg
