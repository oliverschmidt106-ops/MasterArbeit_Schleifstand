// fan.cpp - Implementierung der Gehaeuseluefter-Steuerung (siehe fan.h).
#include "fan.h"

#include "axis.h"
#include "config.h"
#include "driver/gpio.h"

namespace {

Axis* s_tr   = nullptr;  // Tischrotation
Axis* s_fr   = nullptr;  // Faserrotation
bool  s_on   = false;    // gespiegelter GPIO-Zustand (Abfrage ohne GPIO-Read)
bool  s_auto = true;     // true = Automatik, false = manueller Override

void apply(bool on) {
    gpio_set_level(cfg::PIN_FAN, on ? 1 : 0);
    s_on = on;
}

}  // namespace

void fan_init(Axis* tr, Axis* fr) {
    s_tr = tr;
    s_fr = fr;

    gpio_config_t out_cfg = {};
    out_cfg.mode         = GPIO_MODE_OUTPUT;
    out_cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
    out_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    out_cfg.intr_type    = GPIO_INTR_DISABLE;
    out_cfg.pin_bit_mask = (1ULL << cfg::PIN_FAN);
    gpio_config(&out_cfg);

    // Definierter Grundzustand beim Boot: LOW = Luefter aus. Die Zeitspanne
    // VOR diesem Init (Pin hochohmig) deckt der externe Gate-Pull-down des
    // MOSFETs ab (siehe Pin-Doku in config.h).
    apply(false);
    s_auto = true;
}

void fan_update() {
    if (!s_auto) return;   // manueller Override hat Vorrang
    // isRunning() statt Soll-Speed: true genau solange der Motor Schritte
    // macht -- inklusive Auslauframpe nach STOP, exklusive "SPEED gesetzt,
    // aber nie gestartet".
    const bool demand = (s_tr != nullptr && s_tr->isRunning()) ||
                        (s_fr != nullptr && s_fr->isRunning());
    if (demand != s_on) apply(demand);
}

void fan_set(bool on) {
    s_auto = false;
    apply(on);
}

void fan_auto() {
    s_auto = true;   // Zustand stellt der naechste fan_update() (control_task)
}

bool fan_is_on()   { return s_on; }
bool fan_is_auto() { return s_auto; }
