// axis.cpp - Implementierung der Achse auf Basis von FastAccelStepper.
//
// Hinweis zur Lib-Integration (FastAccelStepper, ESP-IDF):
//   - FastAccelStepperEngine::init() startet den internen Rampen-Task.
//   - stepperConnectToPin(stepPin) liefert pro Achse ein FastAccelStepper*.
//   - Der gemeinsame Enable (EN) wird der Lib ueberlassen (Auto-Enable). Alle
//     vier Achsen bekommen denselben PIN_EN; FastAccelStepper schaltet ihn erst
//     stromlos, wenn ALLE Achsen am gemeinsamen Pin stehen (Koordination in der
//     Engine). Kein Haltestrom im Stillstand, keine Positions-Verluste (A4988
//     behaelt den Mikroschritt-Zaehler ueber ~ENABLE).
//
// TN-Besonderheit: closed-loop-Winkelregelung mit AS5600 (siehe unten).
#include "axis.h"

#include <cmath>

#include "config.h"
#include "as5600.h"
#include "FastAccelStepper.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char* TAG = "axis";

// Millisekunden seit Boot (fuer Timeout der Winkelfahrt).
static inline uint32_t now_ms() {
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

Axis::Axis(const char* name, gpio_num_t step_pin, gpio_num_t dir_pin,
           bool dir_invert, AxisMode mode, float min_hz, float max_hz,
           uint32_t accel)
    : name_(name), step_pin_(step_pin), dir_pin_(dir_pin),
      dir_invert_(dir_invert), mode_(mode), min_hz_(min_hz), max_hz_(max_hz),
      accel_(accel) {}

bool Axis::attach(FastAccelStepperEngine& engine) {
    stepper_ = engine.stepperConnectToPin(step_pin_);
    if (stepper_ == nullptr) {
        ESP_LOGE(TAG, "%s: kein Stepper-Slot frei (STEP=%d)", name_, step_pin_);
        return false;
    }
    // DIR-Pin mit Invert-Flag. Zweites Argument = dirPinHighCountsUp:
    // true  -> DIR HIGH zaehlt aufwaerts (A4988)
    // false -> invertiert (z.B. TMC2209)
    stepper_->setDirectionPin(dir_pin_, !dir_invert_);

    // Gemeinsamer Enable-Pin fuer alle vier Achsen -> Auto-Enable der Lib.
    // Reihenfolge wichtig: setEnablePin MUSS vor setAutoEnable kommen.
    // Die Engine deaktiviert den geteilten Pin erst, wenn ALLE Achsen stehen.
    stepper_->setEnablePin(static_cast<uint8_t>(cfg::PIN_EN), cfg::ENABLE_ACTIVE_LOW);
    stepper_->setAutoEnable(cfg::STEPPER_AUTO_ENABLE);
    stepper_->setDelayToDisable(cfg::STEPPER_DISABLE_DELAY_MS);  // Argument in ms
    stepper_->setDelayToEnable(cfg::STEPPER_ENABLE_DELAY_US);    // Argument in us

    stepper_->setAcceleration(accel_);
    stepper_->setSpeedInHz(static_cast<uint32_t>(min_hz_));
    pos_speed255_ = cfg::POS_DEFAULT_SPEED;
    ESP_LOGI(TAG, "%s attached (STEP=%d DIR=%d invert=%d)",
             name_, step_pin_, dir_pin_, dir_invert_);
    return true;
}

void Axis::configurePosition(float steps_per_unit, gpio_num_t limit_min,
                             gpio_num_t limit_max, bool home_dir_positive) {
    steps_per_unit_ = steps_per_unit;
    limit_min_pin_  = limit_min;
    limit_max_pin_  = limit_max;
    home_dir_pos_   = home_dir_positive;
    // Gleicher Sensor auf MIN und MAX -> Zonengatter (TN). Aktiviert die
    // Re-Entry-Logik in driveAllowed(): bei verlassenem Gatter nur Rueckfahrt.
    zone_gate_ = (limit_min != cfg::PIN_NONE) && (limit_min == limit_max);
}

float Axis::hzFrom255(uint8_t value) const {
    if (value == 0) return 0.0f;
    return min_hz_ + (max_hz_ - min_hz_) * (static_cast<float>(value) / 255.0f);
}

// ---------------------------------------------------------------------------
//  Rotationsmodus
// ---------------------------------------------------------------------------
bool Axis::setSpeed255(uint8_t value) {
    if (mode_ != AxisMode::Rotation || stepper_ == nullptr) return false;
    speed255_ = value;
    if (value == 0) {
        stepper_->stopMove();   // sanft auslaufen
        return true;
    }
    stepper_->setSpeedInHz(static_cast<uint32_t>(hzFrom255(value)));
    // Wenn bereits in Bewegung: neue Geschwindigkeit live uebernehmen.
    if (stepper_->isRunning()) {
        stepper_->applySpeedAcceleration();
    }
    return true;
}

bool Axis::setDirection(bool positive) {
    if (mode_ != AxisMode::Rotation) return false;
    dir_positive_ = positive;
    // Bei laufender Achse Richtung sofort neu kommandieren.
    if (stepper_ != nullptr && stepper_->isRunning() && speed255_ > 0) {
        return start();
    }
    return true;
}

bool Axis::start() {
    if (mode_ != AxisMode::Rotation || stepper_ == nullptr) return false;
    if (speed255_ == 0) return false;   // ohne Geschwindigkeit kein Start
    // Treiber-Freigabe uebernimmt Auto-Enable automatisch beim ersten Schritt.
    stepper_->setSpeedInHz(static_cast<uint32_t>(hzFrom255(speed255_)));
    if (dir_positive_) stepper_->runForward();
    else               stepper_->runBackward();
    return true;
}

// ---------------------------------------------------------------------------
//  Positioniermodus
// ---------------------------------------------------------------------------
bool Axis::setPositionSpeed255(uint8_t value) {
    if (mode_ != AxisMode::Position || stepper_ == nullptr) return false;
    pos_speed255_ = value;
    if (value > 0) {
        stepper_->setSpeedInHz(static_cast<uint32_t>(hzFrom255(value)));
        if (stepper_->isRunning()) stepper_->applySpeedAcceleration();
    }
    return true;
}

bool Axis::moveToUnit(float unit) {
    if (mode_ != AxisMode::Position || stepper_ == nullptr) return false;
    if (cl_active_) return false;       // waehrend Winkelregelung kein Direktfahren
    if (homing_ == HomingState::Seeking || homing_ == HomingState::Backoff)
        return false;  // waehrend Referenzfahrt keine Fahrbefehle
    int32_t target = static_cast<int32_t>(unit * steps_per_unit_);
    // Fahrtrichtung aus Ziel vs. Ist ableiten und gegen die Endlage pruefen,
    // damit eine Direktfahrt nicht in eine gesperrte Endlage kommandiert wird.
    bool forward = (target > stepper_->getCurrentPosition());
    if (!driveAllowed(forward)) return false;
    dir_positive_ = forward;
    uint8_t v = (pos_speed255_ > 0) ? pos_speed255_ : cfg::POS_DEFAULT_SPEED;
    stepper_->setAcceleration(accel_);
    stepper_->setSpeedInHz(static_cast<uint32_t>(hzFrom255(v)));
    stepper_->moveTo(target);
    return true;
}

bool Axis::jog(bool forward) {
    if (mode_ != AxisMode::Position || stepper_ == nullptr) return false;
    if (cl_active_) return false;
    if (homing_ == HomingState::Seeking || homing_ == HomingState::Backoff)
        return false;
    // Sensor-Freigabe in Fahrtrichtung? (FV-Endschalter bzw. TN-Gatter/Re-Entry)
    if (!driveAllowed(forward)) return false;
    uint8_t v = (pos_speed255_ > 0) ? pos_speed255_ : cfg::POS_DEFAULT_SPEED;
    stepper_->setAcceleration(accel_);
    stepper_->setSpeedInHz(static_cast<uint32_t>(hzFrom255(v)));
    dir_positive_ = forward;
    if (forward) stepper_->runForward();
    else         stepper_->runBackward();
    return true;
}

bool Axis::home() {
    if (mode_ != AxisMode::Position || stepper_ == nullptr) return false;
    if (cl_active_) return false;
    // Mit moderater Geschwindigkeit Richtung Referenzschalter fahren.
    uint8_t v = (pos_speed255_ > 0) ? pos_speed255_ : cfg::POS_DEFAULT_SPEED;
    stepper_->setAcceleration(accel_);
    stepper_->setSpeedInHz(static_cast<uint32_t>(hzFrom255(v)));
    dir_positive_ = home_dir_pos_;
    if (home_dir_pos_) stepper_->runForward();
    else               stepper_->runBackward();
    homing_ = HomingState::Seeking;
    return true;
}

void Axis::setCalibration(float steps_per_unit) {
    if (steps_per_unit > 0.0f) steps_per_unit_ = steps_per_unit;
}

// ---------------------------------------------------------------------------
//  Low-Level-Primitive fuer die FV-Ablaufsteuerung (fv-Modul)
// ---------------------------------------------------------------------------
// Dauerfahrt mit expliziter Schrittfrequenz (fuer die Homing-Suchphasen).
// Endlagen-Gating bleibt aktiv: in eine gesperrte Richtung wird nicht
// gestartet; das Erreichen der Schranke stoppt via update() (Auto-Stopp).
bool Axis::runAtHz(bool forward, float hz) {
    if (mode_ != AxisMode::Position || stepper_ == nullptr) return false;
    if (cl_active_) return false;
    if (hz <= 0.0f) return false;
    if (!driveAllowed(forward)) return false;
    dir_positive_ = forward;
    stepper_->setAcceleration(accel_);
    stepper_->setSpeedInHz(static_cast<uint32_t>(hz));
    if (forward) stepper_->runForward();
    else         stepper_->runBackward();
    return true;
}

// Zielfahrt (absolute Schrittposition) mit expliziter Schrittfrequenz.
bool Axis::moveToStepsAtHz(int32_t steps, float hz) {
    if (mode_ != AxisMode::Position || stepper_ == nullptr) return false;
    if (cl_active_) return false;
    if (hz <= 0.0f) return false;
    bool forward = (steps > stepper_->getCurrentPosition());
    if (!driveAllowed(forward)) return false;
    dir_positive_ = forward;
    stepper_->setAcceleration(accel_);
    stepper_->setSpeedInHz(static_cast<uint32_t>(hz));
    stepper_->moveTo(steps);
    return true;
}

int32_t Axis::positionSteps() const {
    return (stepper_ != nullptr) ? stepper_->getCurrentPosition() : 0;
}

void Axis::setCurrentPositionSteps(int32_t steps) {
    if (stepper_ != nullptr) stepper_->setCurrentPosition(steps);
}

// Aktuelle Positionier-Geschwindigkeit in Hz (GUI-Speed oder Default).
float Axis::positionSpeedHz() const {
    uint8_t v = (pos_speed255_ > 0) ? pos_speed255_ : cfg::POS_DEFAULT_SPEED;
    return hzFrom255(v);
}

float Axis::positionUnit() const {
    if (stepper_ == nullptr || steps_per_unit_ == 0.0f) return 0.0f;
    return static_cast<float>(stepper_->getCurrentPosition()) / steps_per_unit_;
}

// ---------------------------------------------------------------------------
//  Winkelsensor / Closed-Loop (nur TN)
// ---------------------------------------------------------------------------
void Axis::attachAngleSensor(AS5600* sensor, const AngleControlCfg& ctrl) {
    sensor_ = sensor;
    acfg_   = ctrl;
}

// Rohwert (0..4095) -> gemeldeter Winkel in Grad, zentriert um den Nullpunkt.
// Normalisierung auf (-180,180], damit der Arbeitsbereich nicht am 0/360-Sprung
// haengt; danach Vorzeichen anwenden.
float Axis::computeAngle(uint16_t raw) const {
    float raw_deg = raw * (360.0f / 4096.0f);
    float a = raw_deg - acfg_.offset_deg;
    while (a > 180.0f)   a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
    return static_cast<float>(acfg_.angle_sign) * a;
}

bool Axis::startMoveToAngle(float target_deg) {
    if (mode_ != AxisMode::Position || stepper_ == nullptr) return false;
    if (sensor_ == nullptr) return false;
    if (homing_ == HomingState::Seeking || homing_ == HomingState::Backoff)
        return false;
    if (!last_magnet_ok_) return false;   // ohne erkannten Magnet nicht starten

    // Sollwinkel auf erlaubten Bereich begrenzen.
    if (target_deg < acfg_.min_deg) target_deg = acfg_.min_deg;
    if (target_deg > acfg_.max_deg) target_deg = acfg_.max_deg;

    stepper_->setAcceleration(acfg_.approach_accel);

    cl_target_       = target_deg;
    cl_start_ms_     = now_ms();
    cl_settle_       = 0;
    cl_err_positive_ = (target_deg - last_angle_deg_) > 0.0f;
    cl_move_pending_ = false;
    cl_state_        = ClState::Moving;
    cl_active_       = true;
    ESP_LOGI(TAG, "%s: Winkelfahrt -> %.2f Grad (ist %.2f)",
             name_, target_deg, last_angle_deg_);
    return true;
}

void Axis::setAngleZero() {
    if (sensor_ == nullptr) return;
    uint16_t raw = 0;
    if (sensor_->readRawAngle(raw) == ESP_OK) {
        acfg_.offset_deg = raw * (360.0f / 4096.0f);  // aktueller Rohwert = 0 Grad
        last_angle_deg_  = computeAngle(raw);          // ~0
        ESP_LOGI(TAG, "%s: Nullpunkt gesetzt (offset=%.2f)", name_, acfg_.offset_deg);
    }
}

// Fehlerproportionale Geschwindigkeit fuer die Winkelregelung: ab fast_deg
// Restfehler voller Eilgang (fast_hz), unterhalb tol_deg Feinfahrt
// (approach_hz), dazwischen linear. Begrenzt auf das Achsen-Limit max_hz_.
uint32_t Axis::clSpeedForError(float abs_err_deg) const {
    float lo = acfg_.approach_hz;
    float hi = acfg_.fast_hz;
    if (hi < lo)      hi = lo;        // Fehlkonfiguration abfangen
    if (hi > max_hz_) hi = max_hz_;   // mechanisches Limit der Achse
    float span = acfg_.fast_deg - acfg_.tol_deg;
    float t = (span > 0.0f) ? (abs_err_deg - acfg_.tol_deg) / span : 1.0f;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return static_cast<uint32_t>(lo + (hi - lo) * t);
}

// Geschlossene Regelung: in gedaempften Teilschritten an den Sollwinkel
// heranfahren und nach jeder Bewegung neu messen. Waehrend einer laufenden
// Teilbewegung wird der Restfehler live ueberwacht (Geschwindigkeit
// nachfuehren, bei Ueberschiessen sofort stoppen). Ein adaptiver Gain lernt
// die reale Steps/Grad-Uebersetzung, falls steps_per_unit_ nicht stimmt
// (z.B. Getriebe zwischen Motor und Neige-Welle).
void Axis::updateAngleControl() {
    // Sicherheits-Timeout.
    if (now_ms() - cl_start_ms_ > acfg_.timeout_ms) {
        stepper_->forceStop();
        cl_state_  = ClState::Error;
        cl_active_ = false;
        ESP_LOGW(TAG, "%s: Winkelfahrt Timeout", name_);
        return;
    }
    // Magnet verloren -> sofort abbrechen.
    if (!last_magnet_ok_) {
        stepper_->forceStop();
        cl_state_  = ClState::Error;
        cl_active_ = false;
        ESP_LOGW(TAG, "%s: Magnet nicht erkannt -> Abbruch", name_);
        return;
    }

    float err  = cl_target_ - last_angle_deg_;
    float aerr = fabsf(err);

    // Innerhalb der Toleranz: anhalten und Beruhigung abwarten.
    if (aerr <= acfg_.tol_deg) {
        if (stepper_->isRunning()) {
            stepper_->stopMove();          // sanft anhalten
        } else if (++cl_settle_ >= acfg_.settle_ticks) {
            cl_state_  = ClState::Done;
            cl_active_ = false;
            ESP_LOGI(TAG, "%s: Zielwinkel erreicht (%.2f Grad)", name_, last_angle_deg_);
        }
        return;
    }
    cl_settle_ = 0;

    if (stepper_->isRunning()) {
        // Live-Ueberwachung der laufenden Teilbewegung:
        // 1) Ziel ueberschossen (Fehler-Vorzeichen gedreht) -> sofort anhalten.
        if ((err > 0.0f) != cl_err_positive_) {
            stepper_->stopMove();
            return;
        }
        // 2) Geschwindigkeit dem Restfehler nachfuehren (Eilgang -> Feinfahrt).
        stepper_->setSpeedInHz(clSpeedForError(aerr));
        stepper_->applySpeedAcceleration();
        return;
    }

    // Abgeschlossene Teilbewegung auswerten: kommandierte vs. tatsaechlich
    // gefahrene Grad vergleichen und den Gain nachfuehren. Kompensiert eine
    // falsche steps_per_unit_-Kalibrierung, ohne moveToUnit() zu beeinflussen.
    if (cl_move_pending_) {
        cl_move_pending_ = false;
        float actual = last_angle_deg_ - cl_move_start_deg_;
        if (fabsf(cl_move_cmd_deg_) >= 0.5f) {
            if (fabsf(actual) >= 0.1f) {
                float k = cl_move_cmd_deg_ / actual;
                if (k > 0.0f) {            // nur bei plausibler Richtung anpassen
                    cl_gain_ *= k;
                    if (cl_gain_ < 0.1f)  cl_gain_ = 0.1f;
                    if (cl_gain_ > 20.0f) cl_gain_ = 20.0f;
                }
            } else {
                // Kaum Bewegung trotz deutlichem Kommando: Uebersetzung stark
                // unterschaetzt -> Gain verdoppeln (begrenzt; Timeout sichert ab).
                cl_gain_ *= 2.0f;
                if (cl_gain_ > 20.0f) cl_gain_ = 20.0f;
            }
        }
    }

    // Naechste, gedaempfte Korrektur berechnen. Mindestens 1 Schritt, damit
    // die Regelung oberhalb der Toleranz nicht vorzeitig "fertig" meldet.
    float   cmd_deg = err * acfg_.damping;
    float   steps_f = cmd_deg * steps_per_unit_ * cl_gain_
                      * static_cast<float>(acfg_.motor_sign);
    int32_t steps   = static_cast<int32_t>(lroundf(steps_f));
    if (steps == 0) steps = (steps_f >= 0.0f) ? 1 : -1;

    dir_positive_ = (steps > 0);
    // Sensor-Freigabe in Fahrtrichtung pruefen. Beim TN-Zonengatter erlaubt
    // driveAllowed() ausserhalb der Zone nur die Rueckfahrt zur Mitte (Re-Entry)
    // und sperrt die Fahrt hinaus -> Sicherheit ohne Deadlock.
    if (!driveAllowed(dir_positive_)) {
        stepper_->forceStop(); cl_state_ = ClState::Error; cl_active_ = false;
        ESP_LOGW(TAG, "%s: Sensor sperrt Winkelfahrt-Richtung", name_);
        return;
    }

    // Bezugswerte fuer die Auswertung dieser Teilbewegung merken.
    cl_err_positive_   = (err > 0.0f);
    cl_move_start_deg_ = last_angle_deg_;
    cl_move_cmd_deg_   = cmd_deg;
    cl_move_pending_   = true;

    stepper_->setSpeedInHz(clSpeedForError(aerr));
    stepper_->move(steps);                 // relative Teilbewegung (gerampt)
}

// ---------------------------------------------------------------------------
//  gemeinsam
// ---------------------------------------------------------------------------
bool Axis::stop() {
    if (stepper_ == nullptr) return false;
    stepper_->stopMove();           // mit Rampe anhalten
    if (mode_ == AxisMode::Rotation) speed255_ = 0;
    cl_active_ = false;             // laufende Winkelregelung abbrechen
    if (cl_state_ == ClState::Moving) cl_state_ = ClState::Idle;
    return true;
}

void Axis::forceStop() {
    if (stepper_ == nullptr) return;
    stepper_->forceStop();          // sofort, ohne Rampe
    if (mode_ == AxisMode::Rotation) speed255_ = 0;
    if (homing_ == HomingState::Seeking || homing_ == HomingState::Backoff)
        homing_ = HomingState::Error;
    cl_active_ = false;             // Winkelregelung abbrechen
    if (cl_state_ == ClState::Moving) cl_state_ = ClState::Idle;
}

bool Axis::limitMin() const { return cfg::limit_triggered(limit_min_pin_); }
bool Axis::limitMax() const { return cfg::limit_triggered(limit_max_pin_); }

// Richtungsabhaengige Freigabe (zentrale Stelle fuer FV- und TN-Gating).
//   FV (gerichtete Endschalter): Vorwaerts nur frei, wenn der FWD-Endschalter
//   nicht gesperrt ist; Rueckwaerts symmetrisch ueber den BACK-Endschalter.
//   Die Gegenrichtung bleibt also immer frei, um aus der Endlage zu fahren.
//
//   TN (Zonengatter + AS5600): im Gatter (HIGH) volle Freigabe. Verlaesst die
//   Achse das Gatter (LOW), darf sie NICHT komplett blockieren -> Re-Entry:
//   Anhand des AS5600-Absolutwinkels wird die Seite relativ zur Zonenmitte
//   bestimmt und nur die Bewegung ZURUECK zur Mitte (ins Gatter) erlaubt.
bool Axis::driveAllowed(bool forward) const {
    if (!zone_gate_) {
        // FV: gerichtete Endschalter (limit_max_pin_ = FWD, limit_min_pin_ = BACK).
        return forward ? !limitMax() : !limitMin();
    }

    // TN-Zonengatter: HIGH = im Gatter -> normale Freigabe.
    if (cfg::sensor_tn_drive_allowed()) return true;

    // Ausserhalb des Gatters: Seite nur mit gueltigem Magnet bestimmbar.
    // Ohne Winkel keine sichere Entscheidung -> fail-safe sperren.
    if (sensor_ == nullptr || !last_magnet_ok_) return false;

    // Zonenmitte = Mitte des erlaubten Soll-Winkelbereichs.
    const float center = 0.5f * (acfg_.min_deg + acfg_.max_deg);
    const bool  above_center      = last_angle_deg_ > center;   // Seite der Zone
    const bool  forward_increases = (acfg_.motor_sign > 0);     // Motor-Vorw. -> Winkel?
    // Unterhalb der Mitte muss der Winkel groesser werden (und umgekehrt).
    const bool  need_increase     = !above_center;
    // Erlaubt, wenn die gewuenschte Fahrtrichtung den Winkel Richtung Mitte bewegt.
    return forward == (need_increase == forward_increases);
}

bool Axis::isRunning() const {
    return stepper_ != nullptr && stepper_->isRunning();
}

// ---------------------------------------------------------------------------
//  Periodische Aktualisierung: Sensor + Endlagen-Sicherheit + Homing/CL-SM
// ---------------------------------------------------------------------------
void Axis::update() {
    if (stepper_ == nullptr) return;

    // Winkelsensor pollen (nur wenn vorhanden) -> Ist-Winkel + Magnet puffern.
    if (sensor_ != nullptr) {
        uint16_t raw = 0;
        if (sensor_->readRawAngle(raw) == ESP_OK) {
            last_angle_deg_ = computeAngle(raw);
            last_magnet_ok_ = sensor_->magnetDetected();
        } else {
            last_magnet_ok_ = false;
        }
    }

    if (mode_ != AxisMode::Position) return;

    // Closed-Loop-Winkelregelung hat Vorrang vor Homing/Endlagen-Standardlogik.
    if (cl_active_) { updateAngleControl(); return; }

    switch (homing_) {
    case HomingState::Seeking:
        // Referenzschalter erreicht?
        if ((home_dir_pos_ && limitMax()) || (!home_dir_pos_ && limitMin())) {
            stepper_->forceStop();
            stepper_->setCurrentPosition(0);   // Referenzpunkt = 0
            // Vom Schalter zurueckziehen (entgegen Referenzrichtung).
            int32_t backoff = home_dir_pos_ ? -cfg::HOMING_BACKOFF_STEPS
                                            :  cfg::HOMING_BACKOFF_STEPS;
            stepper_->move(backoff);
            homing_ = HomingState::Backoff;
        }
        break;

    case HomingState::Backoff:
        if (!stepper_->isRunning()) {
            homing_ = HomingState::Done;       // Referenzfahrt fertig
            ESP_LOGI(TAG, "%s homing done", name_);
        }
        break;

    default:
        break;
    }

    // --- Endlagen-/Zonen-Sicherheit (immer, auch ausserhalb Homing) ---
    // Faehrt die Achse in eine gesperrte Richtung (FV-Endschalter ausgeloest
    // oder TN-Gatter verlassen und Bewegung nicht Richtung Zonenmitte), sofort
    // stoppen. driveAllowed() kapselt FV-Endschalter und TN-Re-Entry.
    if (homing_ != HomingState::Seeking && stepper_->isRunning() &&
        !driveAllowed(dir_positive_)) {
        stepper_->forceStop();
        ESP_LOGW(TAG, "%s: Sensor-Sperre in Fahrtrichtung -> Stop", name_);
    }
}
