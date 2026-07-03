// axis.h - Kapselt eine Motorachse auf Basis von FastAccelStepper.
//
// Eine Achse arbeitet in genau einem Modus:
//   Rotation  : Dauerlauf mit Geschwindigkeit/Richtung (TR, FR)
//   Position  : Fahren auf Ziel + Homing + Endlagen (FV, TN)
//
// TN besitzt zusaetzlich einen AS5600-Winkelsensor und fuehrt eine geschlossene
// (closed-loop) Fahrt auf einen Sollwinkel aus -> startMoveToAngle().
//
// Die FastAccelStepper-spezifischen Aufrufe sind ausschliesslich in axis.cpp
// gekapselt; der Rest der Firmware kennt nur dieses Interface.
#pragma once

#include <cstdint>
#include "driver/gpio.h"

class FastAccelStepperEngine;  // Vorwaertsdeklaration (Lib-Header nur in .cpp)
class FastAccelStepper;
class AS5600;                  // Winkelsensor (nur in axis.cpp eingebunden)

enum class AxisMode { Rotation, Position };

// Zustand der nicht-blockierenden Homing-State-Machine.
enum class HomingState { Idle, Seeking, Backoff, Done, Error };

// Zustand der geschlossenen Winkelregelung (closed-loop, TN).
enum class ClState { Idle, Moving, Done, Error };

// Parameter der closed-loop-Winkelregelung (aus config zusammengesetzt).
struct AngleControlCfg {
    float    offset_deg     = -105.29f;  // Sensor-Rohgrad bei mechanisch 0 Grad
    int      angle_sign     = 1;     // Vorzeichen des gemeldeten Winkels (+1/-1)
    int      motor_sign     = -1;     // +1: Motor-Vorwaerts erhoeht Winkel, sonst -1
    float    min_deg        = 0.0f;  // erlaubter Soll-Bereich (Minimum)
    float    max_deg        = 90.0f; // erlaubter Soll-Bereich (Maximum)
    float    tol_deg        = 0.2f;  // Zieltoleranz
    float    approach_hz    = 500.0f;// langsame Annaeherungsgeschwindigkeit
    uint32_t approach_accel = 1500;  // sanfte Rampe (steps/s^2)
    float    damping        = 0.9f;  // Anteil des Fehlers je Teilschritt
    int      settle_ticks   = 3;     // n Messungen in Toleranz = fertig
    uint32_t timeout_ms     = 20000; // Sicherheits-Timeout
};

class Axis {
public:
    Axis(const char* name,
         gpio_num_t step_pin,
         gpio_num_t dir_pin,
         bool dir_invert,
         AxisMode mode,
         float min_hz,
         float max_hz,
         uint32_t accel);

    // Verbindet den Stepper mit der Engine und setzt Pins/Beschleunigung.
    // Rueckgabe false, wenn kein Stepper-Slot frei war.
    bool attach(FastAccelStepperEngine& engine);

    // Zusatzkonfiguration fuer Positionierachsen (Umrechnung + Endlagen).
    // home_dir_positive: Richtung der Referenzfahrt (false = Richtung MIN).
    void configurePosition(float steps_per_unit,
                           gpio_num_t limit_min,
                           gpio_num_t limit_max,
                           bool home_dir_positive);

    // ----- Rotationsmodus -----
    bool setSpeed255(uint8_t value);     // 0..255, 0 = Stop
    bool setDirection(bool positive);    // true = rechts/positiv
    bool start();                        // Dauerlauf in aktueller Richtung/Speed

    // ----- Positioniermodus -----
    bool setPositionSpeed255(uint8_t value);  // Fahrgeschwindigkeit 0..255
    bool moveToUnit(float unit);              // Fahre auf Ziel (mm bzw. Grad)
    bool jog(bool forward);                   // Vor/Zurueck bis Endlage
    bool home();                              // Referenzfahrt starten
    void setCalibration(float steps_per_unit);// steps/mm bzw. steps/deg setzen
    float positionUnit() const;               // Ist-Position aus Schrittzaehler

    // ----- Winkelsensor / Closed-Loop (nur TN) -----
    // Koppelt einen AS5600 an diese Achse und uebernimmt die Regelparameter.
    void attachAngleSensor(AS5600* sensor, const AngleControlCfg& cfg);
    bool  hasAngleSensor()  const { return sensor_ != nullptr; }
    // Aktueller Ist-Winkel in Grad (gepufferter Sensorwert, in update() erneuert).
    float sensorAngleDeg()  const { return last_angle_deg_; }
    bool  sensorMagnetOk()  const { return last_magnet_ok_; }
    ClState angleState()    const { return cl_state_; }
    // Startet die behutsame Fahrt auf 'target_deg' (closed-loop).
    bool  startMoveToAngle(float target_deg);
    // Setzt den aktuellen Ist-Winkel als neuen Nullpunkt (0 Grad).
    void  setAngleZero();

    // ----- gemeinsam -----
    bool stop();        // sanft anhalten (Rampe)
    void forceStop();   // sofort anhalten (Sicherheit)

    // Periodisch aus der Steuer-Task: Sensor lesen, Endlagen, Homing-/CL-SM.
    void update();

    // ----- Status -----
    bool         isRunning()  const;
    AxisMode     mode()       const { return mode_; }
    HomingState  homing()     const { return homing_; }
    bool         limitMin()   const;
    bool         limitMax()   const;
    const char*  name()       const { return name_; }

private:
    float hzFrom255(uint8_t value) const;  // lineares Mapping 0..255 -> Hz
    void  updateAngleControl();            // closed-loop SM (in update())
    float computeAngle(uint16_t raw) const;// Rohwert -> gemeldeter Winkel
    // Darf die Achse in 'forward'-Richtung fahren? Beruecksichtigt die FV-
    // Endschalter und das TN-Zonengatter inkl. AS5600-Re-Entry (Rueckfahrt zur
    // Zonenmitte, wenn das Gatter verlassen wurde).
    bool  driveAllowed(bool forward) const;

    const char* name_;
    gpio_num_t  step_pin_;
    gpio_num_t  dir_pin_;
    bool        dir_invert_;
    AxisMode    mode_;
    float       min_hz_;
    float       max_hz_;
    uint32_t    accel_;

    FastAccelStepper* stepper_ = nullptr;

    // Rotationszustand
    bool    dir_positive_ = true;   // aktuelle Richtung
    uint8_t speed255_     = 0;      // zuletzt gesetzte Geschwindigkeit

    // Positionszustand
    float       steps_per_unit_  = 1.0f;
    gpio_num_t  limit_min_pin_   = GPIO_NUM_NC;
    gpio_num_t  limit_max_pin_   = GPIO_NUM_NC;
    bool        home_dir_pos_    = false;
    uint8_t     pos_speed255_    = 0;
    // Zonengatter (TN): EIN Sensor auf MIN und MAX -> symmetrische Hard-Zone.
    // Aktiviert die richtungsabhaengige Re-Entry-Logik in driveAllowed().
    bool        zone_gate_       = false;

    // Homing-State-Machine
    HomingState homing_ = HomingState::Idle;

    // Winkelsensor + Closed-Loop
    AS5600*         sensor_ = nullptr;
    AngleControlCfg acfg_{};
    volatile float  last_angle_deg_ = 0.0f;  // gepufferter Ist-Winkel
    volatile bool   last_magnet_ok_ = false;
    volatile bool   cl_active_      = false;  // Regelung laeuft
    ClState         cl_state_       = ClState::Idle;
    float           cl_target_      = 0.0f;   // Sollwinkel
    uint32_t        cl_start_ms_    = 0;      // Startzeit (fuer Timeout)
    int             cl_settle_      = 0;      // Zaehler "in Toleranz"
};
