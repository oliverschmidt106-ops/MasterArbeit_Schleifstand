// config.h - Zentrale Konfiguration: Pins, Konstanten, Invert-Flags, Umrechnungen.
//
// ALLE einstellbaren Werte stehen hier als dokumentierte Konstanten.
// Bezeichner englisch, Kommentare deutsch. Achs-Kuerzel: TR/FR/FV/TN.
#pragma once

#include <cstdint>
#include "driver/gpio.h"
#include "driver/uart.h"

namespace cfg {

// ===========================================================================
//  Treiber-Auswahl (zentraler Schalter)
// ===========================================================================
// A4988:           DIR nicht invertiert  -> false
// TMC2209 (Standalone): DIR invertiert    -> true
// Wechsel = nur diese eine Zeile aendern. Pro Achse kann unten zusaetzlich
// nachjustiert werden, falls die Verdrahtung einer Achse abweicht.
constexpr bool DRIVER_IS_TMC2209 = false;

// ===========================================================================
//  Mechanik / Schritt-Umrechnung
// ===========================================================================
constexpr int MICROSTEPS          = 16;                                  // Jumper 1/16
constexpr int FULLSTEPS_PER_REV   = 200;                                 // QSH4218
constexpr int STEPS_PER_REV       = FULLSTEPS_PER_REV * MICROSTEPS;      // = 3200

// FV (Faservorschub): Umrechnung Schritte <-> mm.
// ANPASSEN an die Spindelsteigung der Mechanik!
// Beispiel: Steigung 2 mm/Umdrehung -> 3200 / 2 = 1600 steps/mm.
constexpr float FV_LEADSCREW_PITCH_MM = 2.0f;
constexpr float FV_STEPS_PER_MM       = STEPS_PER_REV / FV_LEADSCREW_PITCH_MM;

// TN (Tischneigung): Umrechnung Schritte <-> Grad.
// MUSS kalibrierbar sein -> hier nur Default-Startwert; zur Laufzeit ueber
// Protokollbefehl  TN:CAL:<steps_per_deg>  ueberschreibbar.
// Default-Platzhalter: keine Untersetzung angenommen (3200/360).
constexpr float TN_STEPS_PER_DEG_DEFAULT = STEPS_PER_REV / 360.0f;

// TR (Tischrotation): Radius fuer optionale Umfangsgeschwindigkeit
// v = 2*pi*n*r  (Maschinenkonstante, in mm). Nur fuer Anzeige/Umrechnung.
constexpr float TR_RADIUS_MM = 50.0f;

// ===========================================================================
//  GPIO-Pinbelegung (ESP32-S3)
// ===========================================================================
// Schrittmotoren: STEP/DIR pro Achse
constexpr gpio_num_t PIN_TR_STEP = GPIO_NUM_4;   // Tischrotation
constexpr gpio_num_t PIN_TR_DIR  = GPIO_NUM_5;
constexpr gpio_num_t PIN_FR_STEP = GPIO_NUM_6;   // Faserrotation
constexpr gpio_num_t PIN_FR_DIR  = GPIO_NUM_7;
constexpr gpio_num_t PIN_FV_STEP = GPIO_NUM_15;  // Faservorschub
constexpr gpio_num_t PIN_FV_DIR  = GPIO_NUM_16;
constexpr gpio_num_t PIN_TN_STEP = GPIO_NUM_17;  // Tischneigung
constexpr gpio_num_t PIN_TN_DIR  = GPIO_NUM_18;

// Gemeinsamer Enable (aktiv LOW) fuer ALLE vier A4988 (CNC-Shield: eine
// gemeinsame EN-Leitung, Arduino-Pin D8). Bei Boot deaktiviert (HIGH).
// TODO(Hardware): Konkreten GPIO der verdrahteten EN-Leitung hier eintragen!
constexpr gpio_num_t PIN_EN = GPIO_NUM_8;   // <-- PLATZHALTER, anpassen

// ===========================================================================
//  Stepper-Auto-Disable (FastAccelStepper Auto-Enable)
// ===========================================================================
// Alle vier Achsen teilen sich PIN_EN. FastAccelStepper bestromt die Treiber
// vor dem ersten Schritt und schaltet sie erst stromlos, wenn ALLE Achsen am
// gemeinsamen EN-Pin stehen (Koordination in FastAccelStepperEngine). Dadurch
// kein Haltestrom/keine Waerme im Stillstand. Position geht nicht verloren:
// der A4988 behaelt seinen Mikroschritt-Zaehler ueber ~ENABLE; FV (Spindel) und
// TN (Schneckengetriebe) sind zusaetzlich selbsthemmend.
constexpr bool     ENABLE_ACTIVE_LOW       = true;  // A4988: LOW = Treiber aktiv
constexpr bool     STEPPER_AUTO_ENABLE     = true;  // Auto-Enable global an
constexpr uint16_t STEPPER_DISABLE_DELAY_MS = 1000; // Verzoegerung bis Abschaltung nach Stillstand (ms)
constexpr uint32_t STEPPER_ENABLE_DELAY_US  = 50;   // Vorlauf nach Enable bis zum ersten Schritt (us)

// Lichtschranken SK-205NA-W (NPN Open-Collector). Jeder Signaleingang hat einen
// externen 4,7k Pull-up nach 3V3 (KEIN 5 V an GPIO; ESP32-S3 ist nicht
// 5-V-tolerant; die 5 V liegen nur an der Sensorversorgung: braun=+5V, blau=GND).
// Einheitliche Konvention aller 3 Sensoren (active-high):
//   Pin LOW  = "Bewegung erlaubt"   ·   Pin HIGH = "gesperrt/stop"
// Die Strahl-Polaritaet wird ueber die Aderwahl gesetzt, NICHT in der Firmware.
// Fail-safe: Drahtbruch / Sensor stromlos -> Pull-up zieht HIGH -> stop.
constexpr gpio_num_t PIN_TN_GATE     = GPIO_NUM_9;   // schwarz (Ader 4), Dark.on : HIGH = ausserhalb Zone -> stop
constexpr gpio_num_t PIN_FV_LIM_FWD  = GPIO_NUM_10;  // weiss  (Ader 2), Light.on: HIGH = Limit FWD  -> stop
constexpr gpio_num_t PIN_FV_LIM_BACK = GPIO_NUM_11;  // weiss  (Ader 2), Light.on: HIGH = Limit BACK -> stop

// Aktiv-Pegel & Software-Entprellung der Lichtschranken (gegen Motor-EMI).
constexpr int      LIMIT_ACTIVE_LEVEL = 1;   // HIGH = ausgeloest/stop
constexpr uint32_t LIMIT_DEBOUNCE_MS  = 3;   // Pegel muss so lange stabil sein, bevor er zaehlt
constexpr uint32_t LIMIT_SAMPLE_MS    = 1;   // Abtastperiode des Entprell-Filters (FreeRTOS-Tick = 1 ms)

// Beleuchtung (Ausgang -> MOSFET)
constexpr gpio_num_t PIN_LIGHT = GPIO_NUM_14;

// Zuordnung Sensoren -> Achs-Endlagen (das Achsmodell kennt MIN/MAX):
//   FV: gerichtete Endschalter. Vorwaertsfahrt (dir_positive) stoppt an MAX,
//       Rueckwaertsfahrt an MIN  ->  FWD = MAX, BACK = MIN.
constexpr gpio_num_t PIN_FV_LIMIT_MIN = PIN_FV_LIM_BACK;  // GPIO 11
constexpr gpio_num_t PIN_FV_LIMIT_MAX = PIN_FV_LIM_FWD;   // GPIO 10
//   TN: EIN Zonengatter als symmetrische Hard-Zone -> auf MIN UND MAX gelegt,
//       damit jede Fahrtrichtung stoppt, sobald HIGH (ausserhalb Zone). Keine
//       Referenzfunktion: der Absolutwinkel kommt vom AS5600.
constexpr gpio_num_t PIN_TN_LIMIT_MIN = PIN_TN_GATE;     // GPIO 9
constexpr gpio_num_t PIN_TN_LIMIT_MAX = PIN_TN_GATE;     // GPIO 9

// "Kein Pin" Marker fuer Achsen ohne Endlage
constexpr gpio_num_t PIN_NONE = GPIO_NUM_NC;       // = -1

// ===========================================================================
//  DIR-Invert pro Achse
// ===========================================================================
// Standard aus dem Treiber-Schalter; bei Bedarf einzeln per != umkehren.
constexpr bool TR_DIR_INVERT = DRIVER_IS_TMC2209;
constexpr bool FR_DIR_INVERT = DRIVER_IS_TMC2209;
constexpr bool FV_DIR_INVERT = DRIVER_IS_TMC2209;
constexpr bool TN_DIR_INVERT = DRIVER_IS_TMC2209;

// ===========================================================================
//  Geschwindigkeit & Beschleunigung pro Achse
// ===========================================================================
// GUI sendet 0..255. Firmware mappt linear auf [MIN_HZ, MAX_HZ]; 0 = Stop.
// ACCEL in steps/s^2 (Rampe gegen Schrittverlust bei hoher Drehzahl).
constexpr float    TR_MIN_HZ = 100.0f;  constexpr float    TR_MAX_HZ = 8000.0f;  constexpr uint32_t TR_ACCEL = 4000;
constexpr float    FR_MIN_HZ = 100.0f;  constexpr float    FR_MAX_HZ = 8000.0f;  constexpr uint32_t FR_ACCEL = 4000;
constexpr float    FV_MIN_HZ = 100.0f;  constexpr float    FV_MAX_HZ = 6000.0f;  constexpr uint32_t FV_ACCEL = 8000;
constexpr float    TN_MIN_HZ = 100.0f;  constexpr float    TN_MAX_HZ = 4000.0f;  constexpr uint32_t TN_ACCEL = 6000;

// Default-Fahrgeschwindigkeit (0..255) der Positionierachsen, falls die GUI
// noch keinen Speed gesetzt hat.
constexpr uint8_t  POS_DEFAULT_SPEED = 128;

// Homing-Rueckzug (steps) nach Erreichen des Endschalters.
constexpr int32_t  HOMING_BACKOFF_STEPS = 200;

// ===========================================================================
//  UART-Befehlskanal (LabVIEW <-> Firmware)
// ===========================================================================
// UART0 ist mit der USB-UART-Bridge des DevKitC verbunden -> der COM-Port,
// den LabVIEW per VISA oeffnet. Logging laeuft separat ueber USB-Serial-JTAG.
constexpr uart_port_t UART_PORT     = UART_NUM_0;
constexpr int         UART_TX_PIN   = 43;     // Standard-TX0 DevKitC-S3
constexpr int         UART_RX_PIN   = 44;     // Standard-RX0 DevKitC-S3
constexpr int         UART_BAUD     = 115200; // im README dokumentiert
constexpr int         UART_RX_BUF   = 1024;
constexpr int         UART_TX_BUF   = 1024;
constexpr char        LINE_TERM     = '\n';   // Termination Character fuer VISA

// Periode der Steuer-/Status-Task (Endlagen + Homing-SM + Sensor), in ms.
constexpr int CONTROL_PERIOD_MS = 10;

// ===========================================================================
//  AS5600 Winkelsensor (I2C) fuer die Neigeachse TN
// ===========================================================================
// Magnetischer Absolut-Encoder, 12 Bit (0..4095 = 0..360 Grad), I2C-Adresse 0x36.
// Verdrahtung: VCC=3,3 V, GND, SDA/SCL (Pull-ups meist auf dem Breakout),
// DIR-Pin des AS5600 fest auf GND (feste Zaehlrichtung). Sensor sitzt 1:1 auf
// der Neige-Ausgangswelle; nutzbarer Bereich < 360 Grad (eine Umdrehung).
constexpr gpio_num_t PIN_I2C_SDA = GPIO_NUM_1;
constexpr gpio_num_t PIN_I2C_SCL = GPIO_NUM_2;
constexpr uint32_t   I2C_FREQ_HZ = 400000;    // Fast-Mode
constexpr uint8_t    AS5600_ADDR = 0x36;      // feste I2C-Adresse

// Nullpunkt (Sensor-Rohgrad bei mechanisch 0 Grad). Zur Laufzeit per TN:AZERO.
constexpr float TN_ANGLE_OFFSET_DEG = 0.0f;
// Vorzeichen des GEMELDETEN Winkels (+1/-1), damit "mehr Neigung" positiv ist.
constexpr int   TN_ANGLE_SIGN = 1;
// Beziehung Motor -> Sensor: +1, wenn Motor-Vorwaerts den gemeldeten Winkel
// VERGROESSERT, sonst -1. (Beim Einrichten einmalig pruefen.)
constexpr int   TN_MOTOR_TO_SENSOR = 1;

// Erlaubter Soll-Winkelbereich (Grad). ANPASSEN an die Mechanik!
constexpr float TN_ANGLE_MIN_DEG = 0.0f;
constexpr float TN_ANGLE_MAX_DEG = 90.0f;

// Closed-Loop-Regelung "behutsam" (TN faehrt geregelt auf den Sollwinkel):
constexpr float    TN_CL_TOLERANCE_DEG  = 0.2f;    // Zieltoleranz (+/- Grad)
constexpr float    TN_CL_APPROACH_HZ    = 500.0f;  // langsame Annaeherung
constexpr uint32_t TN_CL_APPROACH_ACCEL = 1500;    // sanfte Rampe (steps/s^2)
constexpr float    TN_CL_DAMPING        = 0.9f;     // Anteil des Fehlers je Teilschritt
constexpr int      TN_CL_SETTLE_TICKS   = 3;        // n Messungen in Toleranz = fertig
constexpr uint32_t TN_CL_TIMEOUT_MS     = 20000;    // Sicherheits-Timeout

// ===========================================================================
//  Hilfsfunktionen (in config.cpp implementiert)
// ===========================================================================
// Initialisiert alle GPIOs: EN=HIGH (Treiber aus), Light=aus,
// Lichtschranken als Eingang mit Pull-Up.
void init_gpios();

// Manueller Treiber-Enable-Override (Protokoll ENABLE/DISABLE). Im Normalbetrieb
// schaltet FastAccelStepper den gemeinsamen EN-Pin via Auto-Enable selbst; dieser
// Aufruf kann den Pin kurzzeitig forcieren, wird aber ggf. wieder ueberstimmt.
void set_drivers_enabled(bool enabled);
// Liest den realen EN-Pegel (true = Treiber bestromt) -> fuer STATUS-Feld.
bool drivers_enabled();

// Beleuchtung schalten / abfragen.
void set_light(bool on);
bool light_on();

// Einen Abtastschritt des Entprell-Filters ausfuehren (zyklisch aus dem
// limit_task, Periode LIMIT_SAMPLE_MS). Liest die 3 Sensor-GPIOs und uebernimmt
// einen Pegel erst, wenn er LIMIT_DEBOUNCE_MS lang stabil war.
void limits_poll();

// true, wenn die Lichtschranke an 'pin' (entprellt) ausgeloest hat
// (active-high: stabiler Pegel == LIMIT_ACTIVE_LEVEL). Unbekannte/PIN_NONE -> false.
bool limit_triggered(gpio_num_t pin);

}  // namespace cfg
