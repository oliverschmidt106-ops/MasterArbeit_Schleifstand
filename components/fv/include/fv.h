// fv.h - Zustandsmaschine des Faservorschubs (FV): Homing, Park & Resume.
//
// Zustaende:
//   NOT_HOMED --FV_HOME--> HOMING --ok--> HOMED <--FV_PARK/FV_RESUME--> PARKED
//
// Jeder Motorfehler, jede waehrend einer Positionierfahrt ausgeloeste
// Lichtschranke und jeder Neustart setzt auf NOT_HOMED zurueck und
// invalidiert die gespeicherte Position (kein NVS-Persistieren).
//
// Die Befehls-Funktionen liefern die Protokoll-Antwort als statischen String
// ("OK" / "ERR:<GRUND>"); lang laufende Ablaeufe (Referenzfahrt, Park/Resume)
// werden nicht blockierend aus fv_tick() (control_task, 10 ms) weitergeschaltet.
#pragma once

class Axis;

// Protokoll-sichtbare Zustaende (FV_STATE?).
enum class FvState { NotHomed, Homing, Homed, Parked };

// Einmalig nach Axis::configurePosition() aufrufen. 'axis' muss fuer die
// gesamte Laufzeit gueltig bleiben.
void fv_init(Axis* axis);

// Periodisch aus der control_task aufrufen (schaltet Homing/Park/Resume
// weiter und ueberwacht die Schranken waehrend Positionierfahrten).
void fv_tick();

FvState     fv_state();
const char* fv_state_str();     // "NOT_HOMED" / "HOMING" / "HOMED" / "PARKED"
float       fv_position_mm();   // Ist-Position aus dem Schrittzaehler
const char* fv_last_error();    // letzter Abbruchgrund oder "NONE" (FV_ERR?)
float       fv_span_mm();       // beim Homing gemessene Spanne, 0 = unbekannt

// --- Protokoll-Befehle (Rueckgabe = Antwortzeile ohne '\n') ---
const char* fv_cmd_home();                // FV_HOME  (nicht blockierend)
const char* fv_cmd_park();                // FV_PARK  (nur in HOMED)
const char* fv_cmd_resume();              // FV_RESUME (nur in PARKED)
const char* fv_cmd_move_rel(float d_mm);  // FV_MOVE:<+/-mm> (nur in HOMED)
const char* fv_cmd_move_abs(float mm);    // Legacy FV:MOVE/GOTO (nur in HOMED)
const char* fv_cmd_jog(bool forward);     // Legacy FV:FWD/BACK (NOT_HOMED/HOMED)
// Nach einer Laufzeit-Kalibrierung (FV:CAL): Positionsbezug ist ungueltig.
void        fv_invalidate();
