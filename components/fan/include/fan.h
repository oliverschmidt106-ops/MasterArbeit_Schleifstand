// fan.h - Gehaeuseluefter-Steuerung (2x 24-V-Luefter, EIN GPIO, Low-Side-MOSFET).
//
// Reines Digital-Schalten, kein PWM: GPIO HIGH = Luefter an.
// Automatik: Luefter EIN, solange TR ODER FR tatsaechlich laeuft (isRunning(),
// deckt auch die Auslauframpe nach STOP ab), sonst AUS.
// Manueller Override (fan_set) setzt die Automatik aus; fan_auto() gibt sie
// wieder frei.
#pragma once

class Axis;

// Einmalig nach Axis::attach() aufrufen: konfiguriert den GPIO als Ausgang
// und schaltet initial LOW (Luefter aus, kein unkontrolliertes Anlaufen).
// 'tr'/'fr' muessen fuer die gesamte Laufzeit gueltig bleiben.
void fan_init(Axis* tr, Axis* fr);

// Periodisch aus der control_task aufrufen: schaltet den Luefter nach
// TR/FR-Aktivitaet (wirkt nur im Automatik-Modus).
void fan_update();

// Manueller Override: erzwingt EIN/AUS und haelt die Automatik an.
void fan_set(bool on);
// Zurueck in die Automatik; der naechste fan_update() stellt den Zustand.
void fan_auto();

bool fan_is_on();    // aktueller Schaltzustand (gespiegelt, ohne GPIO-Read)
bool fan_is_auto();  // true = Automatik aktiv, false = manueller Override
