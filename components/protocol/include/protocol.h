// protocol.h - Zeilenbasiertes ASCII-Protokoll ueber UART (LabVIEW-kompatibel).
//
// Format Achsbefehl : <ACHSE>:<BEFEHL>[:<WERT>]\n   (ACHSE = TR|FR|FV|TN)
// Globale Befehle    : STOP | STATUS | PING | ENABLE | DISABLE | LIGHT:ON|OFF
//
// Antwort: GENAU eine Zeile je empfangener Zeile, abgeschlossen mit '\n':
//   OK                    Befehl ausgefuehrt
//   ERR:<grund>           Fehler (z.B. ERR:UNKNOWN_CMD, ERR:BAD_VALUE)
//   STATUS:<...>          Antwort auf STATUS
//
// Vollstaendige Befehlstabelle siehe README.md.
#pragma once

#include "axis.h"

// Buendelt die vier Achsen + globale Aktionen fuer den Dispatcher.
struct Machine {
    Axis* tr = nullptr;   // Tischrotation
    Axis* fr = nullptr;   // Faserrotation
    Axis* fv = nullptr;   // Faservorschub
    Axis* tn = nullptr;   // Tischneigung

    // Sofort-Stopp aller Achsen (globaler STOP).
    void stopAll();

    // Achse per Kuerzel finden ("TR"/"FR"/"FV"/"TN"), sonst nullptr.
    Axis* byName(const char* code);
};

// Installiert den UART-Treiber und startet die Befehls-Task.
// 'machine' muss fuer die gesamte Laufzeit gueltig bleiben.
void protocol_start(Machine* machine);
