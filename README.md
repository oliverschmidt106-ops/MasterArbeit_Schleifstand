# ESP32-S3 Firmware – Glasfaser-Schleifstand

ESP-IDF-Firmware (C++/FreeRTOS) für einen motorisierten Schleif-/Polierstand
für Glasfaserspitzen. Vier Schrittmotoren über ein CNC-Shield V3, gesteuert von
einer LabVIEW-GUI über einen zeilenbasierten ASCII-Befehlskanal (UART).

## Achsen

| Kürzel | Funktion        | Modus          |
|--------|-----------------|----------------|
| TR     | Tischrotation   | Dauerrotation  |
| FR     | Faserrotation   | Dauerrotation  |
| FV     | Faservorschub   | Positionierung (mm)  |
| TN     | Tischneigung    | Positionierung (Grad)|

## Projektstruktur

```
.
├── CMakeLists.txt                # Top-Level (Target esp32s3)
├── sdkconfig.defaults            # Konsole auf USB-Serial-JTAG, esp32s3
├── main/
│   ├── CMakeLists.txt
│   └── main.cpp                  # app_main: init, 4 Achsen, Tasks
└── components/
    ├── config/                   # Pins, Konstanten, Invert-Flags, Umrechnungen
    │   ├── include/config.h
    │   └── config.cpp            # GPIO-Init, EN/Light-Helfer
    ├── as5600/                   # AS5600-Winkelsensor (I2C-Master-Treiber)
    │   ├── include/as5600.h
    │   └── as5600.cpp            # Rohwinkel/Grad/Magnet-Status
    ├── axis/                     # Achse als FastAccelStepper-Kapsel
    │   ├── idf_component.yml      # zieht FastAccelStepper (Git-Dependency)
    │   ├── include/axis.h
    │   └── axis.cpp              # Rotation/Position/Endlagen/Homing + TN closed-loop
    ├── fv/                       # FV-Zustandsmaschine (Homing/Park/Resume)
    │   ├── include/fv.h
    │   └── fv.cpp                # NOT_HOMED→HOMING→HOMED⇄PARKED, Softlimits
    └── protocol/                 # UART-Befehlskanal
        ├── include/protocol.h
        └── protocol.cpp          # lesen, parsen, dispatchen, quittieren
```

## Build & Flash

Voraussetzung: ESP-IDF **>= 5.3** installiert und `idf.py` im PATH.

```powershell
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

Beim ersten Build lädt der Component Manager **FastAccelStepper** automatisch
nach `managed_components/` (siehe `components/axis/idf_component.yml`).

> Hinweis: Die genauen FastAccelStepper-Aufrufe sind in `axis.cpp` gekapselt und
> gegen die Lib-Doku zu verifizieren (API-Version kann abweichen). Sollte der
> vom Repo registrierte Komponentenname nicht `FastAccelStepper` lauten, in
> `idf_component.yml` **und** `components/axis/CMakeLists.txt` anpassen.

## Kanäle (wichtig)

- **Befehlskanal**: UART0 (GPIO TX=43 / RX=44) → USB-UART-Bridge des DevKitC →
  der COM-Port, den LabVIEW per VISA öffnet.
- **Logging**: `ESP_LOG` läuft separat über die **USB-Serial-JTAG**-Konsole und
  verschmutzt den Befehlskanal nicht.

## Winkelsensor AS5600 (Achse TN, closed-loop)

Die Neigeachse **TN** besitzt einen magnetischen Absolut-Winkelsensor **AS5600**
(I²C, 12 Bit = 0–4095 ≙ 0–360°, Adresse `0x36`). Damit fährt TN ihren Sollwinkel
**geregelt und behutsam** an, statt nur Schritte zu zählen.

**Verdrahtung (ESP32-S3):**

| AS5600 | ESP32-S3        | Hinweis                                   |
|--------|-----------------|-------------------------------------------|
| VCC    | 3V3             | 3,3-V-Logik                               |
| GND    | GND             |                                           |
| SDA    | **GPIO 1**      | Pull-up (meist auf dem Breakout)          |
| SCL    | **GPIO 2**      | Pull-up (meist auf dem Breakout)          |
| DIR    | GND             | feste Zählrichtung                        |

Der Sensor muss **1:1 auf der Neige-Ausgangswelle** sitzen; nutzbarer Bereich
**< 360°** (eine Umdrehung). Magnet diametral über der Sensormitte.

**Funktionsweise „behutsam" (closed-loop):** `TN:GOTO:<grad>` startet eine
nicht-blockierende Regelung in der Steuer-Task: Ist-Winkel messen → Fehler
bilden → in kleinen, gedämpften Teilschritten mit **niedriger Geschwindigkeit
und sanfter Rampe** annähern → neu messen → bis der Fehler im Toleranzband
(**±0,2°**) liegt. Abbruch bei Endlage, fehlendem Magnet oder Timeout. Fällt der
Sensor beim Start aus, arbeitet TN automatisch open-loop weiter (Schrittzähler).

**Firmware-Funktionen** (`Axis`, in `components/axis`): `sensorAngleDeg()` liefert
den Ist-Winkel; `startMoveToAngle(grad)` startet die Fahrprozedur; `setAngleZero()`
setzt den Nullpunkt.

## Lichtschranken / Endlagen (3 Sensoren)

Drei Gabellichtschranken **SK-205NA-W** dienen als Stop-Mechanismus und Referenz.
Einheitliche Konvention für alle drei Kanäle: **HIGH (~3,2 V) = Bewegung erlaubt**,
**LOW (0 V) = gesperrt/stop**. Jeder Fehler führt auf LOW → stop (fail-safe).

> **Hardware-Korrektur (verbindlich):** Die Ausgänge sind **keine echten
> Open-Collector**, auch wenn das Datenblatt das suggeriert. Messung am realen
> Bauteil: aktiver Zustand → Ausgang **treibt aktiv +5 V**; inaktiver Zustand →
> Ausgang **floatet** (hochohmig). Deshalb **kein Pull-up**, sondern pro Signal
> ein **Spannungsteiler 1,8 kΩ (in Reihe) + 3,3 kΩ (nach GND)**: getriebene 5 V
> → ~3,2 V = HIGH; floatend → 3,3 kΩ zieht auf 0 V = LOW.

**Verdrahtung (ESP32-S3):**

| Sensor            | GPIO | Ader (Ausgang) | Pin-Logik                                             |
|-------------------|------|----------------|-------------------------------------------------------|
| TN-Zonengatter    | **9**  | weiß ②        | im Gatter = HIGH = **erlaubt**; frei = LOW = **stop** (beide Richtungen) |
| FV-Limit **FWD**  | **10** | schwarz ④     | frei = HIGH = **fahren**; Limit = LOW = **FWD stoppen**|
| FV-Limit **BACK** | **11** | schwarz ④     | frei = HIGH = **fahren**; Limit = LOW = **BACK stoppen**|

- Versorgung je Sensor: **braun ① → 5V**, **blau ③ → GND**. Gemeinsame Masse
  (Sensorversorgung, Teiler-GND, ESP-GND) ist vorausgesetzt; der Teiler hält das
  GPIO ≤ 3,3 V (ESP32-S3 ist nicht 5-V-tolerant).
- **Pins ohne internen Pull** (`GPIO_PULLUP_DISABLE` + `GPIO_PULLDOWN_DISABLE`) —
  ein interner Pull-up würde den 3,3-kΩ-Zweig verfälschen; den Pegel definiert
  ausschließlich der externe Teiler.
- **Fail-safe:** Drahtbruch / Sensor stromlos → Ausgang floatet → Teiler → LOW →
  **stop** (alle drei Kanäle).
- **Entprellung:** Mehrheitsfilter gegen Motor-EMI — ein Zustandswechsel gilt erst
  nach **`SENSOR_VOTE_SAMPLES` = 3** gleichen Lesungen; ein einzelner Störimpuls
  löst keinen Stop aus. Ein eigener `limit_task` tastet alle **`SENSOR_SAMPLE_MS`
  = 1 ms** ab (≈ `SENSOR_DEBOUNCE_MS` = 3 ms Fenster).

**Verhalten:**

- **FV** nutzt gerichtete Endschalter: FWD blockiert nur Vorwärts (BACK bleibt
  zur Rückfahrt frei), BACK umgekehrt. `FV_HOME` referenziert **zweistufig**
  gegen das BACK-Limit (GPIO 11): Eilgang bis Kontakt → Freifahren um
  `FV_HOME_RELEASE_MM` → langsame zweite Anfahrt (Auslösepunkt = **0 mm**) →
  Vorfahren auf den Arbeitspunkt `FV_HOME_OFFSET_MM`. Die Schranke bleibt danach
  reiner Sicherheitsanschlag (`FV_PARK_POS_MM` liegt **davor**, nicht darin).
- **TN** hat ein einzelnes **Zonengatter** als symmetrische Hard-Zone (intern auf
  MIN *und* MAX gelegt): solange HIGH (in der Zone), ist TN frei; verlässt TN die
  Zone (LOW), wird **sofort gestoppt**. Damit kein Deadlock entsteht, sperrt LOW
  **nicht** komplett: über den **AS5600-Absolutwinkel** wird die Seite relativ zur
  Zonenmitte (`(TN_ANGLE_MIN+MAX_DEG)/2`) bestimmt und nur die Bewegung **zurück
  zur Mitte** (ins Gatter) erlaubt — **Re-Entry**. Sobald wieder HIGH, volle
  Freigabe. Das Gatter ist **keine** Referenz (die Absolutposition liefert der
  AS5600; `TN:HOME` wird im Normalbetrieb nicht verwendet); der reguläre
  Soll-Bereich wird ohnehin durch `TN_ANGLE_MIN/MAX_DEG` (0–90°) eingegrenzt, das
  Gatter ist der Not-Backstop. *(Sonderfall: fehlt bei LOW der Magnet, ist die
  Seite nicht bestimmbar → fail-safe in beide Richtungen gesperrt.)*

## Serielle Parameter (für LabVIEW VISA)

| Parameter             | Wert      |
|-----------------------|-----------|
| Baudrate              | **115200**|
| Data bits             | 8         |
| Parity                | None      |
| Stop bits             | 1         |
| Flow control          | None      |
| Termination Character | **`\n` (LF, 0x0A)** |

Jede gesendete Zeile endet mit `\n`. Pro gesendeter Zeile kommt **genau eine**
Antwortzeile zurück (`OK`, `ERR:<grund>` oder `STATUS:<...>`), ebenfalls mit `\n`
abgeschlossen → LabVIEW kann nach jedem *VISA Write* deterministisch *VISA Read*
mit Termination Character ausführen. Reines ASCII, kein Binärprotokoll, keine
DLL, kein Sondertreiber.

## Befehlstabelle

Format Achsbefehl: `<ACHSE>:<BEFEHL>[:<WERT>]\n` — ACHSE ∈ {TR, FR, FV, TN}.
Befehle/Achsen sind case-insensitiv.

### Globale Befehle

| Senden        | Antwort        | Wirkung                                   |
|---------------|----------------|-------------------------------------------|
| `PING`        | `OK`           | Verbindungstest (Verbinden)               |
| `STOP`        | `OK`           | **Sofort-Stopp aller Achsen**             |
| `STATUS`      | `STATUS:...`   | Statuszeile (siehe unten)                 |
| `SENS?`       | `SENS:FVF=<0\|1>,FVB=<0\|1>,TNZ=<0\|1>` | Logische Lichtschranken-Zustände (FV vor/zurück, TN-Gatter), je `1`=erlaubt/`0`=stop |
| `ENABLE`      | `OK`           | Treiber freigeben (EN LOW)                |
| `DISABLE`     | `OK`           | Alle stoppen + Treiber sperren (EN HIGH)  |
| `LIGHT:ON`    | `OK`           | Beleuchtung an                            |
| `LIGHT:OFF`   | `OK`           | Beleuchtung aus                           |

### TR / FR – Dauerrotation

| Senden            | Antwort         | Wirkung                                  |
|-------------------|-----------------|------------------------------------------|
| `TR:SPEED:<0-255>`| `OK`            | Geschwindigkeit (0 = Stop). Live bei Lauf|
| `TR:DIR:<R\|L>`   | `OK`            | Richtung rechts/links (auch `1`/`0`,`+`/`-`,`CW`)|
| `TR:START`        | `OK`/`ERR:NO_SPEED` | Dauerlauf starten (Speed muss > 0)   |
| `TR:STOP`         | `OK`            | Sanft anhalten (Rampe)                   |

(FR identisch, Präfix `FR:`.)

### FV – Faservorschub (Zustandsmaschine)

FV besitzt eine Zustandsmaschine `NOT_HOMED → HOMING → HOMED ⇄ PARKED`
(Modul `components/fv`). Jeder Neustart, jede während einer Positionierfahrt
ausgelöste Lichtschranke und `FV:CAL` setzen auf `NOT_HOMED` zurück und
invalidieren die gespeicherte Position.

| Senden           | Antwort                     | Wirkung                                        |
|------------------|-----------------------------|------------------------------------------------|
| `FV_HOME`        | `OK`/`ERR:BUSY`/`ERR:HOME_FAILED` | Zweistufige Referenzfahrt an der MIN-Schranke starten (nicht blockierend; Fortschritt per `FV_STATE?` pollen; Timeout ⇒ `STATE:NOT_HOMED`) |
| `FV_PARK`        | `OK`/`ERR:NOT_HOMED`/`ERR:BUSY` | Position speichern, auf `FV_PARK_POS_MM` fahren, Zustand `PARKED` |
| `FV_RESUME`      | `OK`/`ERR:NOT_PARKED`/`ERR:BUSY` | Zurück zur gespeicherten Position, Endanfahrt immer vorwärts mit `FV_BACKLASH_MM`-Kompensation |
| `FV_MOVE:<±mm>`  | `OK`/`ERR:NOT_HOMED`/`ERR:LIMIT`/`ERR:BUSY` | **Relativfahrt** in mm (nur `HOMED`; Softlimits 0…`FV_MAX_TRAVEL_MM`) |
| `FV_POS?`        | `POS:<mm>`                  | Ist-Position (Schrittzähler, 0 = Auslösepunkt MIN-Schranke) |
| `FV_STATE?`      | `STATE:NOT_HOMED\|HOMING\|HOMED\|PARKED` | Zustand der FV-Maschine |
| `FV:SPEED:<0-255>`| `OK`                       | Fahrgeschwindigkeit                            |
| `FV:FWD`         | `OK`/`ERR:AT_LIMIT`/`ERR:PARKED`/`ERR:BUSY` | Vor (bis Endlage). Alias `VOR`. Nicht in `PARKED`/`HOMING` |
| `FV:BACK`        | `OK`/`ERR:AT_LIMIT`/`ERR:PARKED`/`ERR:BUSY` | Zurück (bis Endlage). Alias `ZUR`              |
| `FV:MOVE:<mm>`   | wie `FV_MOVE`               | **Absolutfahrt** auf Position (mm). Alias `GOTO`; nur `HOMED` |
| `FV:HOME`        | wie `FV_HOME`               | Alias für `FV_HOME`                            |
| `FV:POS?`        | `POS:<mm>`                  | Alias für `FV_POS?`                            |
| `FV:CAL:<steps/mm>` | `OK`/`ERR:BAD_VALUE`     | Schritte/mm setzen (Messuhr-Kalibrierung) ⇒ `NOT_HOMED` |
| `FV:STOP`        | `OK`                        | Sanft anhalten (bricht Park/Resume-Sequenz ab) |

> Jog (`FV:FWD/BACK`) ist auch in `NOT_HOMED` erlaubt (Einrichten/Freifahren),
> kennt aber keine Softlimits: löst dabei in `HOMED` eine Schranke aus, fällt
> der Zustand konservativ auf `NOT_HOMED` zurück.

### TN – Positionierung

Für **TN** (Winkel in Grad):

| Senden               | Antwort                       | Wirkung                                          |
|----------------------|-------------------------------|--------------------------------------------------|
| `TN:GOTO:<grad>`     | `OK`/`ERR:NO_MAGNET`/`ERR:BUSY` | **Closed-Loop** auf Sollwinkel fahren (behutsam). Alias `MOVE` |
| `TN:ANGLE`           | `ANGLE:<grad>`/`ERR:NO_SENSOR`| Aktuellen Ist-Winkel (AS5600) abfragen           |
| `TN:AZERO`           | `OK`/`ERR:NO_SENSOR`          | Aktuelle Position als **0°** setzen (Nullpunkt)  |
| `TN:TILT`            | `OK`/`ERR:AT_LIMIT`           | +Neigen manuell (= `FWD`)                        |
| `TN:LOWER`           | `OK`/`ERR:AT_LIMIT`           | −Senken manuell (= `BACK`)                       |
| `TN:HOME`            | `OK`                          | Referenzfahrt — im Normalbetrieb ungenutzt (TN referenziert über AS5600) |
| `TN:CAL:<steps/deg>` | `OK`                          | Schritte/Grad setzen (Vorsteuerung der Regelung) |
| `TN:SPEED:<0-255>`   | `OK`                          | Geschwindigkeit (manuelles Tippen)               |

> `TN:GOTO` regelt mit dem AS5600 auf den Ist-Winkel. `STOP` oder `TN:STOP`
> bricht eine laufende Winkelfahrt sofort ab.

### STATUS-Antwortzeile

```
STATUS:CONN=1,EN=<0|1>,LIGHT=<0|1>,TR=<0|1>,FR=<0|1>,
FV=<mm>,FVrun=<0|1>,TN=<grad>,TNrun=<0|1>,
FVmin=<0|1>,FVmax=<0|1>,TNmin=<0|1>,TNmax=<0|1>,
HOMEfv=<0..4>,HOMEtn=<0..4>,
TNsens=<grad>,TNmag=<0|1>,TNcl=<0..3>,
SENStn=<0|1>,SENSfwd=<0|1>,SENSback=<0|1>
```
(eine Zeile, ohne Umbrüche). `HOMEtn`: 0=Idle 1=Seeking 2=Backoff 3=Done 4=Error.
`HOMEfv` wird aus der FV-Zustandsmaschine abgebildet: 0=`NOT_HOMED`,
1=`HOMING`, 3=`HOMED`/`PARKED` (Details per `FV_STATE?`).
`TR`/`FR`/`*run` = läuft (1) / steht (0). `*min`/`*max` = Endlage **gesperrt** (1,
d. h. Pegel LOW). `TN` = Winkel aus Schrittzähler, **`TNsens`** = gemessener
Ist-Winkel (AS5600), `TNmag` = Magnet erkannt (1), `TNcl` = Closed-Loop-Zustand
(0=Idle 1=Moving 2=Done 3=Error). **`SENStn`/`SENSfwd`/`SENSback`** = dieselben
logischen Werte wie `SENS?` (1=erlaubt/0=stop).

### Fehlerantworten

`ERR:UNKNOWN_CMD`, `ERR:UNKNOWN_AXIS`, `ERR:MISSING_VALUE`, `ERR:BAD_VALUE`,
`ERR:NO_SPEED`, `ERR:AT_LIMIT`, `ERR:BUSY`, `ERR:HOME_FAILED`, `ERR:NO_SENSOR`,
`ERR:NO_MAGNET`, `ERR:EMPTY`, `ERR:LINE_TOO_LONG`; FV-Zustandsmaschine:
`ERR:NOT_HOMED`, `ERR:NOT_PARKED`, `ERR:PARKED`, `ERR:LIMIT`, `ERR:BLOCKED`.

## Sicherheit

- Treiber bei Boot **deaktiviert** (EN HIGH); erst durch ersten Fahrbefehl bzw.
  `ENABLE` freigegeben.
- `STOP` hält **sofort** alle Achsen an (ohne Rampe) und bricht eine Winkelfahrt ab.
- Lichtschranken **HIGH = erlaubt / LOW = stop**, per Mehrheitsfilter (3 Reads)
  entprellt, fail-safe bei Drahtbruch (floatend → LOW → stop). FV stoppt
  richtungsabhängig (FWD/BACK), das TN-Zonengatter stoppt beim Verlassen der Zone
  und erlaubt nur die AS5600-gestützte Rückfahrt zur Mitte (Re-Entry, kein Deadlock).
- Homing **und** die TN-Winkelregelung laufen als nicht-blockierende
  State-Machines (keine `while`-Warteschleife).
- FV ist encoderlos: löst während einer Positionierfahrt eine Schranke aus oder
  wird die Referenzfahrt unterbrochen, fällt FV konservativ auf `NOT_HOMED`
  zurück und die gespeicherte Park-Position wird invalidiert (kein
  NVS-Persistieren über Neustarts).
- Die TN-Winkelfahrt bricht ab bei Endlage, **fehlendem Magnet** oder Timeout.

## Anzupassende Konstanten (vor Inbetriebnahme)

Alle in `components/config/include/config.h`:

- `DRIVER_IS_TMC2209` — zentraler Schalter A4988 ⇄ TMC2209 (DIR-Invert).
- `FV_LEADSCREW_PITCH_MM` — Spindelsteigung → Schritte/mm. **Aktuell
  Platzhalter (2,0 mm)**: reale Steigung des XR25/M-Antriebs unbekannt, per
  Messuhr kalibrieren (`FV:CAL:<steps/mm>` zur Laufzeit, dann hier eintragen).
- **FV-Zustandsmaschine:** `FV_HOME_SPEED_FAST_HZ` / `FV_HOME_SPEED_SLOW_HZ`
  (Referenzfahrt-Geschwindigkeiten), `FV_HOME_RELEASE_MM` (Freifahrweg),
  `FV_HOME_OFFSET_MM` (Arbeitspunkt), `FV_HOME_MAX_TRAVEL_MM` (Timeout-Budget
  je Suchphase), `FV_PARK_POS_MM` (Parkposition vor der Schranke),
  `FV_BACKLASH_MM` (Umkehrspiel-Kompensation bei `FV_RESUME`, Startwert 0,5 mm,
  per Messuhr verifizieren), `FV_MAX_TRAVEL_MM` (Softlimit, XR25/M: 25 mm).
- `TN_STEPS_PER_DEG_DEFAULT` — Startwert; zur Laufzeit per `TN:CAL` kalibrierbar.
- `PIN_TN_GATE` / `PIN_FV_LIM_FWD` / `PIN_FV_LIM_BACK` — die 3 Lichtschranken
  (SK-205NA-W, GPIO 9/10/11, **HIGH = erlaubt / LOW = stop**, externer
  Spannungsteiler 1k8/3k3, **kein** Pull). TN ist ein symmetrisches Zonengatter
  (auf MIN+MAX gelegt) mit AS5600-Re-Entry, FV nutzt gerichtete Endschalter
  (FWD=MAX, BACK=MIN). `SENSOR_GO_LEVEL` / `SENSOR_STOP_LEVEL` für die Polarität,
  `SENSOR_VOTE_SAMPLES` / `SENSOR_SAMPLE_MS` für den Entprell-Mehrheitsfilter.
- `*_MIN_HZ` / `*_MAX_HZ` / `*_ACCEL` — Drehzahlband und Rampe pro Achse.
- `UART_TX_PIN` / `UART_RX_PIN` / `UART_BAUD` — Befehlskanal.

**AS5600 / TN-Winkelregelung:**

- `PIN_I2C_SDA` / `PIN_I2C_SCL` — I²C-Pins (Default 1 / 2), `I2C_FREQ_HZ`.
- `TN_ANGLE_MIN_DEG` / `TN_ANGLE_MAX_DEG` — erlaubter Soll-Winkelbereich.
- `TN_MOTOR_TO_SENSOR` — `+1`/`-1`: einmalig prüfen, ob Motor-Vorwärts den
  gemessenen Winkel **vergrößert** (sonst `-1`, sonst läuft die Regelung weg).
- `TN_ANGLE_SIGN` — Vorzeichen des gemeldeten Winkels (Anzeige-Konvention).
- `TN_CL_TOLERANCE_DEG` (±0,2°), `TN_CL_APPROACH_HZ`, `TN_CL_APPROACH_ACCEL`,
  `TN_CL_DAMPING`, `TN_CL_SETTLE_TICKS`, `TN_CL_TIMEOUT_MS` — Regelverhalten.
- `TN_CL_FAST_HZ` / `TN_CL_FAST_DEG` — fehlerproportionaler Eilgang: ab
  `TN_CL_FAST_DEG` Restfehler fährt die Regelung mit `TN_CL_FAST_HZ`, nahe am
  Ziel mit `TN_CL_APPROACH_HZ` (dazwischen linear). Zusätzlich lernt ein
  adaptiver Gain die reale Steps/Grad-Übersetzung aus den Teilbewegungen und
  ein Überschwing-Wächter stoppt sofort, wenn das Ziel überfahren wird.
- Nullpunkt: `TN_ANGLE_OFFSET_DEG` oder zur Laufzeit per `TN:AZERO`.
