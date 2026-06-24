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

Drei Gabellichtschranken **SK-205NA-W** (NPN, Open-Collector) dienen als
Stop-Mechanismus und Referenz. Einheitliche Konvention **active-high**:
**Pin LOW = Bewegung erlaubt**, **Pin HIGH = gesperrt/stop**. Die Strahl-Polarität
wird über die **Aderwahl** gesetzt, nicht in der Firmware.

**Verdrahtung (ESP32-S3):**

| Sensor            | GPIO | Ader (Ausgang) | Schalttyp  | Funktion                                              |
|-------------------|------|----------------|------------|-------------------------------------------------------|
| TN-Zonengatter    | **9**  | schwarz ④     | Dark.on    | HIGH = außerhalb Zone → **stop** (beide Richtungen)   |
| FV-Limit **FWD**  | **10** | weiß ②        | Light.on   | HIGH = Limit vorwärts → **FWD stoppen**               |
| FV-Limit **BACK** | **11** | weiß ②        | Light.on   | HIGH = Limit zurück → **BACK stoppen**                |

- Versorgung je Sensor: **braun ① → 5V**, **blau ③ → GND**. Der Signalausgang
  bekommt einen **externen 4,7 kΩ Pull-up nach 3V3** (kein 5 V an den GPIO —
  ESP32-S3 ist nicht 5-V-tolerant). Intern bleibt der Pull-up zusätzlich aktiv.
- **Fail-safe:** Drahtbruch / Sensor stromlos → Pull-up zieht HIGH → **stop**.
- **Entprellung:** Da optisch prellfrei, aber Motor-EMI-empfindlich, wird jeder
  Pegel erst nach **`LIMIT_DEBOUNCE_MS` = 3 ms** stabiler Lesungen übernommen.
  Ein eigener `limit_task` tastet die Pins alle **`LIMIT_SAMPLE_MS` = 1 ms** ab.

**Verhalten:**

- **FV** nutzt gerichtete Endschalter: FWD blockiert nur Vorwärts (BACK bleibt
  zur Rückfahrt frei), BACK umgekehrt. `FV:HOME` referenziert gegen das
  BACK-Limit (GPIO 11) und setzt dort den Nullpunkt.
- **TN** hat ein einzelnes **Zonengatter** als symmetrische Hard-Zone (intern auf
  MIN *und* MAX gelegt): sobald HIGH, wird **jede** Fahrtrichtung gestoppt. Es ist
  **keine** Referenz — die Absolutposition liefert der AS5600; `TN:HOME` wird im
  Normalbetrieb nicht verwendet. Der reguläre Soll-Bereich wird ohnehin durch
  `TN_ANGLE_MIN/MAX_DEG` (0–90°) eingegrenzt, das Gatter ist die Not-Backstop.

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

### FV / TN – Positionierung

| Senden               | Antwort              | Wirkung                                |
|----------------------|----------------------|----------------------------------------|
| `FV:SPEED:<0-255>`   | `OK`                 | Fahrgeschwindigkeit                    |
| `FV:FWD`             | `OK`/`ERR:AT_LIMIT`  | Vor (bis Endlage). Alias `VOR`         |
| `FV:BACK`            | `OK`/`ERR:AT_LIMIT`  | Zurück (bis Endlage). Alias `ZUR`      |
| `FV:MOVE:<mm>`       | `OK`/`ERR:BUSY`      | Auf Position fahren (mm). Alias `GOTO` |
| `FV:HOME`            | `OK`                 | Referenzfahrt (Richtung MIN)           |
| `FV:STOP`            | `OK`                 | Sanft anhalten                         |

Für **TN** zusätzlich/analog (Winkel in Grad):

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
TNsens=<grad>,TNmag=<0|1>,TNcl=<0..3>
```
(eine Zeile, ohne Umbrüche). `HOMExx`: 0=Idle 1=Seeking 2=Backoff 3=Done 4=Error.
`TR`/`FR`/`*run` = läuft (1) / steht (0). `*min`/`*max` = Endlage ausgelöst (1).
`TN` = Winkel aus Schrittzähler, **`TNsens`** = gemessener Ist-Winkel (AS5600),
`TNmag` = Magnet erkannt (1), `TNcl` = Closed-Loop-Zustand (0=Idle 1=Moving
2=Done 3=Error).

### Fehlerantworten

`ERR:UNKNOWN_CMD`, `ERR:UNKNOWN_AXIS`, `ERR:MISSING_VALUE`, `ERR:BAD_VALUE`,
`ERR:NO_SPEED`, `ERR:AT_LIMIT`, `ERR:BUSY`, `ERR:HOME_FAILED`, `ERR:NO_SENSOR`,
`ERR:NO_MAGNET`, `ERR:EMPTY`, `ERR:LINE_TOO_LONG`.

## Sicherheit

- Treiber bei Boot **deaktiviert** (EN HIGH); erst durch ersten Fahrbefehl bzw.
  `ENABLE` freigegeben.
- `STOP` hält **sofort** alle Achsen an (ohne Rampe) und bricht eine Winkelfahrt ab.
- Lichtschranken **active-high** (HIGH = stop), 3 ms entprellt, fail-safe bei
  Drahtbruch. FV stoppt richtungsabhängig (FWD/BACK), das TN-Zonengatter
  symmetrisch in beide Richtungen.
- Homing **und** die TN-Winkelregelung laufen als nicht-blockierende
  State-Machines (keine `while`-Warteschleife).
- Die TN-Winkelfahrt bricht ab bei Endlage, **fehlendem Magnet** oder Timeout.

## Anzupassende Konstanten (vor Inbetriebnahme)

Alle in `components/config/include/config.h`:

- `DRIVER_IS_TMC2209` — zentraler Schalter A4988 ⇄ TMC2209 (DIR-Invert).
- `FV_LEADSCREW_PITCH_MM` — Spindelsteigung → Schritte/mm.
- `TN_STEPS_PER_DEG_DEFAULT` — Startwert; zur Laufzeit per `TN:CAL` kalibrierbar.
- `PIN_TN_GATE` / `PIN_FV_LIM_FWD` / `PIN_FV_LIM_BACK` — die 3 Lichtschranken
  (SK-205NA-W, GPIO 9/10/11, **active-high**: HIGH = stop, externer 4,7k Pull-up
  → 3V3). TN ist ein symmetrisches Zonengatter (auf MIN+MAX gelegt), FV nutzt
  gerichtete Endschalter (FWD=MAX, BACK=MIN). `LIMIT_ACTIVE_LEVEL` /
  `LIMIT_DEBOUNCE_MS` für Polarität und Entprellung.
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
- Nullpunkt: `TN_ANGLE_OFFSET_DEG` oder zur Laufzeit per `TN:AZERO`.
