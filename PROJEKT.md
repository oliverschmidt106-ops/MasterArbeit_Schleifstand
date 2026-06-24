# Master-Prompt: ESP32-S3 Firmware (ESP-IDF) – Glasfaser-Schleifstand

## Kontext

Masterarbeit: motorisierter Schleif-/Polierstand für Glasfaserspitzen. Ein ESP32-S3
treibt 4 Schrittmotoren über ein CNC-Shield V3 und wird von einer bestehenden
LabVIEW-GUI über USB-Serial gesteuert. Baue eine saubere, modulare, gut
dokumentierte ESP-IDF-Firmware als Grundgerüst.

## Framework

- ESP-IDF (idf.py, CMake, FreeRTOS), Target esp32s3, IDF >= 5.3.
- Struktur: Top-CMakeLists.txt, main/ (Komponente mit eigenem CMakeLists.txt),
  components/ für eigene Module und die Stepper-Engine. Konfig per sdkconfig/menuconfig.
- Da die Stepper-Lib C++ ist: main-Komponente in C++ (.cpp).

## Schritt-Engine

- DEFAULT: FastAccelStepper als ESP-IDF-Komponente einbinden (IDF >= 5.3, nutzt
  MCPWM/PCNT bzw. RMT, eigener FreeRTOS-Task für Rampen). Übernimm die genauen
  Integrationsschritte aus der Lib-Dokumentation – nicht raten.
- ALTERNATIVE (nur umsetzen, wenn ich es sage): native Pulserzeugung via MCPWM
  oder RMT mit eigener Beschleunigungslogik.

## Hardware

- ESP32-S3 DevKitC (WROOM), 3,3-V-Logik.
- CNC-Shield V3, 4 Treiber-Slots. Aktuell A4988, später TMC2209 (Standalone).
  WICHTIG: pro Achse ein Richtungs-Invert-Flag in der Konfig (TMC2209 hat DIR
  invertiert) – Wechsel soll nur ein Schalter im Code sein.
- Motoren QSH4218: 200 Vollschritte/Umdrehung, Microstepping 1/16 (Jumper)
  => 3200 Microsteps/Umdrehung (Konstante).
- EN gemeinsam, AKTIV LOW. Treiber bei Boot deaktiviert, erst per Firmware aktivieren.

## Pinbelegung (ESP32-S3 GPIO) – als Konstanten in config

- Tischrotation (TR): STEP=4, DIR=5
- Faserrotation (FR): STEP=6, DIR=7
- Faservorschub (FV): STEP=15, DIR=16
- Tischneigung (TN): STEP=17, DIR=18
- EN (alle): 8 (aktiv LOW)
- Lichtschranken (3x SK-205NA-W, NPN Open-Collector, ACTIVE-HIGH = stop):
  9=TN-Zonengatter, 10=FV-Limit-FWD, 11=FV-Limit-BACK. Je externer 4,7k
  Pull-up -> 3V3 (Fail-safe HIGH), zusaetzlich interner Pull-up. 3-ms-Entprellung.
- Beleuchtung (Ausgang, schaltet MOSFET): 14
  GPIOs über den IDF-gpio-Treiber (gpio_config) initialisieren.

## Achsen und Betriebsarten

1. TR – DAUERROTATION (Geschwindigkeitsmodus), Richtung rechts/links, Start/Stop.
   Optional Umfangsgeschwindigkeit v = 2*pi*n\*r (Radius = Maschinenkonstante).
2. FR – DAUERROTATION, Richtung, Start/Stop.
3. FV – POSITIONIERMODUS, Vor/Zurück/Home, Endlagen Min/Max, Ist-Position.
   Umrechnung Schritte<->mm (Konstante).
4. TN – POSITIONIERMODUS, +Neigen/-Senken/Home/Fahren-auf-Winkel, Endlagen,
   Ist-Winkel zurückmelden. Umrechnung Schritte<->Grad (steps_per_degree,
   oben konfigurierbar, MUSS kalibrierbar sein).

## Geschwindigkeit

- GUI sendet 0–255 (Altbestand). Firmware mappt 0–255 auf reale Schrittfrequenz
  [min_hz, max_hz] pro Achse (konfigurierbar), 0 = Stop.
- Beschleunigungsrampe pro Achse konfigurierbar (löst das Abreißen bei hoher Drehzahl).

## Kommunikation mit LabVIEW

- IDF UART-Treiber (uart_driver_install) auf konfigurierbarem UART als BEFEHLSKANAL.
- Logging (ESP_LOG) auf SEPARATEM Kanal (USB-Serial-JTAG-Konsole), damit es den
  Befehlskanal nicht verschmutzt.
- Protokoll: zeilenbasiert, ASCII, leicht von LabVIEW (VISA Write/Read) erzeugbar.
  Vorschlag `<ACHSE>:<BEFEHL>:<WERT>\n` + globale Befehle (STOP, STATUS, LIGHT).
  Muss alle GUI-Aktionen abdecken (Verbinden; TR/FR Start/Stop/Speed/Dir;
  FV Vor/Zurück/Home/Speed; TN Neigen/Senken/Home/Fahren(Winkel)/Speed; Beleuchtung).
  Jeder Befehl wird mit OK/ERR quittiert. STATUS liefert Ist-Position (FV),
  Ist-Winkel (TN), Endlagen-Status, Verbindungszustand. Befehlstabelle in README.

## Architektur

- Module/Komponenten: config (Pins, Konstanten, Invert-Flags, steps/rev, steps/mm,
  steps/deg, min/max Hz, Beschleunigung) / axis (kapselt FastAccelStepper: Modus
  rotation|position, Invert, Endlagen, Homing-State-Machine) / protocol (UART lesen,
  parsen, dispatchen, quittieren, Status) / main.
- FreeRTOS: ein Task liest/parst UART-Befehle; die Rampen-Task der Lib läuft separat;
  Endschalter-/Status-Check als Task oder Timer. NICHT BLOCKIEREND, Homing als
  State-Machine (keine blockierende while-Schleife).

## Sicherheit

- Treiber bei Boot deaktiviert (EN HIGH).
- Globaler STOP hält sofort alle Achsen an.
- FV und TN dürfen nicht über die Endschalter hinaus; bei Auslösung sofort stoppen.

## Konventionen

- Kommentare auf Deutsch, Bezeichner englisch. Achs-Kürzel TR/FR/FV/TN beibehalten.
- Alle einstellbaren Werte als dokumentierte Konstanten zentral in config.

## Vorgehen

1. ESP-IDF-Projektgerüst + CMakeLists anlegen (Target esp32s3).
2. FastAccelStepper als Komponente einbinden (Integration aus Lib-Doku übernehmen).
3. config mit allen Pins/Konstanten/Flags.
4. axis-Klasse (Rotation + Position + Invert + Endlagen + Homing).
5. protocol (UART-Task, Parser, Dispatcher, Quittung, Status).
6. main: 4 Achsen instanziieren, Tasks starten.
7. README mit Befehlstabelle.

Bring ZUERST eine Achse (TR) komplett zum Laufen, dann die übrigen per Muster.
Prüfe mit `idf.py build`. Frag nach, bevor du bei Unklarheiten große Annahmen triffst.

## LabVIEW-Kompatibilität (verbindlich)

- Anbindung ausschließlich über Standard-UART-Serial. In LabVIEW muss sie sich
  allein mit VISA Configure Serial Port + VISA Write + VISA Read umsetzen lassen –
  KEINE .NET-DLL, KEIN Sondertreiber, KEIN Binärprotokoll.
- Alle Befehle UND alle Antworten sind reine ASCII-Zeilen, abgeschlossen mit '\n'
  (LF), damit LabVIEW VISA Read mit Termination Character arbeiten kann.
- Feste, konfigurierbare Baudrate (z.B. 115200), im README dokumentiert.
- Pro gesendeter Zeile genau EINE Antwortzeile (OK / ERR:<grund> / STATUS:<...>),
  damit LabVIEW nach jedem Write deterministisch einen Read machen kann.
- README enthält: Baudrate, Termination Character und die vollständige
  Befehlstabelle (Senden -> erwartete Antwort) zum 1:1-Nachbau in LabVIEW.
