// main.cpp - Einstiegspunkt: GPIOs init, 4 Achsen instanziieren, Tasks starten.
//
// Architektur:
//   - FastAccelStepper-Engine erzeugt einen eigenen Rampen-Task (in der Lib).
//   - protocol_start() startet den UART-Task (Befehle lesen/parsen/quittieren).
//   - control_task() prueft periodisch Endlagen und schaltet die Homing-SM
//     weiter (nicht blockierend).
#include "axis.h"
#include "config.h"
#include "protocol.h"
#include "as5600.h"

#include "FastAccelStepper.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "main";

// FastAccelStepper-Engine (muss fuer die Laufzeit bestehen bleiben).
static FastAccelStepperEngine engine = FastAccelStepperEngine();

// AS5600-Winkelsensor fuer die Neigeachse TN (muss bestehen bleiben).
static AS5600 tn_sensor;

// --- Die vier Achsen, aus zentralen config-Konstanten aufgebaut ---
static Axis axis_tr("TR", cfg::PIN_TR_STEP, cfg::PIN_TR_DIR, cfg::TR_DIR_INVERT,
                    AxisMode::Rotation, cfg::TR_MIN_HZ, cfg::TR_MAX_HZ, cfg::TR_ACCEL);
static Axis axis_fr("FR", cfg::PIN_FR_STEP, cfg::PIN_FR_DIR, cfg::FR_DIR_INVERT,
                    AxisMode::Rotation, cfg::FR_MIN_HZ, cfg::FR_MAX_HZ, cfg::FR_ACCEL);
static Axis axis_fv("FV", cfg::PIN_FV_STEP, cfg::PIN_FV_DIR, cfg::FV_DIR_INVERT,
                    AxisMode::Position, cfg::FV_MIN_HZ, cfg::FV_MAX_HZ, cfg::FV_ACCEL);
static Axis axis_tn("TN", cfg::PIN_TN_STEP, cfg::PIN_TN_DIR, cfg::TN_DIR_INVERT,
                    AxisMode::Position, cfg::TN_MIN_HZ, cfg::TN_MAX_HZ, cfg::TN_ACCEL);

static Machine machine;

// Hochfrequente Abtast-Task: entprellt die Lichtschranken (Motor-EMI). Laeuft
// schneller als die Steuer-Task, damit der SENSOR_VOTE_SAMPLES-Filter greift.
static void limit_task(void *arg)
{
    const TickType_t period = pdMS_TO_TICKS(cfg::SENSOR_SAMPLE_MS);
    TickType_t last = xTaskGetTickCount();
    for (;;)
    {
        cfg::limits_poll();
        vTaskDelayUntil(&last, period);
    }
}

// Periodische Steuer-Task: Endlagen-Sicherheit + Homing-State-Machine.
static void control_task(void *arg)
{
    const TickType_t period = pdMS_TO_TICKS(cfg::CONTROL_PERIOD_MS);
    TickType_t last = xTaskGetTickCount();
    for (;;)
    {
        axis_tr.update();
        axis_fr.update();
        axis_fv.update();
        axis_tn.update();
        vTaskDelayUntil(&last, period);
    }
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Glasfaser-Schleifstand Firmware startet ...");

    // 1) GPIOs: Treiber AUS (EN HIGH), Licht aus, Lichtschranken als Eingang.
    cfg::init_gpios();

    // 2) Stepper-Engine starten (eigener Rampen-Task der Lib).
    engine.init();

    // 3) Achsen mit der Engine verbinden.
    axis_tr.attach(engine);
    axis_fr.attach(engine);
    axis_fv.attach(engine);
    axis_tn.attach(engine);

    // 4) Positionierachsen konfigurieren (Umrechnung + Endlagen + Homing-Richtung).
    //    FV: gerichtete Endschalter (FWD=MAX/GPIO10, BACK=MIN/GPIO11); Homing
    //        faehrt Richtung MIN-Endlage (home_dir_positive = false).
    axis_fv.configurePosition(cfg::FV_STEPS_PER_MM,
                              cfg::PIN_FV_LIMIT_MIN, cfg::PIN_FV_LIMIT_MAX,
                              /*home_dir_positive=*/false);
    //    TN: ein Zonengatter (GPIO9) auf MIN UND MAX -> symmetrischer Hard-Stop
    //        in beide Richtungen. Reine Sicherheitszone, KEINE Referenzfahrt
    //        (Absolutwinkel liefert der AS5600); HOME wird fuer TN nicht genutzt.
    axis_tn.configurePosition(cfg::TN_STEPS_PER_DEG_DEFAULT,
                              cfg::PIN_TN_LIMIT_MIN, cfg::PIN_TN_LIMIT_MAX,
                              /*home_dir_positive=*/false);

    // 4b) AS5600-Winkelsensor initialisieren und an TN koppeln (closed-loop).
    esp_err_t serr = tn_sensor.begin(cfg::PIN_I2C_SDA, cfg::PIN_I2C_SCL,
                                     cfg::I2C_FREQ_HZ, cfg::AS5600_ADDR);
    if (serr == ESP_OK)
    {
        AngleControlCfg acfg;
        acfg.offset_deg     = cfg::TN_ANGLE_OFFSET_DEG;
        acfg.angle_sign     = cfg::TN_ANGLE_SIGN;
        acfg.motor_sign     = cfg::TN_MOTOR_TO_SENSOR;
        acfg.min_deg        = cfg::TN_ANGLE_MIN_DEG;
        acfg.max_deg        = cfg::TN_ANGLE_MAX_DEG;
        acfg.tol_deg        = cfg::TN_CL_TOLERANCE_DEG;
        acfg.approach_hz    = cfg::TN_CL_APPROACH_HZ;
        acfg.fast_hz        = cfg::TN_CL_FAST_HZ;
        acfg.fast_deg       = cfg::TN_CL_FAST_DEG;
        acfg.approach_accel = cfg::TN_CL_APPROACH_ACCEL;
        acfg.damping        = cfg::TN_CL_DAMPING;
        acfg.settle_ticks   = cfg::TN_CL_SETTLE_TICKS;
        acfg.timeout_ms     = cfg::TN_CL_TIMEOUT_MS;
        axis_tn.attachAngleSensor(&tn_sensor, acfg);
        ESP_LOGI(TAG, "AS5600 an TN gekoppelt (closed-loop aktiv).");
    }
    else
    {
        ESP_LOGE(TAG, "AS5600 init fehlgeschlagen: %s -> TN bleibt open-loop.",
                 esp_err_to_name(serr));
    }

    // 5) Machine-Kontext fuer den Protokoll-Dispatcher fuellen.
    machine.tr = &axis_tr;
    machine.fr = &axis_fr;
    machine.fv = &axis_fv;
    machine.tn = &axis_tn;

    // 6) Tasks starten: Lichtschranken-Abtastung (hohe Prioritaet) + Steuer-Task
    //    + UART-Befehlskanal.
    xTaskCreate(limit_task,   "limit_task",   2048, nullptr, 9, nullptr);
    xTaskCreate(control_task, "control_task", 4096, nullptr, 8, nullptr);
    protocol_start(&machine);

    ESP_LOGI(TAG, "Bereit. Treiber bei Boot deaktiviert (EN HIGH).");
}
