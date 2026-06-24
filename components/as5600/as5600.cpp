// as5600.cpp - Implementierung des AS5600-Treibers (neuer I2C-Master-Treiber).
#include "as5600.h"

#include "esp_log.h"

static const char* TAG = "as5600";

// I2C-Transfer-Timeout (ms). Erfolgreiche Transfers brauchen << 1 ms.
static constexpr int I2C_TIMEOUT_MS = 20;

// --- AS5600 Register ---
static constexpr uint8_t REG_STATUS    = 0x0B;  // MD/ML/MH
static constexpr uint8_t REG_RAW_ANGLE = 0x0C;  // 0x0C(High)/0x0D(Low), 12 Bit

// --- Statusbits (Register 0x0B) ---
static constexpr uint8_t STATUS_MD = 0x20;  // Magnet detected
static constexpr uint8_t STATUS_ML = 0x10;  // Magnet too weak (AGC max)
static constexpr uint8_t STATUS_MH = 0x08;  // Magnet too strong (AGC min)

esp_err_t AS5600::begin(gpio_num_t sda, gpio_num_t scl, uint32_t freq_hz,
                        uint8_t addr) {
    addr_ = addr;

    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.clk_source                   = I2C_CLK_SRC_DEFAULT;
    bus_cfg.i2c_port                     = I2C_NUM_0;
    bus_cfg.sda_io_num                   = sda;
    bus_cfg.scl_io_num                   = scl;
    bus_cfg.glitch_ignore_cnt            = 7;
    bus_cfg.flags.enable_internal_pullup = true;  // zusaetzlich zu Breakout-Pullups

    esp_err_t err = i2c_new_master_bus(&bus_cfg, &bus_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus: %s", esp_err_to_name(err));
        return err;
    }

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address  = addr_;
    dev_cfg.scl_speed_hz    = freq_hz;

    err = i2c_master_bus_add_device(bus_, &dev_cfg, &dev_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_add_device: %s", esp_err_to_name(err));
        return err;
    }

    ready_ = true;
    ESP_LOGI(TAG, "AS5600 init (SDA=%d SCL=%d %lu Hz addr=0x%02X)",
             sda, scl, static_cast<unsigned long>(freq_hz), addr_);
    return ESP_OK;
}

esp_err_t AS5600::readReg16(uint8_t reg, uint16_t& value) {
    if (!ready_) return ESP_ERR_INVALID_STATE;
    uint8_t data[2] = {0, 0};
    esp_err_t err = i2c_master_transmit_receive(dev_, &reg, 1, data, 2,
                                                I2C_TIMEOUT_MS);
    if (err != ESP_OK) return err;
    value = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    return ESP_OK;
}

esp_err_t AS5600::readRawAngle(uint16_t& raw) {
    uint16_t v = 0;
    esp_err_t err = readReg16(REG_RAW_ANGLE, v);
    if (err != ESP_OK) return err;
    raw = v & 0x0FFF;  // nur 12 Bit gueltig
    return ESP_OK;
}

esp_err_t AS5600::readAngleDeg(float& deg) {
    uint16_t raw = 0;
    esp_err_t err = readRawAngle(raw);
    if (err != ESP_OK) return err;
    deg = raw * (360.0f / 4096.0f);
    return ESP_OK;
}

esp_err_t AS5600::readStatus(uint8_t& status) {
    if (!ready_) return ESP_ERR_INVALID_STATE;
    uint8_t reg  = REG_STATUS;
    uint8_t data = 0;
    esp_err_t err = i2c_master_transmit_receive(dev_, &reg, 1, &data, 1,
                                                I2C_TIMEOUT_MS);
    if (err != ESP_OK) return err;
    status = data;
    return ESP_OK;
}

bool AS5600::magnetDetected() {
    uint8_t s = 0;
    if (readStatus(s) != ESP_OK) return false;
    return (s & STATUS_MD) != 0;
}
