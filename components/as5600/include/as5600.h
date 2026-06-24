// as5600.h - Treiber fuer den magnetischen Absolut-Winkelsensor AS5600 (I2C).
//
// 12-Bit-Aufloesung (0..4095 = 0..360 Grad), feste I2C-Adresse 0x36.
// Nutzt den neuen ESP-IDF I2C-Master-Treiber (driver/i2c_master.h, IDF >= 5.2).
// Generisch gehalten: liefert nur Rohwinkel/Grad/Status; die TN-spezifische
// Verrechnung (Offset, Vorzeichen) macht die Achse.
#pragma once

#include <cstdint>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"

class AS5600 {
public:
    // Initialisiert I2C-Master-Bus + Geraet. ESP_OK bei Erfolg.
    esp_err_t begin(gpio_num_t sda, gpio_num_t scl, uint32_t freq_hz,
                    uint8_t addr = 0x36);

    // 12-Bit-Rohwinkel (0..4095) aus dem RAW-ANGLE-Register.
    esp_err_t readRawAngle(uint16_t& raw);

    // Winkel in Grad (0..360) aus dem Rohwert.
    esp_err_t readAngleDeg(float& deg);

    // Status-Rohbyte (Register 0x0B: MD/ML/MH).
    esp_err_t readStatus(uint8_t& status);

    // true, wenn ein Magnet erkannt wurde (MD-Bit gesetzt).
    bool magnetDetected();

    bool ready() const { return ready_; }

private:
    esp_err_t readReg16(uint8_t reg, uint16_t& value);

    i2c_master_bus_handle_t bus_ = nullptr;
    i2c_master_dev_handle_t dev_ = nullptr;
    uint8_t addr_  = 0x36;
    bool    ready_ = false;
};
