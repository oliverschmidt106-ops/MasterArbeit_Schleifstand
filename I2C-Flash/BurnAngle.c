#define AS5600_ADDR        0x36
#define REG_RAW_ANGLE_H     0x0C
#define REG_ZPOS_H          0x01
#define REG_STATUS          0x0B
#define REG_BURN            0xFF

esp_err_t as5600_read_reg16(i2c_master_dev_handle_t dev, uint8_t reg, uint16_t *val) {
    uint8_t data[2];
    esp_err_t err = i2c_master_transmit_receive(dev, &reg, 1, data, 2, 100);
    if (err == ESP_OK) *val = ((data[0] & 0x0F) << 8) | data[1];
    return err;
}

esp_err_t as5600_write_reg16(i2c_master_dev_handle_t dev, uint8_t reg, uint16_t val) {
    uint8_t buf[3] = { reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
    return i2c_master_transmit(dev, buf, 3, 100);
}

// Test: ZPOS setzen (flüchtig)
uint16_t raw;
as5600_read_reg16(dev, REG_RAW_ANGLE_H, &raw);
as5600_write_reg16(dev, REG_ZPOS_H, raw);
vTaskDelay(pdMS_TO_TICKS(1));
// -> ANGLE-Register lesen und prüfen

// Erst wenn ok: Burn
uint8_t status;
i2c_master_transmit_receive(dev, (uint8_t[]){REG_STATUS}, 1, &status, 1, 100);
if (status & 0x20 /* MD bit */) {
    uint8_t burn[2] = { REG_BURN, 0x80 };
    i2c_master_transmit(dev, burn, 2, 100);
    vTaskDelay(pdMS_TO_TICKS(1));
}