#ifndef TMC2209_SETUP_H
#define TMC2209_SETUP_H
#include "esp_err.h"

#include "driver.dto.h"

esp_err_t setup_driver(TMC2209_Driver *driver);
esp_err_t get_driver_settings(TMC2209_Driver *driver);
esp_err_t get_driver_status(TMC2209_Driver *driver);

void set_microsteps(TMC2209_Driver *driver, uint16_t microsteps);
void set_RMS_Current(TMC2209_Driver *driver, uint16_t mA);
void set_target_velocity(TMC2209_Driver *driver, int32_t velocity);

uint8_t current_setting_to_percent(uint8_t current_setting);
uint8_t hold_delay_setting_to_percent(uint8_t hold_delay_setting);

#endif //  TMC2209_SETUP_H
