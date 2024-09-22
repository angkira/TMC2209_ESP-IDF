#ifndef TMC2209_SETUP_H
#define TMC2209_SETUP_H
#include "esp_err.h"

#include "driver.dto.h"

esp_err_t setup_driver(TMC2209_Driver *driver);
esp_err_t get_driver_status(TMC2209_Driver *driver);

void set_microsteps(TMC2209_Driver *driver, uint16_t microsteps);
void set_RMS_Current(TMC2209_Driver *driver, uint16_t mA);

uint8_t current_setting_to_percent(uint8_t current_setting);
uint8_t hold_delay_setting_to_percent(uint8_t hold_delay_setting);

typedef enum
{
  CHOPPER_MODE_STEALTHCHOP,
  CHOPPER_MODE_SPREADCYCLE
} TMC2209_ChopperMode;

void setChopperMode(TMC2209_Driver *driver, TMC2209_ChopperMode mode);
void writeStealthchopConfig(TMC2209_Driver *driver, const TMC2209_StealthchopConfig *config);
void writeSpreadCycleConfig(TMC2209_Driver *driver, const TMC2209_SpreadCycleConfig *config);

uint32_t configure_gconf(const TMC2209_Settings *settings);
uint32_t configure_chopconf(const TMC2209_Settings *settings);
uint32_t configure_ihold_irun(const TMC2209_Settings *settings);
uint32_t configure_tpowerdown(const TMC2209_Settings *settings);
uint32_t configure_sgthrs(const TMC2209_Settings *settings);
uint32_t configure_coolconf(const TMC2209_Settings *settings);
uint32_t configure_pwmconf(const TMC2209_Settings *settings);
uint32_t configure_tpwmthrs(const TMC2209_Settings *settings);
uint32_t get_chopconf_with_microsteps(uint32_t current_chopconf, uint16_t microsteps);

#endif //  TMC2209_SETUP_H
