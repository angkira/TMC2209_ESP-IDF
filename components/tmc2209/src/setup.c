#include "setup.h"
#include "register.h"
#include "driver.dto.h"

#include "driver/gpio.h"

#include "esp_log.h"
#include "driver/uart.h"
#include "freertos/task.h"
#include "esp_err.h"

static const char *TAG = "tmc2209";

esp_err_t setup_driver(TMC2209_Driver *driver)
{
  // UART configuration (assuming UART2, adjust if needed)
  uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
  ESP_ERROR_CHECK(uart_param_config(driver->uart_num, &uart_config));

  // Set UART pins (adjust GPIO numbers if needed)
  ESP_ERROR_CHECK(uart_set_pin(driver->uart_num, 17, 18, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  // Install UART driver
  int uart_driver_status = uart_is_driver_installed(driver->uart_num);

  if (uart_driver_status == 0)
  { // Driver is NOT installed
    // Install UART driver only if it's not already installed
    ESP_ERROR_CHECK(uart_driver_install(driver->uart_num, 256, 0, 0, NULL, 0));
  }
  else if (uart_driver_status < 0)
  {
    // Handle potential error checking the driver status
    return ESP_FAIL;
  }

  // 1. GCONF Register Configuration
  uint32_t gconf = 0;

  // Bit 0: i_scale_analog (Internal vs. External VREF)
  if (driver->settings.analog_current_scaling_enabled)
  {
    gconf |= 1; // Use external VREF
  }

  // Bit 1: internal_Rsense (Internal vs. External Sense Resistors)
  if (driver->settings.internal_sense_resistors_enabled)
  {
    gconf |= (1 << 1); // Use internal sense resistors
  }

  // Bit 3: shaft (Inverse Motor Direction)
  if (driver->settings.inverse_motor_direction_enabled)
  {
    gconf |= (1 << 4);
  }

  // Bit 7: diag0_error (Enable/Disable DIAG0 active on driver errors)
  if (driver->settings.diag0_error_enabled)
  {
    gconf |= (1 << 7);
  }

  // Bit 11: diag0_otpw (Enable/Disable DIAG0 active on over temperatur warning)
  if (driver->settings.diag0_otpw_enabled)
  {
    gconf |= (1 << 11);
  }

  // Bit 12: diag0_stall (Enable/Disable DIAG0 active on stallguard2)
  if (driver->settings.diag0_stall_enabled)
  {
    gconf |= (1 << 12);
  }

  // Bit 13: diag1_stall (Enable/Disable DIAG1 active on stallguard2)
  if (driver->settings.diag1_stall_enabled)
  {
    gconf |= (1 << 13);
  }

  // Bit 14: diag1_index (Enable/Disable DIAG1 active on index)
  if (driver->settings.diag1_index_enabled)
  {
    gconf |= (1 << 14);
  }

  // Bit 15: diag1_onstate (Enable/Disable DIAG1 active on ON_STATE)
  if (driver->settings.diag1_onstate_enabled)
  {
    gconf |= (1 << 15);
  }

  ESP_ERROR_CHECK(writeRegister(driver, REG_GCONF, gconf));

  // writeStealthchopConfig(driver, &driver->settings.stealthchop);
  // writeSpreadCycleConfig(driver, &driver->settings.spreadcycle);
  // 2. CHOPCONF Register Configuration
  uint32_t chopconf = 0;

  // Bits 24-27: MRES (Microstep Resolution)
  uint8_t mres = 0;
  switch (driver->settings.microsteps)
  {
  case 256:
    mres = 0;
    break;
  case 128:
    mres = 1;
    break;
  case 64:
    mres = 2;
    break;
  case 32:
    mres = 3;
    break;
  case 16:
    mres = 4;
    break;
  case 8:
    mres = 5;
    break;
  case 4:
    mres = 6;
    break;
  case 2:
    mres = 7;
    break;
  case 1:
    mres = 8;
    break; // Full step
  default:
    ESP_LOGE(TAG, "Invalid microsteps value: %d", driver->settings.microsteps);
    return ESP_ERR_INVALID_ARG;
  }
  chopconf |= (mres << 24);

  // Bit 17: vsense (Sense Resistor Voltage Selection)
  if (driver->settings.vsense)
  {
    chopconf |= (1 << 17); // High sensitivity, low sense resistor voltage
  }

  // Bit 27: intpol (Interpolation for 256 Microsteps)
  if (driver->settings.interpolate_to_256_microsteps)
  {
    chopconf |= (1 << 27);
  }

  // Bit 28: dedge (Enable Double Edge STEP pulses)
  if (driver->settings.enable_double_edge_step_pulses)
  {
    chopconf |= (1 << 28);
  }

  // Bits 29-31: Short circuit protection settings
  // Configure based on your desired protection level
  // Example:
  chopconf |= 0; // All short protection enabled (default)
  // Or disable specific protections if needed:
  // chopconf |= (1 << 29);  // Disable short to GND protection
  // chopconf |= (1 << 30);  // Disable short to VS protection
  // chopconf |= (1 << 31);  // Disable low-side short protection

  // ESP_ERROR_CHECK(writeRegister(driver, REG_CHOPCONF, chopconf));

  // 3. Enable the chopper based on the enabled mode
  if (driver->settings.stealthchop.enabled)
  {
    // writeStealthchopConfig(driver, &driver->settings.stealthchop);

    // Enable StealthChop using the TOFF value from CHOPCONF
    uint8_t toff = chopconf & 0x0F; // Extract TOFF from CHOPCONF
    ESP_ERROR_CHECK(writeRegister(driver, REG_CHOPCONF, (chopconf & 0xFFFFFFF0) | toff));
  }
  else if (driver->settings.spreadcycle.enabled)
  {
    // writeSpreadCycleConfig(driver, &driver->settings.spreadcycle);

    // Enable SpreadCycle using the TOFF value from CHOPCONF
    uint8_t toff = chopconf & 0x0F; // Extract TOFF from CHOPCONF
    ESP_ERROR_CHECK(writeRegister(driver, REG_CHOPCONF, (chopconf & 0xFFFFFFF0) | toff));
  }
  else
  {
    ESP_LOGE(TAG, "No chopper mode enabled");
    return ESP_ERR_INVALID_ARG;
  }

  // 4. IHOLD_IRUN Register

  uint32_t ihold_irun = 0;

  // Bits 8-12: IRUN (Motor Run Current)
  // Convert RMS current to IRUN using datasheet formula and table
  float sense_resistor = driver->settings.internal_sense_resistors_enabled ? 0.11 : 0.18; // Adjust if using different sense resistors
  uint8_t irun = (uint8_t)((driver->settings.rms_current * 256.0) / 1.41421 / 1000.0 / sense_resistor);
  if (irun > 31)
  {
    irun = 31; // Limit to maximum value
  }
  ihold_irun |= (irun << 8);

  // Bits 0-4: IHOLD (Motor Hold Current)
  // Calculate IHOLD based on irun and ihold_percent
  uint8_t ihold = (uint8_t)((irun * driver->settings.ihold_percent) / 100.0);
  if (ihold > 31)
  {
    ihold = 31; // Limit to maximum value
  }
  ihold_irun |= ihold;

  // Bits 16-19: IHOLDDELAY (Hold Current Delay)
  // Convert iholddelay_percent to IHOLDDELAY
  // Assuming iholddelay_percent represents the percentage of the maximum delay
  // Maximum IHOLDDELAY value is 15, representing 2^15 * 2 clock cycles delay
  uint8_t iholddelay = (uint8_t)((driver->settings.iholddelay_percent / 100.0) * 15);
  ihold_irun |= (iholddelay << 16);

  ESP_ERROR_CHECK(writeRegister(driver, REG_IHOLD_IRUN, ihold_irun));

  // 5. TPOWERDOWN Register

  uint32_t tpowerdown = 0;

  // Bits 0-7: TPOWERDOWN (Power Down Delay)
  // Convert standstill current timeout from seconds to register value (0-255)
  // Formula: TPOWERDOWN = timeout[s] * 2^18 / 10
  tpowerdown = (uint32_t)(driver->settings.standstill_current_timeout * (1 << 18) / 10.0);
  if (tpowerdown > 255)
  {
    tpowerdown = 255; // Limit to maximum value
  }

  ESP_ERROR_CHECK(writeRegister(driver, REG_TPOWERDOWN, tpowerdown));

  // 6. COOLCONF Register (if CoolStep is enabled)

  if (driver->settings.coolstep_enabled)
  {
    uint32_t coolconf = 0;

    // Bits 0-3: semin (Minimum StallGuard Value for CoolStep)
    // Set semin to a non-zero value to enable CoolStep
    // You might want to adjust this value based on your motor and application
    coolconf |= 1; // Example: Set semin to 1 (minimum value to enable)

    // Bits 4-7: semax (Maximum StallGuard Value for CoolStep)
    // Set semax to a value higher than semin
    // You might want to adjust this value based on your motor and application
    coolconf |= (10 << 4); // Example: Set semax to 10

    // Bit 24: seimin (Minimum Current Decrease per Full-Step)
    // Set seimin to control the minimum current decrease
    // You might want to adjust this value based on your motor and application
    coolconf |= (1 << 24); // Example: Set seimin to 1

    // Bit 25: semax (Maximum Current Decrease per Full-Step)
    // Set semax to a value higher than seimin
    // You might want to adjust this value based on your motor and application
    coolconf |= (5 << 25); // Example: Set semax to 5

    // Bit 28: sgt (Enable/Disable StallGuard Threshold)
    if (driver->settings.stallguard_enabled)
    {
      coolconf |= (1 << 28);
    }

    ESP_ERROR_CHECK(writeRegister(driver, REG_COOLCONF, coolconf));
  }

  // 7. SGTHRS Register (if StallGuard is enabled)

  if (driver->settings.stallguard_enabled)
  {
    ESP_ERROR_CHECK(writeRegister(driver, REG_SGTHRS, driver->settings.stallguard_threshold));
  }

  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = (1ULL << driver->step_pin) | (1ULL << driver->dir_pin);
  io_conf.pull_down_en = 0;
  io_conf.pull_up_en = 0;
  gpio_config(&io_conf);

  ESP_LOGI(TAG, "STEP_DIR pins configured");

  return ESP_OK;
}

void set_microsteps(TMC2209_Driver *driver, uint16_t microsteps)
{
  driver->settings.microsteps = microsteps;

  uint32_t MRES = 0;
  switch (microsteps)
  {
  case 256:
    MRES = 0;
    break;
  case 128:
    MRES = 1;
    break;
  case 64:
    MRES = 2;
    break;
  case 32:
    MRES = 3;
    break;
  case 16:
    MRES = 4;
    break;
  case 8:
    MRES = 5;
    break;
  case 4:
    MRES = 6;
    break;
  case 2:
    MRES = 7;
    break;
  case 1:
    MRES = 8;
    break;
  default:
    // Handle invalid microsteps value (you might want to add error handling here)
    return;
  }

  // Update the MRES bits in the CHOPCONF register
  uint32_t chopconf;
  readRegister(driver, REG_CHOPCONF, &chopconf);
  chopconf = (chopconf & 0xF0FFFFFF) | (MRES << 24);
  writeRegister(driver, REG_CHOPCONF, chopconf);
}

void set_RMS_Current(TMC2209_Driver *driver, uint16_t mA)
{
  driver->settings.rms_current = mA;

  // Calculate current register value based on datasheet formula
  // Assuming a default sense resistor of 0.11 ohms (adjust if needed)
  float sense_resistor = 0.11;
  uint32_t current_reg = (uint32_t)((mA * 256.0) / 1.41421 / 1000.0 / sense_resistor);

  // Update the IRUN and IHOLD bits in the IHOLD_IRUN register
  uint32_t ihold_irun;
  readRegister(driver, REG_IHOLD_IRUN, &ihold_irun);
  ihold_irun = (ihold_irun & 0xFF000000) | (current_reg & 0x0000001F) | ((current_reg & 0x0000001F) << 8);
  writeRegister(driver, REG_IHOLD_IRUN, ihold_irun);
}

void setStealthchop(TMC2209_Driver *driver, bool enable)
{
  driver->settings.stealthchop.enabled = enable;

  uint32_t chopconf;
  readRegister(driver, REG_CHOPCONF, &chopconf);

  if (enable)
  {
    chopconf &= ~(1 << 14); // Clear CHM bit for stealthChop
  }
  else
  {
    chopconf |= (1 << 14); // Set CHM bit for spreadCycle
  }

  writeRegister(driver, REG_CHOPCONF, chopconf);
}

esp_err_t get_driver_settings(TMC2209_Driver *driver)
{
  uint32_t reg_value;

  // Read GCONF register
  esp_err_t ret = readRegister(driver, REG_GCONF, &reg_value);
  if (ret != ESP_OK)
  {
    return ret;
  }

  // Extract settings from GCONF
  driver->settings.stealthchop.enabled = ((reg_value >> 3) & 0x01) == 0;
  driver->settings.inverse_motor_direction_enabled = (reg_value >> 4) & 0x01;
  driver->settings.analog_current_scaling_enabled = (reg_value & 0x01);
  driver->settings.internal_sense_resistors_enabled = ((reg_value >> 1) & 0x01);

  // Read CHOPCONF register
  ret = readRegister(driver, REG_CHOPCONF, &reg_value);
  if (ret != ESP_OK)
  {
    return ret;
  }

  // Extract microsteps from CHOPCONF
  uint8_t MRES = (reg_value >> 24) & 0x0F;
  switch (MRES)
  {
  case 0:
    driver->settings.microsteps = 256;
    break;
  case 1:
    driver->settings.microsteps = 128;
    break;
  case 2:
    driver->settings.microsteps = 64;
    break;
  case 3:
    driver->settings.microsteps = 32;
    break;
  case 4:
    driver->settings.microsteps = 16;
    break;
  case 5:
    driver->settings.microsteps = 8;
    break;
  case 6:
    driver->settings.microsteps = 4;
    break;
  case 7:
    driver->settings.microsteps = 2;
    break;
  case 8:
    driver->settings.microsteps = 1;
    break;
  default:
    // Handle invalid MRES value (you might want to add error handling here)
    return ESP_FAIL;
  }

  // Read IHOLD_IRUN register
  ret = readRegister(driver, REG_IHOLD_IRUN, &reg_value);
  if (ret != ESP_OK)
  {
    return ret;
  }

  // Extract current settings from IHOLD_IRUN
  uint8_t IHOLD = reg_value & 0x1F;
  uint8_t IRUN = (reg_value >> 8) & 0x1F;
  uint8_t IHOLDDELAY = (reg_value >> 16) & 0x0F;

  // Convert current settings to percentages (you'll need to implement these helper functions)
  driver->settings.ihold_percent = current_setting_to_percent(IHOLD);
  driver->settings.irun_percent = current_setting_to_percent(IRUN);
  driver->settings.iholddelay_percent = hold_delay_setting_to_percent(IHOLDDELAY);

  // Read PWMCONF register
  ret = readRegister(driver, REG_PWMCONF, &reg_value);
  if (ret != ESP_OK)
  {
    return ret;
  }

  // Extract settings from PWMCONF
  driver->settings.standstill_mode = (reg_value >> 13) & 0x03;
  driver->settings.automatic_pwm_scaling_enabled = (reg_value >> 11) & 0x01;
  driver->settings.automatic_gradient_adaptation_enabled = (reg_value >> 12) & 0x01;
  driver->settings.pwm_offset = reg_value & 0xFF;
  driver->settings.pwm_gradient = (reg_value >> 8) & 0xFF;

  // ... (Read and extract other settings as needed)

  return ESP_OK;
}

uint8_t current_setting_to_percent(uint8_t current_setting)
{
  // The datasheet states:
  // - The I_rms current is scaled linearly with current_setting between 0 and I_rms with current_setting = 31
  // - current_setting = 0: Motor current is 0
  // - current_setting = 31: Motor current is I_rms

  // Ensure the current_setting is within the valid range
  if (current_setting > 31)
  {
    current_setting = 31;
  }

  // Calculate the percentage based on the linear scaling
  return (uint8_t)(((float)current_setting / 31.0f) * 100.0f);
}

uint8_t hold_delay_setting_to_percent(uint8_t hold_delay_setting)
{
  // The datasheet states:
  // - The I_hold delay time after a standstill is scaled exponentially with iholddelay
  // - iholddelay = 0: Delay is 2^0 * 2 clock cycles (fast decay)
  // - iholddelay = 15: Delay is 2^15 * 2 clock cycles (slow decay)

  // Ensure the hold_delay_setting is within the valid range
  if (hold_delay_setting > 15)
  {
    hold_delay_setting = 15;
  }

  // Calculate the percentage based on the exponential scaling
  // We'll map 0 to 0% and 15 to 100%
  float delay_ratio = (1 << hold_delay_setting) / (float)(1 << 15); // Ratio of current delay to maximum delay
  return (uint8_t)(delay_ratio * 100.0f);
}

// Function to enable/disable CoolStep
void setCoolStep(TMC2209_Driver *driver, bool enable)
{
  uint32_t coolconf;
  esp_err_t ret = readRegister(driver, REG_COOLCONF, &coolconf);
  if (ret != ESP_OK)
  {
    // Handle error reading COOLCONF
    return;
  }

  if (enable)
  {
    // Enable CoolStep (set semin to a non-zero value)
    coolconf |= 0x0000000F; // Example: Set semin to 1 (minimum value to enable)
  }
  else
  {
    // Disable CoolStep (set semin to 0)
    coolconf &= 0xFFFFFFF0;
  }

  writeRegister(driver, REG_COOLCONF, coolconf);
}

// Function to configure StallGuard4 threshold
void setStallGuardThreshold(TMC2209_Driver *driver, uint8_t threshold)
{
  writeRegister(driver, REG_SGTHRS, threshold);
}

// Function to get the StallGuard4 result
uint16_t getStallGuardResult(TMC2209_Driver *driver)
{
  uint32_t sg_result_reg;
  esp_err_t ret = readRegister(driver, REG_SG_RESULT, &sg_result_reg);
  if (ret != ESP_OK)
  {
    // Handle error reading SG_RESULT
    return 0; // Or another appropriate error value
  }

  return (uint16_t)(sg_result_reg & 0x3FF); // Extract 10-bit SG_RESULT value
}

void setChopperMode(TMC2209_Driver *driver, TMC2209_ChopperMode mode)
{
  uint32_t gconf;
  ESP_ERROR_CHECK(readRegister(driver, REG_GCONF, &gconf));

  if (mode == CHOPPER_MODE_STEALTHCHOP)
  {
    gconf &= ~(1 << 2); // Clear en_SpreadCycle bit for StealthChop
  }
  else if (mode == CHOPPER_MODE_SPREADCYCLE)
  {
    gconf |= (1 << 2); // Set en_SpreadCycle bit for SpreadCycle
  }
  else
  {
    ESP_LOGE(TAG, "Invalid chopper mode: %d", mode);
    return; // Or handle the error appropriately
  }

  ESP_ERROR_CHECK(writeRegister(driver, REG_GCONF, gconf));
}

// Function to write StealthChop configuration to the TMC2209 driver
void writeStealthchopConfig(TMC2209_Driver *driver, const TMC2209_StealthchopConfig *config)
{

  // Configure PWMCONF register
  uint32_t pwmconf = 0;

  // Bit 10: pwm_autoscale
  if (config->pwm_autoscale)
  {
    pwmconf |= (1 << 10);
  }

  // Bit 18: pwm_autograd
  if (config->pwm_auto_gradient_adaptation)
  {
    pwmconf |= (1 << 18);
  }

  // Bits 15-17: pwm_freq
  pwmconf |= (config->pwm_frequency & 0x03) << 15;

  // Bits 0-7: pwm_ofs
  pwmconf |= config->pwm_offset & 0xFF;

  // Bits 8-14: pwm_grad
  pwmconf |= (config->pwm_gradient & 0x7F) << 8;

  // Bits 20-21: freewheel (Standstill Mode)
  pwmconf |= (config->standstill_mode & 0x03) << 20;

  // Bits 28-31: pwm_lim (PWM Limit)
  pwmconf |= (config->pwm_amplitude_limit & 0x0F) << 28;

  ESP_ERROR_CHECK(writeRegister(driver, REG_PWMCONF, pwmconf));

  // 3. Configure TPWMTHRS register (if hybrid mode is used)
  if (config->velocity_threshold > 0)
  {
    ESP_ERROR_CHECK(writeRegister(driver, REG_TPWMTHRS, config->velocity_threshold));
  }
}

// Function to write SpreadCycle configuration to the TMC2209 driver
void writeSpreadCycleConfig(TMC2209_Driver *driver, const TMC2209_SpreadCycleConfig *config)
{
  // Configure CHOPCONF register
  uint32_t chopconf = 0;

  // Bits 0-3: toff (Slow Decay Time)
  chopconf |= config->slow_decay_time & 0x0F;

  // Bits 14-15: tbl (Blank Time)
  chopconf |= (config->blank_time & 0x03) << 14;

  // Bits 4-7: hstrt (Hysteresis Start Value)
  chopconf |= (config->hysteresis_start & 0x0F) << 4;

  // Bits 8-13: hend (Hysteresis End Value)
  chopconf |= (config->hysteresis_end & 0x3F) << 8;

  ESP_ERROR_CHECK(writeRegister(driver, REG_CHOPCONF, chopconf));
}
