#include "tmc2209.h"
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
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };

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

  // Basic configuration (example - adjust as needed)

  uint32_t gconf_data = configure_gconf(&driver->settings);

  writeRegister(driver, REG_GCONF, gconf_data);
  ESP_LOGI(TAG, "GCONF configurated");

  uint32_t chopconf_data = configure_chopconf(&driver->settings);

  writeRegister(driver, REG_CHOPCONF, chopconf_data);

  ESP_LOGI(TAG, "CHOP_CONF configurated");

  uint32_t ihold_irun_data = configure_ihold_irun(&driver->settings);

  writeRegister(driver, REG_IHOLD_IRUN, ihold_irun_data);

  ESP_LOGI(TAG, "IHOLD configurated");

  uint32_t tpowrdown_data = configure_tpowerdown(&driver->settings);

  writeRegister(driver, REG_TPOWERDOWN, tpowrdown_data);

  ESP_LOGI(TAG, "TPOWERDOWN configurated");

  uint32_t sgthrs_data = configure_sgthrs(&driver->settings);

  writeRegister(driver, REG_TPWMTHRS, sgthrs_data);
  ESP_LOGI(TAG, "PWM configurated");

  uint32_t coolconf_data = configure_coolconf(&driver->settings);

  writeRegister(driver, REG_COOLCONF, coolconf_data);

  ESP_LOGI(TAG, "COOLCONF configurated");

  uint32_t pwmconf_data = configure_pwmconf(&driver->settings);

  writeRegister(driver, REG_PWMCONF, pwmconf_data);

  ESP_LOGI(TAG, "PWMCONF configurated");

  uint32_t tpwmthrs_data = configure_tpwmthrs(&driver->settings);

  writeRegister(driver, REG_TPWMTHRS, tpwmthrs_data);

  ESP_LOGI(TAG, "TPWMTHRS configurated");

  // Configure microstepping and current
  set_microsteps(driver, driver->settings.microsteps);
  ESP_LOGI(TAG, "Microsteps are configured");
  set_RMS_Current(driver, driver->settings.rms_current);
  ESP_LOGI(TAG, "RMS is configured");

  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = (1ULL << driver->step_pin) | (1ULL << driver->dir_pin);
  io_conf.pull_down_en = 0;
  io_conf.pull_up_en = 0;
  gpio_config(&io_conf);

  ESP_LOGI(TAG, "STEP_DIR pins configurated");

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
  driver->settings.stealthchop_enabled = enable;

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

// 1. Function to configure the GCONF register
uint32_t configure_gconf(const TMC2209_Settings *settings)
{
  uint32_t gconf = 0;

  // Bit 0: i_scale_analog (Internal vs. External VREF)
  if (settings->analog_current_scaling_enabled)
  {
    gconf |= 1; // Use external VREF
  }

  // Bit 1: internal_Rsense (Internal vs. External Sense Resistors)
  if (settings->internal_sense_resistors_enabled)
  {
    gconf |= (1 << 1); // Use internal sense resistors
  }

  // Bit 2: en_SpreadCycle (StealthChop vs. SpreadCycle)
  if (settings->spreadcycle.enabled)
  {
    gconf |= (1 << 2); // Enable SpreadCycle
  }

  // Bit 3: shaft (Inverse Motor Direction)
  if (settings->inverse_motor_direction_enabled)
  {
    gconf |= (1 << 4);
  }

  // Bit 5: index_otpw (Enable/Disable INDEX output toggling on over temperature warning)
  if (settings->index_otpw_enabled)
  {
    gconf |= (1 << 5);
  }

  // Bit 6: index_step (Enable/Disable INDEX output toggling on each full step)
  if (settings->index_step_enabled)
  {
    gconf |= (1 << 6);
  }

  // Bit 7: diag0_error (Enable/Disable DIAG0 active on driver errors)
  if (settings->diag0_error_enabled)
  {
    gconf |= (1 << 7);
  }

  // Bit 10: diag1_steps_skipped (Enable/Disable DIAG1 active when steps are skipped in micro step interpolation)
  if (settings->diag1_steps_skipped_enabled)
  {
    gconf |= (1 << 10);
  }

  // Bit 11: diag0_otpw (Enable/Disable DIAG0 active on over temperature warning)
  if (settings->diag0_otpw_enabled)
  {
    gconf |= (1 << 11);
  }

  // Bit 12: diag0_stall (Enable/Disable DIAG0 active on stallguard2)
  if (settings->diag0_stall_enabled)
  {
    gconf |= (1 << 12);
  }

  // Bit 13: diag1_stall (Enable/Disable DIAG1 active on stallguard2)
  if (settings->diag1_stall_enabled)
  {
    gconf |= (1 << 13);
  }

  // Bit 14: diag1_index (Enable/Disable DIAG1 active on index)
  if (settings->diag1_index_enabled)
  {
    gconf |= (1 << 14);
  }

  // Bit 15: diag1_onstate (Enable/Disable DIAG1 active on ON_STATE)
  if (settings->diag1_onstate_enabled)
  {
    gconf |= (1 << 15);
  }

  return gconf;
}

// 2. Function to configure the IHOLD_IRUN register
uint32_t configure_ihold_irun(const TMC2209_Settings *settings)
{
  uint32_t ihold_irun = 0;

  // Bits 8-12: IRUN (Motor Run Current)
  // Convert RMS current to IRUN using datasheet formula and table
  float sense_resistor = settings->internal_sense_resistors_enabled ? 0.0 : 0.11; // Adjust if using different sense resistors
  uint8_t irun = (uint8_t)((settings->rms_current * 256.0) / 1.41421 / 1000.0 / sense_resistor);
  if (irun > 31)
  {
    irun = 31; // Limit to maximum value
  }
  ihold_irun |= (irun << 8);

  // Bits 0-4: IHOLD (Motor Hold Current)
  // Calculate IHOLD based on irun and ihold_percent
  uint8_t ihold = (uint8_t)((irun * settings->ihold_percent) / 100.0);
  if (ihold > 31)
  {
    ihold = 31; // Limit to maximum value
  }
  ihold_irun |= ihold;

  // Bits 16-19: IHOLDDELAY (Hold Current Delay)
  // Convert iholddelay_percent to IHOLDDELAY
  // Assuming iholddelay_percent represents the percentage of the maximum delay
  // Maximum IHOLDDELAY value is 15, representing 2^15 * 2 clock cycles delay
  uint8_t iholddelay = (uint8_t)((settings->iholddelay_percent / 100.0) * 15);
  ihold_irun |= (iholddelay << 16);

  return ihold_irun;
}

// 3. Function to configure the TPOWERDOWN register
uint32_t configure_tpowerdown(const TMC2209_Settings *settings)
{
  uint32_t tpowerdown = 0;

  // Bits 0-7: TPOWERDOWN (Power Down Delay)
  // Convert standstill current timeout from seconds to register value (0-255)
  // Formula: TPOWERDOWN = timeout[s] * 2^18 / 10
  tpowerdown = (uint32_t)(settings->standstill_current_timeout * (1 << 18) / 10.0);
  if (tpowerdown > 255)
  {
    tpowerdown = 255; // Limit to maximum value
  }

  return tpowerdown;
}

uint32_t get_chopconf_with_microsteps(uint32_t current_chopconf, uint16_t microsteps)
{
  uint8_t mres = 0;
  switch (microsteps)
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
    ESP_LOGE(TAG, "Invalid microsteps value: %d", microsteps);
    abort(); // Or handle the error appropriately in your application
  }

  // Update the MRES bits in the CHOPCONF register
  return (current_chopconf & 0xF0FFFFFF) | (mres << 24);
}

// Function to configure the CHOPCONF register
uint32_t configure_chopconf(const TMC2209_Settings *settings)
{
  uint32_t chopconf = 0;

  // Bits 24-27: MRES (Microstep Resolution)
  uint8_t mres = 0;
  switch (settings->microsteps)
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
    ESP_LOGE(TAG, "Invalid microsteps value: %d", settings->microsteps);
    abort(); // Or handle the error appropriately in your application
  }
  chopconf |= (mres << 24);

  // Bit 17: vsense (Sense Resistor Voltage Selection)
  if (settings->vsense)
  {
    chopconf |= (1 << 17); // High sensitivity, low sense resistor voltage
  }

  // Bit 27: intpol (Interpolation for 256 Microsteps)
  if (settings->interpolate_to_256_microsteps)
  {
    chopconf |= (1 << 27);
  }

  // Bit 28: dedge (Enable Double Edge STEP pulses)
  if (settings->enable_double_edge_step_pulses)
  {
    chopconf |= (1 << 28);
  }

  // Bits 29-31: Short circuit protection settings (adjust as needed)
  chopconf |= 0; // All short protection enabled (default)

  return chopconf;
}

// Function to configure the TPWMTHRS register
uint32_t configure_tpwmthrs(const TMC2209_Settings *settings)
{
  // Bits 0-23: TPWMTHRS (StealthChop PWM Threshold)
  return settings->stealthchop.velocity_threshold;
}

// Function to configure the PWMCONF register
uint32_t configure_pwmconf(const TMC2209_Settings *settings)
{
  uint32_t pwmconf = 0;

  // Bit 10: PWM_AUTOSCALE (Automatic PWM Scaling)
  if (settings->stealthchop.pwm_autoscale)
  {
    pwmconf |= (1 << 10);
  }

  // Bit 18: pwm_autograd (Automatic PWM gradient adaptation)
  if (settings->stealthchop.pwm_auto_gradient_adaptation)
  {
    pwmconf |= (1 << 18);
  }

  // Bits 15-17: pwm_freq (PWM frequency selection)
  pwmconf |= (settings->stealthchop.pwm_frequency & 0x03) << 15;

  // Bits 0-7: pwm_ofs (PWM offset)
  pwmconf |= settings->stealthchop.pwm_offset & 0xFF;

  // Bits 8-14: pwm_grad (PWM gradient)
  pwmconf |= (settings->stealthchop.pwm_gradient & 0x7F) << 8;

  // Bits 20-21: freewheel (Standstill Mode)
  pwmconf |= (settings->stealthchop.standstill_mode & 0x03) << 20;

  // Bits 28-31: pwm_lim (PWM Limit)
  pwmconf |= (settings->stealthchop.pwm_amplitude_limit & 0x0F) << 28;

  return pwmconf;
}

// Function to configure the COOLCONF register
uint32_t configure_coolconf(const TMC2209_Settings *settings)
{
  uint32_t coolconf = 0;

  if (settings->coolstep_enabled)
  {
    // Bits 0-3: semin (Minimum StallGuard Value for CoolStep)
    coolconf |= 1; // Example: Set semin to 1

    // Bits 4-7: semax (Maximum StallGuard Value for CoolStep)
    coolconf |= (10 << 4);

    // Bits 5-6: seup (Current Up Step Width)
    coolconf |= (settings->coolstep_current_up_step_width & 0x03) << 5;

    // Bits 16-19: sedec (Current Down Step Speed)
    coolconf |= (settings->coolstep_current_down_step_speed & 0x0F) << 16;

    // Bit 24: seimin (Minimum Current Decrease per Full-Step)
    coolconf |= (1 << 24);

    // Bit 25: semax (Maximum Current Decrease per Full-Step)
    coolconf |= (5 << 25);

    // Bit 28: sgt (Enable/Disable StallGuard Threshold)
    if (settings->stallguard.enabled)
    {
      coolconf |= (1 << 28);
    }
  }

  return coolconf;
}

// Function to configure the SGTHRS register
uint32_t configure_sgthrs(const TMC2209_Settings *settings)
{
  return settings->stallguard.threshold;
}
