#include "tmc2209.h"
#include "string.h"
#include "esp_log.h"

#include "driver/gpio.h"

#define REG_VMAX 0x20

static const char *TAG = "tmc2209";
static const char *TAG_TX = "tmc2209_tx";
static const char *TAG_RX = "tmc2209_rx";

// --- Helper Functions ---

uint8_t tmc2209_calcCRC(uint8_t datagram[], uint8_t len)
{
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++)
  {
    uint8_t current_byte = datagram[i];
    for (uint8_t j = 0; j < 8; j++)
    {
      uint8_t bit = (crc ^ current_byte) & 0x01;
      crc >>= 1;
      if (bit)
      {
        crc ^= 0x8C;
      }
      current_byte >>= 1;
    }
  }
  return crc;
}

// --- Core Library Functions ---

esp_err_t tmc2209_setup(TMC2209_Driver *driver)
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

  // Basic configuration (example - adjust as needed)
  tmc2209_writeRegister(driver, REG_GCONF, 0x0000000C);
  ESP_LOGI(TAG, "GCONF configurated");
  tmc2209_writeRegister(driver, REG_CHOPCONF, 0x000100C5);
  ESP_LOGI(TAG, "CHOP_CONF configurated");
  tmc2209_writeRegister(driver, REG_IHOLD_IRUN, 0x00011F05);
  ESP_LOGI(TAG, "IHOLD configurated");
  tmc2209_writeRegister(driver, REG_TPOWERDOWN, 0x0000000A);
  ESP_LOGI(TAG, "TPOWERDOWN configurated");

  tmc2209_writeRegister(driver, REG_TPWMTHRS, 0x000001F4);
  ESP_LOGI(TAG, "PWM configurated");

  // Configure microstepping and current
  tmc2209_setMicrosteps(driver, driver->settings.microsteps);
  ESP_LOGI(TAG, "Microsteps are configured");
  tmc2209_setRMS_Current(driver, driver->settings.rms_current);
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

void tmc2209_rotate(TMC2209_Driver *driver, int32_t steps, uint32_t speed)
{
  // Set DIR pin based on the sign of steps
  gpio_set_level(driver->dir_pin, steps >= 0 ? 0 : 1);

  tmc2209_setTargetVelocity(driver, speed);

  // Get the current tick count
  TickType_t xLastWakeTime = xTaskGetTickCount();
  // Calculate delay between steps (in microseconds)
  uint32_t delay_us = 1000000 / speed / driver->settings.microsteps;

  // Ensure delay is at least one tick
  if (delay_us < portTICK_PERIOD_MS)
  {
    delay_us = portTICK_PERIOD_MS; // Minimum delay of one tick
  }

  uint32_t microsteps = abs(steps) * driver->settings.microsteps;

  // Loop for the absolute value of steps
  for (int32_t i = 0; i < microsteps; i++)
  {
    // Toggle STEP pin
    gpio_set_level(driver->step_pin, 1);
    esp_rom_delay_us(10); // Short pulse, adjust if needed
    gpio_set_level(driver->step_pin, 0);

    // Introduce delay
    vTaskDelayUntil(&xLastWakeTime, delay_us / portTICK_PERIOD_MS);
  }

  // Switch back to UART control mode (VACTUAL = 0)
  tmc2209_writeRegister(driver, REG_VACTUAL, 0);
}

void tmc2209_enable(TMC2209_Driver *driver)
{
  tmc2209_writeRegister(driver, REG_GCONF, 0x0000000F);
}

void tmc2209_disable(TMC2209_Driver *driver)
{
  tmc2209_writeRegister(driver, REG_GCONF, 0x0000000C);
}

// ... (Previous code from tmc2209.c)

void tmc2209_moveAtVelocity(TMC2209_Driver *driver, int32_t velocity)
{
  tmc2209_writeRegister(driver, REG_VMAX, abs(velocity));
  tmc2209_writeRegister(driver, REG_XDIRECT, (velocity >= 0) ? 0 : 0xFFFFFFFF); // Direction
}

esp_err_t tmc2209_readRegister(TMC2209_Driver *driver, uint8_t address, uint32_t *data)
{
  uint8_t tx_buf[5] = {0x05, address, 0x00, 0x00, 0x00};
  uint8_t rx_buf[9];

  tx_buf[4] = tmc2209_calcCRC(tx_buf, 4);

  int tx_bytes = uart_write_bytes(driver->uart_num, (const char *)tx_buf, sizeof(tx_buf));
  if (tx_bytes < 0)
  {
    return ESP_FAIL;
  }

  ESP_LOG_BUFFER_HEX(TAG_TX, tx_buf, sizeof(tx_buf)); // Log transmitted data

  vTaskDelay(10 / portTICK_PERIOD_MS);

  int rx_bytes = uart_read_bytes(driver->uart_num, rx_buf, sizeof(rx_buf), 20 / portTICK_PERIOD_MS);
  if (rx_bytes < 0)
  {
    return ESP_FAIL;
  }

  ESP_LOG_BUFFER_HEX(TAG_RX, rx_buf, rx_bytes); // Log received data

  if (tmc2209_calcCRC(rx_buf, 8) != rx_buf[8])
  {
    return ESP_ERR_INVALID_CRC;
  }

  *data = (rx_buf[5] << 24) | (rx_buf[6] << 16) | (rx_buf[7] << 8);

  return ESP_OK;
}

esp_err_t tmc2209_writeRegister(TMC2209_Driver *driver, uint8_t address, uint32_t data)
{
  uint8_t tx_buf[9] = {0x05, address | 0x80, (data >> 24) & 0xFF, (data >> 16) & 0xFF, (data >> 8) & 0xFF, data & 0xFF, 0x00};
  uint8_t rx_buf[5];

  tx_buf[8] = tmc2209_calcCRC(tx_buf, 8);

  int tx_bytes = uart_write_bytes(driver->uart_num, (const char *)tx_buf, sizeof(tx_buf));
  if (tx_bytes < 0)
  {
    return ESP_FAIL;
  }

  ESP_LOG_BUFFER_HEX(TAG_TX, tx_buf, sizeof(tx_buf));

  vTaskDelay(10 / portTICK_PERIOD_MS);

  int rx_bytes = uart_read_bytes(driver->uart_num, rx_buf, sizeof(rx_buf), 20 / portTICK_PERIOD_MS);
  if (rx_bytes < 0)
  {
    return ESP_FAIL;
  }

  ESP_LOG_BUFFER_HEX(TAG_RX, rx_buf, rx_bytes);

  for (int i = 0; i < 5; i++)
  {
    if (tx_buf[i] != rx_buf[i])
    {
      return ESP_ERR_INVALID_RESPONSE;
    }
  }

  return ESP_OK;
}

void tmc2209_setMicrosteps(TMC2209_Driver *driver, uint16_t microsteps)
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
  tmc2209_readRegister(driver, REG_CHOPCONF, &chopconf);
  chopconf = (chopconf & 0xF0FFFFFF) | (MRES << 24);
  tmc2209_writeRegister(driver, REG_CHOPCONF, chopconf);
}

void tmc2209_setRMS_Current(TMC2209_Driver *driver, uint16_t mA)
{
  driver->settings.rms_current = mA;

  // Calculate current register value based on datasheet formula
  // Assuming a default sense resistor of 0.11 ohms (adjust if needed)
  float sense_resistor = 0.11;
  uint32_t current_reg = (uint32_t)((mA * 256.0) / 1.41421 / 1000.0 / sense_resistor);

  // Update the IRUN and IHOLD bits in the IHOLD_IRUN register
  uint32_t ihold_irun;
  tmc2209_readRegister(driver, REG_IHOLD_IRUN, &ihold_irun);
  ihold_irun = (ihold_irun & 0xFF000000) | (current_reg & 0x0000001F) | ((current_reg & 0x0000001F) << 8);
  tmc2209_writeRegister(driver, REG_IHOLD_IRUN, ihold_irun);
}

void tmc2209_setStealthchop(TMC2209_Driver *driver, bool enable)
{
  driver->settings.stealthchop_enabled = enable;

  uint32_t chopconf;
  tmc2209_readRegister(driver, REG_CHOPCONF, &chopconf);

  if (enable)
  {
    chopconf &= ~(1 << 14); // Clear CHM bit for stealthChop
  }
  else
  {
    chopconf |= (1 << 14); // Set CHM bit for spreadCycle
  }

  tmc2209_writeRegister(driver, REG_CHOPCONF, chopconf);
}

esp_err_t tmc2209_getSettings(TMC2209_Driver *driver)
{
  uint32_t reg_value;

  // Read GCONF register
  esp_err_t ret = tmc2209_readRegister(driver, REG_GCONF, &reg_value);
  if (ret != ESP_OK)
  {
    return ret;
  }

  // Extract settings from GCONF
  driver->settings.stealthchop_enabled = ((reg_value >> 3) & 0x01) == 0;
  driver->settings.inverse_motor_direction_enabled = (reg_value >> 4) & 0x01;
  driver->settings.analog_current_scaling_enabled = (reg_value & 0x01);
  driver->settings.internal_sense_resistors_enabled = ((reg_value >> 1) & 0x01);

  // Read CHOPCONF register
  ret = tmc2209_readRegister(driver, REG_CHOPCONF, &reg_value);
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
  ret = tmc2209_readRegister(driver, REG_IHOLD_IRUN, &reg_value);
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
  ret = tmc2209_readRegister(driver, REG_PWMCONF, &reg_value);
  if (ret != ESP_OK)
  {
    return ret;
  }

  // Extract settings from PWMCONF
  driver->settings.standstill_mode = (reg_value >> 13) & 0x03;
  driver->settings.automatic_current_scaling_enabled = (reg_value >> 11) & 0x01;
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

esp_err_t tmc2209_getStatus(TMC2209_Driver *driver)
{
  uint32_t reg_value;

  // Read DRV_STATUS register
  esp_err_t ret = tmc2209_readRegister(driver, REG_DRV_STATUS, &reg_value);
  if (ret != ESP_OK)
  {
    return ret;
  }

  // Extract status flags from DRV_STATUS
  // ... (Implement based on the Arduino library's logic and the TMC2209 datasheet)

  // Read other relevant registers and extract status information as needed
  // ...

  return ESP_OK;
}

// ... (Previous code from tmc2209.c)

// Function to set the motor's target velocity in microsteps per second
void tmc2209_setTargetVelocity(TMC2209_Driver *driver, int32_t velocity)
{
  // The datasheet mentions that VACTUAL allows moving the motor by UART control
  // It gives the motor velocity in +-(2^23)-1 [usteps / t]
  // 0: Normal operation. Driver reacts to STEP input.
  // /=0: Motor moves with the velocity given by VACTUAL
  // The motor direction is controlled by the sign of VACTUAL

  // With internal oscillator:
  // VACTUAL[2209] = 0.715Hz / v[Hz]

  // Convert velocity (usteps/s) to VACTUAL
  int32_t vactual = (int32_t)((0.715f / (float)velocity) * (1 << 23));

  // Ensure VACTUAL is within the valid range
  if (vactual > ((1 << 23) - 1))
  {
    vactual = (1 << 23) - 1;
  }
  else if (vactual < -((1 << 23) - 1))
  {
    vactual = -((1 << 23) - 1);
  }

  tmc2209_writeRegister(driver, REG_VACTUAL, vactual);
}

// Function to enable/disable CoolStep
void tmc2209_setCoolStep(TMC2209_Driver *driver, bool enable)
{
  uint32_t coolconf;
  esp_err_t ret = tmc2209_readRegister(driver, REG_COOLCONF, &coolconf);
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

  tmc2209_writeRegister(driver, REG_COOLCONF, coolconf);
}

// Function to configure StallGuard4 threshold
void tmc2209_setStallGuardThreshold(TMC2209_Driver *driver, uint8_t threshold)
{
  tmc2209_writeRegister(driver, REG_SGTHRS, threshold);
}

// Function to get the StallGuard4 result
uint16_t tmc2209_getStallGuardResult(TMC2209_Driver *driver)
{
  uint32_t sg_result_reg;
  esp_err_t ret = tmc2209_readRegister(driver, REG_SG_RESULT, &sg_result_reg);
  if (ret != ESP_OK)
  {
    // Handle error reading SG_RESULT
    return 0; // Or another appropriate error value
  }

  return (uint16_t)(sg_result_reg & 0x3FF); // Extract 10-bit SG_RESULT value
}
