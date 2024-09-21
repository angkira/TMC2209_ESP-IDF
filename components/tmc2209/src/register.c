#include "driver.dto.h"

#include "register.h"

#include "string.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/uart.h"

static const char *TAG = "tmc2209";
static const char *TAG_TX = "tx";
static const char *TAG_RX = "rx";

uint8_t calcCRC(uint8_t datagram[], uint8_t len)
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

esp_err_t readRegister(TMC2209_Driver *driver, uint8_t address, uint32_t *data)
{
  uint8_t tx_buf[5] = {0x05, address, 0x00, 0x00, 0x00};
  uint8_t rx_buf[9];

  tx_buf[4] = calcCRC(tx_buf, 4);

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

  if (calcCRC(rx_buf, 8) != rx_buf[8])
  {
    return ESP_ERR_INVALID_CRC;
  }

  *data = (rx_buf[5] << 24) | (rx_buf[6] << 16) | (rx_buf[7] << 8);

  return ESP_OK;
}

esp_err_t writeRegister(TMC2209_Driver *driver, uint8_t address, uint32_t data)
{
  uint8_t tx_buf[9] = {0x05, address | 0x80, (data >> 24) & 0xFF, (data >> 16) & 0xFF, (data >> 8) & 0xFF, data & 0xFF, 0x00};
  uint8_t rx_buf[5];

  tx_buf[8] = calcCRC(tx_buf, 8);

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
