#include "tmc2209.h"
#include "esp_log.h"

static const char *TAG = "main";

#define TMC2209_ADDRESS 0x00

void app_main()
{
  TMC2209_Driver motorDriver;
  motorDriver.uart_num = UART_NUM_1;
  motorDriver.dir_pin = 6;
  motorDriver.step_pin = 7;

  // Initialize driver settings
  motorDriver.settings.driver_address = TMC2209_ADDRESS;
  motorDriver.settings.rms_current = 800;
  motorDriver.settings.microsteps = 64;
  motorDriver.settings.stealthchop_enabled = true;
  // Setup the driver
  esp_err_t err = tmc2209_setup(&motorDriver);
  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "Error setting up TMC2209 driver: %s", esp_err_to_name(err));
    return;
  }

  // // Enable the driver
  tmc2209_enable(&motorDriver);
  ESP_LOGI(TAG, "TMC2209 driver enabled");

  // Basic motor control sequence
  ESP_LOGI(TAG, "Starting motor movement...");
  // tmc2209_moveAtVelocity(&motorDriver, 10000);

  tmc2209_rotate(&motorDriver, 20000, 100000);

  // vTaskDelay(20000 / portTICK_PERIOD_MS);

  // tmc2209_moveAtVelocity(&motorDriver, 0);

  // Disable the driver when finished
  tmc2209_disable(&motorDriver);
  ESP_LOGI(TAG, "TMC2209 driver disabled");
}
