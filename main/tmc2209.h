#ifndef TMC2209_ESP_IDF_H
#define TMC2209_ESP_IDF_H

#include "driver/gpio.h"
#include "driver/uart.h"

// Register addresses
#define REG_GCONF 0x00
#define REG_GSTAT 0x01
#define REG_IOIN 0x04
#define REG_IHOLD_IRUN 0x10
#define REG_TPOWERDOWN 0x11
#define REG_TSTEP 0x12
#define REG_TPWMTHRS 0x13
#define REG_TCOOLTHRS 0x14
#define REG_THIGH 0x15
#define REG_XDIRECT 0x2d
#define REG_MSLUT0 0x60
#define REG_MSLUT1 0x61
#define REG_MSLUT2 0x62
#define REG_MSLUT3 0x63
#define REG_MSLUT4 0x64
#define REG_MSLUT5 0x65
#define REG_MSLUT6 0x66
#define REG_MSLUT7 0x67
#define REG_MSLUTSEL 0x68
#define REG_MSLUTSTART 0x69
#define REG_MSCNT 0x6a
#define REG_MSCURACT 0x6b
#define REG_CHOPCONF 0x6c
#define REG_COOLCONF 0x6d
#define REG_DCCTRL 0x6e
#define REG_DRV_STATUS 0x6f
#define REG_PWMCONF 0x70
#define REG_PWM_SCALE 0x71
#define REG_ENCM_CTRL 0x72
#define REG_LOST_STEPS 0x73
#define REG_VACTUAL 0x22 // Register for setting target velocity
#define REG_SGTHRS  0x40 // Register for configuring StallGuard4 threshold
#define REG_SG_RESULT 0x41 // Register for StallGuard result

typedef struct {
    uint8_t driver_address;
    uint16_t rms_current;
    uint16_t microsteps;
    bool stealthchop_enabled;
    bool inverse_motor_direction_enabled;
    bool analog_current_scaling_enabled;
    bool internal_sense_resistors_enabled;
    uint8_t ihold_percent;
    uint8_t irun_percent;
    uint8_t iholddelay_percent;
    uint8_t standstill_mode;
    bool automatic_current_scaling_enabled;
    bool automatic_gradient_adaptation_enabled;
    uint8_t pwm_offset;
    uint8_t pwm_gradient;
    bool coolstep_enabled;
    uint8_t stallguard_threshold;
    // ... (Add other settings as needed)
} TMC2209_Settings;

typedef struct {
    bool standstill;
    bool sg2_result;
    // ... (Other status information as needed)
} TMC2209_Status;

// --- TMC2209 class adaptation ---

typedef struct {
    uart_port_t uart_num;
    TMC2209_Settings settings;
    TMC2209_Status status;
    gpio_num_t step_pin;
    gpio_num_t dir_pin;
} TMC2209_Driver;

// --- Function prototypes ---

esp_err_t tmc2209_setup(TMC2209_Driver *driver);
void tmc2209_enable(TMC2209_Driver *driver);
void tmc2209_disable(TMC2209_Driver *driver);
void tmc2209_moveAtVelocity(TMC2209_Driver *driver, int32_t velocity); 
// ... (Other configuration and control functions as needed)
esp_err_t tmc2209_readRegister(TMC2209_Driver *driver, uint8_t address, uint32_t *data);
esp_err_t tmc2209_writeRegister(TMC2209_Driver *driver, uint8_t address, uint32_t data);
void tmc2209_setMicrosteps(TMC2209_Driver *driver, uint16_t microsteps);
void tmc2209_setRMS_Current(TMC2209_Driver *driver, uint16_t mA);
void tmc2209_setTargetVelocity(TMC2209_Driver *driver, int32_t velocity);

void tmc2209_rotate(TMC2209_Driver *driver, int32_t steps, uint32_t speed);

esp_err_t tmc2209_getSettings(TMC2209_Driver *driver);
esp_err_t tmc2209_getStatus(TMC2209_Driver *driver);

uint8_t current_setting_to_percent(uint8_t current_setting);
uint8_t hold_delay_setting_to_percent(uint8_t hold_delay_setting);

#endif // TMC2209_ESP_IDF_H
