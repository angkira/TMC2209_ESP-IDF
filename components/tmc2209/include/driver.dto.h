#ifndef TMC2209_DRIVER_DTO_H
#define TMC2209_DRIVER_DTO_H

#include <stdint.h>
#include <stdbool.h>

#include "driver/gpio.h"
#include "driver/uart.h"

typedef struct
{
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
} TMC2209_Settings;

typedef struct
{
    bool standstill;
    bool sg2_result;
    // ... (Other status information as needed)
} TMC2209_Status;

typedef struct
{
    uart_port_t uart_num;
    TMC2209_Settings settings;
    TMC2209_Status status;
    gpio_num_t step_pin;
    gpio_num_t dir_pin;
} TMC2209_Driver;

#endif //  TMC2209_DRIVER_DTO_H
