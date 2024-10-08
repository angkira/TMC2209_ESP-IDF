#ifndef TMC2209_DRIVER_DTO_H
#define TMC2209_DRIVER_DTO_H

#include <stdint.h>
#include <stdbool.h>

#include "driver/gpio.h"
#include "driver/uart.h"

// Stallguard Configuration Structure
typedef struct
{
    bool enabled;             // Enable/disable StallGuard
    uint8_t threshold;        // StallGuard threshold value (SG_THRS register)
    bool filter_enabled;      // Enable/disable StallGuard filter (SGFILT bit in CHOPCONF)
    bool hysteresis_enabled;  // Enable/disable StallGuard hysteresis (SGHST bit in COOLCONF)
    uint8_t hysteresis_low;   // StallGuard hysteresis low value (SG_HYS register, bits 0-7)
    uint8_t hysteresis_start; // StallGuard hysteresis start value (SG_HYS register, bits 8-15)
} TMC2209_StallguardConfig;

// StealthChop Configuration Structure
typedef struct
{
    bool enabled;                      // Enable/disable StealthChop
    uint32_t velocity_threshold;       // Velocity threshold for switching to SpreadCycle (if used in hybrid mode)
    uint8_t pwm_amplitude_limit;       // PWM amplitude limit when switching on from SpreadCycle
    uint8_t pwm_regulation_gradient;   // Maximum PWM amplitude change per half wave
    bool pwm_auto_gradient_adaptation; // Enable/disable automatic PWM gradient adaptation
    bool pwm_autoscale;                // Enable/disable automatic PWM amplitude scaling
    uint8_t pwm_frequency;             // PWM frequency selection
    uint8_t pwm_gradient;              // User-defined amplitude gradient
    uint8_t pwm_offset;                // User-defined amplitude offset
    uint8_t standstill_mode;           // Standstill mode (normal, freewheeling, or coil short)
} TMC2209_StealthchopConfig;

// SpreadCycle Configuration Structure
typedef struct
{
    bool enabled;             // Enable/disable SpreadCycle
    uint8_t slow_decay_time;  // Slow decay time (toff)
    uint8_t blank_time;       // Comparator blank time (tbl)
    uint8_t hysteresis_start; // Hysteresis start value (hstrt)
    uint8_t hysteresis_end;   // Hysteresis end value (hend)
} TMC2209_SpreadCycleConfig;

typedef struct
{
    uint8_t driver_address;
    uint16_t rms_current;
    uint16_t microsteps;

    bool stealthchop_enabled;
    bool inverse_motor_direction_enabled;
    bool analog_current_scaling_enabled;
    bool internal_sense_resistors_enabled;
    bool index_otpw_enabled;
    uint8_t ihold_percent;
    bool index_step_enabled;
    uint8_t irun_percent;
    uint8_t iholddelay_percent;

    bool coolstep_enabled;

    float standstill_current_timeout;
    bool vsense;
    bool diag0_error_enabled;
    bool diag0_otpw_enabled;
    bool diag0_stall_enabled;
    bool diag1_stall_enabled;
    bool diag1_index_enabled;
    bool diag1_onstate_enabled;
    bool diag1_steps_skipped_enabled;

    bool interpolate_to_256_microsteps;
    bool enable_double_edge_step_pulses;
    uint8_t coolstep_current_up_step_width;
    uint8_t coolstep_current_down_step_speed;

    TMC2209_StallguardConfig stallguard;
    TMC2209_StealthchopConfig stealthchop;
    TMC2209_SpreadCycleConfig spreadcycle;
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
