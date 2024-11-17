#ifndef TMC2209_CONTROL_H
#define TMC2209_CONTROL_H

#include "driver.dto.h"
#include "setup.h"

void enable_driver(TMC2209_Driver *driver);
void disable_driver(TMC2209_Driver *driver);
void moveAtVelocity(TMC2209_Driver *driver, int32_t velocity);

void set_target_velocity(TMC2209_Driver *driver, int32_t velocity);

void rotate_by_steps(TMC2209_Driver *driver, int32_t steps, uint32_t speed_steps_per_second);

void rotate_by_angle(TMC2209_Driver *driver, float angle, uint32_t speed_rpm);

#endif // TMC2209_CONTROL_H
