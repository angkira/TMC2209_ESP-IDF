#ifndef TMC2209_REGISTER_H
#define TMC2209_REGISTER_H

#include "driver.dto.h"

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
#define REG_VMAX 0x20

esp_err_t readRegister(TMC2209_Driver *driver, uint8_t address, uint32_t *data);
esp_err_t writeRegister(TMC2209_Driver *driver, uint8_t address, uint32_t data);

#endif // TMC2209_REGISTER_H
