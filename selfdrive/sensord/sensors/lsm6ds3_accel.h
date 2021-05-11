#pragma once

#include "selfdrive/sensord/sensors/i2c_sensor.h"

// Address of the chip on the bus
#define LSM6DS3_ACCEL_I2C_ADDR       0x6A

// Registers of the chip
#define LSM6DS3_ACCEL_I2C_REG_ID        0x0F
#define LSM6DS3_ACCEL_I2C_REG_CTRL1_XL  0x10
#define LSM6DS3_ACCEL_I2C_REG_OUTX_L_XL 0x28

// Constants
#define LSM6DS3_ACCEL_CHIP_ID        0x69
#define LSM6DS3_ACCEL_ODR_104HZ      (0b0100 << 4)


class LSM6DS3_Accel : public I2CSensor {
  uint8_t get_device_address() {return LSM6DS3_ACCEL_I2C_ADDR;}
public:
  LSM6DS3_Accel(I2CBus *bus);
  int init();
  void get_event(cereal::SensorEventData::Builder &event);
};
