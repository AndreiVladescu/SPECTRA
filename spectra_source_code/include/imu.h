#include <Arduino.h>

#include "const_data.h"
#include <MPU9250.h>

#include <Wire.h>

uint8_t imuInitialize();

void getImuGyro(float *roll, float *pitch, float *yaw);