#include "imu.h"

MPU9250 mpu;

uint8_t imuInitialize()
{
    if (!mpu.setup(0x68))
        return 1;
}

void getImuGyro(float *roll, float *pitch, float *yaw)
{
    if (mpu.update())
    {
        static uint32_t prev_ms = millis();

        *roll = mpu.getRoll();
        *pitch = mpu.getPitch();
        *yaw = mpu.getYaw();
    }
}
