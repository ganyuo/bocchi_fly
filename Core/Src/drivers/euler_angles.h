
#ifndef __EULER_ANGLES_H__
#define __EULER_ANGLES_H__

#include "drivers/mpu6500.h"

struct Euler_angles_s
{
    float yaw, pitch, roll;
};

/* 欧拉角 */
typedef struct Euler_angles_s Euler_angles_t;

void Euler_angles_update_by_gyro(Euler_angles_t *euler_angles, mpu6500_sensor_data_t *sensor_data, float intveral);

#endif /* __EULER_ANGLES_H__ */
