
#ifndef __MPU6050_DMP_H__
#define __MPU6050_DMP_H__

#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"
#include <stdint.h>

uint8_t mpu_dmp_init(void);
uint8_t mpu_dmp_get_data(float *pitch, float *roll, float *yaw);

#endif /* __MPU6050_DMP_H__ */

