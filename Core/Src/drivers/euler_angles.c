
#include "euler_angles.h"
#include "mpu6500.h"

/**
 * @brief 使用陀螺仪数据更新欧拉角
 */
void Euler_angles_update_by_gyro(Euler_angles_t *euler_angles, mpu6500_sensor_data_t *sensor_data, float intveral)
{
    euler_angles->pitch += sensor_data->gx * intveral;
    euler_angles->roll  -= sensor_data->gy * intveral;
    euler_angles->yaw   += sensor_data->gz * intveral;
}
