#include "euler_angles.h"
#include "mpu6500.h"
#include <math.h>

/**
 * @brief 使用陀螺仪数据更新欧拉角
 */
void Euler_angles_update_by_gyro(Euler_angles_t *euler_angles, mpu6500_sensor_data_t *sensor_data, float interval)
{
    euler_angles->pitch += sensor_data->gx * interval;
    euler_angles->roll  -= sensor_data->gy * interval;
    euler_angles->yaw   += sensor_data->gz * interval;
}

/**
 * @brief 使用加速度计数据解算欧拉角
 */
void Euler_angles_update_by_acceler(Euler_angles_t *euler_angles, mpu6500_sensor_data_t *sensor_data)
{
    euler_angles->yaw = 0.0f;
    euler_angles->pitch = atan2f(sensor_data->ay, sensor_data->az) / 3.14159 * 180.0f;
    euler_angles->roll = atan2f(sensor_data->ax, sensor_data->az) / 3.14159 * 180.0f;
}

/**
 * @brief 使用互补滤波解算欧拉角
 */
void complementary_filter(Euler_angles_t *euler_angles, mpu6500_sensor_data_t *sensor_data, float interval)
{
    static Euler_angles_t gyro_ea = {0.0f, 0.0f, 0.0f};
    Euler_angles_t acceler_ea;

    Euler_angles_update_by_gyro(&gyro_ea, sensor_data, interval);
    Euler_angles_update_by_acceler(&acceler_ea, sensor_data);

    euler_angles->yaw = gyro_ea.yaw;
    euler_angles->pitch = 0.95238 * gyro_ea.pitch + (1.0 - 0.95238) * acceler_ea.pitch;
    euler_angles->roll = 0.95238 * gyro_ea.roll + (1.0 - 0.95238) * acceler_ea.roll;
}
