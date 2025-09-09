#include <stdio.h>
#include <string.h>
#include "mpu6500_test.h"
#include "stm32f4xx.h"
#include "usart.h"
#include "process/process.h"
#include "drivers/mpu6500.h"
#include "drivers/euler_angles.h"

void mpu6500_sensor_data_test()
{
    mpu6500_sensor_data_t mpu6500_sensor_data;

    mpu6500_init();

    while(1)
    {
        // PERIODIC(1000)
        mpu6500_read_sensor_data(&mpu6500_sensor_data);

        char uart_buff[256];
        sprintf(uart_buff, "%f, %f, %f, %f, %f, %f, %f\n", 
            mpu6500_sensor_data.ax,
            mpu6500_sensor_data.ay,
            mpu6500_sensor_data.az,
            mpu6500_sensor_data.temperature,
            mpu6500_sensor_data.gx,
            mpu6500_sensor_data.gy,
            mpu6500_sensor_data.gz);

        HAL_UART_Transmit(&huart6, (uint8_t *)uart_buff, strlen(uart_buff), HAL_MAX_DELAY);
        HAL_Delay(1000);
    }
}

void Euler_angles_test()
{
    mpu6500_sensor_data_t mpu6500_sensor_data;
    Euler_angles_t euler_angles = {0, 0, 0};

    mpu6500_init();

    while(1)
    {
        // PERIODIC(50)
        mpu6500_read_sensor_data(&mpu6500_sensor_data);
        // Euler_angles_update_by_gyro(&euler_angles, &mpu6500_sensor_data, 0.05);
        // Euler_angles_update_by_acceler(&euler_angles, &mpu6500_sensor_data);
        complementary_filter(&euler_angles, &mpu6500_sensor_data, 0.05);

        char uart_buff[256];
        sprintf(uart_buff, "%f, %f, %f\n", 
            euler_angles.yaw,
            euler_angles.pitch,
            euler_angles.roll);

        HAL_UART_Transmit(&huart6, (uint8_t *)uart_buff, strlen(uart_buff), HAL_MAX_DELAY);
        HAL_Delay(45);
    }
}

