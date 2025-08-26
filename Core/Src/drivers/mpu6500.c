#include "mpu6500.h"

/*****************以下接口与平台相关，移植时需要修改**********************/
#include "stm32f4xx_hal.h"
#include "spi.h"

#define MPU6500_SPI &hspi2

/**
 * @brief 向mpu6500发送数据
 * 
 * @param data 需要发生的数据起始地址
 * @param size 发送数据的长度
 */
void mpu6500_send_data(uint8_t *data, uint16_t size)
{
    HAL_SPI_Transmit(MPU6500_SPI, data, size, HAL_MAX_DELAY);
}

/**
 * @brief 从mpu6500接收数据
 * 
 * @param data 存储接收到数据的起始地址
 * @param size 接收数据的长度
 */
void mpu6500_recvice_data(uint8_t *data,  uint16_t size)
{
    HAL_SPI_Receive(MPU6500_SPI, data, size, HAL_MAX_DELAY);
}

/**
 * @brief 按毫秒延时
 * 
 * @param ms 延时的毫秒数
 */
#ifndef delay_ms
#define delay_ms(ms) HAL_Delay(ms)
#endif

/************************************end************************************/

struct mpu6500_sensor_data_t
{
    uint16_t ax, ay, az;
    uint16_t temp;
    uint16_t gx, gy, gz;
} mpu6500_sensor_data;

/**
 * @brief 读取mpu6500一个寄存器的值
 * 
 * @param reg_addr 寄存器地址
 * @return uint8_t 读取到的值
 */
uint8_t mpu6500_read_register(uint8_t reg_addr)
{
    uint8_t value;
    mpu6500_send_data(&reg_addr, 1);
    mpu6500_recvice_data(&value, 1);
    return value;
}

/**
 * @brief 向mpu6500的一个寄存器写入数据
 * 
 * @param reg_addr 寄存器地址
 * @param value 写入的数据
 */
void mpu6500_write_register(uint8_t reg_addr, uint8_t value)
{
    uint8_t temp_data[2] = {reg_addr, value};
    mpu6500_send_data(temp_data, 2);
}

/**
 * @brief 初始化mpu6500
 * 
 * @return uint8_t 初始化成功返回0
 */
uint8_t mpu6500_init()
{
    // 复位
    mpu6500_write_register(MPU_RA_PWR_MGMT_1, 0x80);
    delay_ms(100);

    // 设置陀螺仪满量程范围
    mpu6500_write_register(MPU_RA_GYRO_CONFIG, 0x18);

    // 设置加速度传感器满量程范围
    mpu6500_write_register(MPU_RA_ACCEL_CONFIG, 0x18);

    // 设置采样率为8kHz, 计算公式：8000 / (MPU_RA_SMPLRT_DIV + 1)
    mpu6500_write_register(MPU_RA_SMPLRT_DIV, 0);

    // // 设置CLKSEL，PLL X轴为参考
    // mpu6500_write_register(MPU_RA_PWR_MGMT_1, 0x01);

    // 设置加速度与陀螺仪都工作
    mpu6500_write_register(MPU_RA_PWR_MGMT_2, 0x00);

    // 唤醒mpu6500
    mpu6500_write_register(MPU_RA_PWR_MGMT_1, 0x00);

    // 验证是否有mpu6500
    if (mpu6500_read_register(MPU_RA_WHO_AM_I) != MPU6500_WHO_AM_I_CONST)
    {
        return 1;
    }

}

void mpu6500_upate_sensor_data()
{

}

