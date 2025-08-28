#include "mpu6500.h"

/*****************以下接口与平台相关，移植时需要修改**********************/
#include "stm32f4xx_hal.h"
#include "stm32f405xx.h"
#include "spi.h"
#include "gpio.h"

#define MPU6500_SPI &hspi2

/**
 * @brief 向mpu6500写入数据
 * 
 * @param tx_data 需要写入的数据
 * @param size 写入数据的长度
 */
void mpu6500_write_data(uint8_t *tx_data, uint16_t size)
{
    HAL_SPI_Transmit(MPU6500_SPI, tx_data, size, HAL_MAX_DELAY);
}

/**
 * @brief 从mpu6500读取数据
 * 
 * @param rx_data 存储读取到数据
 * @param size 读取数据的长度
 */
void mpu6500_read_data(const uint8_t *rx_data,  uint16_t size)
{
    HAL_SPI_Receive(MPU6500_SPI, rx_data, size, HAL_MAX_DELAY);
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

/**
 * @brief 读取mpu6500一个寄存器的值
 * 
 * @param reg_addr 寄存器地址
 * @return uint8_t 读取到的值
 */
uint8_t mpu6500_read_register(uint8_t reg_addr)
{
    reg_addr |= 0x80; // 读命令：寄存器地址最高位置1
    uint8_t rx_data;
    mpu6500_write_data(&reg_addr, 1);
    mpu6500_read_data(&rx_data, 1);
    return rx_data;
}

/**
 * @brief 向mpu6500的一个寄存器写入数据
 * 
 * @param reg_addr 寄存器地址
 * @param value 写入的数据
 */
void mpu6500_write_register(uint8_t reg_addr, uint8_t value)
{
    uint8_t tx_data[2] = {reg_addr & 0x7F, value}; // 写命令：寄存器地址最高位清0
    mpu6500_write_data(tx_data, 2);
}

/**
 * @brief 初始化mpu6500
 * 
 * @return uint8_t 初始化成功返回0
 */
uint8_t mpu6500_init()
{
    delay_ms(100);
    // 复位
    mpu6500_write_register(MPU_RA_PWR_MGMT_1, 0x80);
    delay_ms(100);

    // 唤醒mpu6500
    mpu6500_write_register(MPU_RA_PWR_MGMT_1, 0x00);

    // 验证是否有mpu6500
    if (mpu6500_read_register(MPU_RA_WHO_AM_I) != MPU6500_WHO_AM_I_CONST)
    {
        return 1;
    }

    // 设置时钟源，使用PLL作为时钟源
    mpu6500_write_register(MPU_RA_PWR_MGMT_1, 0x01);

    // 设置陀螺仪量程范围
    // 0 - 250dps
    // 1 - 500dps
    // 2 - 1000dps
    // 3 - 2000dps
    mpu6500_write_register(MPU_RA_GYRO_CONFIG, 3 << 3);

    // 设置加速度计量程范围
    // 0 - 2g
    // 1 - 4g
    // 2 - 8g
    // 3 - 16g
    mpu6500_write_register(MPU_RA_ACCEL_CONFIG, 0 << 3);

    // 设置陀螺仪采样率为1kHz, 计算公式：8000 / (MPU_RA_SMPLRT_DIV + 1)
    mpu6500_write_register(MPU_RA_SMPLRT_DIV, 7);

    // 设置加速度与陀螺仪都工作
    mpu6500_write_register(MPU_RA_PWR_MGMT_2, 0x00);
    delay_ms(50);
}

/**
 * @brief 读取mpu6500的测量结果
 * 
 * @param sensor_data 测量的结果
 */
void mpu6500_read_sensor_data(mpu6500_sensor_data_t *sensor_data)
{
    uint8_t reg_addr = MPU_RA_ACCEL_XOUT_H | 0x80;
    uint8_t rx_data[14];
    mpu6500_write_data(reg_addr, 1);
    mpu6500_read_data(rx_data, sizeof(rx_data));

    int16_t ax, ay, az;
    int16_t temp;
    int16_t gx, gy, gz;

    ax = ((int16_t)rx_data[0] << 8) | rx_data[1];
    ay = ((int16_t)rx_data[2] << 8) | rx_data[3];
    az = ((int16_t)rx_data[4] << 8) | rx_data[5];
    temp = ((int16_t)rx_data[6] << 8) | rx_data[7];   
    gx = ((int16_t)rx_data[8] << 8) | rx_data[9];
    gy = ((int16_t)rx_data[10] << 8) | rx_data[11];
    gz = ((int16_t)rx_data[12] << 8) | rx_data[13];

    sensor_data->ax = ax *6.1035e-5f;
    sensor_data->ay = ay *6.1035e-5f;
    sensor_data->az = az *6.1035e-5f;
    sensor_data->temperature = temp / 333.87f + 21.0f;
    sensor_data->gx = gx *6.1035e-2f;
    sensor_data->gy = gy *6.1035e-2f;
    sensor_data->gz = gz *6.1035e-2f;
}
