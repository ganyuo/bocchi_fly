#include "mpu6500.h"

void mpu6500_send_data();
void mpu6500_recvice_data();
void mpu6500_init();
void mpu6500_read_register();
void mpu6500_write_register();

/*****************以下接口与平台相关，移植时需要修改**********************/
#include "stm32f4xx_hal.h"
#include "i2c.h"

/**
 * @brief 使用I2C发送数据
 * 
 * @param data 需要发生的数据起始地址
 * @param size 发送数据的长度
 */
static void i2c_send_data(uint8_t *data, uint16_t size)
{
    HAL_I2C_Master_Transmit(&hi2c1, MPU6500_I2C_ADDRESS, data, size, HAL_MAX_DELAY);
}

/**
 * @brief 使用I2C接收数据
 * 
 * @param data 存储接收到数据的起始地址
 * @param size 接收数据的长度
 */
static void i2c_receive_data(uint8_t *data, uint16_t size)
{
    HAL_I2C_Master_Receive(&hi2c1, MPU6500_I2C_ADDRESS, data, size, HAL_MAX_DELAY);
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




/*****************以下接口与平台无关，移植时可无需修改**********************/

/**
 * @brief 向mpu6500的寄存器写入数据
 * 
 * @param reg_addr 寄存器地址
 * @param value 写入的数据
 */
void mpu6500_write_register(uint8_t reg_addr, uint8_t value)
{
    uint8_t data[2] = {reg_addr, value};
    i2c_send_data(data, 2);
}

/**
 * @brief 读取mpu6500寄存器的值
 * 
 * @param reg_addr 寄存器地址
 * @return uint8_t 读取到的值
 */
uint8_t mpu6500_read_register(uint8_t reg_addr)
{
    uint8_t data;
    i2c_send_data(&reg_addr, 1);
    i2c_receive_data(&data, 1);
    return data;
}

/**
 * @brief 初始化mpu6500
 * 
 */
void mpu6500_init()
{
    // 复位
    mpu6500_write_register(MPU6500_RA_PWR_MGMT_1, 0x80);
    delay_ms(100);

    // 唤醒mpu6500
    mpu6500_write_register(MPU6500_RA_PWR_MGMT_1, 0x00);

    // 设置陀螺仪满量程范围
    mpu6500_write_register(MPU6500_GYRO_CFG_REG, 0x18);

    // 设置加速度传感器满量程范围
    mpu6500_write_register(MPU6500_ACCEL_CFG_REG, 0x18);

    // 设置采样率为100Hz, 计算公式：1000 / (99 + 1)
    mpu6500_write_register(MPU6500_SAMPLE_RATE_REG, 99);

    // 设置设置mpu6500的数字低通滤波器频率为98Hz
    mpu6500_write_register(MPU6500_CFG_REG, 0x02);

    // 关闭所有中断
    mpu6500_write_register(MPU6500_INT_EN_REG, 0x00);

    // I2C主模式关闭
    mpu6500_write_register(MPU6500_USER_CTRL_REG, 0x00);

    // 关闭FIFO
    mpu6500_write_register(MPU6500_FIFO_EN_REG, 0x00);

    // INT引脚低电平有效
    mpu6500_write_register(MPU6500_INTBP_CFG_REG, 0x00);

    // 设置CLKSEL，PLL X轴为参考
    mpu6500_write_register(MPU6500_RA_PWR_MGMT_1, 0x01);

    // 设置加速度与陀螺仪都工作
    mpu6500_write_register(MPU6500_PWR_MGMT2_REG, 0x00);
}

/* 进行大小端转换 */
#define bswap16(x) ((uint16_t) ((((x) >> 8) & 0xff) | (((x) & 0xff) << 8)))

/**
 * @brief 读取陀螺仪数据
 * 
 * @param gyro_data 陀螺仪数据存储地址，存储顺序为XYZ
 */
void mpu6500_read_gyroscope(int16_t *gyro_data)
{
    uint8_t reg_addr = MPU6500_GYRO_XOUTH_REG;
    i2c_send_data(&reg_addr, 1);
    i2c_receive_data((uint8_t *)gyro_data, 6);
    gyro_data[0] = bswap16(gyro_data[0]);
    gyro_data[1] = bswap16(gyro_data[1]);
    gyro_data[2] = bswap16(gyro_data[2]);
}

/**
 * @brief 读取加速度数据
 * 
 * @param acc_data 加速度数据存储地址，存储顺序为XYZ
 */
void mpu6500_read_acceleration(int16_t *acc_data)
{
    uint8_t reg_addr = MPU6500_ACCEL_XOUTH_REG;
    i2c_send_data(&reg_addr, 1);
    i2c_receive_data((uint8_t *)acc_data, 6);
    acc_data[0] = bswap16(acc_data[0]);
    acc_data[1] = bswap16(acc_data[1]);
    acc_data[2] = bswap16(acc_data[2]);
}

/**
 * @brief 读取MPU6500上的温度
 * 
 * @return float 温度值单位为℃
 */
float mpu6500_read_temperature()
{
    int16_t reg_value;
    uint8_t reg_addr = MPU6500_TEMP_OUTH_REG;
    i2c_send_data(&reg_addr, 1);
    i2c_receive_data((uint8_t *)&reg_value, 2);
    reg_value = bswap16(reg_value);
    return (float)reg_value / 340.0 + 36.53;
}
