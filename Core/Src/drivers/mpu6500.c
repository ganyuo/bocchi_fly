#include "mpu6500.h"

/*****************************************以下接口与平台相关，移植时需要修改*****************************************/
#include "stm32f4xx_hal.h"
#include "stm32f405xx.h"
#include "spi.h"
#include "gpio.h"

/**
 * @brief 按毫秒延时
 * 
 * @param ms 延时的毫秒数
 */
#ifndef delay_ms
#define delay_ms(ms) HAL_Delay(ms)
#endif

#define MPU6500_SPI hspi2

/**
 * @brief 向mpu6500的寄存器写入数据
 * 
 * @param reg_addr 寄存器地址
 * @param tx_data 需要写入的数据
 * @param size 写入数据的长度
 */
void mpu6500_write_register(uint8_t reg_addr, uint8_t *tx_data, uint16_t size)
{
    reg_addr = (reg_addr & 0x7F); // 写命令：寄存器地址最高位清0
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&MPU6500_SPI, &reg_addr, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&MPU6500_SPI, tx_data, size, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    delay_ms(1);
}

/**
 * @brief 读取mpu6500寄存器的值
 * 
 * @param reg_addr 寄存器地址
 * @param rx_data 存储读取到数据
 * @param size 读取数据的长度
 */
void mpu6500_read_register(uint8_t reg_addr, uint8_t *rx_data,  uint16_t size)
{
    reg_addr = (reg_addr | 0x80); // 读命令：寄存器地址最高位置
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&MPU6500_SPI, &reg_addr, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&MPU6500_SPI, rx_data, size, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
}

/***************************************************end***************************************************/



/**
 * @brief 初始化mpu6500
 * 
 * @return uint8_t 初始化成功返回0
 */
uint8_t mpu6500_init()
{
    uint8_t temp_data = 0x00;
    // 复位
    temp_data = 0x80;
    mpu6500_write_register(MPU_RA_PWR_MGMT_1, &temp_data, 1);
    delay_ms(100);

    // 唤醒mpu6500
    temp_data = 0x00;
    mpu6500_write_register(MPU_RA_PWR_MGMT_1, &temp_data, 1);

    // 验证是否有mpu6500
    mpu6500_read_register(MPU_RA_WHO_AM_I, &temp_data, 1);
    if (temp_data != MPU6500_WHO_AM_I_CONST)
    {
        return 1;
    }

    // 设置时钟源，使用PLL作为时钟源
    temp_data = 0x01;
    mpu6500_write_register(MPU_RA_PWR_MGMT_1, &temp_data, 1);

    // 设置陀螺仪量程范围
    // 0 - 250dps
    // 1 - 500dps
    // 2 - 1000dps
    // 3 - 2000dps
    temp_data = 3 << 3;
    mpu6500_write_register(MPU_RA_GYRO_CONFIG, &temp_data, 1);

    // 设置加速度计量程范围
    // 0 - 2g
    // 1 - 4g
    // 2 - 8g
    // 3 - 16g
    temp_data = 0 << 3;
    mpu6500_write_register(MPU_RA_ACCEL_CONFIG, &temp_data, 1);

    // 设置陀螺仪采样率为1kHz, 计算公式：8000 / (MPU_RA_SMPLRT_DIV + 1)
    temp_data = 7;
    mpu6500_write_register(MPU_RA_SMPLRT_DIV, &temp_data, 1);

    // 设置加速度与陀螺仪都工作
    temp_data = 0x00;
    mpu6500_write_register(MPU_RA_PWR_MGMT_2, &temp_data, 1);
    delay_ms(10);
    return 0;
}

/**
 * @brief 读取mpu6500的测量结果
 * 
 * @param sensor_data 测量的结果
 */
void mpu6500_read_sensor_data(mpu6500_sensor_data_t *sensor_data)
{
    uint8_t rx_data[14];
    mpu6500_read_register(MPU_RA_ACCEL_XOUT_H, rx_data, 14);

    int16_t ax = ((int16_t)rx_data[0] << 8) | rx_data[1];
    int16_t ay = ((int16_t)rx_data[2] << 8) | rx_data[3];
    int16_t az = ((int16_t)rx_data[4] << 8) | rx_data[5];
    int16_t temperature = ((int16_t)rx_data[6] << 8) | rx_data[7];
    int16_t gx = ((int16_t)rx_data[8] << 8) | rx_data[9];
    int16_t gy = ((int16_t)rx_data[10] << 8) | rx_data[11];
    int16_t gz = ((int16_t)rx_data[12] << 8) | rx_data[13];

    sensor_data->ax = (float)ax * 6.1035e-5f;
    sensor_data->ay = (float)ay * 6.1035e-5f;
    sensor_data->az = (float)az * 6.1035e-5f;
    sensor_data->gx = (float)gx * 6.1035e-2f;
    sensor_data->gy = (float)gy * 6.1035e-2f;
    sensor_data->gz = (float)gz * 6.1035e-2f;
    sensor_data->temperature = (float)temperature / 333.87f + 21.0f;
}
