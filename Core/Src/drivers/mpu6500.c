#include "mpu6500.h"
#include "stm32f4xx_hal.h"
#include "spi.h"


/**************************硬件相关接口***************************/
#define MPU6500_SPI &hspi2

void mpu6500_send_data(uint8_t *data, uint16_t size)
{
    HAL_SPI_Transmit(MPU6500_SPI, data, size, HAL_MAX_DELAY);
}
void mpu6500_recvice_data(uint8_t *data,  uint16_t size)
{
    HAL_SPI_Receive(MPU6500_SPI, data, size, HAL_MAX_DELAY);
}

#ifndef delay_ms
#define delay_ms(ms) HAL_Delay(ms)
#endif

/******************************************************************/

struct
{
    uint16_t gx, gy, gz;
    uint16_t temp;
    uint16_t ax, ay, az;
    uint16_t reserve;
} mpu6500_sensor_data;

void mpu6500_read_register(uint8_t address, uint8_t *data, uint16_t size)
{
    mpu6500_send_data(&address, 1);
    mpu6500_recvice_data(data, size);
}
void mpu6500_write_register(uint8_t address, uint8_t *data, uint16_t size)
{
    uint8_t temp_data[128];
    temp_data[0] = address;
    memcpy(temp_data + 1, data, size);
    mpu6500_send_data(temp_data, size + 1);
}

void mpu6500_init()
{
    mpu6500_write_register(MPU_RA_PWR_MGMT_1, 0x80, 1);
    delay_ms(100);
    mpu6500_write_register(MPU_RA_PWR_MGMT_1, 0x00, 1);
}

void mpu6500_upate_sensor_data()
{

}

