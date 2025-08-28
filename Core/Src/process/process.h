
#ifndef __PROCESS_H__
#define __PROCESS_H__

#include "stm32f4xx.h"
#include "stdint.h"

#define PERIODIC(ms) \
                    static uint32_t next = 0; \
                    if (HAL_GetTick() < next) \
                    {                         \
                        return ;              \
                    }                         \
                    next += ms;               \

#endif /* __PROCESS_H__ */
