//
// Created by mmx on 2018/3/28.
//

#ifndef MMXSTM32F103_IRDASEND_H
#define MMXSTM32F103_IRDASEND_H

#include <stdbool.h>
#include "stm32f1xx_hal.h"

#define MAX_SEND_PULSE   40

typedef struct IrdaPulseTickDef{
    uint16_t period;
    uint16_t on;
}IrdaPulseTickDef_t;

bool IrdaSendPulse(IrdaPulseTickDef_t *pulse, uint32_t cnt);
bool IrdaIsSendBusy(void);

#endif //MMXSTM32F103_IRDASEND_H
