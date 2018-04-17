//
// Created by mmx on 2018/3/28.
//

#include <stm32f1xx_hal_tim.h>
#include "IrdaCarrierMgr.h"

extern TIM_HandleTypeDef htim2;

static uint32_t sActiveTarget = 0;

void IrdaCarrierEnable(CarrierPurpose target)
{
    if (sActiveTarget == 0){
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    }
    sActiveTarget |= target;
}

void IrdaCarrierDisable(CarrierPurpose target)
{
    sActiveTarget &= ~target;
    if (sActiveTarget == 0){
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
    }
}