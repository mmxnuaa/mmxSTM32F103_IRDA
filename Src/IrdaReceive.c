//
// Created by mmx on 2018/3/28.
//

#include <stm32f1xx_hal_tim.h>
#include "IrdaReceive.h"
#include "log.h"

extern TIM_HandleTypeDef htim3;
extern DMA_HandleTypeDef hdma_tim3_ch4_up;
#define RECEIVE_TIM htim3

static struct _STATE{
    uint32_t msOnStart;
    uint32_t msOn;
    uint32_t tLastCheck;
    uint32_t tLastLeft;
    uint32_t iLastRptIdx;
    uint16_t tLastFallingVal;
    bool     bFallingValid;
}sRevState = {0,0,0,0};

#define MAX_TICK_CNT 100
static uint16_t sTicks[MAX_TICK_CNT];

static void startdma(){
    TIM_HandleTypeDef *htim = &RECEIVE_TIM;
    /* configure the DMA Burst Mode */
    htim->Instance->DCR = TIM_DMABASE_CCR3 | TIM_DMABURSTLENGTH_2TRANSFERS;

    HAL_DMA_Start(&hdma_tim3_ch4_up, (uint32_t)&htim->Instance->DMAR, (uint32_t)sTicks, MAX_TICK_CNT);

    htim->State = HAL_TIM_STATE_READY;

    /* Enable the TIM DMA Request */
    __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_CC4);
}

void IrdaReceiveTurnOn(uint32_t msOn){
    if (sRevState.msOnStart == 0){
        HAL_TIM_IC_Stop(&RECEIVE_TIM, TIM_CHANNEL_3);
        HAL_TIM_IC_Stop(&RECEIVE_TIM, TIM_CHANNEL_4);
        __HAL_TIM_CLEAR_FLAG(&RECEIVE_TIM, (TIM_FLAG_CC3|TIM_FLAG_CC4|TIM_FLAG_CC3OF|TIM_FLAG_CC4OF));
        startdma();
        HAL_TIM_IC_Start(&RECEIVE_TIM, TIM_CHANNEL_3);
        HAL_TIM_IC_Start(&RECEIVE_TIM, TIM_CHANNEL_4);
        sRevState.msOnStart = HAL_GetTick();
        sRevState.msOn = msOn;
        USBRsp("IrdaRecvState: On");
        return;
    }

    uint32_t tpass = HAL_GetTick() - sRevState.msOnStart;
    if (tpass > sRevState.msOn){
        sRevState.msOnStart = HAL_GetTick();
        sRevState.msOn = msOn;
    } else {
        sRevState.msOnStart = HAL_GetTick();
        sRevState.msOn = sRevState.msOn - tpass + msOn;
    }
    USBRsp("IrdaRecvState: On");
}

static void irdaReportResult(uint32_t dmaLeft) {
    if (MAX_TICK_CNT <= dmaLeft){
        return;
    }
    uint32_t validCnt = MAX_TICK_CNT - dmaLeft;
    while (sRevState.iLastRptIdx + 2 <= validCnt){
        uint16_t up = sTicks[sRevState.iLastRptIdx++];
        uint16_t down = sTicks[sRevState.iLastRptIdx++];
        if (sRevState.bFallingValid){
            USBRsp("IrdaSigLow: %d", up - sRevState.tLastFallingVal);
        }
        USBRsp("IrdaSigHigh: %d", down - up);
        sRevState.bFallingValid = true;
        sRevState.tLastFallingVal = down;
    }
}

static void restartDma(){

}

void IrdaReceiveTurnOff(void){
    sRevState.msOnStart = 0;
    HAL_TIM_IC_Stop(&RECEIVE_TIM, TIM_CHANNEL_3);
    HAL_TIM_IC_Stop(&RECEIVE_TIM, TIM_CHANNEL_4);
    __HAL_TIM_CLEAR_FLAG(&RECEIVE_TIM, (TIM_FLAG_CC3|TIM_FLAG_CC4|TIM_FLAG_CC3OF|TIM_FLAG_CC4OF));
    HAL_DMA_Abort(&hdma_tim3_ch4_up);
    sRevState.tLastCheck = 0;
    USBRsp("IrdaRecvState: Off");
}


void IrdaReceiveCheck(void){
    if (sRevState.msOnStart == 0){
        return;
    }
    uint32_t dmaLeft = __HAL_DMA_GET_COUNTER(&hdma_tim3_ch4_up);
    irdaReportResult(dmaLeft);
    if (dmaLeft ==0){
        restartDma();
        return;
    }

    if (HAL_GetTick() - sRevState.msOnStart > sRevState.msOn ){
        IrdaReceiveTurnOff();
        return;
    }
    if (sRevState.bFallingValid && sRevState.tLastCheck != 0 && sRevState.tLastLeft == dmaLeft && HAL_GetTick() - sRevState.tLastCheck > 50 ){
        USBRsp("IrdaSigEndSequence");
        sRevState.bFallingValid = false;
    }

    if (dmaLeft < MAX_TICK_CNT/2 && sRevState.tLastCheck != 0 && sRevState.tLastLeft == dmaLeft && HAL_GetTick() - sRevState.tLastCheck > 50 ){
        restartDma();
        return;
    }
    sRevState.tLastCheck = HAL_GetTick();
    sRevState.tLastLeft = dmaLeft;
}
