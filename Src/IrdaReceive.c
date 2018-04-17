//
// Created by mmx on 2018/3/28.
//

#include "stm32f1xx_hal.h"
#include <stm32f1xx_hal_tim.h>
#include "IrdaReceive.h"
#include "log.h"
#include "IrdaCarrierMgr.h"

extern TIM_HandleTypeDef htim3;
extern DMA_HandleTypeDef hdma_tim3_ch4_up;
#define RECEIVE_TIM htim3

#define MAX_PULS_CNT 100
static struct _STATE{
    uint32_t msOnStart;
    uint32_t msOn;
    uint32_t tLastNewData;
    uint32_t iLastRptIdx;
    uint16_t tLastFallingVal;
    bool     bFallingValid;
}sRevState = { 0,0,0,0};

static uint16_t sTicks[MAX_PULS_CNT];

static void startdma(){
    TIM_HandleTypeDef *htim = &RECEIVE_TIM;
    /* configure the DMA Burst Mode */
    htim->Instance->DCR = TIM_DMABASE_CCR3 | TIM_DMABURSTLENGTH_2TRANSFERS;

    HAL_DMA_Start(&hdma_tim3_ch4_up, (uint32_t)&htim->Instance->DMAR, (uint32_t)sTicks, MAX_PULS_CNT);

    htim->State = HAL_TIM_STATE_READY;

    /* Enable the TIM DMA Request */
    __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_CC4);
}

static void hwTurnOffTimAndDma(){
    HAL_TIM_IC_Stop(&RECEIVE_TIM, TIM_CHANNEL_3);
    HAL_TIM_IC_Stop(&RECEIVE_TIM, TIM_CHANNEL_4);
    __HAL_TIM_CLEAR_FLAG(&RECEIVE_TIM, (TIM_FLAG_CC3|TIM_FLAG_CC4|TIM_FLAG_CC3OF|TIM_FLAG_CC4OF));
    HAL_DMA_Abort(&hdma_tim3_ch4_up);
}

static void hwStartTimAndDma(){
    startdma();
    HAL_TIM_IC_Start(&RECEIVE_TIM, TIM_CHANNEL_3);
    HAL_TIM_IC_Start(&RECEIVE_TIM, TIM_CHANNEL_4);
}

static void hwRestartTimAndDma(){
    hwTurnOffTimAndDma();
    hwStartTimAndDma();
}

static void hwRestartDma(){
    HAL_DMA_Abort(&hdma_tim3_ch4_up);
    sRevState.iLastRptIdx = 0;
    startdma();
}

void IrdaReceiveTurnOn(uint32_t msOn){
    if (sRevState.msOnStart == 0){
        hwRestartTimAndDma();
        sRevState.msOnStart = NowMs();
        sRevState.msOn = msOn;
        IrdaCarrierEnable(CP_RECEIVE);
        USBRsp("IrdaRecvState: On");
        return;
    }

    uint32_t tpass = NowMs() - sRevState.msOnStart;
    if (tpass > sRevState.msOn){
        sRevState.msOnStart = NowMs();
        sRevState.msOn = msOn;
    } else {
        sRevState.msOnStart = NowMs();
        sRevState.msOn = sRevState.msOn - tpass + msOn;
    }
    USBRsp("IrdaRecvState: On");
}

static void irdaReportResult(uint32_t dmaLeft) {
    if (MAX_PULS_CNT <= dmaLeft){
        return;
    }
    uint32_t validCnt = MAX_PULS_CNT - dmaLeft;
    while (sRevState.iLastRptIdx + 2 <= validCnt){
        uint16_t up = sTicks[sRevState.iLastRptIdx++];
        uint16_t down = sTicks[sRevState.iLastRptIdx++];
        if (sRevState.bFallingValid){
            USBRsp("IrdaSigLow: %u", (uint32_t)(up - sRevState.tLastFallingVal));
        }
        USBRsp("IrdaSigHigh: %u", (uint32_t)(down - up));
        sRevState.bFallingValid = true;
        sRevState.tLastFallingVal = down;
        sRevState.tLastNewData = NowMs();
    }
}


void IrdaReceiveTurnOff(void){
    sRevState.msOnStart = 0;
    IrdaCarrierDisable(CP_RECEIVE);
    hwTurnOffTimAndDma();
    sRevState.tLastNewData = 0;
    USBRsp("IrdaRecvState: Off");
}


void IrdaReceiveCheck(void){
    if (sRevState.msOnStart == 0){
        return;
    }
    uint32_t dmaLeft = __HAL_DMA_GET_COUNTER(&hdma_tim3_ch4_up);
    irdaReportResult(dmaLeft);
    if (dmaLeft ==0){
        //BUFFER FULL, report end
        USBRsp("IrdaSigEndSequence:Buff full");
        sRevState.bFallingValid = false;
        hwRestartDma();
        return;
    }

    if (NowMs() - sRevState.msOnStart > sRevState.msOn ){
        IrdaReceiveTurnOff();
        return;
    }
    if (sRevState.bFallingValid && sRevState.tLastNewData != 0 && NowMs() - sRevState.tLastNewData > 50 ){
        USBRsp("IrdaSigEndSequence");
        sRevState.bFallingValid = false;
    }

    if (dmaLeft < MAX_PULS_CNT/2 && sRevState.tLastNewData != 0 && NowMs() - sRevState.tLastNewData > 50 ){
        hwRestartDma();
        return;
    }
}
