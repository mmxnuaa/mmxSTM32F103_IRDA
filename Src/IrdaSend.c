//
// Created by mmx on 2018/3/28.
//


#include "stm32f1xx_hal.h"

#include <stm32f1xx_ll_tim.h>
#include <stm32f1xx_hal_tim.h>
#include "string.h"
#include "log.h"
#include "IrdaCarrierMgr.h"
#include "IrdaSend.h"

//*********************************************
// Use TIM4 CH1 and DMA to generate Irda remote signal
#define IRDA_SIGNAL_TIM   htim4

extern TIM_HandleTypeDef htim4;
extern DMA_HandleTypeDef hdma_tim4_ch1;
static volatile bool sbSendBusy = false;

#pragma pack(2)
typedef struct _RAW_TICK_DMA_PACK_t{
    uint16_t arr;   //arr register value
    uint16_t dummy; //gap between arr and ccr1, just put dummy value
    uint16_t ccr1;   //arr register value
}RAW_TICK_DMA_PACK_t;
static RAW_TICK_DMA_PACK_t sDMAPulseRawData[MAX_SEND_PULSE+1]; //ONE extra for DMA end pulse
#pragma pack()

static void TIM_DMASendIrdaCplt(DMA_HandleTypeDef *hdma)
{
  TIM_HandleTypeDef* htim = ( TIM_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
  htim->State= HAL_TIM_STATE_READY;
  HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);
  IrdaCarrierDisable(CP_SEND);
  sbSendBusy = false;
}

static bool irdaSendRawTick(RAW_TICK_DMA_PACK_t *pData, uint32_t cnt){
  if (cnt < 2){
    LogE("Invalid irda raw tick to send: %d", cnt);
    return false;
  }

  sbSendBusy = true;
//  LogI("Send irda raw tick: len = %d", cnt);
  TIM_HandleTypeDef *htim = &IRDA_SIGNAL_TIM;

  HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);
  HAL_DMA_Abort_IT(&hdma_tim4_ch1);
  __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC1);
  __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_UPDATE);

  IrdaCarrierEnable(CP_SEND);

  /* configure the DMA Burst Mode */
  htim->Instance->DCR = TIM_DMABASE_ARR | TIM_DMABURSTLENGTH_3TRANSFERS;

  //Write first pulse param
  __HAL_TIM_SET_AUTORELOAD(htim, pData[0].arr);
  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, pData[0].ccr1);
  HAL_TIM_GenerateEvent(htim, TIM_EVENTSOURCE_UPDATE);

  hdma_tim4_ch1.XferCpltCallback = TIM_DMASendIrdaCplt;
  HAL_DMA_Start_IT(&hdma_tim4_ch1,
                   (uint32_t)&pData[1],
                   (uint32_t)&htim->Instance->DMAR,
                   (cnt-1)*(sizeof(RAW_TICK_DMA_PACK_t)/sizeof(uint16_t)) );

  htim->State = HAL_TIM_STATE_READY;

  /* Enable the TIM Capture/Compare 1 DMA request */
  __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_CC1);

  HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
  return true;
}

bool IrdaSendPulse(IrdaPulseTickDef_t *pulse, uint32_t cnt){
  if (cnt > MAX_SEND_PULSE){
    LogE("No support so many pulse: %d", cnt);
    return false;
  }
  if (IrdaIsSendBusy()){
    LogE("Send busy");
    return false;
  }

  uint32_t pulse_cnt = 0;
  RAW_TICK_DMA_PACK_t *pRaw = sDMAPulseRawData;
  for (int i = 0; i <cnt ; ++i) {
    pRaw->arr = pulse->period;
    pRaw->ccr1 = pulse->on;
    if (pRaw->ccr1 == 0){
      pRaw->ccr1 = 1;
    }
    if (pRaw->arr <= pRaw->ccr1){
      pRaw->arr = (uint16_t) (pRaw->ccr1 + 1);
    }
    pRaw++;
    pulse_cnt++;
    pulse++;
  }
  pRaw->arr = 10;
  pRaw->ccr1 = 0;
  pulse_cnt++;
  return irdaSendRawTick(sDMAPulseRawData, pulse_cnt);
}

bool IrdaIsSendBusy() {
  return sbSendBusy;
}
