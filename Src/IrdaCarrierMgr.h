//
// Created by mmx on 2018/3/28.
//

#ifndef MMXSTM32F103_IRDACARRIERMGR_H
#define MMXSTM32F103_IRDACARRIERMGR_H

//BIT MASK VALUE
typedef enum {
    CP_SEND = 0x1,
    CP_RECEIVE = 0x2,
}CarrierPurpose;

void IrdaCarrierEnable(CarrierPurpose target);
void IrdaCarrierDisable(CarrierPurpose target);

#define NowMs() (HAL_GetTick()|0x1)

#endif //MMXSTM32F103_IRDACARRIERMGR_H
