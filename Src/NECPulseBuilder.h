//
// Created by mmx on 2018/4/18.
//

#ifndef MMXSTM32F103_NECPULSEBUILDER_H
#define MMXSTM32F103_NECPULSEBUILDER_H

#include "IrdaSend.h"

uint32_t NECEncode(const char *param/* "addr, command" */, IrdaPulseTickDef_t *pulse /*OUT*/, uint32_t maxCnt);

#endif //MMXSTM32F103_NECPULSEBUILDER_H
