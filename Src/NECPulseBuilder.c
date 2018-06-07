//
// Created by mmx on 2018/4/18.
//

#include "NECPulseBuilder.h"
#include "log.h"

#define CARRIER_FREQ  38 //Unit: Khz
#define US2Tick(us) ((us)*CARRIER_FREQ/1000)

uint32_t NECEncode(const char *param, IrdaPulseTickDef_t *pulse, uint32_t maxCnt) {
    uint32_t address, cmd;
    if (maxCnt <= 34){ //start + 32 + end
        LogE("Nec need more space for pulse");
        return 0;
    }
    if (sscanf(param, "%u,%u", &address, &cmd) != 2){
        LogE("Nec extract param fail: %s", param);
        return 0;
    }
    if (address > 0xffff || cmd > 0xff){
        LogE("Nec invalid param value: %s", param);
        return 0;
    }
    if (address > 0xff){
        LogI("Nec use extend format: %s", param);
    } else {
        address = (((~address)&0xff)<<8) | (((address)&0xff));
    }
    uint32_t bits = (((~cmd)&0xff)<<24)
                    |(((cmd)&0xff)<<16)
                    | address;

    pulse[0].period = US2Tick(9000+4500);
    pulse[0].on = US2Tick(9000);

    for (int i = 1; i <33 ; ++i) {
        pulse[i].on = US2Tick(560);
        if (bits & 0x1){
            pulse[i].period = US2Tick(2250);
        } else {
            pulse[i].period = US2Tick(1120);
        }
        bits >>= 1;
    }
    pulse[33].period = US2Tick(1120);
    pulse[33].on = US2Tick(560);

    return 34;
}
