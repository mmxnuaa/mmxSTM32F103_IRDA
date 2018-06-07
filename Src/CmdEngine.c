//
// Created by mmx on 2018/4/17.
//

#include <stdint.h>
#include <usbd_cdc_if.h>
#include "CmdEngine.h"
#include "log.h"
#include "IrdaReceive.h"
#include "NECPulseBuilder.h"

#define CMD_BUF_SIZE 256

static bool sbSending = false;
static void procCmd(const char *cmd){
    LogI("Got cmd: %s", cmd);
    if (strcmp(cmd, "IrdaReceiveStart") == 0){
        IrdaReceiveTurnOn(50000);
    }else if (strcmp(cmd, "IrdaReceiveStop") == 0){
        IrdaReceiveTurnOff();
    } else if (strncmp(cmd, "NEC:", 4)==0){
        IrdaPulseTickDef_t pulse[MAX_SEND_PULSE];
        uint32_t cnt = NECEncode(cmd+4, pulse, MAX_SEND_PULSE);
        if (cnt == 0){
            USBRsp("SendFail: NEC encode fail");
            return;
        }

        if (IrdaIsSendBusy()){
            USBRsp("SendFail: Send busy");
            return;
        }

        if (!IrdaSendPulse(pulse, cnt)){
            USBRsp("SendFail: Send pulses fail");
            return;
        }
        sbSending =  true;
    } else if (strcmp(cmd, "HowAreYou") == 0){
        USBRsp("Fine");
    }
}

void CmdEngineCheck(void) {
    static uint32_t len = 0;
    static char cmdBuf[CMD_BUF_SIZE];
    if (!UsbRxPending && UsbRxValidCnt > 0){
        const char *pRx = (const char *) GetUsbRxBuff();
        while (UsbRxValidCnt--){
            char ch = *pRx++;
            if ( ch == '\0' || ch ==  '\n' || ch == '\r' ){
                if (len > 0){
                    cmdBuf[len] = '\0';
                    procCmd(cmdBuf);
                    len = 0;
                }
                continue;
            }
            cmdBuf[len++] = ch;
            if (len >= CMD_BUF_SIZE){
                LogE("cmd too long!");
                len = 0;
            }
        }
    }
    if (sbSending && !IrdaIsSendBusy()){
        sbSending = false;
        USBRsp("SendFinish");
    }

    UsbReceiveNewBlock();
}
