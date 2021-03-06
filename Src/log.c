
#include "log.h"



#ifdef MMX_LOG_EN
#include "stm32f1xx_hal.h"
#include <stm32f1xx_hal_uart.h>
#include <usbd_cdc_if.h>

extern UART_HandleTypeDef huart1;

#define LOG_TMP_BUF_SIZE (128)
//static char logBuf[ LOG_TMP_BUF_SIZE + 5 ] ="abcd";
void mmxLog(char type, const char *format, ...){
    char logBuf[ LOG_TMP_BUF_SIZE + 5 ] ="abcd";
    va_list list;
    va_start(list, format);
    int len = vsnprintf(logBuf + 5, LOG_TMP_BUF_SIZE, format, list);
    va_end(list);
    if (len>0){
        logBuf[0] = 'a';
        logBuf[1] = 'b';
        logBuf[2] = 'c';
        logBuf[3] = 'd';
        logBuf[4] = type;
        len += 5;
        if (len > LOG_TMP_BUF_SIZE - 6) len = LOG_TMP_BUF_SIZE - 6;
        memcpy(logBuf+len, "ABCDE\0", 6);
//        kfifo_put(&slogfifo, (uint8_t *) logBuf, (uint32_t) (len + 5));
        HAL_UART_Transmit(&huart1, (uint8_t *)logBuf, (uint32_t) (len + 5), 2000);
    }
}

void LogHex(const char *head, uint8_t *pData, uint16_t len, const char *tail){
    char logBuf[ LOG_TMP_BUF_SIZE + 5 ] ="abcd";
    int lh = strlen(head);
    int lt = strlen(tail);
    if (len + lh + lt > LOG_TMP_BUF_SIZE) {
        LogE(" LOG HEX overflow: %s: len = %d : %s", head, len, tail);
        return;
    }

    memcpy(logBuf, head, lh);
    memcpy(logBuf+lh, pData, len);
    memcpy(logBuf+lh+len, tail, lt);

    HAL_UART_Transmit(&huart1, (uint8_t *)logBuf, (uint32_t) (len + lh + lt), 2000);
}

void LogInit(void){

}

void usbCDCRsp(const char *format, ...){
    char logBuf[ LOG_TMP_BUF_SIZE + 5 ] ="abcd";
    va_list list;
    va_start(list, format);
    int len = vsnprintf(logBuf + 5, LOG_TMP_BUF_SIZE, format, list);
    va_end(list);
    if (len>0){
        logBuf[0] = 'a';
        logBuf[1] = 'b';
        logBuf[2] = 'c';
        logBuf[3] = 'd';
        logBuf[5 + len] = '\n';
        if (CDC_Send(logBuf + 5, (uint16_t) len+1)>0){
            logBuf[4] = 'U';
        } else{
            logBuf[4] = 'u';
        }

        len += 5;
        if (len > LOG_TMP_BUF_SIZE - 6) len = LOG_TMP_BUF_SIZE - 6;
        memcpy(logBuf+len, "ABCDE\0", 6);
//        kfifo_put(&slogfifo, (uint8_t *) logBuf, (uint32_t) (len + 5));
        HAL_UART_Transmit(&huart1, (uint8_t *)logBuf, (uint32_t) (len + 5), 2000);
    }
}

#endif // #ifdef MMX_LOG_EN
