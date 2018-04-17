#ifndef _LOG_H_
#define _LOG_H_

//void debug_log(char *str);

#define MMX_LOG_EN

#ifdef MMX_LOG_EN

#include <string.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
void mmxLog(char type, const char *format, ...);
void LogHex(const char *head, uint8_t *pData, uint16_t len, const char *tail);

#define LogI(...)  mmxLog('I', __VA_ARGS__)
#define LogE(...)  mmxLog('E', __VA_ARGS__)

#define USBRsp(...)
#else
#define LogI(...)
#define LogE(...)
#define LogHex(h, d, l, t)
#endif

void LogInit(void);
#endif
