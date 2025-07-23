#ifndef __DEBUG_H__
#define __DEBUG_H__


//#define DEBUG

#include <stdio.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int (*pn532_debug_printf_t)(const char *fmt, ...);

extern pn532_debug_printf_t pn532_debug_printf;

static inline void pn532_set_debug_printf(pn532_debug_printf_t func)
{
    pn532_debug_printf = func;
}

#ifdef DEBUG
#define DMSG(...)         do { if (pn532_debug_printf) pn532_debug_printf(__VA_ARGS__); } while (0)
#define DMSG_STR(str)     do { if (pn532_debug_printf) pn532_debug_printf("%s\n", (str)); } while (0)
#define DMSG_HEX(num)     do { if (pn532_debug_printf) pn532_debug_printf(" %02X", (uint8_t)(num)); } while (0)
#define DMSG_INT(num)     do { if (pn532_debug_printf) pn532_debug_printf(" %d", (int)(num)); } while (0)
#else
#define DMSG(...)
#define DMSG_STR(str)
#define DMSG_HEX(num)
#define DMSG_INT(num)
#endif

#ifdef __cplusplus
}
#endif

#endif
