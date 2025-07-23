#ifndef __PN532_HSU_H__
#define __PN532_HSU_H__

#include "PN532Interface.h"
#include <stdint.h>
#include "stm32f1xx_hal.h"

#define PN532_HSU_DEBUG

#define PN532_HSU_READ_TIMEOUT (1000)

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t command;
} pn532_hsu_t;

void pn532_hsu_init(pn532_hsu_t *hsu, UART_HandleTypeDef *huart);
void pn532_hsu_begin(pn532_hsu_t *hsu);
void pn532_hsu_wakeup(pn532_hsu_t *hsu);
int8_t pn532_hsu_write_command(pn532_hsu_t *hsu, const uint8_t *header, uint8_t hlen,
                               const uint8_t *body, uint8_t blen);
int16_t pn532_hsu_read_response(pn532_hsu_t *hsu, uint8_t *buf, uint8_t len, uint16_t timeout);
void pn532_hsu_create_interface(pn532_hsu_t *hsu, pn532_interface_t *iface);

#endif
