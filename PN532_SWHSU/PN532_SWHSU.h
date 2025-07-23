#ifndef __PN532_SWHSU_H__
#define __PN532_SWHSU_H__

#include "PN532Interface.h"
#include <stdint.h>
#include "stm32f1xx_hal.h"

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t command;
} pn532_swhsu_t;

void pn532_swhsu_init(pn532_swhsu_t *dev, UART_HandleTypeDef *huart);
void pn532_swhsu_create_interface(pn532_swhsu_t *dev, pn532_interface_t *iface);
int8_t pn532_swhsu_write_command(pn532_swhsu_t *dev, const uint8_t *header, uint8_t hlen,
                                 const uint8_t *body, uint8_t blen);
int16_t pn532_swhsu_read_response(pn532_swhsu_t *dev, uint8_t *buf, uint8_t len, uint16_t timeout);
void pn532_swhsu_wakeup(pn532_swhsu_t *dev);
void pn532_swhsu_begin(pn532_swhsu_t *dev);

#endif
