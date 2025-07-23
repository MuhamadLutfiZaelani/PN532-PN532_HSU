#ifndef __PN532_SPI_H__
#define __PN532_SPI_H__

#include "PN532Interface.h"
#include <stdint.h>
#include "stm32f1xx_hal.h"

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *ss_port;
    uint16_t ss_pin;
    uint8_t command;
} pn532_spi_t;

void pn532_spi_init(pn532_spi_t *dev, SPI_HandleTypeDef *hspi, GPIO_TypeDef *port, uint16_t pin);
void pn532_spi_create_interface(pn532_spi_t *dev, pn532_interface_t *iface);
int8_t pn532_spi_write_command(pn532_spi_t *dev, const uint8_t *header, uint8_t hlen,
                               const uint8_t *body, uint8_t blen);
int16_t pn532_spi_read_response(pn532_spi_t *dev, uint8_t *buf, uint8_t len, uint16_t timeout);
void pn532_spi_wakeup(pn532_spi_t *dev);
void pn532_spi_begin(pn532_spi_t *dev);

#endif
