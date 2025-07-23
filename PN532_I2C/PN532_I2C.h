#ifndef __PN532_I2C_H__
#define __PN532_I2C_H__

#include "PN532Interface.h"
#include <stdint.h>
#include "stm32f1xx_hal.h"

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t command;
    uint8_t address;
} pn532_i2c_t;

void pn532_i2c_init(pn532_i2c_t *dev, I2C_HandleTypeDef *hi2c, uint8_t address);
void pn532_i2c_create_interface(pn532_i2c_t *dev, pn532_interface_t *iface);
int8_t pn532_i2c_write_command(pn532_i2c_t *dev, const uint8_t *header, uint8_t hlen,
                               const uint8_t *body, uint8_t blen);
int16_t pn532_i2c_read_response(pn532_i2c_t *dev, uint8_t *buf, uint8_t len, uint16_t timeout);
void pn532_i2c_wakeup(pn532_i2c_t *dev);
void pn532_i2c_begin(pn532_i2c_t *dev);

#endif
