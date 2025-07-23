#ifndef MODBUS_CRC_H
#define MODBUS_CRC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint16_t ModbusCRC(const uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif // MODBUS_CRC_H
