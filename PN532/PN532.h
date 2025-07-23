#ifndef __PN532_C_H__
#define __PN532_C_H__

#include <stdint.h>
#include <stdbool.h>
#include "PN532Interface.h"

// PN532 Commands
#define PN532_COMMAND_GETFIRMWAREVERSION    (0x02)
#define PN532_COMMAND_SAMCONFIGURATION      (0x14)
#define PN532_COMMAND_INLISTPASSIVETARGET   (0x4A)
#define PN532_COMMAND_RFCONFIGURATION       (0x32)

typedef struct {
    pn532_interface_t *iface;
    uint8_t uid[7];
    uint8_t uid_len;
    uint8_t packet_buffer[64];
} pn532_t;

void pn532_init(pn532_t *nfc, pn532_interface_t *iface);
void pn532_begin(pn532_t *nfc);
uint32_t pn532_get_firmware_version(pn532_t *nfc);
bool pn532_SAMConfig(pn532_t *nfc);
bool pn532_set_passive_activation_retries(pn532_t *nfc, uint8_t max_retries);
bool pn532_read_passive_target_id(pn532_t *nfc, uint8_t cardbaudrate,
                                  uint8_t *uid, uint8_t *uid_length, uint16_t timeout);

void pn532_print_hex(const uint8_t *data, uint32_t numBytes);

#endif
