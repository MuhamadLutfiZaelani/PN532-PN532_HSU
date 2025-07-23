#include "PN532.h"
#include <string.h>
#include "PN532_debug.h"

void pn532_init(pn532_t *nfc, pn532_interface_t *iface)
{
    nfc->iface = iface;
    nfc->uid_len = 0;
}

void pn532_begin(pn532_t *nfc)
{
    if(nfc->iface && nfc->iface->begin)
        nfc->iface->begin(nfc->iface);
    if(nfc->iface && nfc->iface->wakeup)
        nfc->iface->wakeup(nfc->iface);
}

uint32_t pn532_get_firmware_version(pn532_t *nfc)
{
    uint8_t cmd = PN532_COMMAND_GETFIRMWAREVERSION;
    if(nfc->iface->write_command(nfc->iface,&cmd,1,NULL,0))
        return 0;
    if(nfc->iface->read_response(nfc->iface,nfc->packet_buffer,4,1000)<0)
        return 0;
    uint32_t res = nfc->packet_buffer[0];
    res <<=8; res |= nfc->packet_buffer[1];
    res <<=8; res |= nfc->packet_buffer[2];
    res <<=8; res |= nfc->packet_buffer[3];
    return res;
}

bool pn532_SAMConfig(pn532_t *nfc)
{
    uint8_t cmd[] = {PN532_COMMAND_SAMCONFIGURATION,0x01,0x14,0x01};
    if(nfc->iface->write_command(nfc->iface,cmd,4,NULL,0))
        return false;
    return nfc->iface->read_response(nfc->iface,nfc->packet_buffer,8,1000) >= 0;
}

bool pn532_set_passive_activation_retries(pn532_t *nfc, uint8_t max_retries)
{
    uint8_t cmd[] = {PN532_COMMAND_RFCONFIGURATION, 5, 0xFF, 0x01, max_retries};
    if(nfc->iface->write_command(nfc->iface, cmd, sizeof(cmd), NULL, 0))
        return false;
    return nfc->iface->read_response(nfc->iface,
                                     nfc->packet_buffer,
                                     sizeof(nfc->packet_buffer),
                                     1000) >= 0;
}

bool pn532_read_passive_target_id(pn532_t *nfc, uint8_t cardbaudrate,
                                  uint8_t *uid, uint8_t *uid_length, uint16_t timeout)
{
    uint8_t cmd[] = {PN532_COMMAND_INLISTPASSIVETARGET,1,cardbaudrate};
    if(nfc->iface->write_command(nfc->iface,cmd,3,NULL,0))
        return false;
    if(nfc->iface->read_response(nfc->iface,nfc->packet_buffer,sizeof(nfc->packet_buffer),timeout)<0)
        return false;
    if(nfc->packet_buffer[0]!=1)
        return false;
    *uid_length = nfc->packet_buffer[5];
    for(uint8_t i=0;i<*uid_length;i++)
        uid[i]=nfc->packet_buffer[6+i];
    return true;
}

void pn532_print_hex(const uint8_t *data, uint32_t numBytes)
{
    for(uint32_t i = 0; i < numBytes; i++) {
        if (pn532_debug_printf) {
            pn532_debug_printf(" %02X", data[i]);
        }
    }
    if (pn532_debug_printf) {
        pn532_debug_printf("\n");
    }
}

