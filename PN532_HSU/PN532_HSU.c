#include "PN532_HSU.h"
#include "PN532_debug.h"
#include <string.h>

void pn532_hsu_init(pn532_hsu_t *hsu, UART_HandleTypeDef *huart)
{
    hsu->huart = huart;
    hsu->command = 0;
}

void pn532_hsu_begin(pn532_hsu_t *hsu)
{
    /* UART should already be configured with 115200 baud by MXCube */
    (void)hsu;
}

void pn532_hsu_wakeup(pn532_hsu_t *hsu)
{
    uint8_t buf[5] = {0x55,0x55,0x00,0x00,0x00};
    HAL_UART_Transmit(hsu->huart, buf, sizeof(buf), HAL_MAX_DELAY);
}

static int8_t hsu_receive(pn532_hsu_t *hsu, uint8_t *buf, int len, uint16_t timeout)
{
    if(HAL_UART_Receive(hsu->huart, buf, len, timeout)==HAL_OK)
        return len;
    return PN532_TIMEOUT;
}

static int8_t hsu_read_ack(pn532_hsu_t *hsu)
{
    const uint8_t ack[] = {0,0,0xFF,0,0xFF,0};
    uint8_t tmp[6];
    if(hsu_receive(hsu,tmp,6,PN532_ACK_WAIT_TIME)!=6)
        return PN532_TIMEOUT;
    if(memcmp(tmp,ack,6)!=0)
        return PN532_INVALID_ACK;
    return 0;
}

int8_t pn532_hsu_write_command(pn532_hsu_t *hsu, const uint8_t *header, uint8_t hlen,
                               const uint8_t *body, uint8_t blen)
{
    hsu->command = header[0];
    uint8_t preamble[] = {PN532_PREAMBLE,PN532_STARTCODE1,PN532_STARTCODE2};
    HAL_UART_Transmit(hsu->huart, preamble, sizeof(preamble), HAL_MAX_DELAY);
    uint8_t length = hlen + blen + 1;
    uint8_t lcs = ~length + 1;
    HAL_UART_Transmit(hsu->huart, &length, 1, HAL_MAX_DELAY);
    HAL_UART_Transmit(hsu->huart, &lcs, 1, HAL_MAX_DELAY);

    uint8_t tfi = PN532_HOSTTOPN532;
    HAL_UART_Transmit(hsu->huart, &tfi, 1, HAL_MAX_DELAY);
    uint8_t sum = tfi;
    HAL_UART_Transmit(hsu->huart, (uint8_t*)header, hlen, HAL_MAX_DELAY);
    for(uint8_t i=0;i<hlen;i++) sum += header[i];
    if(body && blen){
        HAL_UART_Transmit(hsu->huart, (uint8_t*)body, blen, HAL_MAX_DELAY);
        for(uint8_t i=0;i<blen;i++) sum += body[i];
    }
    uint8_t checksum = ~sum + 1;
    HAL_UART_Transmit(hsu->huart, &checksum, 1, HAL_MAX_DELAY);
    uint8_t post = PN532_POSTAMBLE;
    HAL_UART_Transmit(hsu->huart, &post, 1, HAL_MAX_DELAY);

    return hsu_read_ack(hsu);
}

int16_t pn532_hsu_read_response(pn532_hsu_t *hsu, uint8_t *buf, uint8_t len, uint16_t timeout)
{
    uint8_t tmp[3];
    if(hsu_receive(hsu,tmp,3,timeout)!=3)
        return PN532_TIMEOUT;
    if(tmp[0]!=0 || tmp[1]!=0 || tmp[2]!=0xFF)
        return PN532_INVALID_FRAME;
    uint8_t length_bytes[2];
    if(hsu_receive(hsu,length_bytes,2,timeout)!=2)
        return PN532_TIMEOUT;
    if((uint8_t)(length_bytes[0]+length_bytes[1])!=0)
        return PN532_INVALID_FRAME;
    uint8_t length_payload = length_bytes[0]-2;
    if(length_payload>len)
        return PN532_NO_SPACE;
    uint8_t cmd_bytes[2];
    if(hsu_receive(hsu,cmd_bytes,2,timeout)!=2)
        return PN532_TIMEOUT;
    uint8_t cmd = hsu->command + 1;
    if(cmd_bytes[0]!=PN532_PN532TOHOST || cmd_bytes[1]!=cmd)
        return PN532_INVALID_FRAME;
    if(hsu_receive(hsu,buf,length_payload,timeout)!=length_payload)
        return PN532_TIMEOUT;
    uint8_t sum = PN532_PN532TOHOST + cmd;
    for(uint8_t i=0;i<length_payload;i++) sum += buf[i];
    uint8_t crc[2];
    if(hsu_receive(hsu,crc,2,timeout)!=2)
        return PN532_TIMEOUT;
    if((uint8_t)(sum+crc[0])!=0 || crc[1]!=0)
        return PN532_INVALID_FRAME;
    return length_payload;
}

static void iface_begin(pn532_interface_t *iface){
    pn532_hsu_t *hsu = (pn532_hsu_t*)iface->context;
    pn532_hsu_begin(hsu);
}
static void iface_wakeup(pn532_interface_t *iface){
    pn532_hsu_t *hsu = (pn532_hsu_t*)iface->context;
    pn532_hsu_wakeup(hsu);
}
static int8_t iface_write(pn532_interface_t *iface,const uint8_t *header,uint8_t hlen,const uint8_t *body,uint8_t blen){
    return pn532_hsu_write_command((pn532_hsu_t*)iface->context,header,hlen,body,blen);
}
static int16_t iface_read(pn532_interface_t *iface,uint8_t *buf,uint8_t len,uint16_t timeout){
    return pn532_hsu_read_response((pn532_hsu_t*)iface->context,buf,len,timeout);
}

void pn532_hsu_create_interface(pn532_hsu_t *hsu, pn532_interface_t *iface)
{
    iface->context = hsu;
    iface->begin = iface_begin;
    iface->wakeup = iface_wakeup;
    iface->write_command = iface_write;
    iface->read_response = iface_read;
}
