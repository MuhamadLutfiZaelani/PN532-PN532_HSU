#include "PN532_SWHSU.h"

void pn532_swhsu_init(pn532_swhsu_t *dev, UART_HandleTypeDef *huart)
{
    dev->huart = huart;
    dev->command = 0;
}

void pn532_swhsu_begin(pn532_swhsu_t *dev){ (void)dev; }
void pn532_swhsu_wakeup(pn532_swhsu_t *dev){ uint8_t buf[5]={0x55,0x55,0,0,0}; HAL_UART_Transmit(dev->huart,buf,5,HAL_MAX_DELAY); }

int8_t pn532_swhsu_write_command(pn532_swhsu_t *dev,const uint8_t *header,uint8_t hlen,const uint8_t *body,uint8_t blen)
{
    return PN532_TIMEOUT; // not implemented
}

int16_t pn532_swhsu_read_response(pn532_swhsu_t *dev,uint8_t *buf,uint8_t len,uint16_t t)
{
    return PN532_TIMEOUT; // not implemented
}

static void iface_begin(pn532_interface_t *iface){ pn532_swhsu_begin((pn532_swhsu_t*)iface->context); }
static void iface_wakeup(pn532_interface_t *iface){ pn532_swhsu_wakeup((pn532_swhsu_t*)iface->context); }
static int8_t iface_write(pn532_interface_t *iface,const uint8_t *h,uint8_t hl,const uint8_t *b,uint8_t bl){ return pn532_swhsu_write_command((pn532_swhsu_t*)iface->context,h,hl,b,bl); }
static int16_t iface_read(pn532_interface_t *iface,uint8_t *buf,uint8_t len,uint16_t t){ return pn532_swhsu_read_response((pn532_swhsu_t*)iface->context,buf,len,t); }

void pn532_swhsu_create_interface(pn532_swhsu_t *dev, pn532_interface_t *iface)
{
    iface->context=dev;
    iface->begin=iface_begin;
    iface->wakeup=iface_wakeup;
    iface->write_command=iface_write;
    iface->read_response=iface_read;
}
