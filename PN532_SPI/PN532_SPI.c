#include "PN532_SPI.h"
#include <string.h>

static void select(pn532_spi_t *dev){ HAL_GPIO_WritePin(dev->ss_port,dev->ss_pin,GPIO_PIN_RESET); }
static void deselect(pn532_spi_t *dev){ HAL_GPIO_WritePin(dev->ss_port,dev->ss_pin,GPIO_PIN_SET); }

void pn532_spi_init(pn532_spi_t *dev, SPI_HandleTypeDef *hspi, GPIO_TypeDef *port, uint16_t pin)
{
    dev->hspi = hspi;
    dev->ss_port = port;
    dev->ss_pin = pin;
    dev->command = 0;
    deselect(dev);
}

void pn532_spi_begin(pn532_spi_t *dev)
{
    deselect(dev);
}

void pn532_spi_wakeup(pn532_spi_t *dev)
{
    select(dev); HAL_Delay(2); deselect(dev);
}

int8_t pn532_spi_write_command(pn532_spi_t *dev, const uint8_t *header, uint8_t hlen,
                               const uint8_t *body, uint8_t blen)
{
    dev->command = header[0];
    uint8_t pre[] = {0x01,PN532_PREAMBLE,PN532_STARTCODE1,PN532_STARTCODE2};
    select(dev);
    HAL_SPI_Transmit(dev->hspi, pre, sizeof(pre), HAL_MAX_DELAY);
    uint8_t length = hlen+blen+1;
    uint8_t lcs = ~length+1;
    HAL_SPI_Transmit(dev->hspi, &length,1,HAL_MAX_DELAY);
    HAL_SPI_Transmit(dev->hspi, &lcs,1,HAL_MAX_DELAY);
    uint8_t tfi=PN532_HOSTTOPN532; HAL_SPI_Transmit(dev->hspi,&tfi,1,HAL_MAX_DELAY);
    uint8_t sum=tfi;
    HAL_SPI_Transmit(dev->hspi,(uint8_t*)header,hlen,HAL_MAX_DELAY); for(uint8_t i=0;i<hlen;i++) sum+=header[i];
    if(body&&blen){HAL_SPI_Transmit(dev->hspi,(uint8_t*)body,blen,HAL_MAX_DELAY); for(uint8_t i=0;i<blen;i++) sum+=body[i];}
    uint8_t checksum=~sum+1; HAL_SPI_Transmit(dev->hspi,&checksum,1,HAL_MAX_DELAY);
    uint8_t post=PN532_POSTAMBLE; HAL_SPI_Transmit(dev->hspi,&post,1,HAL_MAX_DELAY);
    deselect(dev);
    return 0;
}

static int16_t spi_read(pn532_spi_t *dev,uint8_t *buf,uint16_t len,uint16_t timeout)
{
    HAL_StatusTypeDef s = HAL_SPI_Receive(dev->hspi,buf,len,timeout);
    return s==HAL_OK?len:PN532_TIMEOUT;
}

int16_t pn532_spi_read_response(pn532_spi_t *dev, uint8_t *buf, uint8_t len, uint16_t timeout)
{
    uint8_t tmp[6];
    select(dev);
    uint8_t readcmd=0x03;
    HAL_SPI_Transmit(dev->hspi,&readcmd,1,HAL_MAX_DELAY);
    if(spi_read(dev,tmp,6,timeout)!=6){ deselect(dev); return PN532_TIMEOUT; }
    if(tmp[1]!=0 || tmp[2]!=0xFF){ deselect(dev); return PN532_INVALID_FRAME; }
    uint8_t length_payload=tmp[3];
    if((uint8_t)(length_payload+tmp[4])!=0){ deselect(dev); return PN532_INVALID_FRAME; }
    uint8_t cmd = dev->command+1;
    if(spi_read(dev,tmp,2,timeout)!=2){ deselect(dev); return PN532_TIMEOUT; }
    if(tmp[0]!=PN532_PN532TOHOST || tmp[1]!=cmd){ deselect(dev); return PN532_INVALID_FRAME; }
    length_payload -=2;
    if(length_payload>len){ deselect(dev); return PN532_NO_SPACE; }
    if(spi_read(dev,buf,length_payload,timeout)!=length_payload){ deselect(dev); return PN532_TIMEOUT; }
    uint8_t sum=PN532_PN532TOHOST+cmd; for(uint8_t i=0;i<length_payload;i++) sum+=buf[i];
    if(spi_read(dev,tmp,2,timeout)!=2){ deselect(dev); return PN532_TIMEOUT; }
    deselect(dev);
    if((uint8_t)(sum+tmp[0])!=0 || tmp[1]!=0) return PN532_INVALID_FRAME;
    return length_payload;
}

static void iface_begin(pn532_interface_t *iface){ pn532_spi_begin((pn532_spi_t*)iface->context); }
static void iface_wakeup(pn532_interface_t *iface){ pn532_spi_wakeup((pn532_spi_t*)iface->context); }
static int8_t iface_write(pn532_interface_t *iface,const uint8_t *h,uint8_t hl,const uint8_t *b,uint8_t bl){ return pn532_spi_write_command((pn532_spi_t*)iface->context,h,hl,b,bl); }
static int16_t iface_read(pn532_interface_t *iface,uint8_t *buf,uint8_t len,uint16_t t){ return pn532_spi_read_response((pn532_spi_t*)iface->context,buf,len,t); }

void pn532_spi_create_interface(pn532_spi_t *dev, pn532_interface_t *iface)
{
    iface->context=dev;
    iface->begin=iface_begin;
    iface->wakeup=iface_wakeup;
    iface->write_command=iface_write;
    iface->read_response=iface_read;
}
