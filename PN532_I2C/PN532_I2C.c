#include "PN532_I2C.h"
#include <string.h>

void pn532_i2c_init(pn532_i2c_t *dev, I2C_HandleTypeDef *hi2c, uint8_t address)
{
    dev->hi2c = hi2c;
    dev->command = 0;
    dev->address = address>>1; // PN532 address shifted
}

void pn532_i2c_begin(pn532_i2c_t *dev)
{
    (void)dev; // I2C assumed initialized
}

void pn532_i2c_wakeup(pn532_i2c_t *dev)
{
    HAL_Delay(500);
}

static int8_t i2c_write(pn532_i2c_t *dev, uint8_t data)
{
    return HAL_I2C_Master_Transmit(dev->hi2c, dev->address<<1, &data, 1, HAL_MAX_DELAY)==HAL_OK;
}

int8_t pn532_i2c_write_command(pn532_i2c_t *dev, const uint8_t *header, uint8_t hlen,
                               const uint8_t *body, uint8_t blen)
{
    dev->command = header[0];
    uint8_t frame[8+hlen+blen];
    uint8_t idx=0;
    frame[idx++]=PN532_PREAMBLE;
    frame[idx++]=PN532_STARTCODE1;
    frame[idx++]=PN532_STARTCODE2;
    uint8_t length = hlen+blen+1;
    frame[idx++]=length;
    frame[idx++]=~length+1;
    frame[idx++]=PN532_HOSTTOPN532;
    uint8_t sum=PN532_HOSTTOPN532;
    memcpy(&frame[idx],header,hlen); idx+=hlen; for(uint8_t i=0;i<hlen;i++) sum+=header[i];
    if(body && blen){ memcpy(&frame[idx],body,blen); idx+=blen; for(uint8_t i=0;i<blen;i++) sum+=body[i]; }
    frame[idx++]=~sum+1;
    frame[idx++]=PN532_POSTAMBLE;
    return HAL_I2C_Master_Transmit(dev->hi2c, dev->address<<1, frame, idx, HAL_MAX_DELAY)==HAL_OK ? 0 : -1;
}

static int16_t i2c_receive(pn532_i2c_t *dev, uint8_t *buf, uint16_t len, uint16_t timeout)
{
    if(HAL_I2C_Master_Receive(dev->hi2c, dev->address<<1, buf, len, timeout)==HAL_OK)
        return len;
    return PN532_TIMEOUT;
}

int16_t pn532_i2c_read_response(pn532_i2c_t *dev, uint8_t *buf, uint8_t len, uint16_t timeout)
{
    uint8_t hdr[6];
    if(i2c_receive(dev,hdr,6,timeout)!=6) return PN532_TIMEOUT;
    if(hdr[0]!=0 || hdr[1]!=0 || hdr[2]!=0xFF) return PN532_INVALID_FRAME;
    uint8_t length_payload = hdr[3];
    if((uint8_t)(length_payload + hdr[4])!=0) return PN532_INVALID_FRAME;
    uint8_t cmd = dev->command + 1;
    if(i2c_receive(dev,hdr,2,timeout)!=2) return PN532_TIMEOUT;
    if(hdr[0]!=PN532_PN532TOHOST || hdr[1]!=cmd) return PN532_INVALID_FRAME;
    length_payload -=2;
    if(length_payload>len) return PN532_NO_SPACE;
    if(i2c_receive(dev,buf,length_payload,timeout)!=length_payload) return PN532_TIMEOUT;
    uint8_t sum=PN532_PN532TOHOST+cmd; for(uint8_t i=0;i<length_payload;i++) sum+=buf[i];
    uint8_t tail[2];
    if(i2c_receive(dev,tail,2,timeout)!=2) return PN532_TIMEOUT;
    if((uint8_t)(sum+tail[0])!=0 || tail[1]!=0) return PN532_INVALID_FRAME;
    return length_payload;
}

static void iface_begin(pn532_interface_t *iface){ pn532_i2c_begin((pn532_i2c_t*)iface->context); }
static void iface_wakeup(pn532_interface_t *iface){ pn532_i2c_wakeup((pn532_i2c_t*)iface->context); }
static int8_t iface_write(pn532_interface_t *iface,const uint8_t *h,uint8_t hl,const uint8_t *b,uint8_t bl){ return pn532_i2c_write_command((pn532_i2c_t*)iface->context,h,hl,b,bl); }
static int16_t iface_read(pn532_interface_t *iface,uint8_t *buf,uint8_t len,uint16_t t){ return pn532_i2c_read_response((pn532_i2c_t*)iface->context,buf,len,t); }

void pn532_i2c_create_interface(pn532_i2c_t *dev, pn532_interface_t *iface)
{
    iface->context=dev;
    iface->begin=iface_begin;
    iface->wakeup=iface_wakeup;
    iface->write_command=iface_write;
    iface->read_response=iface_read;
}
