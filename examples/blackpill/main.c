#include "PN532/PN532.h"
#include "PN532_HSU/PN532_HSU.h"
#include "NDEF/c_version/ndef_c.h"

extern UART_HandleTypeDef huart1;

int main(void)
{
    HAL_Init();
    // SystemClock_Config();
    // MX_GPIO_Init();
    // MX_USART1_UART_Init();

    pn532_hsu_t hsu;
    pn532_interface_t iface;
    pn532_hsu_init(&hsu, &huart1);
    pn532_hsu_create_interface(&hsu, &iface);

    pn532_t nfc;
    pn532_init(&nfc, &iface);
    pn532_begin(&nfc);
    pn532_SAMConfig(&nfc);

    uint8_t uid[7];
    uint8_t uid_len;
    if(pn532_read_passive_target_id(&nfc, 0x00, uid, &uid_len, 1000))
    {
        pn532_print_hex(uid, uid_len);
    }

    ndef_message_t msg;
    ndef_message_init(&msg);
    ndef_message_add_text_record(&msg, "Hello NFC");
    uint8_t buffer[64];
    uint16_t len = ndef_message_encode(&msg, buffer);
    // buffer now contains encoded NDEF message

    while(1)
    {
    }
}
