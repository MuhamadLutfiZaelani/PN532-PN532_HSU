#include <stdio.h>
#include "NDEF/c_version/ndef_c.h"

int main(void)
{
    ndef_message_t msg;
    ndef_message_init(&msg);
    ndef_message_add_text_record(&msg, "Hello NFC");

    uint8_t buffer[256];
    uint16_t len = ndef_message_encode(&msg, buffer);

    for(uint16_t i = 0; i < len; i++)
        printf("%02X ", buffer[i]);
    printf("\n");
    return 0;
}

