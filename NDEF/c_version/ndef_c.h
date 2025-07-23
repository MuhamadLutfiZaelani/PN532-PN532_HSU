#ifndef NDEF_C_H
#define NDEF_C_H

#include <stdint.h>
#include <stdbool.h>

#define NDEF_MAX_RECORDS 4
#define NDEF_MAX_PAYLOAD 128

typedef struct {
    uint8_t tnf;
    uint8_t type_length;
    uint8_t payload_length;
    uint8_t type[16];
    uint8_t payload[NDEF_MAX_PAYLOAD];
} ndef_record_t;

typedef struct {
    ndef_record_t records[NDEF_MAX_RECORDS];
    uint8_t record_count;
} ndef_message_t;

void ndef_message_init(ndef_message_t *msg);
bool ndef_message_add_text_record(ndef_message_t *msg, const char *text);
uint16_t ndef_message_encode(const ndef_message_t *msg, uint8_t *out);

#endif
