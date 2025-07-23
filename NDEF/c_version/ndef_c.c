#include "ndef_c.h"
#include <string.h>

void ndef_message_init(ndef_message_t *msg)
{
    msg->record_count = 0;
}

bool ndef_message_add_text_record(ndef_message_t *msg, const char *text)
{
    if(msg->record_count >= NDEF_MAX_RECORDS) return false;
    ndef_record_t *rec = &msg->records[msg->record_count++];
    rec->tnf = 1; // well known
    const char type[] = "T";
    rec->type_length = 1;
    memcpy(rec->type, type, 1);
    rec->payload_length = strlen(text)+1;
    rec->payload[0] = 0x02; // UTF8, no lang code
    strncpy((char*)&rec->payload[1], text, NDEF_MAX_PAYLOAD-1);
    return true;
}

static uint16_t encode_record(const ndef_record_t *rec, uint8_t mb, uint8_t me, uint8_t *out)
{
    uint8_t flags = (mb?0x80:0) | (me?0x40:0) | rec->tnf;
    uint8_t *p = out;
    *p++ = flags;
    *p++ = rec->type_length;
    *p++ = rec->payload_length;
    memcpy(p, rec->type, rec->type_length); p += rec->type_length;
    memcpy(p, rec->payload, rec->payload_length); p += rec->payload_length;
    return p - out;
}

uint16_t ndef_message_encode(const ndef_message_t *msg, uint8_t *out)
{
    uint8_t *p = out;
    for(uint8_t i=0;i<msg->record_count;i++)
    {
        p += encode_record(&msg->records[i], i==0, i==(msg->record_count-1), p);
    }
    return p - out;
}

