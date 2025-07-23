# Simple example Makefile for the BlackPill demo

SRCS = main.c modbus_crc.c
OBJS = $(SRCS:.c=.o)

# Add paths for the PN532 library sources if they are available in your project
# These are only placeholders and may need adjustment for your environment
PN532_SRCS = ../../PN532/PN532.c \
              ../../PN532/PN532_debug.c \
              ../../PN532_HSU/PN532_HSU.c

CFLAGS ?= -I. -I../../PN532 -I../../PN532_HSU

all: blackpill.elf

blackpill.elf: $(OBJS) $(PN532_SRCS)
	$(CC) $(CFLAGS) $^ -o $@

clean:
	rm -f $(OBJS) blackpill.elf
