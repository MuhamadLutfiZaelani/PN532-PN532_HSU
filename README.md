# PN532-PN532_HSU
=======
## NFC library for Arduino

This is an Arduino library for PN532 to use NFC technology. It's based on 
[Adafruit_NFCShield_I2C](http://goo.gl/pk3FdB)
, improved by [Seeed Studio](http://goo.gl/zh1iQh), added HSU(High Speed Uart) driver by [Elechouse](http://elechouse.com). 

It works with:

+ [Elechouse NFC Module](http://goo.gl/i0EQgd)
+ [Seeed Studio NFC Shield](http://goo.gl/Cac2OH)
+ [Xadow NFC](http://goo.gl/qBZMt0)
+ [Adafruit PN532 NFC/RFID controller breakout board](http://goo.gl/tby9Sw)

### Features
+ Support all interfaces of PN532 (I2C, SPI, HSU )
+ Read/write Mifare Classic Card
+ Works with [Don's NDEF Library](http://goo.gl/jDjsXl) or the C implementation in `NDEF/c_version`
+ Support Peer to Peer communication(exchange data with android 4.0+)
+ Support [mbed platform](http://goo.gl/kGPovZ)

### Getting Started
1. **Download [zip file](https://github.com/elechouse/PN532/archive/PN532_HSU.zip) and 
extract the three folders(PN532, PN532_SPI, PN532_HSU and PN532_I2C) into libraries of Arduino.**
2. Downlaod [Don's NDEF library](http://goo.gl/ewxeAe) and extract it into libraries of Arduino's into a new folder called "NDEF" (Note if you leave this folder as NDEF-Master Arduino will not be able to use it as a library)
2. Follow the examples of the PN532 library.  A minimal demonstration of the C
   NDEF implementation is provided in `examples/ndef_c_example`.

Additional Arduino sketches are located under `PN532/examples`. STM32Cube HAL example projects can be found in the `examples` folder such as `examples/blackpill`, `examples/stm32_i2c` and `examples/stm32_spi`.

### To do
+ Card emulation

## HSU Interface

HSU is short for High Speed Uart. HSU interface needs only 4 wires to connect PN532 with Arduino, [Sensor Shield](http://goo.gl/i0EQgd) can make it more easier. For some Arduino boards like [Leonardo][Leonardo], [DUE][DUE], [Mega][Mega] ect, there are more than one `Serial` on these boards, so we can use this additional Serial to control PN532, HSU uses 115200 baud rate .

To use the `Serial1` control PN532, refer to the code below.

	#include <PN532_HSU.h>
	#include <PN532.h>
	
	PN532_HSU pn532hsu(Serial1);
	PN532 nfc(pn532hsu);

	void setup(void)
	{
		nfc.begin();
		//...
	}

If your Arduino has only one serial interface and you want to keep it for control or debugging with the Serial Monitor, you can use the [`SoftwareSerial`][SoftwareSerial] library to control the PN532 by emulating a serial interface. Include `PN532_SWHSU.h` instead of `PN532_HSU.h`:

	#include <SoftwareSerial.h>
	#include <PN532_SWHSU.h>
	#include <PN532.h>
	
	SoftwareSerial SWSerial( 10, 11 ); // RX, TX

	PN532_SWHSU pn532swhsu( SWSerial );
	PN532 nfc( pn532swhsu );

	void setup(void)
	{
		nfc.begin();
		//...
	}

[Mega]: http://arduino.cc/en/Main/arduinoBoardMega
[DUE]: http://arduino.cc/en/Main/arduinoBoardDue
[Leonardo]: http://arduino.cc/en/Main/arduinoBoardLeonardo
[SoftwareSerial]: https://www.arduino.cc/en/Reference/softwareSerial


## STM32 HAL

The repository includes C implementations suitable for STM32Cube HAL.  Add
`PN532/PN532.c`, `PN532/PN532_debug.c` and the interface driver you want from
`PN532_I2C/PN532_I2C.c`, `PN532_SPI/PN532_SPI.c` or
`PN532_HSU/PN532_HSU.c`.  When compiling for STM32 make sure only these `.c`
files are used.  All Arduino specific `.cpp` sources should be removed or
ignored so that there are no Arduino dependencies.

See [docs/STM32_HAL.md](docs/STM32_HAL.md) for peripheral initialisation and a
minimal usage example.
CubeIDE samples using the HAL drivers are provided in `examples/stm32_i2c` and `examples/stm32_spi`.

## NDEF C version

For bare-metal or STM32 targets without the Arduino framework the repository
contains a minimal C implementation of NDEF message encoding.  Add
`NDEF/c_version/ndef_c.c` to your project and include `ndef_c.h` to construct
text records and encode NDEF messages.
