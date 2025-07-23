# BlackPill RS-485 Example

This example shows how to interface the PN532 module on an STM32 *BlackPill* board. Communication with a host happens over Modbus RTU on RS‑485 while the PN532 is connected via UART.

## Modbus registers

The firmware exposes a small holding‑register map using slave ID **10**:

| Register | Function                                   |
|---------:|--------------------------------------------|
| `0`      | Relay status (write with Function 0x06)    |
| `1`      | UID bytes 0–1 (read with Function 0x03)    |
| `2`      | UID bytes 2–3 (read with Function 0x03)    |
| `3`      | Reserved for future UID bytes              |

Writing a value from `0x0000`..`0x0007` to register `0` turns the four relays on or off. When a card is detected the firmware updates registers `1` and `2` with the most recent UID. If no card is present register `1` and `2` remain unchanged and register `0` of the UID block is set to zero.

## RS‑485 UART settings

Modbus traffic uses `USART1` at **9600 baud**, `8E1` framing (8 data bits, even parity, one stop bit). The DE pin must be toggled manually to switch between transmit and receive.

## Polling UID from a master

A master should periodically read registers `1`–`3` using Function Code `0x03`. When register `1` returns non‑zero, combine the words from registers `1` and `2` to obtain the 4‑byte UID. Reading register `0` provides the current relay state.

