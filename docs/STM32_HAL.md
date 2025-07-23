# Using the library with STM32 HAL

This repository ships plain C implementations of the PN532 drivers so
that they can be compiled in STM32Cube HAL projects without the Arduino
framework.  Add the following source files to your application
depending on the interface you plan to use:

- `PN532/PN532.c`
- `PN532/PN532_debug.c`
- `PN532_I2C/PN532_I2C.c` *(I2C mode)*
- `PN532_SPI/PN532_SPI.c` *(SPI mode)*
- `PN532_HSU/PN532_HSU.c` *(UART/HSU mode)*
- `NDEF/c_version/ndef_c.c` *(optional NDEF helpers)*

All C headers from these directories should be available on the include
path.  Make sure that **none** of the `.cpp` files are compiled as they
contain Arduino specific code and will pull in unnecessary
dependencies.

## Peripheral initialization
Below are minimal examples for setting up the HAL handles.  Adjust the
configuration as needed for your board.

```c
I2C_HandleTypeDef hi2c1;
void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
}

SPI_HandleTypeDef hspi2;
void MX_SPI2_Init(void)
{
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 7;
    HAL_SPI_Init(&hspi2);
}

UART_HandleTypeDef huart1;
void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);
}
```

Once the peripherals are initialised you can attach them to the
library:

```c
pn532_i2c_t nfc_i2c;
pn532_interface_t iface;

pn532_i2c_init(&nfc_i2c, &hi2c1, 0x48);  // address depends on board
pn532_i2c_create_interface(&nfc_i2c, &iface);

pn532_t nfc;
pn532_init(&nfc, &iface);
pn532_begin(&nfc);
pn532_SAMConfig(&nfc);

uint8_t uid[7];
uint8_t uid_len;

if (pn532_read_passive_target_id(&nfc, 0x00, uid, &uid_len, 1000)) {
    // UID is now available in 'uid'
}
```
