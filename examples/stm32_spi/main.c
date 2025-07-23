#include "PN532_SPI.h"
#include "PN532.h"
#include "stm32f1xx_hal.h"

extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart2;
extern GPIO_TypeDef* PN532_SS_PORT;
extern uint16_t PN532_SS_PIN;

static void pn532_spi_cs_select(void)
{
    HAL_GPIO_WritePin(PN532_SS_PORT, PN532_SS_PIN, GPIO_PIN_RESET);
}

static void pn532_spi_cs_deselect(void)
{
    HAL_GPIO_WritePin(PN532_SS_PORT, PN532_SS_PIN, GPIO_PIN_SET);
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_SPI2_Init();
    MX_USART2_UART_Init();
    MX_GPIO_Init();

    pn532_spi_t nfc_spi;
    pn532_interface_t iface;
    pn532_spi_init(&nfc_spi, &hspi2, pn532_spi_cs_select, pn532_spi_cs_deselect);
    pn532_spi_create_interface(&nfc_spi, &iface);

    pn532_t nfc;
    pn532_init(&nfc, &iface);
    pn532_begin(&nfc);
    pn532_SAMConfig(&nfc);

    uint8_t uid[7];
    uint8_t uid_len;

    while (1)
    {
        if (pn532_read_passive_target_id(&nfc, 0x00, uid, &uid_len, 1000))
        {
            HAL_UART_Transmit(&huart2, uid, uid_len, HAL_MAX_DELAY);
            const uint8_t newline[2] = {'\r','\n'};
            HAL_UART_Transmit(&huart2, newline, 2, HAL_MAX_DELAY);
        }
    }
}
