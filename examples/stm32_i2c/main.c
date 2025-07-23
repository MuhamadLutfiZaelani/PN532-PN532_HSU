#include "PN532_I2C.h"
#include "PN532.h"
#include "stm32f1xx_hal.h"

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_I2C1_Init();
    MX_USART2_UART_Init();

    pn532_i2c_t nfc_dev;
    pn532_interface_t iface;
    pn532_i2c_init(&nfc_dev, &hi2c1, 0x48); /* adjust address for your board */
    pn532_i2c_create_interface(&nfc_dev, &iface);

    pn532_t nfc;
    pn532_init(&nfc, &iface);
    pn532_begin(&nfc);
    pn532_SAMConfig(&nfc);

    uint8_t uid[7];
    uint8_t uid_len;

    while(1)
    {
        if(pn532_read_passive_target_id(&nfc, 0x00, uid, &uid_len, 1000))
        {
            HAL_UART_Transmit(&huart2, uid, uid_len, HAL_MAX_DELAY);
            const uint8_t newline[2] = {'\r','\n'};
            HAL_UART_Transmit(&huart2, newline, 2, HAL_MAX_DELAY);
        }
    }
}
