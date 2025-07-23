/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "PN532_HSU.h"
#include "PN532.h"

#include "modbus_crc.h"
#include <string.h>        // (opsional) untuk memset, dll.
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Konfigurasi dasar
#define SLAVE_ID       10          // Modbus slave ID
#define RX_BUF_SIZE    256        // Ukuran buffer RX DMA (maksimum frame Modbus)
#define UART_BAUDRATE  9600       // Baud rate UART (sesuaikan dengan master)

// Definisi pin untuk kemudahan
#define RS485_DIR_PORT 		GPIOA
#define RS485_DIR_PIN 		GPIO_PIN_8

#define RELAY1_PORT 		GPIOB
#define RELAY1_PIN 			GPIO_PIN_0

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
// Tambahan untuk NFC UID
PN532_HSU pn532_hsu;
PN532 nfc;

// Buffer RX untuk menerima data Modbus
static uint8_t rxBuf[RX_BUF_SIZE];
// Variabel holding register (16-bit) untuk status relay (0=OFF, 1=ON)
static volatile uint16_t holdingReg0 = 0;
// Register tambahan untuk menyimpan hasil baca kartu NFC
static uint16_t holding_registers[10] = {0};
// Penanda waktu untuk polling NFC
static uint32_t lastNfcTick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t uart1_tx_data[8];
uint8_t uart1_rx_data[15];
uint16_t recv_data[2];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    // Proses data yang diterima
    recv_data[0] = uart1_rx_data[3] << 8 | uart1_rx_data[4];
    recv_data[1] = uart1_rx_data[5] << 8 | uart1_rx_data[6];

    // Panggil lagi agar siap menerima data selanjutnya
    HAL_UART_Receive_IT(&huart1, uart1_rx_data, sizeof(uart1_rx_data));
  }
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  // Set awal: Relay OFF, RS485 dalam mode receive
  holdingReg0 = 0;
  HAL_GPIO_WritePin(GPIOB, RELAY_A_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, RELAY_B_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, RELAY_C_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, RELAY_D_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RELAY1_PORT, RELAY1_PIN, GPIO_PIN_SET);   // Pastikan relay mati, kalo kwh meter schneider
//  HAL_GPIO_WritePin(RELAY1_PORT, RELAY1_PIN, GPIO_PIN_RESET);   // Pastikan relay mati, kalo kwh meter chint
  HAL_GPIO_WritePin(RS485_DIR_PORT, RS485_DIR_PIN, GPIO_PIN_RESET); // DE=Low (receive mode)

  // Mengaktifkan interrupt Idle Line untuk UART1
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  // Memulai UART1 dalam mode DMA (menerima data Modbus ke rxBuf)
  HAL_UART_Receive_DMA(&huart1, rxBuf, RX_BUF_SIZE);

  PN532_HSU_init(&pn532_hsu, &huart2);
  PN532_init(&nfc, (PN532Interface *)&pn532_hsu);

  PN532_begin(&nfc);

  uint32_t version = PN532_getFirmwareVersion(&nfc);
  if (version) {
	  printf("Found PN5%02lx, firmware ver %lu.%lu\n",
			 (version >> 24) & 0xFF,
			 (version >> 16) & 0xFF,
			 (version >> 8) & 0xFF);
  } else {
	  printf("PN532 not found\n");
  }

  PN532_SAMConfig(&nfc);
  PN532_setPassiveActivationRetries(&nfc, 0xFF);

  uint8_t uid[7];
  uint8_t uidLen;

  printf("Waiting for an ISO14443A card\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (HAL_GetTick() - lastNfcTick > 1000) {
      lastNfcTick = HAL_GetTick();
      if (PN532_readPassiveTargetID(&nfc, PN532_MIFARE_ISO14443A, uid, &uidLen, 1000)) {
        holding_registers[0] = 1;  // kartu terdeteksi
        holding_registers[1] = ((uint16_t)uid[0] << 8) | uid[1];
        holding_registers[2] = ((uint16_t)uid[2] << 8) | uid[3];
      } else {
        holding_registers[0] = 0;  // tidak ada kartu
      }
    }

    if (HAL_GPIO_ReadPin(GPIOA, Emergency_Pin) == GPIO_PIN_SET)
    {
      HAL_GPIO_WritePin(GPIOB, RELAY_D_Pin, GPIO_PIN_SET);
    }
    else
    {
      HAL_GPIO_WritePin(GPIOB, RELAY_D_Pin, GPIO_PIN_RESET);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_EVEN;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RELAY1_Pin|RELAY_A_Pin|RELAY_B_Pin|RELAY_C_Pin 
                          |RELAY_D_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Emergency_Pin */
  GPIO_InitStruct.Pin = Emergency_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Emergency_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RELAY1_Pin RELAY_A_Pin RELAY_B_Pin RELAY_C_Pin 
                           RELAY_D_Pin */
  GPIO_InitStruct.Pin = RELAY1_Pin|RELAY_A_Pin|RELAY_B_Pin|RELAY_C_Pin 
                          |RELAY_D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RS485_DIR_Pin */
  GPIO_InitStruct.Pin = RS485_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(RS485_DIR_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
// Callback/ISR: Interrupt Handler UART1 (di-overwrite untuk tangani IDLE)


// Callback kustom: dipanggil saat frame UART lengkap diterima (terdeteksi idle line)
void UART_IdleCallback(void) {
    // Berhenti menerima DMA untuk memproses data yang sudah masuk
    HAL_UART_DMAStop(&huart1);
    // Hitung jumlah byte data yang diterima sebelum IDLE (panjang frame)
    uint16_t rxLen = RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
    if (rxLen == 0) {
        // Tidak ada data (keluar saja)
        HAL_UART_Receive_DMA(&huart1, rxBuf, RX_BUF_SIZE);
        return;
    }

    // ** Proses data Modbus yang diterima dalam rxBuf[0..rxLen-1] **
    if (rxLen >= 4 && rxBuf[0] == SLAVE_ID) {
        // Cek CRC frame (2 byte CRC terakhir dalam frame)
        if (rxLen >= 4) {  // minimal ada address, function, CRC dll.
            uint16_t recv_crc = rxBuf[rxLen-2] | (rxBuf[rxLen-1] << 8);
            uint16_t calc_crc = ModbusCRC(rxBuf, rxLen-2);  // hitung CRC dari data tanpa CRC field
            if (calc_crc == recv_crc) {
                // CRC valid dan ID cocok -> lanjut proses function code
                uint8_t func = rxBuf[1];
                // Siapkan buffer untuk respon (ukuran max 256, tapi frame kita pendek)
                uint8_t txBuf[16];
                uint16_t txLen = 0;
                // ** Handle Function Code 0x03: Read Holding Register **
                if (func == 0x03) {
					// Pastikan permintaan read register 0 sebanyak 1 register
					if (rxLen == 8) {
						uint16_t startAddr = (rxBuf[2] << 8) | rxBuf[3];
						uint16_t count = (rxBuf[4] << 8) | rxBuf[5];
                                                if (startAddr == 0 && count == 1) {
                                                        // Susun respon: [ID, 0x03, byte count, data_hi, data_lo, CRC_lo, CRC_hi]
                                                        uint16_t value = holdingReg0;  // nilai register 0 (status relay)
                                                        txBuf[0] = SLAVE_ID;
                                                        txBuf[1] = 0x03;
                                                        txBuf[2] = 2;          // byte count (2 bytes data)
                                                        txBuf[3] = (uint8_t)(value >> 8);
                                                        txBuf[4] = (uint8_t)(value & 0xFF);
                                                        // Hitung CRC respon untuk 5 bytes pertama (ID,FC,Count,DataHi,DataLo)
                                                        uint16_t resp_crc = ModbusCRC(txBuf, 5);
                                                        txBuf[5] = (uint8_t)(resp_crc & 0xFF);       // CRC Low
                                                        txBuf[6] = (uint8_t)((resp_crc >> 8) & 0xFF);// CRC High
                                                        txLen = 7;
                                                } else if (startAddr >= 1 && (startAddr + count) <= 4) {
                                                        // Membaca register NFC (register 1..3)
                                                        txBuf[0] = SLAVE_ID;
                                                        txBuf[1] = 0x03;
                                                        txBuf[2] = count * 2;
                                                        for (uint16_t i = 0; i < count; i++) {
                                                                uint16_t val = holding_registers[startAddr - 1 + i];
                                                                txBuf[3 + 2 * i] = (uint8_t)(val >> 8);
                                                                txBuf[4 + 2 * i] = (uint8_t)(val & 0xFF);
                                                        }
                                                        uint16_t resp_crc = ModbusCRC(txBuf, 3 + (count * 2));
                                                        txBuf[3 + (count * 2)] = (uint8_t)(resp_crc & 0xFF);
                                                        txBuf[4 + (count * 2)] = (uint8_t)((resp_crc >> 8) & 0xFF);
                                                        txLen = 5 + (count * 2);
                                                }
                                                // (Jika address/count tidak didukung, tidak mengirim respon - request diabaikan)
					}
				}
				// ** Handle Function Code 0x06: Write Single Register **
				else if (func == 0x06) {
					// Pastikan format frame benar untuk write single register (6 bytes payload)
					if (rxLen == 8) {
						uint16_t regAddr = (rxBuf[2] << 8) | rxBuf[3];
						uint16_t regValue = (rxBuf[4] << 8) | rxBuf[5];
						if (regAddr == 0) {
							// Hanya mendukung register 0
//                            if (regValue == 0x0000 || regValue == 0x0001) {
							if (regValue < 0x0008) {
								// Perbarui status relay
								holdingReg0 = regValue;
								if (holdingReg0 == 0x0001) {
									HAL_GPIO_WritePin(GPIOB, RELAY_A_Pin, GPIO_PIN_SET);   // Relay ON
								} else if (holdingReg0 == 0x0000){
									HAL_GPIO_WritePin(GPIOB, RELAY_A_Pin, GPIO_PIN_RESET); // Relay OFF
								}
								if (holdingReg0 == 0x0003) {
									HAL_GPIO_WritePin(GPIOB, RELAY_B_Pin, GPIO_PIN_SET);   // Relay ON
								} else if (holdingReg0 == 0x0002){
									HAL_GPIO_WritePin(GPIOB, RELAY_B_Pin, GPIO_PIN_RESET); // Relay OFF
								}
								if (holdingReg0 == 0x0005) {
									HAL_GPIO_WritePin(GPIOB, RELAY_C_Pin, GPIO_PIN_SET);   // Relay ON
								} else if (holdingReg0 == 0x0004){
									HAL_GPIO_WritePin(GPIOB, RELAY_C_Pin, GPIO_PIN_RESET); // Relay OFF
								}
								if (holdingReg0 == 0x0007) {
									HAL_GPIO_WritePin(GPIOB, RELAY_D_Pin, GPIO_PIN_SET);   // Relay ON
								} else if (holdingReg0 == 0x0006){
									HAL_GPIO_WritePin(GPIOB, RELAY_D_Pin, GPIO_PIN_RESET); // Relay OFF
								}
								// Susun respon sebagai echo dari permintaan: [ID, 0x06, Addr_Hi, Addr_Lo, Val_Hi, Val_Lo, CRC_lo, CRC_hi]
								memcpy(txBuf, rxBuf, 6);   // copy 6 byte pertama dari request
								uint16_t resp_crc = ModbusCRC(txBuf, 6);
								txBuf[6] = (uint8_t)(resp_crc & 0xFF);
								txBuf[7] = (uint8_t)((resp_crc >> 8) & 0xFF);
								txLen = 8;
							}
							// (Jika nilai di luar 0/1, diabaikan tanpa respon)
						}
					}
				}
                // TODO: (Opsional) Dapat ditambahkan handler untuk function code lain atau exception response

                // Jika ada respon yang perlu dikirim (txLen > 0), kirim via UART
                if (txLen > 0) {
                    // Aktifkan mode transmit RS485 (DE = High)
                    HAL_GPIO_WritePin(RS485_DIR_PORT, RS485_DIR_PIN, GPIO_PIN_SET);
                    // Transmisi data respon secara blocking (tunggu hingga terkirim semua)
                    HAL_UART_Transmit(&huart1, txBuf, txLen, HAL_MAX_DELAY);
                    // Tunggu hingga UART benar-benar selesai mengirim (Transfer Complete flag)
                    while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET) {
                        // loop tunggu TC=1
                    }
                    // Kembalikan transceiver ke mode terima (DE = Low)
                    HAL_GPIO_WritePin(RS485_DIR_PORT, RS485_DIR_PIN, GPIO_PIN_RESET);
                }
            }
            // Jika CRC tidak valid, frame diabaikan (tidak respon)
        }
    }
    // Restart DMA reception untuk frame berikutnya (persiapkan buffer lagi)
    HAL_UART_Receive_DMA(&huart1, rxBuf, RX_BUF_SIZE);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
