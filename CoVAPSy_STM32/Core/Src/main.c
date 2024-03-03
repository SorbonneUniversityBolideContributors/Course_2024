/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CoVAPSy_buzzer.h"
#include "CoVAPSy_bno055.h"
#include "u8g.h"
#include "stdio.h"
#include <string.h>
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DISTANCE_1_TOUR_AXE_TRANSMISSION_MM 79.748
#define ADRES_SRF10 0x70
#define ADDRESS_TF051 0x3C
#define SPEED_DELTAT_THRESHOLD_MS 26
#define US_DELTAT_THRESHOLD_MS 4
#define US_SMOOTHING 0.1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t vitesse_mesuree_mm_s = 0;
uint32_t lectures_ADC[3];
uint8_t SPI_TxBuffer[16] = {0x35,0,0,0,0,0,0,0,0,0,0,0,0x55,0x55,0x55,0x55}; 	//buffer pour la transmission SPI
uint8_t SPI_RxBuffer[16] ={}; 					// Buffer pour la réception SPI
uint16_t yaw, distance_US_cm, US_value;
uint32_t mesure_vitesse_ms=0, mesure_US_ticks=0;
uint32_t elapsed_time_speed_ms, elapsed_time_us_ms;

static u8g_t u8g;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void CRC_Init(void) {

	//Enable CRC clock, then reset CRC buffer.
    __HAL_RCC_CRC_CLK_ENABLE();
    CRC->CR |= CRC_CR_RESET;
}


void SRF10_Init(void) {
	//Set correct ranges.
	uint8_t donnees_Tx_i2c[2];
	donnees_Tx_i2c[0]=0x02;
	donnees_Tx_i2c[1]=0xB;
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)ADRES_SRF10<<1, donnees_Tx_i2c, 2, 1000); // HAL should really be replaced by direct register access.
//	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
//	HAL_I2C_Master_Receive(&hi2c1, (uint16_t)ADRES_SRF10<<1, donnees_Rx_i2c, 1, 1);

	donnees_Tx_i2c[0]=0x01;
	donnees_Tx_i2c[1]=0x09;
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)ADRES_SRF10<<1, donnees_Tx_i2c, 2, 1000); // HAL should really be replaced by direct register access.
//	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
//	HAL_I2C_Master_Receive(&hi2c1, (uint16_t)ADRES_SRF10<<1, donnees_Rx_i2c, 1, 1);

}




uint8_t control =0;
void u8g_Delay(uint16_t val) {
    HAL_Delay(val);
}
void u8g_xMicroDelay(uint16_t val) {
    static uint32_t i, j;
    static uint32_t freq;
    freq = HAL_RCC_GetSysClockFreq() / 1000000;

    for (i = 0; i < val;) {
        for (j = 0; j < freq; ++j) {
            ++j;
        }
        ++i;
    }
}
void u8g_MicroDelay(void) {
    u8g_xMicroDelay(1);
}
void u8g_10MicroDelay(void) {
    u8g_xMicroDelay(10);
}

uint8_t u8g_com_arm_stm32_sh_i2c_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr) {
    switch (msg) {
    case U8G_COM_MSG_STOP:
        break;

    case U8G_COM_MSG_INIT:
        u8g_MicroDelay();
        break;

    case U8G_COM_MSG_ADDRESS:
        if (arg_val == 0) {
            control = 0;
        } else {
            control = 0x40;
        }
        u8g_10MicroDelay();
        break;

    case U8G_COM_MSG_WRITE_BYTE: {
        HAL_I2C_Mem_Write(&hi2c1, ADDRESS_TF051<<1, control, 1, &arg_val, 1, 10000);
    }
        break;

    case U8G_COM_MSG_WRITE_SEQ:
    case U8G_COM_MSG_WRITE_SEQ_P: {
        HAL_I2C_Mem_Write(&hi2c1, ADDRESS_TF051<<1, control, 1, arg_ptr, arg_val, 10000);
    }

        break;
    }
    return 1;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//	uint32_t bp1,bp2,bp1_old=0,bp2_old=0;
	uint8_t donnees_Tx_i2c[2];
	uint8_t donnees_Rx_i2c[2];
//	char text[50];
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
  MX_ADC1_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();



//  MX_CRC_Init();


  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Base_Start(&htim7);



  buzzer_start();
  buzzer_gamme();

  CRC_Init();




  HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
  HAL_Delay(10);
  HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);


  bno055_init();
  SRF10_Init();

  u8g_InitComFn(&u8g, &u8g_dev_sh1106_128x64_i2c, u8g_com_arm_stm32_sh_i2c_fn);
  u8g_Begin(&u8g);

  u8g_FirstPage(&u8g);
  		  do {
  			  u8g_SetFont(&u8g, u8g_font_profont29);
  			  u8g_DrawStr(&u8g, 0, 29, "SORBONNE");
  			  u8g_DrawStr(&u8g, 0, 59, "BOLIDE 1"); 
          // u8g_DrawStr(&u8g, 0, 59, "BOLIDE 2"); 
  } while (u8g_NextPage(&u8g));

  uint16_t number = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  buzzer_stop();
  while (1)
  {
	  //démarrage de la conversion ADC des 3 canaux
	  HAL_ADC_Start_DMA(&hadc1, lectures_ADC, 3);

	  //lecture ultrason
	  elapsed_time_us_ms = HAL_GetTick() - mesure_US_ticks;
	  if (elapsed_time_us_ms > US_DELTAT_THRESHOLD_MS) {
		  HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
		  donnees_Tx_i2c[0]=0x02;
		  while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
		  HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)ADRES_SRF10<<1, donnees_Tx_i2c, 1, 1000); // HAL should really be replaced by direct register access.
		  while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
		  HAL_I2C_Master_Receive(&hi2c1, (uint16_t)ADRES_SRF10<<1, donnees_Rx_i2c, 2, 1);
		  if (donnees_Rx_i2c[0] != 0x06) {
			  US_value = (uint16_t)(donnees_Rx_i2c[0]<<8) + donnees_Rx_i2c[1];
			  distance_US_cm = US_value * US_SMOOTHING + (1 - US_SMOOTHING) * distance_US_cm;

		  }
	  }

	  yaw = bno055_lecture_16bits(EULER_HEADING_16bits);


	  // Using SysTick instead of a Hardware timer because we don't need sub ms accuracy.

	  //Change the SPEED_DELTAT_THRESHOLD_MS to a higher value if you want to detect lower speeds.
	  //Default value of 26 ms is enough to detect lowest powered reverse speed.
	  //BTW, the speed is not signed, so you need to take into account the fact that the car can move backwards.



	  elapsed_time_speed_ms = HAL_GetTick() - mesure_vitesse_ms;
	  if(elapsed_time_speed_ms > SPEED_DELTAT_THRESHOLD_MS)
	  {
		  vitesse_mesuree_mm_s = 0;
	  }






	  //Demande de lecture ultrason
	  if (elapsed_time_us_ms > US_DELTAT_THRESHOLD_MS) {
		  	  donnees_Tx_i2c[0]=0x00;
		  	  donnees_Tx_i2c[1]=0x51;
		  	  while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
		  	  HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)ADRES_SRF10<<1, donnees_Tx_i2c, 2, 1000);
		  	  mesure_US_ticks = HAL_GetTick();
	  }


	  //attente de la fin de la conversion ADC, si jamais ce n'est pas encore fini
	  HAL_ADC_PollForConversion(&hadc1, 1);

	  number++;


	  //Reset CRC;
	  CRC->CR = 1;


	  SPI_TxBuffer[0] = (uint8_t)((lectures_ADC[2] >> 8) & 0xFF);
	  SPI_TxBuffer[1] = (uint8_t)(lectures_ADC[2] & 0xFF); //battery_voltage (unit ?)
	  SPI_TxBuffer[2] = (uint8_t)((yaw >> 8) & 0xFF);
	  SPI_TxBuffer[3] = (uint8_t)(yaw & 0xFF);

	  CRC->DR = (uint32_t)((((uint16_t) lectures_ADC[2] << 16)) | ((uint16_t) yaw));


	  SPI_TxBuffer[4] = (uint8_t)((((uint16_t)(lectures_ADC[0])) >> 8) & 0xFF);
	  SPI_TxBuffer[5] = (uint8_t)(((uint16_t)(lectures_ADC[0])) & 0xFF);    // Octet de poids faible de l'IR gauche
	  SPI_TxBuffer[6] = (uint8_t)((((uint16_t)(lectures_ADC[1])) >> 8) & 0xFF);
	  SPI_TxBuffer[7] = (uint8_t)(((uint16_t)(lectures_ADC[1])) & 0xFF);    // Octet de poids faible de l'IR droit

	  CRC->DR = (uint32_t)(((uint16_t) lectures_ADC[0] << 16) | ((uint16_t) lectures_ADC[1])); //Put 4 next bytes in the CRC register


	  SPI_TxBuffer[8] = (uint8_t)((vitesse_mesuree_mm_s >> 8) & 0xFF); // Octet de poids fort de vitesse_mesuree_mm_s
	  SPI_TxBuffer[9] = (uint8_t)(vitesse_mesuree_mm_s & 0xFF);    // Octet de poids faible de vitesse_mesuree_mm_s
	  SPI_TxBuffer[10] = (uint8_t)((distance_US_cm >> 8) & 0xFF);
	  SPI_TxBuffer[11] = (uint8_t)((distance_US_cm &0xFF));

	  CRC->DR = (uint32_t)(((uint16_t) vitesse_mesuree_mm_s << 16) | ((uint16_t) distance_US_cm));


	  uint32_t checksum = CRC->DR; //Read from register to get computer value

	  //Send the checksum over SPI.
	  SPI_TxBuffer[15] = (uint8_t)(checksum & 0xFF);
	  SPI_TxBuffer[14] = (uint8_t)((checksum >> 8) & 0xFF);
	  SPI_TxBuffer[13] = (uint8_t)((checksum >> 16) & 0xFF);
	  SPI_TxBuffer[12] = (uint8_t)((checksum >> 24) & 0xFF);




	  HAL_SPI_Transmit(&hspi3, (uint8_t *)SPI_TxBuffer, 16,10);
	  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);

	  /**
	  ******************************************************************************
	  * THE FOLLOWING BIT OF CODE IS NECESSARY TO FLUSH THE FIFO BUFFER
	  * THIS IS REALLY LOW LEVEL STUFF, AND IT TOOK ME AGES TO FIGURE OUT
	  * DO NOT DELETE THIS IF YOU DON'T HAVE AN ACTUAL SOLUTION, REALLY,
	  * YOU'RE GOING TO PULL YOUR HAIR OUT TRYING TO FIX THE MESS IT WILL
	  * INEVITABLY CREATE. PROCEED AT YOUR OWN RISK
	  ******************************************************************************
	  */
	  __HAL_RCC_SPI3_FORCE_RESET();
	  __HAL_RCC_SPI3_RELEASE_RESET();
	  MX_SPI3_Init();


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim)
{
	static uint32_t mesure_precedente_us=0, mesure_us, indice=0, i;
	static uint32_t tableau_intervalles_us[16]={};
	static float coefficient_distance_par_intervalle_um = DISTANCE_1_TOUR_AXE_TRANSMISSION_MM *1000 / 16.0;
	static uint32_t somme_intervalles_us = 0;
	uint32_t nb_intervalles=0;
	//	HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
	mesure_us = TIM2->CCR1; // ou TIM2->CCR1
	mesure_vitesse_ms = HAL_GetTick();
	uint32_t delta_us = mesure_us-mesure_precedente_us;

	if((delta_us) >= 300) //si mesure cohérente (pas un glitch):
	{
		if((mesure_us > (mesure_precedente_us+100000)) || ((mesure_us-100000) > mesure_precedente_us)) //cas d'un nouveau départ (the second term is for overflow)
		{
			memset(tableau_intervalles_us, 0, sizeof(tableau_intervalles_us));
			indice=0;
		}
		else //cas où on tourne depuis plus d'un intervalle
		{
			tableau_intervalles_us[indice] = delta_us; //on sauvegarde la nouvelle mesure dans le tableau
			//On fait une moyenne sur 10 ms au plus ou 16 valeurs.
			somme_intervalles_us = 0;
			i= indice;

			do{
				if(!tableau_intervalles_us[i])
						break;
				somme_intervalles_us += tableau_intervalles_us[i];

				i--;
				if (i < 0) i = 15;
				nb_intervalles++;

			}while ((somme_intervalles_us<10000) && (nb_intervalles < 16));

			indice = (indice+1)%16; // on incrémente l'indice avec retour à 0 pour indice = 16

			vitesse_mesuree_mm_s = 1000 * coefficient_distance_par_intervalle_um * nb_intervalles / somme_intervalles_us;
		}
		mesure_precedente_us = mesure_us;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
