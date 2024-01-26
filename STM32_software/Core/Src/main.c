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
#include "data.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BP_ENFONCE 0
#define BP_RELACHE 1
#define BUTEE_DROITE 1630
#define BUTEE_GAUCHE 1160
#define DISTANCE_1_TOUR_AXE_TRANSMISSION_MM 79
#define ADRES_SRF10 0x70
#define ADDRESS_TF051 0x3C

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Variables globales
 * vitesse_mesuree_m_s : vitesse lineaire mesuree par la fourche optique
 * lectures_ADC[3] : les deux premières cases contiennent les données des capteurs infrarouges à l'arrière
 * roll : roulis de la voiture (EULER)
 * yaw : lacet (EULER)
 * pitch : tangage (EULER)
 * acc_x, acc_y : accélération liée à la gravité
 * lia_x, lia_y : accélération linéaire liée à l'accélération moteur
 * gyr_z : vitesse de rotation autour de l'axe z
 * distance_US : distance mesurée par le capteur ultrason ou sonar à l'arrière de la voiture
 * temp : température de l'IMU
 */

float vitesse_mesuree_m_s = 0;
uint16_t vitesse_mesuree_mm_s = 0;
uint32_t lectures_ADC[3];
uint8_t SPI_TxBuffer_data[12]={}; 	//buffer pour la transmission SPI
uint8_t SPI_RxBuffer[12] ={}; 		// Buffer pour la réception SPI
int16_t roll, yaw, pitch, distance_US, acc_x, acc_y, lia_x, lia_y, gyr_z;
uint8_t temp;
uint32_t drapeau = 0;
static u8g_t u8g;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t control = 0;
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

void displayInitializationMessage(u8g_t *u8g) {
    u8g_FirstPage(u8g);
    do {
        u8g_SetFont(u8g, u8g_font_profont12);
        u8g_DrawStr(u8g, 0, 24, "Sorbonne Bolide 1");
        u8g_DrawStr(u8g, 0, 36, "Initialisation OK");
    } while (u8g_NextPage(u8g));
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	uint32_t bp1,bp2,bp1_old=0,bp2_old=0;
	uint8_t donnees_Tx_i2c[8];
	uint8_t donnees_Rx_i2c[8];
	char text[50];
	enum State Init_state = OK;

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
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);


  // Buzzer starting sequence (Pirate des caraibes)
  buzzer_start();
  buzzer_gamme();

  // Initialization of the IMU sensor
  Init_state = bno055_init();

  if(Init_state == KO){
	  u8g_FirstPage(&u8g);
		  do {
			  u8g_SetFont(&u8g, u8g_font_profont12);
			  u8g_DrawStr(&u8g, 0, 24, "Sorbonne Bolide 1");
			  u8g_DrawStr(&u8g, 0, 36, "!!! Bad Init IMU !!!");
		     } while (u8g_NextPage(&u8g));
		  HAL_Delay(3000);
  	  }

  // Initialization of the screen
  u8g_InitComFn(&u8g, &u8g_dev_sh1106_128x64_i2c, u8g_com_arm_stm32_sh_i2c_fn);
  u8g_Begin(&u8g);

  if(Init_state == OK){
		u8g_FirstPage(&u8g);
		  do {
			  u8g_SetFont(&u8g, u8g_font_profont12);
			  u8g_DrawStr(&u8g, 0, 24, "Sorbonne Bolide 1");
			  u8g_DrawStr(&u8g, 0, 36, "Initialisation OK");
		  } while (u8g_NextPage(&u8g));
		HAL_Delay(3000);
       }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //démarrage de la conversion ADC des 3 canaux
	  HAL_ADC_Start_DMA(&hadc1, lectures_ADC, 3);

	  //lecture des boutons
	  bp1 = HAL_GPIO_ReadPin(BP1_GPIO_Port, BP1_Pin);
	  bp2 = HAL_GPIO_ReadPin(BP2_GPIO_Port, BP2_Pin);

	  //détection front descendant sur bp1
	  if((bp1 == BP_ENFONCE) && (bp1_old == BP_RELACHE))
	  {
		  buzzer_start_frequency_Hz(1760);
		  HAL_Delay(500);
		  buzzer_stop();
	  }

	  //détection front descendant sur bp2
	  if((bp2 == BP_ENFONCE) && (bp2_old == BP_RELACHE))
	  {
		  //changement d'état de la Led4
		  HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
	  }
	  //lecture ultrason
	donnees_Tx_i2c[0]=0x02;
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)ADRES_SRF10<<1, donnees_Tx_i2c, 1, 1000);
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
	HAL_I2C_Master_Receive(&hi2c1, (uint16_t)ADRES_SRF10<<1, donnees_Rx_i2c, 2, 1);
	distance_US = (uint16_t)(donnees_Rx_i2c[0]<<8) + donnees_Rx_i2c[1];

	  roll = bno055_lecture_16bits(EULER_ROLL_16bits);
	  yaw = bno055_lecture_16bits(EULER_YAW_16bits);
	  pitch = bno055_lecture_16bits(EULER_PITCH_16bits);

	  acc_x = bno055_lecture_16bits(ACC_X_16bits);
	  acc_y = bno055_lecture_16bits(ACC_Y_16bits);

	  lia_x = bno055_lecture_16bits(LIA_X_16bits);
	  lia_y = bno055_lecture_16bits(LIA_Y_16bits);

	  // si il n'y a pas eu de lecture de la vitesse récemment
	  if(drapeau==0)
	  {
		  vitesse_mesuree_mm_s = 0;
		  vitesse_mesuree_m_s = 0;
	  }
	  drapeau = 0;

	  u8g_FirstPage(&u8g);
		do {
			u8g_SetFont(&u8g, u8g_font_profont12);
			sprintf(text,"Sorbonne Bolide 1");
			u8g_DrawStr(&u8g, 0, 12,  text);
			sprintf(text,"vitesse %5u mm/s", (unsigned int)vitesse_mesuree_mm_s);
			u8g_DrawStr(&u8g, 0, 24,  text);
			sprintf(text,"yaw_RAW : %5d", yaw);
			u8g_DrawStr(&u8g, 0, 36,  text);
			//sprintf(text,"Sonar : %5d cm", distance_US);
			//u8g_DrawStr(&u8g, 0, 36,  text);
			sprintf(text,"IR gauche : %5u", (unsigned int)lectures_ADC[0]);
			u8g_DrawStr(&u8g, 0, 48,  text);
			sprintf(text,"IR droit : %5u", (unsigned int)lectures_ADC[1]);
			u8g_DrawStr(&u8g, 0, 60,  text);
		} while (u8g_NextPage(&u8g));

	  //Demande de lecture ultrason
	  donnees_Tx_i2c[0]=0x00;
	  donnees_Tx_i2c[1]=0x51;
	  while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
	  HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)ADRES_SRF10<<1, donnees_Tx_i2c, 2, 1000);

	  HAL_Delay(100);

	  //attente de la fin de la conversion ADC, si jamais ce n'est pas encore fini
	  //HAL_ADC_PollForConversion(&hadc1, 1);

	  //sauvegarde des valeurs de bp1 et bp2 pour la détection des fronts
	  bp1_old = bp1;
	  bp2_old = bp2;


	  /************* Communication SPI avec la Raspberry Pi *************/

	  /* Ici, on choisi les données capteurs qui vont être envoyées vers la Raspi
	  *  Il est possible de choisir différentes données parmis celles récupérées
	  *  Pour l'instant, les données envoyées sont : yaw, linear acceleration y (lia_y), vitesse, distance_US, IR_gauche et IR_droit
	  *  On découpe les données en 2 (ce sont des uint16_t et on a besoin d'uint8_t pour la communication SPI)
	  */

	  // Remplir le buffer de transmission avec les données des capteurs
	  //SPI_TxBuffer_data[0] = (uint8_t)(roll & 0xFF);           // Octet de poids faible de roll
	  //SPI_TxBuffer_data[1] = (uint8_t)((roll >> 8) & 0xFF);    // Octet de poids fort de roll

	  SPI_TxBuffer_data[0] = (uint8_t)(yaw & 0xFF);            // Octet de poids faible de yaw
	  SPI_TxBuffer_data[1] = (uint8_t)((yaw >> 8) & 0xFF);     // Octet de poids fort de yaw

	  //SPI_TxBuffer_data[4] = (uint8_t)(pitch & 0xFF);          // Octet de poids faible de pitch
	  //SPI_TxBuffer_data[5] = (uint8_t)((pitch >> 8) & 0xFF);   // Octet de poids fort de pitch

	  SPI_TxBuffer_data[2] = (uint8_t)(lia_y & 0xFF);    // Octet de poids faible de acceleration lineaire
	  SPI_TxBuffer_data[3] = (uint8_t)((lia_y >> 8) & 0xFF); // Octet de poids fort de acceleration lineaire

	  SPI_TxBuffer_data[4] = (uint8_t)(vitesse_mesuree_mm_s & 0xFF);    // Octet de poids faible de vitesse_mesuree_mm_s
	  SPI_TxBuffer_data[5] = (uint8_t)((vitesse_mesuree_mm_s >> 8) & 0xFF); // Octet de poids fort de vitesse_mesuree_mm_s

	  SPI_TxBuffer_data[6] = (uint8_t)(distance_US & 0xFF);    // Octet de poids faible de distance_US
	  SPI_TxBuffer_data[7] = (uint8_t)((distance_US >> 8) & 0xFF); // Octet de poids fort de distance_US

	  SPI_TxBuffer_data[8] = (uint8_t)(((uint16_t)(lectures_ADC[0])) & 0xFF);    // Octet de poids faible de l'IR gauche
	  SPI_TxBuffer_data[9] = (uint8_t)((((uint16_t)(lectures_ADC[0])) >> 8) & 0xFF); // Octet de poids fort de l'IR gauche

	  SPI_TxBuffer_data[10] = (uint8_t)(((uint16_t)(lectures_ADC[1])) & 0xFF);    // Octet de poids faible de l'IR droit
	  SPI_TxBuffer_data[11] = (uint8_t)((((uint16_t)(lectures_ADC[1])) >> 8) & 0xFF); // Octet de poids fort de l'IR droit


	  if(HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t *)SPI_TxBuffer_data, (uint8_t *)SPI_RxBuffer, 12) != HAL_OK)
	  	  {
		  	  printf("erreur SPI");
	  	  }



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
	static float coefficient_distance_par_intervalle_us = DISTANCE_1_TOUR_AXE_TRANSMISSION_MM *1000 / 16.0;
	static uint32_t somme_intervalles_us = 0;
	uint32_t nb_intervalles=0;
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin,1);
	mesure_us = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1); // ou TIM2->CCR1
	if((mesure_us-mesure_precedente_us) >= 300) //si mesure cohérente (pas un glitch):
	{
		if((mesure_us > (mesure_precedente_us+100000)) || ((mesure_us-100000) > mesure_precedente_us)) //cas d'un nouveau départ
		{
			for(indice=0;indice<16;indice++)
			{
				tableau_intervalles_us[indice] = 0;
			}
			indice=0;
		}
		else //cas où on tourne depuis plus d'un intervalle
		{
			drapeau = 1;
			tableau_intervalles_us[indice] = mesure_us - mesure_precedente_us; //on sauvegarde la nouvelle mesure dans le tableau
			//On fait une moyenne sur 10 ms au plus ou 16 valeurs.
			somme_intervalles_us = 0;
			i= indice;
			do{
				if(tableau_intervalles_us[i]==0)
						break;
				somme_intervalles_us += tableau_intervalles_us[i];
				i = (i - 1)%16;
				nb_intervalles++;
			}while ((somme_intervalles_us<100000) && (nb_intervalles < 16));
			indice = (indice+1)%16; // on incrémente l'indice avec retour à 0 pour indice = 16
			vitesse_mesuree_m_s = coefficient_distance_par_intervalle_us * nb_intervalles / somme_intervalles_us;
			vitesse_mesuree_mm_s = 1000*vitesse_mesuree_m_s;
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
