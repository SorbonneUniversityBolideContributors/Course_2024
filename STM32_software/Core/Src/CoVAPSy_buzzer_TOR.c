/*
 * CoVAPSy_buzzer.c
 *
 *  Created on: May 18, 2023
 *      Author: ajuton
 */


#include "CoVAPSy_buzzer.h"

//Periode max 65 535 µs => Frequence entre 15 et 1 MHz
void buzzer_start_frequency_Hz(float frequency_Hz){
	uint32_t periode_buzzer;
	periode_buzzer = FREQ_TIMER_6/frequency_Hz;
	HAL_TIM_Base_Stop_IT(&htim6);
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 31;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = periode_buzzer;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
	{
	    Error_Handler();
	}
	HAL_TIM_Base_Start_IT(&htim6);
}

void buzzer_start(void){

	HAL_TIM_Base_Start_IT(&htim6);
}

void buzzer_stop(void){
	HAL_TIM_Base_Stop_IT(&htim6);
}

void buzzer_gamme(void){

/*	buzzer_start_frequency_Hz(NOTE_DO3);
	HAL_Delay(200);
	buzzer_start_frequency_Hz(NOTE_RE3);
	HAL_Delay(200);
	buzzer_start_frequency_Hz(NOTE_MI3);
	HAL_Delay(200);
	buzzer_start_frequency_Hz(NOTE_FA3);
	HAL_Delay(200);
	buzzer_start_frequency_Hz(NOTE_SOL3);
	HAL_Delay(200);
	buzzer_start_frequency_Hz(NOTE_LA3);
	HAL_Delay(200);
	buzzer_start_frequency_Hz(NOTE_SI3);
	HAL_Delay(200);
	buzzer_start_frequency_Hz(NOTE_DO4);
	HAL_Delay(200);
	buzzer_stop();
*/
	buzzer_start_frequency_Hz(NOTE_E4);
	HAL_Delay(200);
	buzzer_start_frequency_Hz(NOTE_G4);
	HAL_Delay(200);
	buzzer_start_frequency_Hz(NOTE_A4);
	HAL_Delay(200);
	buzzer_start_frequency_Hz(NOTE_A4);
	HAL_Delay(200);
	buzzer_start_frequency_Hz(0);
	HAL_Delay(200);
	buzzer_start_frequency_Hz(NOTE_A4);
	HAL_Delay(200);
	buzzer_start_frequency_Hz(NOTE_B4);
	HAL_Delay(200);
	buzzer_start_frequency_Hz(NOTE_C5);
	HAL_Delay(200);
	buzzer_start_frequency_Hz(NOTE_C5);
	HAL_Delay(200);
	buzzer_start_frequency_Hz(0);
	HAL_Delay(200);
	buzzer_start_frequency_Hz(NOTE_C5);
	HAL_Delay(200);
	buzzer_start_frequency_Hz(NOTE_D5);
	HAL_Delay(200);
	buzzer_start_frequency_Hz(NOTE_B4);
	HAL_Delay(200);
	buzzer_start_frequency_Hz(NOTE_B4);
	HAL_Delay(200);
	buzzer_start_frequency_Hz(0);
	HAL_Delay(200);
	buzzer_start_frequency_Hz(NOTE_A4);
	HAL_Delay(200);
	buzzer_start_frequency_Hz(NOTE_G4);
	HAL_Delay(200);
	buzzer_start_frequency_Hz(NOTE_A4);
	HAL_Delay(200);
	buzzer_start_frequency_Hz(0);
	HAL_Delay(200);
/*
	buzzer_start_frequency_Hz(NOTE_E4);
		HAL_Delay(200);
		buzzer_start_frequency_Hz(NOTE_G4);
		HAL_Delay(200);
		buzzer_start_frequency_Hz(NOTE_A4);
		HAL_Delay(200);
		buzzer_start_frequency_Hz(NOTE_A4);
		HAL_Delay(200);
		buzzer_start_frequency_Hz(0);
		HAL_Delay(200);
		buzzer_start_frequency_Hz(NOTE_A4);
		HAL_Delay(200);
		buzzer_start_frequency_Hz(NOTE_B4);
		HAL_Delay(200);
		buzzer_start_frequency_Hz(NOTE_C5);
		HAL_Delay(200);
		buzzer_start_frequency_Hz(NOTE_C5);
		HAL_Delay(200);
		buzzer_start_frequency_Hz(0);
		HAL_Delay(200);
		buzzer_start_frequency_Hz(NOTE_C5);
		HAL_Delay(200);
		buzzer_start_frequency_Hz(NOTE_D5);
		HAL_Delay(200);
		buzzer_start_frequency_Hz(NOTE_B4);
		HAL_Delay(200);
		buzzer_start_frequency_Hz(NOTE_B4);
		HAL_Delay(200);
		buzzer_start_frequency_Hz(0);
		HAL_Delay(200);
		buzzer_start_frequency_Hz(NOTE_A4);
		HAL_Delay(200);
		buzzer_start_frequency_Hz(NOTE_G4);
		HAL_Delay(200);
		buzzer_start_frequency_Hz(NOTE_A4);
		HAL_Delay(200);
		buzzer_start_frequency_Hz(0);
		HAL_Delay(200);

		buzzer_start_frequency_Hz(NOTE_E4);
			HAL_Delay(200);
			buzzer_start_frequency_Hz(NOTE_G4);
			HAL_Delay(200);
			buzzer_start_frequency_Hz(NOTE_A4);
			HAL_Delay(200);
			buzzer_start_frequency_Hz(NOTE_A4);
			HAL_Delay(200);
			buzzer_start_frequency_Hz(0);
			HAL_Delay(200);
			buzzer_start_frequency_Hz(NOTE_A4);
			HAL_Delay(200);
			buzzer_start_frequency_Hz(NOTE_C5);
			HAL_Delay(200);
			buzzer_start_frequency_Hz(NOTE_D5);
			HAL_Delay(200);
			buzzer_start_frequency_Hz(NOTE_D5);
			HAL_Delay(200);
			buzzer_start_frequency_Hz(0);
			HAL_Delay(200);
			buzzer_start_frequency_Hz(NOTE_D5);
			HAL_Delay(200);
			buzzer_start_frequency_Hz(NOTE_E5);
			HAL_Delay(200);
			buzzer_start_frequency_Hz(NOTE_F5);
			HAL_Delay(200);
			buzzer_start_frequency_Hz(NOTE_F5);
			HAL_Delay(200);
			buzzer_start_frequency_Hz(0);
			HAL_Delay(200);
			buzzer_start_frequency_Hz(NOTE_E5);
			HAL_Delay(200);
			buzzer_start_frequency_Hz(NOTE_D5);
			HAL_Delay(200);
			buzzer_start_frequency_Hz(NOTE_E5);
			HAL_Delay(200);
			buzzer_start_frequency_Hz(NOTE_A4);
			HAL_Delay(200);
			buzzer_start_frequency_Hz(0);
			HAL_Delay(200);

			buzzer_start_frequency_Hz(NOTE_A4);
				HAL_Delay(200);
				buzzer_start_frequency_Hz(NOTE_B4);
				HAL_Delay(200);
				buzzer_start_frequency_Hz(NOTE_C5);
				HAL_Delay(200);
				buzzer_start_frequency_Hz(NOTE_C5);
				HAL_Delay(200);
				buzzer_start_frequency_Hz(0);
				HAL_Delay(200);
				buzzer_start_frequency_Hz(NOTE_D5);
				HAL_Delay(200);
				buzzer_start_frequency_Hz(NOTE_E5);
				HAL_Delay(200);
				buzzer_start_frequency_Hz(NOTE_A4);
				HAL_Delay(200);
				buzzer_start_frequency_Hz(0);
				HAL_Delay(200);
				buzzer_start_frequency_Hz(NOTE_A4);
				HAL_Delay(200);
				buzzer_start_frequency_Hz(NOTE_C5);
				HAL_Delay(200);
				buzzer_start_frequency_Hz(NOTE_B4);
				HAL_Delay(200);
				buzzer_start_frequency_Hz(NOTE_B4);
				HAL_Delay(200);
				buzzer_start_frequency_Hz(0);
				HAL_Delay(200);
				buzzer_start_frequency_Hz(NOTE_C5);
				HAL_Delay(200);
				buzzer_start_frequency_Hz(NOTE_A4);
				HAL_Delay(200);
				buzzer_start_frequency_Hz(NOTE_B4);
				HAL_Delay(200);
				buzzer_start_frequency_Hz(0);
				HAL_Delay(200);
*/
	buzzer_stop();

}
