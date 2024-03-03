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
	buzzer_start_frequency_Hz(NOTE_FA3);
}
