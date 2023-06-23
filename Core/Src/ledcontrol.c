/*
 * ledcontrol.c
 *
 *  Created on: Jun 23, 2023
 *      Author: michael_uman
 */

#include <stdbool.h>
#include "main.h"
#include "cmsis_os.h"
#include "ledcontrol.h"

extern volatile bool fadeInProgress;
extern TIM_HandleTypeDef htim2; // pwm timer
extern TIM_HandleTypeDef htim4; // fade timer
extern volatile int16_t ledDuty;
extern volatile int16_t inc;
extern volatile bool fadeInProgress;

void ledFullOn()
{
	  htim2.Instance->CCR1 = 1000;
}

void ledFullOff()
{
	  htim2.Instance->CCR1 = 0;
}

void beginLedFadeOut()
{
	ledDuty = 1000;
	inc = -20;
	htim2.Instance->CCR1 = ledDuty;
	ledDuty += inc;
	fadeInProgress = true;
	HAL_TIM_Base_Start_IT(&htim4);
}

void beginLedFadeIn()
{
	ledDuty = 0;
	inc = 20;
	htim2.Instance->CCR1 = ledDuty;
	ledDuty += inc;
	fadeInProgress = true;
	HAL_TIM_Base_Start_IT(&htim4);
}
