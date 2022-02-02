/*
 * elapsed_time.c
 *
 *  Created on: 18 мая 2021 г.
 *      Author: Arthur Sayfutdinov
 */
#include "main.h"


void Elapsed_Time_Init(void)
{
	//таймеры измерения времени
	LL_TIM_EnableCounter(ELAPSED_TIME_MEAS_TIMER1);
	LL_TIM_EnableCounter(ELAPSED_TIME_MEAS_TIMER2);

}
