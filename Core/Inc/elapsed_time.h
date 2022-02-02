/*
 * elapsed_time.h
 *
 *  Created on: 18 мая 2021 г.
 *      Author: Arthur Sayfutdinov
 */

#ifndef INC_ELAPSED_TIME_H_
#define INC_ELAPSED_TIME_H_

#define ELAPSED_TIME_MEAS_TIMER1	TIM3
#define ELAPSED_TIME_MEAS_TIMER2	TIM6

#define Tic1()						ELAPSED_TIME_MEAS_TIMER1->CNT=0
#define Tic2()						ELAPSED_TIME_MEAS_TIMER2->CNT=0

#define Toc1()						ELAPSED_TIME_MEAS_TIMER1->CNT
#define Toc2()						ELAPSED_TIME_MEAS_TIMER2->CNT

void Elapsed_Time_Init(void);
#endif /* INC_ELAPSED_TIME_H_ */
