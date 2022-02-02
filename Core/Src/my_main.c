/*
 * my_main.c
 *
 *  Created on: Jan 31, 2022
 *      Author: Dukalis
 */
#include "main.h"

uint8_t t1_ms = 0;

void my_main(void)
{
  RS485_Init();
  Elapsed_Time_Init();
  while (1)
  {
    if (t1_ms)
    {
      t1_ms = 0;
      Configurator_Serv();
      Distance_Detector_Serv();
    }
  }

}

