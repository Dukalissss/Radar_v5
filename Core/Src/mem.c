/*
 * mem.c
 *
 *  Created on: Jan 31, 2022
 *      Author: Dukalis
 */

#include "main.h"

uint32_t FirstPage = 0, NbOfPages = 0;
FLASH_EraseInitTypeDef EraseInitStruct = {0};
uint32_t flAddress = 0, PageError = 0;
uint32_t data32 = 0;


void load_variables (char *str2,long lens,uint32_t address)
{
    uint16_t *t;
    t=(uint16_t *)str2;
    flAddress=address;

    while (flAddress < address+lens)
    {
        data32 = *(__IO uint32_t *)flAddress;
        *t++=(uint16_t)(data32&0x0000FFFF);
        *t++=(uint16_t)((data32 >> 16)&0x0000FFFF);
        flAddress = flAddress + 4;
    }
}
uint32_t GetPage(uint32_t address)
{
  return ((address-0x08000000)/FLASH_PAGE_SIZE);
}
void write_variables (char *wrstr,int lens, uint32_t address)
{
    uint64_t    DATA_64=0;
    uint16_t *t;
    uint16_t data_count=0;
    t=(uint16_t *)wrstr;
    HAL_FLASH_Unlock();
    FirstPage = GetPage(address);
    /* Get the number of pages to erase from 1st page */
    /* Fill EraseInit structure*/
    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Page        = FirstPage;
    EraseInitStruct.NbPages     = 1;
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
    {
        /*
          Error occurred while page erase.
          User can add here some code to deal with this error.
          PageError will contain the faulty page and then to know the code error on this page,
          user can call function 'HAL_FLASH_GetError()'
        */
        /* Infinite loop */
        while (1)
        {

        }
    }

    flAddress = address;
    while (flAddress < address+lens)
    {
        DATA_64=(uint64_t)(t[data_count++]);
        DATA_64|=(uint64_t)(t[data_count++]) << 16;
        DATA_64|=(uint64_t)(t[data_count++]) << 32;
        DATA_64|=(uint64_t)(t[data_count++]) << 48;

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, flAddress, DATA_64) == HAL_OK)
        {
            flAddress = flAddress + 8;
        }
        else
        {
            /* Error occurred while writing data in Flash memory.
            User can add here some code to deal with this error */
            while (1)
            {
            }
        }
    }
    HAL_FLASH_Lock();
}
