#include "stdint.h"

#define USART_RS485     USART1
#define RS485_BUF_SIZE  300

typedef struct
{
  uint8_t ready;
  uint8_t buf_in[RS485_BUF_SIZE];
  uint8_t buf_out[RS485_BUF_SIZE];
  uint16_t ptr_out_tx2;
  uint16_t len_tx_485;
  uint16_t rx_index;
  uint8_t *tx_485;
  uint8_t delay;
} RS485_STRUC;

void RS485_UART_Handler(void);
void RS485_Systick_Handler(void);
void pusk_485(uint8_t *str_485, uint16_t lenn);
void RS485_Init(void);

extern RS485_STRUC rs485;

