#include "main.h"

RS485_STRUC rs485;

void RS485_Init(void)
{
  LL_GPIO_ResetOutputPin(dir_485_GPIO_Port, dir_485_Pin);
  LL_USART_EnableIT_RXNE(USART_RS485);
  rs485.ready=1;
}
void RS485_Systick_Handler(void)
{
  if(rs485.delay) 
  {
    rs485.delay--;
    if (rs485.delay==1)
      LL_GPIO_SetOutputPin(dir_485_GPIO_Port, dir_485_Pin);
    if (!rs485.delay) 
    {
      rs485.ready=0;
      LL_USART_EnableIT_TXE(USART_RS485);
    }
  }
}
void RS485_UART_Handler(void)
{
  if ((LL_USART_IsActiveFlag_RXNE(USART_RS485))&&(LL_USART_IsEnabledIT_RXNE(USART_RS485)))
  {
    //записываем полученный байт в массив buf_485_in
    rs485.buf_in[rs485.rx_index]=LL_USART_ReceiveData8(USART_RS485);
    //принимаемые байты записываются в массив buf_485_in по кругу
    if (++rs485.rx_index == RS485_BUF_SIZE) rs485.rx_index=0;    
  }
  if ((LL_USART_IsActiveFlag_TXE(USART_RS485))&&(LL_USART_IsEnabledIT_TXE(USART_RS485)))
  {
    if (rs485.ptr_out_tx2<rs485.len_tx_485)
    {
      LL_USART_TransmitData8(USART_RS485, *rs485.tx_485++);
      if (++rs485.ptr_out_tx2==rs485.len_tx_485)
      {
        /* Disable TXE interrupt */
        LL_USART_DisableIT_TXE(USART_RS485);

        /* Enable TC interrupt */
        LL_USART_EnableIT_TC(USART_RS485);
      }
    }    
  }
  if ((LL_USART_IsActiveFlag_TC(USART_RS485))&&(LL_USART_IsEnabledIT_TC(USART_RS485)))
  {
	    /* Clear TC flag */
	    LL_USART_ClearFlag_TC(USART_RS485);
	    /* Call function in charge of handling end of transmission of sent character
	       and prepare next charcater transmission */
	    //устанавливаем PB5 в логический ноль
	    //это режим приема]
	    LL_GPIO_ResetOutputPin(dir_485_GPIO_Port, dir_485_Pin);
	    LL_USART_DisableIT_TC(USART_RS485);
	    rs485.ready=1;    
  }
  USART_RS485->ICR=0xFFFFFFFF;
}
void pusk_485(uint8_t *str_485, uint16_t lenn)
{
	while (!rs485.ready) {};
	rs485.ready=0;
	rs485.ptr_out_tx2=0; //счетчик отправленных байтов
	rs485.tx_485=(uint8_t*)str_485; //отправляемая строка
	rs485.len_tx_485=lenn; //ее длина
	rs485.delay=6; //задержка перед отправкой (мс)
}
