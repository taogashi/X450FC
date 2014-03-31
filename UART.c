#include "UART.h"
#include "stm32f4xx.h"

#define UART_TX_BUFFER_LENGTH 100
/*
 *
 */
struct
{
	char buffer[UART_TX_BUFFER_LENGTH];
	u16 header;
	u16 tail;
}uart_user_tx_buffer;

char uart_dma_buffer[UART_TX_BUFFER_LENGTH];
u8 DMA_Transmit_Complete = 1;

void USART3_Config(void) //串口设置
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

//--------------------------------USART3----------------------------------------------
	/* Configure USART3 Tx (PD8) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
   /* Configure USART3 Rx (PD9) as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

   /* Connect PXx to USARTx_Tx   */
   GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
 
   /* Connect PXx to USARTx_Rx*/
   GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);

	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx;
	
	USART_Init(USART3,&USART_InitStructure);
	/* Enable USART2 */
	USART_Cmd(USART3, ENABLE);		
}

void USART3_IT_Config(void)//串口中断设置，可选
{
	NVIC_InitTypeDef NVIC_InitStructure;

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); /*接收中断使能*/  // |

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; 	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);
}

#if USE_USART1
/************************************************************************************************************/
/*                                    USART1 DMA 															*/
/************************************************************************************************************/
void USART1_DMA_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	/* Configure USART1 Tx (PA9) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* Configure USART1 Rx (PA10) as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Connect PXx to USARTx_Tx   */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	/* Connect PXx to USARTx_Rx*/
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	USART_DeInit(USART1);

	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	
	USART_Init(USART1,&USART_InitStructure);
	
	DMA_DeInit(DMA2_Stream7);

	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = ((uint32_t)USART1 + 0x04);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)uart_dma_buffer;	//designate later
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = (uint16_t)10;	//designate later
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	
	DMA_Init(DMA2_Stream7, &DMA_InitStructure);
	
	/* Enable USART1 */
	USART_Cmd(USART1, ENABLE);	
	
	uart_user_tx_buffer.header = 0;
	uart_user_tx_buffer.tail = 0;
}

void USART1_DMA_IT_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); /*接收中断使能*/  // |

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; 	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);

	DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

#else

/************************************************************************************************************/
/*                                    USART2 DMA 															*/
/************************************************************************************************************/
void USART2_DMA_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	/* Configure USART2 Tx (PD5) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	/* Configure USART2 Rx (PD6) as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* Connect PXx to USARTx_Tx   */
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
	/* Connect PXx to USARTx_Rx*/
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);

	USART_DeInit(USART2);

	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	
	USART_Init(USART2,&USART_InitStructure);
	
	DMA_DeInit(DMA1_Stream6);

	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = ((uint32_t)USART2 + 0x04);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)uart_dma_buffer;	//designate later
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = (uint16_t)10;	//designate later
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	
	DMA_Init(DMA1_Stream6, &DMA_InitStructure);
	
	/* Enable USART2 */
	USART_Cmd(USART2, ENABLE);	
	
	uart_user_tx_buffer.header = 0;
	uart_user_tx_buffer.tail = 0;
}

void USART2_DMA_IT_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); /*接收中断使能*/  // |

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; 	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);

	DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
#endif

u8 UartSend(char* content, u16 num)
{
	s16 occupied_buffer;
	u16 total_byte;
	
	u16 i;
	
	occupied_buffer = uart_user_tx_buffer.header - uart_user_tx_buffer.tail;
	if(occupied_buffer < 0)
		occupied_buffer += UART_TX_BUFFER_LENGTH;
	total_byte = occupied_buffer + num;
	
	/*if DMA is ready, put data to DMA buffer anyway*/
	if(DMA_Transmit_Complete == 1)
	{
		/* Disable DMA stream
		 * this step is neccessary
		 * DMA stream must be disabled before calling DMA_SetCurrDataCounter()
		 * ref@ 
		 */
#if USE_USART1
		DMA_Cmd(DMA2_Stream7, DISABLE);
#else
		DMA_Cmd(DMA1_Stream6, DISABLE);
#endif
		DMA_Transmit_Complete = 0;
		
		/*sum of new content and content in the buffer is less than UART_TX_BUFFER_LENGTH*/
		if(total_byte <= UART_TX_BUFFER_LENGTH)
		{
			/*all data can be put into dma buffer*/
			for(i=0; i<occupied_buffer; i++)
			{
				uart_dma_buffer[i] = uart_user_tx_buffer.buffer[uart_user_tx_buffer.tail++];
				if(uart_user_tx_buffer.tail >= UART_TX_BUFFER_LENGTH)
					uart_user_tx_buffer.tail = 0;
			}
			for(i=0; i<num; i++)
			{
				uart_dma_buffer[i+occupied_buffer] = content[i];
			}
			/*buffer has been clear*/
			uart_user_tx_buffer.header = 0;
			uart_user_tx_buffer.tail = 0;
			
			/*set DMA transmit byte count*/
#if USE_USART1
			DMA_SetCurrDataCounter(DMA2_Stream7, (uint16_t)total_byte);
#else
			DMA_SetCurrDataCounter(DMA1_Stream6, (uint16_t)total_byte);
#endif
		}	
		/*put data to DMA buffer as much as we can, the rest into uart2 buffer*/
		else
		{
			for(i=0; i<occupied_buffer; i++)
			{
				uart_dma_buffer[i] = uart_user_tx_buffer.buffer[uart_user_tx_buffer.tail++];
				if(uart_user_tx_buffer.tail >= UART_TX_BUFFER_LENGTH)
					uart_user_tx_buffer.tail = 0;
			}
			for(i=0; i<UART_TX_BUFFER_LENGTH-occupied_buffer; i++)
			{
				uart_dma_buffer[i+occupied_buffer] = content[i];
			}
			uart_user_tx_buffer.header = 0;
			uart_user_tx_buffer.tail = 0;
			for(i=0; i<total_byte-UART_TX_BUFFER_LENGTH; i++)
			{
				uart_user_tx_buffer.buffer[uart_user_tx_buffer.header++] = content[UART_TX_BUFFER_LENGTH-occupied_buffer+i];
			}
#if USE_USART1
			DMA_SetCurrDataCounter(DMA2_Stream7, (uint16_t)UART_TX_BUFFER_LENGTH);
#else
			DMA_SetCurrDataCounter(DMA1_Stream6, (uint16_t)UART_TX_BUFFER_LENGTH);
#endif
		}
#if USE_USART1
		USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
		DMA_Cmd(DMA2_Stream7, ENABLE);
#else
		USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
		DMA_Cmd(DMA1_Stream6, ENABLE);
#endif
		return 1;
	}
	/*if DMA is still transmitting, put all content to uart buffer
	 *if exceed, return false
	 */
	else
	{
		if(total_byte > UART_TX_BUFFER_LENGTH)
			return 0;
		else
		{
			for(i=0; i<num; i++)
			{
				uart_user_tx_buffer.buffer[uart_user_tx_buffer.header++] = content[i];
				if(uart_user_tx_buffer.header >= UART_TX_BUFFER_LENGTH)
					uart_user_tx_buffer.header = 0;
			}
			return 1;
		}
	}
}

#if USE_USART1
void DMA2_Stream7_IRQHandler(void)
{
	/*DMA1 Streamer7 transmit complete*/
	if(DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7) != RESET)
	{
		DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7);
		DMA_Transmit_Complete = 1;
	}
}
#else
void DMA1_Stream6_IRQHandler(void)
{
	/*DMA1 Streamer6 transmit complete*/
	if(DMA_GetITStatus(DMA1_Stream6, DMA_IT_TCIF6) != RESET)
	{
		DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6);
		DMA_Transmit_Complete = 1;
	}
}
#endif
