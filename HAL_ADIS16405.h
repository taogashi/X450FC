
/* Define to prevent recursive inclusion*/
#ifndef __HAL_ADIS16405_H_
#define __HAL_ADIS16405_H_

#ifdef __cplusplus
 extern "C" {
#endif 
  
/* Includes */
#include "stm32f4xx.h"
#include "OSConfig.h"

/**
* @addtogroup ADIS16405 
* @{
*/
	 
#define ADIS16405_SPI_INT_MODE
	 
#define ADIS16405_SPI                	SPI1
#define ADIS16405_SPI_RCC_Periph     	RCC_APB2Periph_SPI1
#define ADIS16405_SPI_RCC_Port			RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOA

#define ADIS16405_SPI_NSS_Pin_Port      GPIOA
#define ADIS16405_SPI_NSS_Pin        	GPIO_Pin_15
#define ADIS16405_SPI_NSS_Pin_Source 	GPIO_PinSource15
#define ADIS16405_SPI_NSS_AF		 	GPIO_AF_SPI1

#define ADIS16405_SPI_CLK_Pin_Port      GPIOB
#define ADIS16405_SPI_CLK_Pin        	GPIO_Pin_3
#define ADIS16405_SPI_CLK_Pin_Source   	GPIO_PinSource3
#define ADIS16405_SPI_CLK_AF			GPIO_AF_SPI1

#define ADIS16405_SPI_MISO_Pin_Port     GPIOB
#define ADIS16405_SPI_MISO_Pin        	GPIO_Pin_4
#define ADIS16405_SPI_MISO_Pin_Source   GPIO_PinSource4
#define ADIS16405_SPI_MISO_AF			GPIO_AF_SPI1

#define ADIS16405_SPI_MOSI_Pin_Port     GPIOB
#define ADIS16405_SPI_MOSI_Pin        	GPIO_Pin_5
#define ADIS16405_SPI_MOSI_Pin_Source   GPIO_PinSource5
#define ADIS16405_SPI_MOSI_AF			GPIO_AF_SPI1

//128~656kHz
#define ADIS16405_SPI_BaudRatePrescaler	SPI_BaudRatePrescaler_128	

#define ADIS16405_INT_PRIOR 	configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY-3 //configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=5
#define ADIS16405_IRQChannel 	SPI1_IRQn
#define ADIS16405_IRQHandler	SPI1_IRQHandler


#endif /* __HAL_ADIS16405_H */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
