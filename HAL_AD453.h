
/* Define to prevent recursive inclusion*/
#ifndef __HAL_AD453_H_
#define __HAL_AD453_H_

#ifdef __cplusplus
 extern "C" {
#endif 
  
/* Includes */
#include "stm32f4xx.h"

/**
* @addtogroup AD453 
* @{
*/

#define AD453_SPI                	SPI1
#define AD453_SPI_RCC_Periph     	RCC_APB2Periph_SPI1
#define AD453_SPI_RCC_Port			RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOA

#define AD453_SPI_NSS_Pin_Port      GPIOA
#define AD453_SPI_NSS_Pin        	GPIO_Pin_15
#define AD453_SPI_NSS_Pin_Source 	GPIO_PinSource15
#define AD453_SPI_NSS_AF		 	GPIO_AF_SPI1

#define AD453_SPI_CLK_Pin_Port      GPIOB
#define AD453_SPI_CLK_Pin        	GPIO_Pin_3
#define AD453_SPI_CLK_Pin_Source   	GPIO_PinSource3
#define AD453_SPI_CLK_AF			GPIO_AF_SPI1

#define AD453_SPI_MISO_Pin_Port     GPIOB
#define AD453_SPI_MISO_Pin        	GPIO_Pin_4
#define AD453_SPI_MISO_Pin_Source   GPIO_PinSource4
#define AD453_SPI_MISO_AF			GPIO_AF_SPI1

#define AD453_SPI_MOSI_Pin_Port     GPIOB
#define AD453_SPI_MOSI_Pin        	GPIO_Pin_5
#define AD453_SPI_MOSI_Pin_Source   GPIO_PinSource5
#define AD453_SPI_MOSI_AF			GPIO_AF_SPI1

#define AD453_SPI_BaudRatePrescaler	SPI_BaudRatePrescaler_8


#endif /* __HAL_AD453_H */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
