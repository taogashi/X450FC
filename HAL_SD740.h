
/* Define to prevent recursive inclusion*/
#ifndef __HAL_SD740_H_
#define __HAL_SD740_H_

#ifdef __cplusplus
 extern "C" {
#endif 
  
/* Includes */
#include "stm32f4xx.h"

/**
* @addtogroup SD740 
* @{
*/

/**
* @addtogroup  SD740_I2C_Define
* @{
*/

#define SD740_SPI                	SPI1
#define SD740_SPI_RCC_Periph     	RCC_APB2Periph_SPI1
#define SD740_SPI_RCC_Port			RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOA

#define SD740_SPI_NSS_Pin_Port      GPIOA
#define SD740_SPI_NSS_Pin        	GPIO_Pin_15
#define SD740_SPI_NSS_Pin_Source 	GPIO_PinSource15
#define SD740_SPI_NSS_AF		 	GPIO_AF_SPI1

#define SD740_SPI_CLK_Pin_Port      GPIOB
#define SD740_SPI_CLK_Pin        	GPIO_Pin_3
#define SD740_SPI_CLK_Pin_Source   	GPIO_PinSource3
#define SD740_SPI_CLK_AF			GPIO_AF_SPI1

#define SD740_SPI_MISO_Pin_Port     GPIOB
#define SD740_SPI_MISO_Pin        	GPIO_Pin_4
#define SD740_SPI_MISO_Pin_Source   GPIO_PinSource4
#define SD740_SPI_MISO_AF			GPIO_AF_SPI1

#define SD740_SPI_MOSI_Pin_Port     GPIOB
#define SD740_SPI_MOSI_Pin        	GPIO_Pin_5
#define SD740_SPI_MOSI_Pin_Source   GPIO_PinSource5
#define SD740_SPI_MOSI_AF			GPIO_AF_SPI1

#define SD740_SPI_BaudRatePrescaler	SPI_BaudRatePrescaler_64


#endif /* __HAL_SD740_H */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
