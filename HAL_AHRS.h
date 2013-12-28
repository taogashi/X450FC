/* Define to prevent recursive inclusion*/
#ifndef _HAL_AHRS_H_
#define _HAL_AHRS_H_

#ifdef __cplusplus
 extern "C" {
#endif 
  
/* Includes */
#include "stm32f4xx.h"
#include "OSConfig.h"
/**
* @addtogroup AHRS 
* @{
*/

#define AHRS_SPI_INT_MODE
	 
#define AHRS_SPI                	SPI2
#define AHRS_SPI_RCC_Periph     	RCC_APB1Periph_SPI2
#define AHRS_SPI_RCC_Port			RCC_AHB1Periph_GPIOB

#define AHRS_SPI_NSS_Pin_Port      	GPIOB
#define AHRS_SPI_NSS_Pin        	GPIO_Pin_12
#define AHRS_SPI_NSS_Pin_Source 	GPIO_PinSource12
#define AHRS_SPI_NSS_AF		 		GPIO_AF_SPI2

#define AHRS_SPI_CLK_Pin_Port      	GPIOB
#define AHRS_SPI_CLK_Pin        	GPIO_Pin_13
#define AHRS_SPI_CLK_Pin_Source   	GPIO_PinSource13
#define AHRS_SPI_CLK_AF				GPIO_AF_SPI2

#define AHRS_SPI_MISO_Pin_Port     	GPIOB
#define AHRS_SPI_MISO_Pin        	GPIO_Pin_14
#define AHRS_SPI_MISO_Pin_Source   	GPIO_PinSource14
#define AHRS_SPI_MISO_AF			GPIO_AF_SPI2

#define AHRS_SPI_MOSI_Pin_Port     	GPIOB
#define AHRS_SPI_MOSI_Pin        	GPIO_Pin_15
#define AHRS_SPI_MOSI_Pin_Source   	GPIO_PinSource15
#define AHRS_SPI_MOSI_AF			GPIO_AF_SPI2

#define AHRS_SPI_BaudRatePrescaler	SPI_BaudRatePrescaler_32

#define AHRS_INT_PRIOR 		configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY-2 //configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=5
#define AHRS_IRQChannel 	SPI2_IRQn
#define AHRS_IRQHandler		SPI2_IRQHandler

#endif /* __HAL_AHRS_H */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
