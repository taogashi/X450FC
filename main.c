/**
  ******************************************************************************
  * @file    STM32F4-Discovery FreeRTOS demo\main.c
  * @author  T.O.M.A.S. Team
  * @version V1.1.0
  * @date    14-October-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"


/* FreeRTOS includes */
#include "OSConfig.h"
#include "pwm.h"

#include "ledTask.h"
#include "sensor.h"
#include "diskTask.h"
#include "buttonTask.h"
#include "AHRSEKF.h"
#include "flightConTask.h"
#include "uartTask.h"
#include "INSEKF.h"

/** @addtogroup STM32F4-Discovery_Demo
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

void prvSetupHardware( void );
void Delay_ns(u32 n);

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */

int main(void)
{ 
	/* initialize hardware... */
	prvSetupHardware();
	
	/*queu to sychronize EXTI and button task*/	
	xButtonQueueISR = xQueueCreate(3,sizeof(u8));

	/*uart task send the GPS data to other tasks*/
	xUartGPSQueue = xQueueCreate(1,sizeof(GPSDataType));
	/*uart task send the parameters to other task*/	
	xUartParaQueue = xQueueCreate(1,sizeof(OptionalPara));
	/*uart task send waypoint to flightConTask*/
	xUartWayPointQueue = xQueueCreate(10,sizeof(WayPointType));
	/*read sensor data and send to other tasks*/
	xSenToAhrsQueue = xQueueCreate(1,sizeof(SensorDataType*));

	/*AHRS should send data to flight control task to stabilize the att*/
	AHRSToFlightConQueue = xQueueCreate(1,sizeof(AttConDataType));
	/*AHRS should send data to INS task to offer inertial data*/
	AHRSToINSQueue = xQueueCreate(1,sizeof(float *));
	/*INS task should send data to flight control task to support the position control*/
	INSToFlightConQueue = xQueueCreate(1,sizeof(PosConDataType));

	/*there are two queues to communicate with the tf card*/
	/*the first one to read/write basic parameters like PID para, etc.*/
	xDiskWrQueue1 = xQueueCreate(1,sizeof(OptionalPara));
	/*the second one to make a data log on the tf card*/
	xDiskLogQueue = xQueueCreate(1,sizeof(char)*100);
//	/* Start the tasks defined within this file/specific to this demo. */
	xTaskCreate(vLED1Task, ( signed portCHAR * ) "LED", configMINIMAL_STACK_SIZE, (void *)NULL,tskIDLE_PRIORITY+1, NULL );
	xTaskCreate(vSenAHRSRead, ( signed portCHAR * ) "AHRSread", configMINIMAL_STACK_SIZE+32, (void *)NULL,tskIDLE_PRIORITY+3, NULL );

	xTaskCreate(vAEKFAligTask, ( signed portCHAR * ) "AHRSaligment", configMINIMAL_STACK_SIZE+32, (void *)NULL,tskIDLE_PRIORITY+2, NULL );
	xTaskCreate(vINSAligTask, ( signed portCHAR * ) "INSaligment", configMINIMAL_STACK_SIZE+32, (void *)NULL,tskIDLE_PRIORITY+2, NULL );

	xTaskCreate(vFlyConTask, ( signed portCHAR * ) "flightControl", configMINIMAL_STACK_SIZE+64, (void *)NULL,tskIDLE_PRIORITY+4, NULL );

	xTaskCreate(vUartRecTask,(signed portCHAR *)"uart_rec", configMINIMAL_STACK_SIZE+128,(void *)NULL,tskIDLE_PRIORITY+2,NULL);
	xTaskCreate(vDiskOperation,(signed portCHAR *)"file", configMINIMAL_STACK_SIZE+4096,(void *)NULL,tskIDLE_PRIORITY+1,NULL);
	xTaskCreate(vButtonEXTIHandler,(signed portCHAR *)"button", configMINIMAL_STACK_SIZE+32,(void *)NULL,tskIDLE_PRIORITY+2,NULL);
	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was not enough heap space to create the idle task. */
	return 0;  
}

/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
//	u32 i;
	volatile size_t xFreeStackSpace;

	/* This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amout of FreeRTOS heap that 
	remains unallocated. */
	xFreeStackSpace = xPortGetFreeHeapSize();
//	for(i=0;i<0x5FFFFF;i++);
//	printf(".");
	if( xFreeStackSpace > 100 )
	{
		/* By now, the kernel has allocated everything it is going to, so
		if there is a lot of heap remaining unallocated then
		the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		reduced accordingly. */
	}
}


/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software 
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	for( ;; );
}

/*-----------------------------------------------------------*/
static void prvSetupHardware( void )
{
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

//	TIM2_Config();
	TIM3_Config();
	TIM4_Config();
	TIM5_Config();
	USART2_DMA_Config();
	USART2_DMA_IT_Config();
}
/*-----------------------------------------------------------*/

void Delay_ns(u32 n)
{
	for(;n>0;n--);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
