/**
  ******************************************************************************
  * File   : Templates/at32f4xx_it.c 
  * Version: V1.3.0
  * Date   : 2021-03-18
  * Brief  : Interrupt program body
  ******************************************************************************
  */ 


/* Includes ------------------------------------------------------------------*/
#include "at32f4xx_it.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "at32_board.h"

//extern void xPortSysTickHandler(void);
//extern BaseType_t xTaskGetSchedulerState(void);
/** @addtogroup AT32F413_StdPeriph_Templates
  * @{
  */
/** @addtogroup GPIO_LED_Toggle
  * @{
  */
void TimingDelay_Decrement(void);

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
//void SVC_Handler(void)
//{
//}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
//void PendSV_Handler(void)
//{
//}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
//void SysTick_Handler(void)
//{
//}

//void TMR6_GLOBAL_IRQHandler(void)
//{
//    if(TMR_GetFlagStatus(TMR6, TMR_INT_Overflow) != RESET)
//    {
//        if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)
//        {
//            xPortSysTickHandler();
//        }
//        TMR_ClearFlag(TMR6, TMR_INT_Overflow);
//    }
//}

/**
  * @}
  */
void USART1_IRQHandler(void)
{
    if( USART_GetFlagStatus(USART1,USART_FLAG_IDLEF) != RESET ){
        usart1_callback();
	}
}

void USART2_IRQHandler(void)
{
    if( USART_GetFlagStatus(USART2,USART_FLAG_IDLEF) != RESET ){
        usart2_callback();
	}
}

void DMA1_Channel3_2_IRQHandler(void)
{
    if( DMA_GetFlagStatus(DMA1_INT_TC2) != RESET )
    {
        //SPI_DMA_CALLBACK();
        DMA_ClearITPendingBit(DMA1_INT_TC2);
    }

}

void DMA1_Channel7_4_IRQHandler(void)
{
    if( DMA_GetFlagStatus(DMA_INT_TC) != RESET )
    {
        DMA_ClearITPendingBit(DMA_INT_TC);
    }
}
/**
  * @}
  */

