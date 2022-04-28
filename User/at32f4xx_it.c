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
#include "at32_board.h"
#include "uart_manager.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

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



/**
  * @}
  */
void USART1_IRQHandler(void)
{
    u8 i;
    BaseType_t	pxHigherPriorityTaskWoken = pdFALSE;
    QueueHandle_t xReturn = uart_port_queue_get(UART1_ID);
    u16	USART1_CurrDataCounter = 0;
    if( USART_GetFlagStatus(USART1,USART_FLAG_IDLEF) != RESET ){
        DMA_ChannelEnable(USART1_DMA_R_CHANNEL, DISABLE);
        DMA_ClearFlag( USART1_DMA_REC_FINISH );
        USART1_CurrDataCounter = USART1_RECBUFF_SIZE - DMA_GetCurrDataCounter(USART1_DMA_R_CHANNEL);
        xQueueSendFromISR(xReturn,uart_port_rxbuff_get(UART1_ID),&pxHigherPriorityTaskWoken);
//        Usart1_WaitReadNumber+=USART2_CurrDataCounter;
//        Usart1_ReceiveBufferState = USART2_IsNotWriteing;

        USART1_DMA_R_CHANNEL->TCNT = USART1_RECBUFF_SIZE;
        DMA_ChannelEnable(USART1_DMA_R_CHANNEL, ENABLE);

        USART_ReceiveData( USART1_COM ); // Clear IDLE interrupt flag bit
        if( pxHigherPriorityTaskWoken == pdTRUE )
            taskYIELD();
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

