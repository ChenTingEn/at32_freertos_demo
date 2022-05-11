#ifndef __USART_H
#define __USART_H
#include  "main.h"

#define  USART1_COM                USART1
#define  USART1_BAUDRATE           115200
#define  USART1_CLK                RCC_APB2PERIPH_USART1
#define  USART1_APBxClkCmd         RCC_APB2PeriphClockCmd

#define  USART1_GPIO_CLK           (RCC_AHBPERIPH_GPIOA)
#define  USART1_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
    
#define  USART1_TX_GPIO_PORT       GPIOA   
#define  USART1_TX_GPIO_PIN        GPIO_Pins_9
#define  USART1_RX_GPIO_PORT       GPIOA
#define  USART1_RX_GPIO_PIN        GPIO_Pins_10
#define  USART1_Priority           3

#define  USART1_IRQ                 USART1_IRQn
#define  USART1_DMA_R_CHANNEL_IRQ   DMA1_Channel3_2_IRQn
#define  USART1_DMA_W_CHANNEL_IRQ   DMA1_Channel3_2_IRQn
#define  USART1_DMA_R_CHANNEL       DMA1_Channel3
#define  USART1_DMA_W_CHANNEL       DMA1_Channel2
#define  USART1_DMA_SEND_FINISH     DMA1_FLAG_TC2
#define  USART1_DMA_REC_FINISH     DMA1_FLAG_TC3
#define  USART1_RECBUFF_SIZE        128
/**************************************************/
#define  USART2_COM                USART2
#define  USART2_BAUDRATE           115200
#define  USART2_CLK                RCC_APB1PERIPH_USART2
#define  USART2_APBxClkCmd         RCC_APB1PeriphClockCmd

#define  USART2_GPIO_CLK           (RCC_AHBPERIPH_GPIOA)
#define  USART2_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
    
#define  USART2_TX_GPIO_PORT       GPIOA   
#define  USART2_TX_GPIO_PIN        GPIO_Pins_2
#define  USART2_RX_GPIO_PORT       GPIOA
#define  USART2_RX_GPIO_PIN        GPIO_Pins_3
#define  USART2_Priority           3

#define  USART2_IRQ                USART2_IRQn
#define  USART2_DMA_R_CHANNEL_IRQ   DMA1_Channel7_4_IRQn
#define  USART2_DMA_W_CHANNEL_IRQ   DMA1_Channel7_4_IRQn
#define  USART2_DMA_R_CHANNEL       DMA1_Channel5
#define  USART2_DMA_W_CHANNEL       DMA1_Channel4
#define  USART2_DMA_SEND_FINISH     DMA1_FLAG_TC4
#define  USART2_DMA_REC_FINISH      DMA1_FLAG_TC5
#define  USART2_RECBUFF_SIZE        128

/**************************************************/
#define  USART3_COM                 NULL//USART3
#define  USART3_BAUDRATE           115200
#define  USART3_CLK                RCC_APB1PERIPH_USART3
#define  USART3_APBxClkCmd         RCC_APB1PeriphClockCmd

#define  USART3_GPIO_CLK           (RCC_APB2PERIPH_GPIOB)
#define  USART3_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
    
#define  USART3_TX_GPIO_PORT       GPIOB   
#define  USART3_TX_GPIO_PIN        GPIO_Pins_10
#define  USART3_RX_GPIO_PORT       GPIOB
#define  USART3_RX_GPIO_PIN        GPIO_Pins_11

#define  USART3_Priority           3
#define  USART3_IRQ                USART3_IRQn

/**************************************************/
#define  USART4_COM                 NULL//USART4
#define  USART4_BAUDRATE           115200
#define  USART4_CLK                RCC_APB1PERIPH_UART4
#define  USART4_APBxClkCmd         RCC_APB1PeriphClockCmd

#define  USART4_GPIO_CLK           (RCC_APB2PERIPH_GPIOC)
#define  USART4_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
    
#define  USART4_TX_GPIO_PORT       GPIOC   
#define  USART4_TX_GPIO_PIN        GPIO_Pins_10
#define  USART4_RX_GPIO_PORT       GPIOC
#define  USART4_RX_GPIO_PIN        GPIO_Pins_11

#define  USART4_Priority           3
#define  USART4_IRQ                UART4_IRQn

/**************************************************/
#define  USART5_COM                 USART5
#define  USART5_BAUDRATE           115200
#define  USART5_CLK                RCC_APB1PERIPH_UART5
#define  USART5_APBxClkCmd         RCC_APB1PeriphClockCmd

#define  USART5_GPIO_CLK           (RCC_APB2PERIPH_GPIOC|RCC_APB2PERIPH_GPIOD)
#define  USART5_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
    
#define  USART5_TX_GPIO_PORT       GPIOC   
#define  USART5_TX_GPIO_PIN        GPIO_Pins_12
#define  USART5_RX_GPIO_PORT       GPIOD
#define  USART5_RX_GPIO_PIN        GPIO_Pins_2

#define  USART5_Priority           3
#define  USART5_IRQ                UART5_IRQn

/**************************************************/
#define DMA1_CLOCK               RCC_AHBPERIPH_DMA1
#define DMA2_CLOCK               RCC_AHBPERIPH_DMA2


//#define USART1_DR_ADDRESS        (USART1_BASE+0x04)
//#define USART2_DR_ADDRESS        (USART2_BASE+0x04)

#define USART1_DR_ADDRESS        &(USART1->DT)
#define USART2_DR_ADDRESS        &(USART2->DT)


void usart1_dma_config(struct uart_port *port);
void usart1_config(struct uart_port *port);
void usart2_config(struct uart_port *port);
void usart2_dma_config(struct uart_port *port);
u8 usart1_enable_get(void);
u8 usart2_enable_get(void);
u8 usart1_send_paper(u8 *send_data,u16 send_data_len);
u8 usart2_send_paper(u8 *send_data,u16 send_data_len);
u8 check_usart_dma_send_state(struct uart_port *port);
#endif