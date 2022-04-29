#ifndef __UART_MANAGER_H
#define __UART_MANAGER_H
#include "at32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#define UART1_ID  0
#define UART2_ID  1
#define UART3_ID  2
#define UART4_ID  3
#define UART_MAX_ID  4

#define UART_RECV_QUEUE_MSG_DATA_MAX_LEN 32
#define UART_RECV_QUEUE_MAX_LEN 8

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

#define USART1_MAX_RX_LEN  8
#define USART2_MAX_RX_LEN  8
#define USART1_MAX_TX_LEN  8
#define USART2_MAX_TX_LEN  8

#define BUFF_SIZE 6

//extern USART_Type * DEBUG_USARTx;


typedef void (*recv_callback_isr_t)(u8 *data,u8 data_len);

struct uart_queue_msg
{
    s8 ID;
    u8 data[UART_RECV_QUEUE_MSG_DATA_MAX_LEN];
    u8 len;
};

struct uart_parameter
{
	u32 uart_baud_rate;
    u16 art_word_length;
    u16 uart_stop_bits;
    u16 UART_Parity;
    u8 uart_id;
    
    u8 is_use_recv_queue;
    u8 recv_queue_len;
} ;

struct uart_port
{
    struct uart_parameter parameter;
	USART_Type *com_port;
    
    u8 *dma_recv_buff;
    u8 *dma_send_buff;
    u16  dma_recv_max_len;
    u16  dma_send_max_len;
    DMA_Channel_Type *DMA_r_Channel;
    DMA_Channel_Type *DMA_w_Channel;
    u32 dma_send_finish_flag;
    u32 dma_periph_addr;
    
    QueueHandle_t recv_queue;
    
    recv_callback_isr_t recv_callback_isr;
};

struct _uart_manager
{
    struct uart_port *port[UART_MAX_ID];
};


void uart_manager_init(void);
int uart_register(struct uart_parameter *parameter,u16 recv_packet_max_len,u16 send_packet_max_len,recv_callback_isr_t recv_callback_isr);
int uart_wait_recv_data(u8 uart_id,u8 *data,TickType_t timeout);
int uart_send_data(u8 uart_id,u8 *send_data,u16 send_data_len,u16 timeout);
void usart1_config(struct uart_port *port);
void usart2_config(struct uart_port *port);
void uart1_parameter_init(struct uart_parameter *p);
void uart2_parameter_init(struct uart_parameter *p);
int uart_dma_init(struct uart_port *port);
void usart1_dma_config(struct uart_port *port);
void usart2_dma_config(struct uart_port *port);
u8 uart_unregister(u8 uart_id);

//void Usart_SendByte( USART_Type * pUSARTx, u8 ch);
//void Usart_SendString( USART_Type * pUSARTx, char *str);
QueueHandle_t uart_port_queue_get(u8 uart_id);
u8* uart_port_rxbuff_get(u8 uart_id);
u8 uart_unregister(u8 uart_id);
int uart_reset_parameter(struct uart_parameter *parameter);
void usart1_callback(void);
    

#endif 

