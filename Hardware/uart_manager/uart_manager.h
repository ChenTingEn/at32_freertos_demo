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



#define USART1_MAX_RX_LEN  32
#define USART2_MAX_RX_LEN  32
#define USART1_MAX_TX_LEN  32
#define USART2_MAX_TX_LEN  32

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
void uart1_parameter_init(struct uart_parameter *p);
void uart2_parameter_init(struct uart_parameter *p);
int uart_dma_init(struct uart_port *port);
u8 uart_unregister(u8 uart_id);
//void Usart_SendByte( USART_Type * pUSARTx, u8 ch);
//void Usart_SendString( USART_Type * pUSARTx, char *str);
QueueHandle_t uart_port_queue_get(u8 uart_id);
u8* uart_port_rxbuff_get(u8 uart_id);
u8 uart_unregister(u8 uart_id);
int uart_reset_parameter(struct uart_parameter *parameter);
void usart1_callback(void);
void usart2_callback(void);
u16 uart_port_rxbuffsize_get(u8 uart_id);

#endif 

