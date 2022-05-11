#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "uart_manager.h"
#include "string.h"
#include "usart.h"
static struct _uart_manager uart_manager;
static int uart_dma_init(struct uart_port *port);
static void uart_init_config(struct uart_port *port);


//typedef struct __FILE FILE;

void uart_manager_init(void)
{
    char i = 0;
    
    for(i=0;i<UART_MAX_ID;i++)
        uart_manager.port[i] = NULL;
}

void uart1_parameter_init(struct uart_parameter *p)
{
    p->uart_baud_rate = USART1_BAUDRATE;
    p->art_word_length = USART_WordLength_8b;
    p->uart_stop_bits = USART_StopBits_1;
    p->UART_Parity = USART1_Priority;
    p->uart_id = UART1_ID;
    p->is_use_recv_queue = 1;
    p->recv_queue_len = BUFF_SIZE;
}

void uart2_parameter_init(struct uart_parameter *p)
{
    p->uart_baud_rate = USART2_BAUDRATE;
    p->art_word_length = USART_WordLength_8b;
    p->uart_stop_bits = USART_StopBits_1;
    p->UART_Parity = USART2_Priority;
    p->uart_id = UART2_ID;
    p->is_use_recv_queue = 1;
    p->recv_queue_len = BUFF_SIZE;
}

QueueHandle_t uart_port_queue_get(u8 uart_id)//取队列
{
    return uart_manager.port[uart_id]->recv_queue;
}

u8* uart_port_rxbuff_get(u8 uart_id)//取buff地址指针
{
    return uart_manager.port[uart_id]->dma_recv_buff;
}

u16 uart_port_rxbuffsize_get(u8 uart_id)
{
    return uart_manager.port[uart_id]->dma_recv_max_len;
}

int uart_register(struct uart_parameter *parameter,u16 recv_packet_max_len,u16 send_packet_max_len,recv_callback_isr_t recv_callback_isr)
{
    struct uart_port *port;

    if(parameter == NULL || parameter->uart_id >= UART_MAX_ID) //结构体为空
        return -1;
    
    if(uart_manager.port[parameter->uart_id] != NULL)//已经被注册了
        return -1;

    port = (struct uart_port *)pvPortMalloc(sizeof(struct uart_port));//创建指针结构体变量空间
    if(port == NULL)//创建失败
        return -1;

    memset(port, 0, sizeof(struct uart_port));
    memcpy(&port->parameter, parameter, sizeof(struct uart_parameter));//转移parameter结构体到指针变量内
    switch(port->parameter.uart_id)
    {
        case UART1_ID:
            port->com_port = USART1_COM;
            break;
        case UART2_ID:
            port->com_port = USART2_COM;
            break;
        case UART3_ID:
            port->com_port = USART3_COM;
            break;
        case UART4_ID:
            port->com_port = USART4_COM;
            break;
        default:
            port->com_port = NULL;
            goto free_uart_port;
            //break;
    }
    
    if(port->parameter.is_use_recv_queue)//判断是否开启接收消息队列
    {
        if(port->parameter.recv_queue_len > UART_RECV_QUEUE_MAX_LEN)
            goto free_uart_port;
        
        port->recv_queue = xQueueCreate(port->parameter.recv_queue_len,sizeof(struct uart_queue_msg));//创建队列
        if(port->recv_queue == NULL)//判断队列句柄为空
            goto free_uart_port;
    }
    else
        port->recv_queue = NULL;
    
    port->recv_callback_isr = recv_callback_isr;//回调函数

    if(port->parameter.is_use_recv_queue)//判断是否开启接收消息队列，是就赋DMA收发最大值，否则用户设置
    {
        port->dma_recv_max_len = UART_RECV_QUEUE_MSG_DATA_MAX_LEN;
        port->dma_send_max_len = UART_RECV_QUEUE_MSG_DATA_MAX_LEN;
    }
    else
    {
        port->dma_recv_max_len = recv_packet_max_len;
        port->dma_send_max_len = send_packet_max_len;
    }
    
    if(uart_dma_init(port) < 0)//判断dma内存地址初始化是否成功
    {
        goto free_uart_port;
    }
    uart_init_config(port);
    uart_manager.port[port->parameter.uart_id] = port;//接班
    return 0;
free_uart_port:
    vPortFree(port);
    return -1;
}

u8 uart_unregister(u8 uart_id)
{
    uart_manager.port[uart_id] = NULL;
    if(uart_manager.port[uart_id] != NULL)
        return -1;
    else
        return 0;
}

int uart_reset_parameter(struct uart_parameter *parameter)
{
    vPortFree(&parameter);
    if(parameter != NULL)
        return -1;
    else
        return 0;
}

int uart_wait_recv_data(u8 uart_id,u8 *data,TickType_t timeout)
{
    struct uart_port *port;
    struct uart_queue_msg msg;
    
    if(uart_id >= UART_MAX_ID || data == NULL )
        return -1;
    
    port = uart_manager.port[uart_id];
    if(port == NULL)
        return -1;
    
    if(port->recv_queue == NULL)
        return -1;
    if(xQueueReceive(port->recv_queue,&msg,timeout) == pdPASS)
    {   
        memcpy(data,msg.data,msg.len);
        return msg.len;
    }
    else
        return -1;
}

int uart_send_data(u8 uart_id,u8 *send_data,u16 send_data_len,u16 timeout)
{
    struct uart_port *port;
    u16 timeout_count = 0;
    
    if(uart_id >= UART_MAX_ID || send_data == NULL)
        return -1;
    
    port = uart_manager.port[uart_id];
    if(port == NULL)
        return -1;
    
    switch(uart_id)
    {
        case UART1_ID:
            usart1_send_paper(send_data,send_data_len);
            break;
        case UART2_ID:
            usart2_send_paper(send_data,send_data_len);
            break;
    }
    
    while(check_usart_dma_send_state(port) == RESET)//超时
    {
        timeout_count++;
        if(timeout_count > timeout)
        {
            return -1;
        }
        else
        {
            vTaskDelay(1);
        }
    }
    return 0;
}

static void uart_init_config(struct uart_port *port)//串口初始化分支
{
    switch(port->parameter.uart_id)
    {
        case UART1_ID:
            usart1_config(port);
            break;
        case UART2_ID:
            usart2_config(port);
            break;
        //case UART3_ID:
        //    USART3_Configuration(&port->parameter);
        //    break;
        //case UART4_ID:
        //    USART4_Configuration(&port->parameter);
        //     break;
        }
}

static int uart_dma_init(struct uart_port *port)//DMA初始化分支
{ 
    port->dma_send_buff = (u8 *)pvPortMalloc(port->dma_send_max_len);//发送BUFF分配
    if(port->dma_send_buff == NULL)
    {
        goto malloc_send_buff_error;
    }
    port->dma_recv_buff = (u8 *)pvPortMalloc(port->dma_recv_max_len);//接收BUFF分配
    if(port->dma_recv_buff == NULL)
    {
        goto malloc_recv_buff_error;
    }

    switch(port->parameter.uart_id)
    {
        case UART1_ID:
            port->DMA_r_Channel = USART1_DMA_R_CHANNEL;
            port->DMA_w_Channel = USART1_DMA_W_CHANNEL;
            port->dma_send_finish_flag = USART1_DMA_SEND_FINISH;
            port->dma_periph_addr = (u32)USART1_DR_ADDRESS;
            usart1_dma_config(port);
            break;
        case UART2_ID:
            port->DMA_r_Channel = USART2_DMA_R_CHANNEL;
            port->DMA_w_Channel = USART2_DMA_W_CHANNEL;
            port->dma_send_finish_flag = USART2_DMA_SEND_FINISH;
            port->dma_periph_addr = (u32)USART2_DR_ADDRESS;
            usart2_dma_config(port);
            break;
        //case UART3_ID:
        //    USART3_Configuration(&port->parameter);
        //    break;
        //case UART4_ID:
        //    USART4_Configuration(&port->parameter);
        //     break;
    }
    return 0;
malloc_recv_buff_error:
    vPortFree(port->dma_send_buff);
malloc_send_buff_error:
    return -1;
}


void usart1_callback(void)
{
    u16 USART1_CurrDataCounter;
    BaseType_t	pxHigherPriorityTaskWoken = pdFALSE;
    //    struct uart_queue_msg *qmsg;
    
//    DMA_ChannelEnable(USART1_DMA_R_CHANNEL, DISABLE);
//    DMA_ClearFlag( USART1_DMA_REC_FINISH );
//    USART1_CurrDataCounter = USART1_RECBUFF_SIZE - DMA_GetCurrDataCounter(USART1_DMA_R_CHANNEL);
//    QueueHandle_t xReturn = uart_port_queue_get(UART1_ID);
        

//        qmsg = (struct uart_queue_msg *)pvPortMalloc(sizeof(struct uart_queue_msg));
//        if(qmsg == NULL)
//            return ;
//        
//        qmsg->ID = UART1_ID;
//        qmsg->len = USART1_CurrDataCounter;
//        memcpy(qmsg->data,uart_port_rxbuff_get(UART1_ID),USART1_CurrDataCounter);
//        
//        xQueueSendFromISR(xReturn,qmsg->data,&pxHigherPriorityTaskWoken);
    xSemaphoreGiveFromISR(xsemaphore_recf1_get(),&pxHigherPriorityTaskWoken);
        
//    USART1_DMA_R_CHANNEL->TCNT = USART1_RECBUFF_SIZE;
//    DMA_ChannelEnable(USART1_DMA_R_CHANNEL, ENABLE);

    USART_ReceiveData( USART1_COM ); // Clear IDLE interrupt flag bit
    if( pxHigherPriorityTaskWoken == pdTRUE )
        taskYIELD();

}

void usart2_callback(void)
{
    u16 USART2_CurrDataCounter;
    BaseType_t	pxHigherPriorityTaskWoken = pdFALSE;
    //    struct uart_queue_msg *qmsg;
    
//    DMA_ChannelEnable(USART2_DMA_R_CHANNEL, DISABLE);
//    DMA_ClearFlag( USART2_DMA_REC_FINISH );
//    USART2_CurrDataCounter = USART2_RECBUFF_SIZE - DMA_GetCurrDataCounter(USART2_DMA_R_CHANNEL);
//    QueueHandle_t xReturn = uart_port_queue_get(UART1_ID);
        

//        qmsg = (struct uart_queue_msg *)pvPortMalloc(sizeof(struct uart_queue_msg));
//        if(qmsg == NULL)
//            return ;
//        
//        qmsg->ID = UART1_ID;
//        qmsg->len = USART1_CurrDataCounter;
//        memcpy(qmsg->data,uart_port_rxbuff_get(UART1_ID),USART1_CurrDataCounter);
//        
//        xQueueSendFromISR(xReturn,qmsg->data,&pxHigherPriorityTaskWoken);
    xSemaphoreGiveFromISR(xsemaphore_recf2_get(),&pxHigherPriorityTaskWoken);
        
//    USART2_DMA_R_CHANNEL->TCNT = USART2_RECBUFF_SIZE;
//    DMA_ChannelEnable(USART2_DMA_R_CHANNEL, ENABLE);

    USART_ReceiveData( USART2_COM ); // Clear IDLE interrupt flag bit
    if( pxHigherPriorityTaskWoken == pdTRUE )
        taskYIELD();

}

//void DMA_USART_Tx_Data(u32 size)
//{
//	while(USART1_TX_FLAG){};//等待上一次发送完成（USART1_TX_FLAG为1即还在发送数据）
//    USART_ClearFlag(USART1, USART_FLAG_TRAC); //清除USART标志位   
//	USART1_TX_FLAG=1;							//USART1发送标志（启动发送）

//    //QueueReceive(uart1_queue_handle, &usart1_tx_buff, 1);
//    
//	DMA1_Channel4->CMBA  =(u32) u1sendbuff;	//设置要发送的数据地址			
//    DMA_SetCurrDataCounter(DMA1_Channel4, size); //设置要发送的字节数目
//	DMA_ChannelEnable(DMA1_Channel4, ENABLE);				//开始DMA发送

//}


        
//void Usart_SendByte( USART_Type * pUSARTx, u8 ch)
// {
///* 发送一个字节数据到 USART */
//    USART_SendData(pUSARTx,ch);
// 
// /* 等待发送数据寄存器为空 */
//    while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TDE) == RESET);
// }
// 
// /***************** 发送字符串 **********************/
// void Usart_SendString( USART_Type * pUSARTx, char *str)
// {
//    unsigned int k=0;
//    do {
//        Usart_SendByte( pUSARTx, *(str + k) );
//        k++;}  while (*(str + k)!='\0');
// 
// /* 等待发送完成 */
//    while (USART_GetFlagStatus(pUSARTx,USART_FLAG_TRAC)==RESET) {
// }
//}


//int fputc(int ch, FILE *f)
//{
//		/* 发送一个字节数据到串口 */
//		USART_SendData(DEBUG_USARTx, (uint8_t) ch);
//		
//		/* 等待发送完毕 */
//		while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_TRAC) == RESET);		
//	
//		return (ch);
//}
//
//int fgetc(FILE *f)
//{
//		/* 等待串口输入数据 */
//		while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_RDNE) == RESET);
//
//		return (int)USART_ReceiveData(DEBUG_USARTx);
//}













