#include  "usart.h"

u8 usart_enable_get(u8 uart_id)
{
    switch(uart_id)
    {
        case UART1_ID:
            if( (USART1->CTRL1) & 0x0E )
                return 0;
            else
                return -1;
            break;
        case UART2_ID:
            if( (USART2->CTRL1) & 0x0E )
                return 0;
            else
                return -1;
            break;
    }
}


void usart1_config(struct uart_port *port)
{
	GPIO_InitType GPIO_InitStructure;
	NVIC_InitType NVIC_InitStructure;
	USART_InitType USART_InitStructure;
    
    RCC_AHBPeriphClockCmd(USART1_GPIO_CLK,ENABLE);
	RCC_APB2PeriphClockCmd(USART1_CLK,ENABLE);//GPIOA USART1
	//GPIO_PinsRemapConfig(GPIO_Remap_USART1, ENABLE);
    GPIO_PinAFConfig(GPIOA, GPIO_PinsSource9, GPIO_AF_1);
  /* Connect PXx to USART1_Rx */
    GPIO_PinAFConfig(GPIOA, GPIO_PinsSource10, GPIO_AF_1);
	//USART1_TX   GPIOA.9
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pins = USART1_TX_GPIO_PIN;         //TX
	GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OutType = GPIO_OutType_PP;
    GPIO_InitStructure.GPIO_Pull = GPIO_Pull_NOPULL;
	GPIO_Init(USART1_TX_GPIO_PORT, &GPIO_InitStructure);

    /* Configure the UART1 RX pin */
    GPIO_InitStructure.GPIO_Pins = USART1_RX_GPIO_PIN;        //RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OutType = GPIO_OutType_PP;
    GPIO_InitStructure.GPIO_Pull = GPIO_Pull_NOPULL;
    GPIO_Init(USART1_RX_GPIO_PORT, &GPIO_InitStructure);
	
	//Usart1 NVIC ??
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= port->parameter.UART_Parity;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
	NVIC_Init(&NVIC_InitStructure);
	
	//USART ?????
	USART_InitStructure.USART_BaudRate = port->parameter.uart_baud_rate;
	USART_InitStructure.USART_WordLength = port->parameter.art_word_length;
	USART_InitStructure.USART_StopBits = port->parameter.uart_stop_bits;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    
	USART_ClearFlag(USART1,USART_FLAG_TRAC);
	USART_INTConfig(USART1,USART_INT_IDLEF,ENABLE);
	USART_Cmd(USART1, ENABLE);
    USART_DMACmd(USART1,USART_DMAReq_Tx|USART_DMAReq_Rx,ENABLE);
}


void usart2_config(struct uart_port *port)
{
	GPIO_InitType GPIO_InitStructure;
	NVIC_InitType NVIC_InitStructure;
	USART_InitType USART_InitStructure;
    
    RCC_AHBPeriphClockCmd(USART2_GPIO_CLK,ENABLE);
	RCC_APB1PeriphClockCmd(USART2_CLK,ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOD,ENABLE);//??GPIOD?USART1??
//    RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_AFIO,ENABLE);
//	GPIO_PinsRemapConfig(GPIO_Remap_USART2, ENABLE);
  /* Connect PXx to USART2_Tx */
    GPIO_PinAFConfig(GPIOA, GPIO_PinsSource2, GPIO_AF_1);
  /* Connect PXx to USART2_Rx */
    GPIO_PinAFConfig(GPIOA, GPIO_PinsSource3, GPIO_AF_1);
    
    GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pins = USART2_TX_GPIO_PIN;         //TX
	GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OutType = GPIO_OutType_PP;
    GPIO_InitStructure.GPIO_Pull = GPIO_Pull_NOPULL;
	GPIO_Init(USART2_TX_GPIO_PORT, &GPIO_InitStructure);

    /* Configure the UART1 RX pin */
    GPIO_InitStructure.GPIO_Pins = USART2_RX_GPIO_PIN;        //RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OutType = GPIO_OutType_PP;
    GPIO_InitStructure.GPIO_Pull = GPIO_Pull_NOPULL;
    GPIO_Init(USART2_RX_GPIO_PORT, &GPIO_InitStructure);
    
	//Usart2 NVIC ??
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = port->parameter.UART_Parity;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//USART ?????
	USART_InitStructure.USART_BaudRate = port->parameter.uart_baud_rate;
	USART_InitStructure.USART_WordLength = port->parameter.art_word_length;
	USART_InitStructure.USART_StopBits = port->parameter.uart_stop_bits;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);
    
	USART_ClearFlag(USART2,USART_FLAG_TRAC);
	USART_INTConfig(USART2,USART_INT_IDLEF,ENABLE);
	USART_Cmd(USART2, ENABLE);
    USART_DMACmd(USART2,USART_DMAReq_Tx|USART_DMAReq_Rx,ENABLE);
}

void usart1_dma_config(struct uart_port *port)
{
    DMA_InitType DMA_InitStructure;
    NVIC_InitType NVIC_InitStructure;
    

    RCC_AHBPeriphClockCmd(DMA1_CLOCK, ENABLE);
    
//    NVIC_InitStructure.NVIC_IRQChannel = USART1_DMA_R_CHANNEL_IRQ;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
//    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_DMA_W_CHANNEL_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
    
    /*RX_Init*/
    DMA_Reset(USART1_DMA_R_CHANNEL);
    DMA_InitStructure.DMA_PeripheralBaseAddr = port->dma_periph_addr;
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32) port->dma_recv_buff;         
    DMA_InitStructure.DMA_Direction = DMA_DIR_PERIPHERALSRC;       
    DMA_InitStructure.DMA_BufferSize = port->dma_recv_max_len;         
    DMA_InitStructure.DMA_PeripheralInc = DMA_PERIPHERALINC_DISABLE;         
    DMA_InitStructure.DMA_MemoryInc = DMA_MEMORYINC_ENABLE;         
    DMA_InitStructure.DMA_PeripheralDataWidth =DMA_PERIPHERALDATAWIDTH_BYTE;         
    DMA_InitStructure.DMA_MemoryDataWidth = DMA_MEMORYDATAWIDTH_BYTE;         
    DMA_InitStructure.DMA_Mode = DMA_MODE_CIRCULAR;         
    DMA_InitStructure.DMA_Priority = DMA_PRIORITY_MEDIUM;        
    DMA_InitStructure.DMA_MTOM = DMA_MEMTOMEM_DISABLE;
    DMA_Init(USART1_DMA_R_CHANNEL, &DMA_InitStructure);
    DMA_ChannelEnable(USART1_DMA_R_CHANNEL,ENABLE);
    

    /*TX_Init*/
    DMA_Reset(USART1_DMA_W_CHANNEL);
    DMA_InitStructure.DMA_PeripheralBaseAddr = port->dma_periph_addr;
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32) port->dma_send_buff;         
    DMA_InitStructure.DMA_Direction = DMA_DIR_PERIPHERALDST;       
    DMA_InitStructure.DMA_BufferSize = port->dma_send_max_len;         
    DMA_InitStructure.DMA_PeripheralInc = DMA_PERIPHERALINC_DISABLE;         
    DMA_InitStructure.DMA_MemoryInc = DMA_MEMORYINC_ENABLE;         
    DMA_InitStructure.DMA_PeripheralDataWidth = DMA_PERIPHERALDATAWIDTH_BYTE;         
    DMA_InitStructure.DMA_MemoryDataWidth = DMA_MEMORYDATAWIDTH_BYTE;         
    DMA_InitStructure.DMA_Mode = DMA_MODE_NORMAL;         
    DMA_InitStructure.DMA_Priority = DMA_PRIORITY_MEDIUM;        
    DMA_InitStructure.DMA_MTOM = DMA_MEMTOMEM_DISABLE;
    
    DMA_Init( USART1_DMA_W_CHANNEL, &DMA_InitStructure);
    USART_DMACmd(port->com_port, USART_DMAReq_Rx, ENABLE);
    DMA_ChannelEnable( USART1_DMA_W_CHANNEL, DISABLE);
}

void usart2_dma_config(struct uart_port *port)
{
    DMA_InitType DMA_InitStructure;
    NVIC_InitType NVIC_InitStructure;
    
  
    RCC_AHBPeriphClockCmd(DMA1_CLOCK, ENABLE);
  
//    NVIC_InitStructure.NVIC_IRQChannel = USART2_DMA_R_CHANNEL_IRQ;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
//    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART2_DMA_W_CHANNEL_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

    /*RX_Init*/
    DMA_Reset(USART2_DMA_R_CHANNEL);    
    DMA_InitStructure.DMA_PeripheralBaseAddr = port->dma_periph_addr;
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32) port->dma_recv_buff;         
    DMA_InitStructure.DMA_Direction = DMA_DIR_PERIPHERALSRC;       
    DMA_InitStructure.DMA_BufferSize = port->dma_recv_max_len;         
    DMA_InitStructure.DMA_PeripheralInc = DMA_PERIPHERALINC_DISABLE;         
    DMA_InitStructure.DMA_MemoryInc = DMA_MEMORYINC_ENABLE;         
    DMA_InitStructure.DMA_PeripheralDataWidth = DMA_PERIPHERALDATAWIDTH_BYTE;         
    DMA_InitStructure.DMA_MemoryDataWidth = DMA_MEMORYDATAWIDTH_BYTE;         
    DMA_InitStructure.DMA_Mode = DMA_MODE_CIRCULAR;         
    DMA_InitStructure.DMA_Priority = DMA_PRIORITY_MEDIUM;        
    DMA_InitStructure.DMA_MTOM = DMA_MEMTOMEM_DISABLE;
    
    DMA_Init( USART2_DMA_R_CHANNEL, &DMA_InitStructure);
    DMA_ChannelEnable( USART2_DMA_R_CHANNEL,ENABLE);
    
    /*TX_Init*/
    DMA_Reset(USART2_DMA_W_CHANNEL);
    DMA_InitStructure.DMA_PeripheralBaseAddr = port->dma_periph_addr;
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32) port->dma_send_buff;         
    DMA_InitStructure.DMA_Direction = DMA_DIR_PERIPHERALDST;       
    DMA_InitStructure.DMA_BufferSize = port->dma_send_max_len;         
    DMA_InitStructure.DMA_PeripheralInc = DMA_PERIPHERALINC_DISABLE;         
    DMA_InitStructure.DMA_MemoryInc = DMA_MEMORYINC_ENABLE;         
    DMA_InitStructure.DMA_PeripheralDataWidth = DMA_PERIPHERALDATAWIDTH_BYTE;         
    DMA_InitStructure.DMA_MemoryDataWidth = DMA_MEMORYDATAWIDTH_BYTE;         
    DMA_InitStructure.DMA_Mode = DMA_MODE_NORMAL;         
    DMA_InitStructure.DMA_Priority = DMA_PRIORITY_MEDIUM;
    DMA_InitStructure.DMA_MTOM = DMA_MEMTOMEM_DISABLE;
    
    DMA_Init( USART2_DMA_W_CHANNEL, &DMA_InitStructure);
    USART_DMACmd(port->com_port, USART_DMAReq_Rx, ENABLE);
    DMA_ChannelEnable( USART2_DMA_W_CHANNEL, DISABLE);
}
