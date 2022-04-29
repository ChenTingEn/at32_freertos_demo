/**
 **************************************************************************
 * File Name    : main.c
 * Description  : None
 * Date         : 2021-02-22
 * Version      : V1.0.0
 **************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include  "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
//#include "at32_board.h"

/* USER CODE END PM */

/* Private define ------------------------------------------------------------*/
#define USART1_TASK_STACK_SIZE 100
#define USART2_TASK_STACK_SIZE 100
#define LED_TASK_STACK_SIZE 100
/* Private function prototypes -----------------------------------------------*/
u8 u1recebuff[USART2_MAX_RX_LEN] = {0};
u8 u2recebuff[USART2_MAX_RX_LEN] = {0};
u8 usart2_waitread = 0;
extern struct _uart_manager uart_manager;
/* Private functions ---------------------------------------------------------*/

/*create task handle*/
static TaskHandle_t uart1_task_handle = NULL;
static TaskHandle_t uart2_task_handle = NULL;
static TaskHandle_t LED_task_handle = NULL;
static SemaphoreHandle_t xsemaphore_recf1 = NULL;
static SemaphoreHandle_t xsemaphore_recf2 = NULL;

//extern USART_Type * DEBUG_USARTx;

/*defined a USART_Type struct parameter*/

struct uart_parameter usart_p1 = {0},usart_p2 = {0};

/* uart1 task */
static void uart1_task_Function(void *pvParameters);
/* uart2 task */
static void uart2_task_Function(void *pvParameters);
/* LED task */
static void LED_task_Function(void *pvParameters);
/* pa6 unlock */
static void pa6init(void);
static void TMR6_Init(void);


SemaphoreHandle_t xsemaphore_recf1_get(){
    return xsemaphore_recf1;
}

SemaphoreHandle_t xsemaphore_recf2_get(){
    return xsemaphore_recf2;
}

int main(void)
{
    BaseType_t xReturn = pdFAIL;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    //TMR6_Init();
    uart_manager_init();
    //pa6init();
    
    taskENTER_CRITICAL(); //进入临界区
    xReturn = xTaskCreate( uart1_task_Function,"Task1",USART1_TASK_STACK_SIZE,NULL,1,&uart1_task_handle);
    xReturn = xTaskCreate( uart2_task_Function,"Task2",USART2_TASK_STACK_SIZE,NULL,2,&uart2_task_handle); 
    xReturn = xTaskCreate( LED_task_Function,"Task3",LED_TASK_STACK_SIZE,NULL,2,&LED_task_handle); 
    taskEXIT_CRITICAL();//退出临界区
    
    if (pdPASS == xReturn)
        vTaskStartScheduler();//开启调度
    else
        return -1;
    
    while(1);
}

static void uart1_task_Function(void *pvParameters)
{
    uart1_parameter_init(&usart_p1);//初始化结构体
    vSemaphoreCreateBinary(xsemaphore_recf1);//创建信号量
    if(uart_register(&usart_p1,4,4,NULL) != 0){
        while(1);
    }
    //Usart_SendString(USART2,"helloA\n");
    while(1)
    {
        if(xSemaphoreTake(xsemaphore_recf1,portMAX_DELAY) != pdFALSE){
            xQueueReceive(uart_port_queue_get(UART1_ID),u1recebuff,portMAX_DELAY);
            
        }
        vTaskDelay(10);
    }
}
 

static void uart2_task_Function(void *pvParameters)
{
    uart2_parameter_init(&usart_p2);//初始化结构体
    
    vSemaphoreCreateBinary(xsemaphore_recf2);//创建信号量
    if( xsemaphore_recf2 == NULL)
        while(1);
    
    if(uart_register(&usart_p2,4,4,NULL) != 0){
        while(1);
    }
    //Usart_SendString(USART2,"helloA\n");

    while(1)
    {
        if(xSemaphoreTake(xsemaphore_recf2, 1) != pdFALSE)
        {
            usart2_waitread = uart_wait_recv_data(UART2_ID,u2recebuff,1);
        }
            
        vTaskDelay(1);
    }
}

static void LED_task_Function(void *pvParameters)
{
    while(1)
    {
        vTaskDelay(1000);
    }
}

//static void pa6init()
//{
//    GPIO_InitType GPIO_InitStructure;
//    RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOA,ENABLE);
//    GPIO_InitStructure.GPIO_Pins = GPIO_Pins_6; //PA.6
//    GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT_PP;
//    GPIO_Init(GPIOA, &GPIO_InitStructure);
//    
//    GPIO_SetBits(GPIOA,GPIO_Pins_6);
//}

static void TMR6_Init(void)
{
    NVIC_InitType NVIC_InitStructure;
    TMR_TimerBaseInitType  TMR_TimeBaseStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_TMR6,ENABLE);
    
    NVIC_InitStructure.NVIC_IRQChannel = TMR6_GLOBAL_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    TMR_TimeBaseStructInit(&TMR_TimeBaseStructure);
    TMR_TimeBaseStructure.TMR_DIV = 0;
    TMR_TimeBaseStructure.TMR_CounterMode = TMR_CounterDIR_Up;
    TMR_TimeBaseStructure.TMR_Period = 119;         //1ms
    TMR_TimeBaseStructure.TMR_ClockDivision = 0;
    TMR_TimeBaseStructure.TMR_RepetitionCounter = 0;

    TMR_TimeBaseInit(TMR6, &TMR_TimeBaseStructure);
    TMR_INTConfig(TMR6, TMR_INT_Overflow, ENABLE);
    TMR_Cmd(TMR6, ENABLE);
}