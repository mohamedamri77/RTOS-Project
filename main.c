/**
 ******************************************************************************
 * @file    main.c
 * @author  Ac6
 * @version V1.0
 * @date    01-December-2013
 * @brief   Default main function.
 ******************************************************************************
 */


#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "string.h"
#include "stdio.h"
TaskHandle_t xTaskhandle1=NULL;
TaskHandle_t xTaskhandle2=NULL;
// Task Function Prototype
void vTask1_handler(void *parameter);
void vTask2_handler(void *parameter);
static void prvSetupHardware(void );
static void prvSetupUart(void);
void Sendmess(char *msg);
// define AREA
#define TRUE 1
#define FALSE 0
#define AVAILABLE TRUE
#define NOT_AVAILABLE FALSE
//#define USE_SEMIHOSTING
// Global Variables Area
char msg[100]= "aaaaaaaaaaaa";
char usr_msg[250]={0};
uint8_t USE_UART=AVAILABLE;

// extern Declaration used for semi-hosting

int main(void)
{
#ifdef USE_SEMIHOSTING
	initialise_monitor_handles();
	printf("HI ya si Mohamed\n");
#endif
	DWT->CTRL |= (1<<0); // Enable the Cycle Count Register  CYCCNT
	// 1. Reset The Clock Configuration to the Default Reset State = 16 MHz
	RCC_DeInit();
	// 2. update the systemcoreClock
	SystemCoreClockUpdate();
	prvSetupHardware();
	sprintf(usr_msg,"Hhhhhhhhhhhhhhhhhhhhhhhhhhhhh\r\n");
	Sendmess(usr_msg);

	SEGGER_SYSVIEW_Conf();
	SEGGER_SYSVIEW_Start();
	// 3. Create Task 1 and Task 2

	//	 BaseType_t xTaskCreate(	TaskFunction_t pxTaskCode,
	//							const char * const pcName,
	//							const configSTACK_DEPTH_TYPE usStackDepth,
	//							void * const pvParameters,
	//							UBaseType_t uxPriority,
	//							TaskHandle_t * const pxCreatedTask )
	xTaskCreate(vTask1_handler,"Task-1",configMINIMAL_STACK_SIZE,NULL,2,&xTaskhandle1);
	xTaskCreate(vTask2_handler,"Task-2",configMINIMAL_STACK_SIZE,NULL,2,&xTaskhandle2);

	vTaskStartScheduler();





	for(;;);
}
void vTask1_handler(void *params)
{
	while(1){
		if(USE_UART==AVAILABLE){
			USE_UART=NOT_AVAILABLE;
			Sendmess("Hello from task 1\r\n");
			USE_UART=AVAILABLE;
			taskYIELD();
		}
	}


}
void vTask2_handler(void *params)
{
	while(1){
		if(USE_UART==AVAILABLE){
			USE_UART=NOT_AVAILABLE;
			Sendmess("Hello from task 2\r\n");
			USE_UART=AVAILABLE;
			taskYIELD();
		}
	}
}
static void prvSetupUart(void)
{
	GPIO_InitTypeDef gpio_uart_pins;
	USART_InitTypeDef uart_init;
	// 1. enable the UART2 and the GPIOA peripheral Clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);

	// PA2 is UART2_TX, PA3 is UART2_RX
	memset(&gpio_uart_pins,0,sizeof(gpio_uart_pins));
	//2. Alternate function configuration of MCU to behave as UART2 Tx and Rx
	gpio_uart_pins.GPIO_Pin=GPIO_Pin_2 | GPIO_Pin_3;
	gpio_uart_pins.GPIO_Mode=GPIO_Mode_AF;
	gpio_uart_pins.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &gpio_uart_pins);

	//3. AF mode Stetings for the pins
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //PA2
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //PA3
	//4. UART Peripheral initialization
	//USART_Init(USART_TypeDef* USARTx, USART_InitTypeDef* USART_InitStruct)
	memset(&uart_init,0,sizeof(uart_init));
	uart_init.USART_BaudRate=115200;
	uart_init.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	uart_init.USART_Mode=USART_Mode_Rx | USART_Mode_Tx;
	uart_init.USART_Parity=USART_Parity_No;
	uart_init.USART_StopBits=USART_StopBits_1;
	uart_init.USART_WordLength=USART_WordLength_8b;
	USART_Init( USART2, &uart_init);
	//5. Enable the UART peripheral
	USART_Cmd(USART2,ENABLE);
}
static void prvSetupHardware(void )
{

	// Setup the chosen Hardware
	prvSetupUart();


}
void Sendmess(char *msg)
{
	for(uint32_t i=0; i<strlen(msg);i++)
	{
		while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) !=SET);
		USART_SendData(USART2,msg[i]);
	}
}

