/*
* Example RTOS Atmel Studio
*/

#include <asf.h>
#include <string.h>
#include "conf_board.h"

/********************************************************
Define
********************************************************/
// LEDs
#define LED_PIO PIOC
#define LED_PIO_ID ID_PIOC
#define LED_IDX 8
#define LED_IDX_MASK (1 << LED_IDX)

#define LED1_PIO PIOA
#define LED1_PIO_ID ID_PIOA
#define LED1_IDX 0
#define LED1_IDX_MASK (1 << LED1_IDX)

#define LED2_PIO PIOC
#define LED2_PIO_ID ID_PIOC
#define LED2_IDX 30
#define LED2_IDX_MASK (1 << LED2_IDX)

#define LED3_PIO PIOB
#define LED3_PIO_ID ID_PIOB
#define LED3_IDX 2
#define LED3_IDX_MASK (1 << LED3_IDX)


//BUTs
#define BUT_PIO PIOA
#define BUT_PIO_ID ID_PIOA
#define BUT_PIO_IDX 11
#define BUT_PIO_IDX_MASK (1u << BUT_PIO_IDX)

#define BUT1_PIO PIOD
#define BUT1_PIO_ID 16
#define BUT1_PIO_IDX 28
#define BUT1_PIO_IDX_MASK (1u << BUT1_PIO_IDX)

#define BUT2_PIO PIOC
#define BUT2_PIO_ID ID_PIOC
#define BUT2_PIO_IDX 31
#define BUT2_PIO_IDX_MASK (1 << BUT2_PIO_IDX)

#define BUT3_PIO PIOA
#define BUT3_PIO_ID ID_PIOA
#define BUT3_PIO_IDX 19
#define BUT3_PIO_IDX_MASK (1 << BUT3_PIO_IDX)

//TASKs
#define TASK_LED_STACK_SIZE                (1024/sizeof(portSTACK_TYPE))
#define TASK_LED_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_LED1_STACK_SIZE (1024 / sizeof(portSTACK_TYPE))
#define TASK_LED1_STACK_PRIORITY (tskIDLE_PRIORITY)

#define TASK_LED2_STACK_SIZE (1024 / sizeof(portSTACK_TYPE))
#define TASK_LED2_STACK_PRIORITY (tskIDLE_PRIORITY)

#define TASK_LED3_STACK_SIZE (1024 / sizeof(portSTACK_TYPE))
#define TASK_LED3_STACK_PRIORITY (tskIDLE_PRIORITY)

#define TASK_MONITOR_STACK_SIZE            (2048/sizeof(portSTACK_TYPE))
#define TASK_MONITOR_STACK_PRIORITY        (tskIDLE_PRIORITY)


#define TASK_UARTRX_STACK_SIZE (1024 / sizeof(portSTACK_TYPE))
#define TASK_UARTRX_STACK_PRIORITY (tskIDLE_PRIORITY)

#define TASK_EXECUTE_STACK_SIZE (1024 / sizeof(portSTACK_TYPE))
#define TASK_EXECUTE_STACK_PRIORITY (tskIDLE_PRIORITY)




/************************************************************
Queue
************************************************************/
QueueHandle_t xQueueRx;
QueueHandle_t xQueueCommand;
QueueHandle_t xQueueLED1;
QueueHandle_t xQueueLED2;
QueueHandle_t xQueueLED3;
QueueHandle_t xQueueLED1on;
QueueHandle_t xQueueLED2on;
QueueHandle_t xQueueLED3on;

/** Semaforo a ser usado pela task led 
    tem que ser var global! */
SemaphoreHandle_t xSemaphore;
SemaphoreHandle_t xSemaphore1;
SemaphoreHandle_t xSemaphore2;
SemaphoreHandle_t xSemaphore3;

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/**
 * \brief Called if stack overflow during execution
 */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,signed char *pcTaskName){
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	 * identify which task has overflowed its stack.
	 */
	for (;;) {
	}
}

/**
 * \brief This function is called by FreeRTOS idle task
 */
extern void vApplicationIdleHook(void){
	pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);

}

/**
 * \brief This function is called by FreeRTOS each tick
 */
extern void vApplicationTickHook(void){
	
}

extern void vApplicationMallocFailedHook(void){
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}









/**
 * \brief This task, when activated, send every ten seconds on debug UART
 * the whole report of free heap and total tasks status
 */
static void task_monitor(void *pvParameters){
	static portCHAR szList[256];
	UNUSED(pvParameters);

	/* Block for 3000ms. */
	const TickType_t xDelay = 3000 / portTICK_PERIOD_MS;

	while(1){
		printf("--- task ## %u\n", (unsigned int)uxTaskGetNumberOfTasks());
		vTaskList((signed portCHAR *)szList);
		printf(szList);
		vTaskDelay(xDelay);
	}
}
/***********************************************************
FUNCTIONS
***********************************************************/
void pin_toggle(Pio *pio, uint32_t mask) {
	if (pio_get_output_data_status(pio, mask)) pio_clear(pio, mask);
	else pio_set(pio, mask);
}


/*******************************************************
TASKs + Callback
*******************************************************/
/**
* callback do botao
* libera semaforo: xSemaphore
*/
void but_callback(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	printf("but_callback \n");
	xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);
	printf("semafaro tx \n");
}

void but1_callback(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	printf("but1_callback \n");
	xSemaphoreGiveFromISR(xSemaphore1, &xHigherPriorityTaskWoken);
	printf("semafaro tx \n");
}

void but2_callback() {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	printf("but2_callback \n");
	xSemaphoreGiveFromISR(xSemaphore2, &xHigherPriorityTaskWoken);
	printf("semafaro2 tx \n");
}

void but3_callback() {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	printf("but3_callback \n");
	xSemaphoreGiveFromISR(xSemaphore3, &xHigherPriorityTaskWoken);
	printf("semafaro3 tx \n");
}

static void task_led(void *pvParameters) {
  /* We are using the semaphore for synchronisation so we create a binary
  semaphore rather than a mutex.  We must make sure that the interrupt
  does not attempt to use the semaphore before it is created! */
  xSemaphore = xSemaphoreCreateBinary();

  /* devemos iniciar a interrupcao no pino somente apos termos alocado
  os recursos (no caso semaforo), nessa funcao inicializamos 
  o botao e seu callback*/
  /* init botão */
  pmc_enable_periph_clk(BUT_PIO_ID);
  pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_IDX_MASK, PIO_PULLUP);
  pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIO_IDX_MASK, PIO_IT_FALL_EDGE, but_callback);
  pio_enable_interrupt(BUT_PIO, BUT_PIO_IDX_MASK);
  NVIC_EnableIRQ(BUT_PIO_ID);
  NVIC_SetPriority(BUT_PIO_ID, 4); // Prioridade 4

  if (xSemaphore == NULL)
    printf("falha em criar o semaforo \n");

  for (;;) {
    if( xSemaphoreTake(xSemaphore, ( TickType_t ) 500) == pdTRUE ){
      LED_Toggle(LED0);
    }
  }
}

static void task_led1(void *pvParameters){
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_configure(LED1_PIO, PIO_OUTPUT_0, LED1_IDX_MASK, PIO_DEFAULT);
	pio_set(LED1_PIO, LED1_IDX_MASK);

	/* Block for 2000ms. */
	xQueueLED1 = xQueueCreate(5, sizeof(int));
	xQueueLED1on = xQueueCreate(5, sizeof(int));
	int i, on;

	while(1){
		if (xQueueReceive(xQueueLED1, &(i), (TickType_t)500)) {
			if (i) {
				pin_toggle(LED1_PIO, LED1_IDX_MASK);
			}
		}
		if (xQueueReceive(xQueueLED1on, &(on), (TickType_t)500)) {
			if (on) {
				pio_clear(LED1_PIO, LED1_IDX_MASK);
				} else {
				pio_set(LED1_PIO, LED1_IDX_MASK);
			}
		}
	}
}


static void task_led2(void *pvParameters){
	pmc_enable_periph_clk(LED2_PIO_ID);
	pio_configure(LED2_PIO, PIO_OUTPUT_0, LED2_IDX_MASK, PIO_DEFAULT);
	pio_set(LED2_PIO, LED2_IDX_MASK);

	/* Block for 2000ms. */
	xQueueLED2 = xQueueCreate(5, sizeof(int));
	xQueueLED2on = xQueueCreate(5, sizeof(int));
	int i,on;

	while(1){
		if (xQueueReceive(xQueueLED2, &(i), (TickType_t)500)) {
			if (i) {
				pin_toggle(LED2_PIO, LED2_IDX_MASK);
			}
		}
		if (xQueueReceive(xQueueLED2on, &(on), (TickType_t)500)) {
			if (on) {
				pio_clear(LED2_PIO, LED2_IDX_MASK);
				} else {
				pio_set(LED2_PIO, LED2_IDX_MASK);
			}
		}
	}
}
static void task_led3(void *pvParameters){
	pmc_enable_periph_clk(LED3_PIO_ID);
	pio_configure(LED3_PIO, PIO_OUTPUT_0, LED3_IDX_MASK, PIO_DEFAULT);
	pio_set(LED3_PIO, LED3_IDX_MASK);

	/* Block for 2000ms. */
	xQueueLED3 = xQueueCreate(5, sizeof(int));
	xQueueLED3on = xQueueCreate(5, sizeof(int));
	int i, on;

	while(1){
		if (xQueueReceive(xQueueLED3, &(i), (TickType_t)500)) {
			if (i) {
				pin_toggle(LED3_PIO, LED3_IDX_MASK);
			}
		}
		
		if (xQueueReceive(xQueueLED3on, &(on), (TickType_t)500)) {
			if (on) {
				pio_clear(LED3_PIO, LED3_IDX_MASK);
				} else {
				pio_set(LED3_PIO, LED3_IDX_MASK);
			}
		}
	}
}


/********************************************************************
USART
********************************************************************/
static void USART1_init(void){
	/* Configura USART1 Pinos */
	sysclk_enable_peripheral_clock(ID_PIOB);
	sysclk_enable_peripheral_clock(ID_PIOA);
	pio_set_peripheral(PIOB, PIO_PERIPH_D, PIO_PB4);  // RX
	pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA21); // TX
	MATRIX->CCFG_SYSIO |= CCFG_SYSIO_SYSIO4;

	/* Configura opcoes USART */
	const sam_usart_opt_t usart_settings = {
		.baudrate = 115200,
		.char_length = US_MR_CHRL_8_BIT,
		.parity_type = US_MR_PAR_NO,
		.stop_bits = US_MR_NBSTOP_1_BIT,
	.channel_mode = US_MR_CHMODE_NORMAL};

	/* Ativa Clock periferico USART0 */
	sysclk_enable_peripheral_clock(ID_USART1);

	stdio_serial_init(CONF_UART, &usart_settings);

	/* Enable the receiver and transmitter. */
	usart_enable_tx(USART1);
	usart_enable_rx(USART1);

	/* map printf to usart */
	ptr_put = (int (*)(void volatile *, char)) & usart_serial_putchar;
	ptr_get = (void (*)(void volatile *, char *)) & usart_serial_getchar;

	/* ativando interrupcao */
	usart_enable_interrupt(USART1, US_IER_RXRDY);
	NVIC_SetPriority(ID_USART1, 4);
	NVIC_EnableIRQ(ID_USART1);
}
void USART1_Handler(void){
	uint32_t ret = usart_get_status(USART1);

	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	char c;

	// Verifica por qual motivo entrou na interrupçcao?
	// RXRDY ou TXRDY

	//  Dados disponível para leitura
	if (ret & US_IER_RXRDY)
	{
		usart_serial_getchar(USART1, &c);
		xQueueSendFromISR(xQueueRx, &c, &xHigherPriorityTaskWoken);
		// printf("%c", c);
	}
	else if (ret & US_IER_TXRDY)
	{
	}
}

uint32_t usart1_puts(uint8_t *pstring) {
	uint32_t i;

	while (*(pstring + i))
	if (uart_is_tx_empty(USART1)) usart_serial_putchar(USART1, *(pstring + i++));
}


/**********************************************************
 * \brief Configure the console UART.
 ************************************************************/
static void configure_console(void){
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
#if (defined CONF_UART_CHAR_LENGTH)
		.charlength = CONF_UART_CHAR_LENGTH,
#endif
		.paritytype = CONF_UART_PARITY,
#if (defined CONF_UART_STOP_BITS)
		.stopbits = CONF_UART_STOP_BITS,
#endif
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
#if defined(__GNUC__)
	setbuf(stdout, NULL);
#else
	/* Already the case in IAR's Normal DLIB default configuration: printf()
	 * emits one character at a time.
	 */
#endif
}
/*********************************************************************
TASK
*********************************************************************/
static void task_uartRX(void *pvParameters) {
	char rxMsg;
	char msgBuffer[64] = {0};
	int i = 0;

	xQueueRx = xQueueCreate(32, sizeof(char));

	while (1) {
		if (xQueueReceive(xQueueRx, &rxMsg, (TickType_t)500)) {
			printf("Recebido: %c\n", rxMsg);

			if (rxMsg == '\n') {
				msgBuffer[i] = 0;
				xQueueSend(xQueueCommand, &msgBuffer, 0);
				i = 0;
				} else {
				msgBuffer[i] = rxMsg;
				i++;
			}
		}
	}
}



static void task_execute(void *pvParameters) {
	char msgBuffer[64];
	
	int led1 = 0;
	int led2 = 0;
	int led3 = 0;

	int on1;
	xQueueCommand = xQueueCreate(5, sizeof(char[64]));

	while (1) {
		if (xQueueReceive(xQueueCommand, &msgBuffer, (TickType_t)500)) {
			printf("Mensagem: %s\n", msgBuffer);
			
			if (strcmp(msgBuffer, "led 1 toggle") == 0) {
				led1 = !led1;
				xQueueSend(xQueueLED1, &led1, 0);
			}
			
			if (strcmp(msgBuffer, "led 2 toggle") == 0) {
				led2 = !led2;
				xQueueSend(xQueueLED2, &led2, 0);
			}
			
			if (strcmp(msgBuffer, "led 3 toggle") == 0) {
				led3 = !led3;
				xQueueSend(xQueueLED3, &led3, 0);
			}
			
			
			if (strcmp(msgBuffer, "led 1 on") == 0) {
				on1 = 1;
				xQueueSend(xQueueLED1on, &on1, 0);
			}
			
			if (strcmp(msgBuffer, "led 2 on") == 0) {
				on1 = 1;
				xQueueSend(xQueueLED2on, &on1, 0);
			}
			
			if (strcmp(msgBuffer, "led 3 on") == 0) {
				on1 = 1;
				xQueueSend(xQueueLED3on, &on1, 0);
			}
			
			if (strcmp(msgBuffer, "led 1 off") == 0) {
				on1 = 0;
				xQueueSend(xQueueLED1on, &on1, 0);
			}

			if (strcmp(msgBuffer, "led 2 off") == 0) {
				on1 = 0;
				xQueueSend(xQueueLED2on, &on1, 0);
			}
			
			if (strcmp(msgBuffer, "led 3 off") == 0) {
				on1 = 0;
				xQueueSend(xQueueLED3on, &on1, 0);
			}
		}
	}
}

/**
 *  \brief FreeRTOS Real Time Kernel example entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
int main(void){
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	//configure_console();
	USART1_init();
	
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_configure(LED1_PIO, PIO_OUTPUT_0, LED1_IDX_MASK, PIO_DEFAULT);

	/* Output demo information. */
	printf("-- Freertos Example --\n\r");
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);


	/* Create task to monitor processor activity */
	if (xTaskCreate(task_monitor, "Monitor", TASK_MONITOR_STACK_SIZE, NULL, TASK_MONITOR_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create Monitor task\r\n");
	}

	/* Create task to make led blink */
	if (xTaskCreate(task_led, "Led", TASK_LED_STACK_SIZE, NULL, TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}
	
	if(xTaskCreate(task_led1, "Led1", TASK_LED1_STACK_SIZE, NULL, TASK_LED1_STACK_PRIORITY, NULL)){
		printf("Failed to create led1 task\r\n");
	}

	/* Create task to make led blink */
	if (xTaskCreate(task_led2, "Led2", TASK_LED2_STACK_SIZE, NULL, TASK_LED2_STACK_PRIORITY, NULL) != pdPASS){
		printf("Failed to create test led2 task\r\n");
	}

	/* Create task to make led blink */
	if (xTaskCreate(task_led3, "Led3", TASK_LED3_STACK_SIZE, NULL, TASK_LED3_STACK_PRIORITY, NULL) != pdPASS){
		printf("Failed to create test led3 task\r\n");
	}

	// Create UART RX task
	if (xTaskCreate(task_uartRX, "UART-RX", TASK_UARTRX_STACK_SIZE, NULL, TASK_UARTRX_STACK_PRIORITY, NULL) != pdPASS){
		printf("Failed to create UART-RX task\r\n");
	}
	
	// Create EXECUTE task
	if (xTaskCreate(task_execute, "EXECUTE", TASK_EXECUTE_STACK_SIZE, NULL, TASK_EXECUTE_STACK_PRIORITY, NULL) != pdPASS){
		printf("Failed to create EXECUTE task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();
	while(1){
		
	}
	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
