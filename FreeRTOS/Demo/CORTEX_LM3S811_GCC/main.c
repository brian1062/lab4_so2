/*
 * FreeRTOS V202212.01
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */


#include <stdio.h> 

/* Environment includes. */
#include "DriverLib.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Demo app includes. */
#include "integer.h"
#include "PollQ.h"
#include "semtest.h"
#include "BlockQ.h"

/* Delay between cycles of the 'check' task. */
#define mainCHECK_DELAY						( ( TickType_t ) 5000 / portTICK_PERIOD_MS )

/* UART configuration - note this does not use the FIFO so is not very
efficient. */
#define mainBAUD_RATE				( 19200 )
#define mainFIFO_SET				( 0x10 )

/* Demo task priorities. */
#define mainQUEUE_POLL_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define mainCHECK_TASK_PRIORITY		( tskIDLE_PRIORITY + 3 )
#define mainSEM_TEST_PRIORITY		( tskIDLE_PRIORITY + 1 )
#define mainBLOCK_Q_PRIORITY		( tskIDLE_PRIORITY + 2 )

/* Demo board specifics. */
#define mainPUSH_BUTTON             GPIO_PIN_4

/* Misc. */
#define mainQUEUE_SIZE				( 3 )
#define mainDEBOUNCE_DELAY			( ( TickType_t ) 150 / portTICK_PERIOD_MS )
#define mainNO_DELAY				( ( TickType_t ) 0 )
//this is for 10HZ
#define SENSOR_TIME ((TickType_t)100 / portTICK_PERIOD_MS)

#define X_VALUE 75 //96-21 75 values total in the graph
/*
 * Configure the processor and peripherals for this demo.
 */
static void prvSetupHardware( void );


/*
 * The task that controls access to the LCD.
 */


/* String that is transmitted on the UART. */
static volatile char *pcNextChar;

/* The semaphore used to wake the button handler task from within the GPIO
interrupt handler. */
SemaphoreHandle_t xButtonSemaphore;

/* The queue used to send strings to the print task for display on the LCD. */
QueueHandle_t xPrintQueue;
QueueHandle_t xFilterQueue;
#define T_MAX                		34
#define T_MIN               		18
#define N_MAX						10
static int size_N = 2;
static int temperature = 	26;
//TASKS
static void vSensorTask( void *pvParameters );
static void vFilterTask( void *pvParameters );
static void vGraphTask ( void *pvParameters );
//-----
static uint32_t _dwRandNext=1 ;
//FUNCTIONS
uint32_t rand_number( void );
void intToAscii(int number, char *buffer, int bufferSize);
void PositionInGraph(uint8_t tmp, uint8_t pos_graph[2]);
void GraphValues(uint8_t temp, uint8_t *bufTmp, int *x_start);
int getAverageValue(int *values, int values_size, int n_value);

/*--------------Main function----------------------*/

int main( void )
{
	/* Configure the clocks, UART and GPIO. */
	prvSetupHardware();

	/* Create the semaphore used to wake the button handler task from the GPIO
	ISR. */
	// vSemaphoreCreateBinary( xButtonSemaphore );
	// xSemaphoreTake( xButtonSemaphore, 0 );

	/* Create the queue used to pass message to vPrintTask. */
	xPrintQueue  = xQueueCreate( mainQUEUE_SIZE, sizeof( char * ) );
	xFilterQueue = xQueueCreate( mainQUEUE_SIZE, sizeof( char * ) );

	/* Start the tasks defined within the file. */
	xTaskCreate( vSensorTask, "Sensor", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY +1, NULL );
	xTaskCreate( vGraphTask,  "Graph", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY  -1, NULL );
	xTaskCreate( vFilterTask, "Filter", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY , NULL );

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was insufficient heap to start the
	scheduler. */

	return 0;
}
/*----------------------------------------------------------------*/
//https://www.freertos.org/uxTaskGetStackHighWaterMark.html
/*------------------------TASKS-----------------------------------*/
static void vSensorTask( void *pvParameters )
{
	UBaseType_t uxHighWaterMarkSensor;
	TickType_t xLastExecutionTime;
	portBASE_TYPE xErrorOccurred = pdFALSE;
	/* Initialise xLastExecutionTime so the first call to vTaskDelayUntil()
	works correctly. */
	xLastExecutionTime = xTaskGetTickCount();
	uxHighWaterMarkSensor = uxTaskGetStackHighWaterMark( NULL );
	if(uxHighWaterMarkSensor < 1){
		vTaskDelete(NULL);//stop task
	}

	for( ;; )
	{
		/* Perform this check every mainCHECK_DELAY milliseconds. */
		vTaskDelayUntil( &xLastExecutionTime, SENSOR_TIME );
		//read sensor
		temperature = (rand_number()% 2)? temperature + 1 : temperature - 1;
		if(temperature > T_MAX){
			temperature = T_MAX;
		} else if (temperature < T_MIN){
			temperature = T_MIN;
		}

		uxHighWaterMarkSensor = uxTaskGetStackHighWaterMark(NULL);
		if(uxHighWaterMarkSensor < 1){
			vTaskDelete(NULL);//stop task
		}
		if (xQueueSend(xFilterQueue, &temperature, portMAX_DELAY) != pdPASS)
		{
			OSRAMClear();
			OSRAMStringDraw("FULL", 0, 0);
			while (true){};
		}
	}
}

static void vFilterTask( void *pvParameters )
{	
	UBaseType_t uxHighWaterMarkSensor;
	int temp;
	int values_temp[N_MAX]={};
	int temp_filtered;



	uxHighWaterMarkSensor = uxTaskGetStackHighWaterMark( NULL );
	if(uxHighWaterMarkSensor < 1){
		vTaskDelete(NULL);//stop task
	}

	while (true)
	{
		if(xQueueReceive(xFilterQueue, &temp, portMAX_DELAY) != pdPASS){
			while (true){};
		}
		//update values
		for(int i =1; i < N_MAX; i++){
			values_temp[i] = values_temp[i-1];
		}
		values_temp[0]=temp;
		//filter
		temp_filtered = getAverageValue(values_temp, N_MAX, size_N);
		//send to print
		if(xQueueSend(xPrintQueue, &temp_filtered, portMAX_DELAY) != pdPASS){
			OSRAMClear();
			OSRAMStringDraw("FULL", 0, 0);
			while (true){};
		}

		uxHighWaterMarkSensor = uxTaskGetStackHighWaterMark(NULL);
		if(uxHighWaterMarkSensor < 1){
			vTaskDelete(NULL);//stop task
		}
	}
	

}

// From https://github.com/istarc/freertos/blob/master/FreeRTOS/Demo/CORTEX_A5_SAMA5D3x_Xplained_IAR/AtmelFiles/libboard_sama5d3x-ek/source/rand.c
// generate random number
uint32_t rand_number( void )
{
    _dwRandNext = _dwRandNext * 1103515245 + 12345 ;

    return (uint32_t)(_dwRandNext/131072) % 65536 ;
}

/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Setup the PLL. */
	SysCtlClockSet( SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_6MHZ );


	/* Enable the UART.  */
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	/* Set GPIO A0 and A1 as peripheral function.  They are used to output the
	UART signals. */
	GPIODirModeSet( GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_DIR_MODE_HW );

	/* Configure the UART for 8-N-1 operation. */
	UARTConfigSet( UART0_BASE, mainBAUD_RATE, UART_CONFIG_WLEN_8 | UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE );


	/* Enable Tx interrupts. */
	HWREG( UART0_BASE + UART_O_IM ) |= UART_INT_TX;
	IntPrioritySet( INT_UART0, configKERNEL_INTERRUPT_PRIORITY );
	IntEnable( INT_UART0 );


	/* Initialise the LCD> */
    OSRAMInit( false );
    OSRAMStringDraw("www.FreeRTOS.org", 0, 0);
	OSRAMStringDraw("LM3S811 demo", 16, 1);
}
/*-----------------------------------------------------------*/


void vUART_ISR(void)
{
unsigned long ulStatus;

	/* What caused the interrupt. */
	ulStatus = UARTIntStatus( UART0_BASE, pdTRUE );

	/* Clear the interrupt. */
	UARTIntClear( UART0_BASE, ulStatus );

	/* Was a Tx interrupt pending? */
	if( ulStatus & UART_INT_TX )
	{
		/* Send the next character in the string.  We are not using the FIFO. */
		if( *pcNextChar != 0 )
		{
			if( !( HWREG( UART0_BASE + UART_O_FR ) & UART_FR_TXFF ) )
			{
				HWREG( UART0_BASE + UART_O_DR ) = *pcNextChar;
			}
			pcNextChar++;
		}
	}
}
/*-----------------------------------------------------------*/

void vGPIO_ISR( void )
{
portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	/* Clear the interrupt. */
	GPIOPinIntClear(GPIO_PORTC_BASE, mainPUSH_BUTTON);

	/* Wake the button handler task. */
	xSemaphoreGiveFromISR( xButtonSemaphore, &xHigherPriorityTaskWoken );

	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
/*-----------------------------------------------------------*/

static void vGraphTask( void *pvParameters )
{

	uint8_t temp;
	uint8_t bufTmp[2]; //2bytes
	char *N_temp[4]; //4bytes to be safe(255'\0')
	int x_start = 21;
	

	UBaseType_t uxHighWaterMark;
	uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
	if(uxHighWaterMark < 1)
		while (true);

	OSRAMClear();
	for( ;; )
	{
		/* Wait for a message to arrive. */
		xQueueReceive( xPrintQueue, &temp, portMAX_DELAY );

		/* Write the message to the LCD. */
		intToAscii(temp, N_temp, 3);
		OSRAMStringDraw("T:", 0, 0);
		OSRAMStringDraw(N_temp, 9, 0);
		OSRAMStringDraw("N:", 0, 1); //next line lcd
		intToAscii(size_N, N_temp, 3);
		OSRAMStringDraw(N_temp, 8, 1);
		
		GraphValues(temp, bufTmp, &x_start);


		uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
		if(uxHighWaterMark < 1){
		while (true);
		}
	}
}

void intToAscii(int number, char *buffer, int bufferSize)
{
    if (number == 0)
    {
        buffer[0] = '0';
        buffer[1] = '\0';
        return;
    }

    int index = 0;

    // separate the digits
    while (number > 0 && index < bufferSize - 1){
        buffer[index++] = '0' + (number % 10); //obtain the last digit
        number /= 10;
    }

    buffer[index] = '\0';

    // Reorder the digits
    for (int i = 0; i < index / 2; i++)
    {
        char temp = buffer[i];
        buffer[i] = buffer[index - i - 1];
        buffer[index - i - 1] = temp;
    }
}

void PositionInGraph(uint8_t tmp, uint8_t pos_graph[2]){
	uint8_t lRow = 0x80; //0b10000000
	uint8_t uRow = 0;
	if (tmp == T_MAX)
	{
		pos_graph[0] = 0x01;
		pos_graph[1] = 0x80;
		return;
	}
	
	int pxl = 7 - ((tmp-T_MIN) % 8); //map the correct pixel

	if (tmp < 26)
	{
		lRow |= (1 << pxl);
	}
	else{
		uRow |= (1 << pxl);
	}

	pos_graph[0] = uRow;
	pos_graph[1] = lRow;
}

void GraphValues(uint8_t temp, uint8_t *bufTmp, int *x_start)
{
    unsigned char y[] = {0xFF, 0xFF};
    OSRAMImageDraw(y, 20, 0, 1, 2);

    PositionInGraph(temp, bufTmp);
    uint8_t x[] = {0x00, 0x80};
    OSRAMImageDraw(bufTmp, *x_start, 0, 1, 2);
    for (int i = *x_start + 1; i < 96; i++)
    {
        OSRAMImageDraw(x, i, 0, 1, 2);
    }
    if (*x_start < 96)
        (*x_start)++;
    else
        *x_start = 21; //reset
}

int getAverageValue(int *values, int values_size, int n_value)
{
	int sum = 0;
	if (n_value>values_size){
		while (true);//error window > values_size		
	}
	for(int i = 0; i < n_value; i++){
		sum += values[i];
	}

	return sum/n_value;
}
/*-----------------------------------------------------------*/
