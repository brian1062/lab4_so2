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


/*
 * This project contains an application demonstrating the use of the
 * FreeRTOS.org mini real time scheduler on the Luminary Micro LM3S811 Eval
 * board.  See http://www.FreeRTOS.org for more information.
 *
 * main() simply sets up the hardware, creates all the demo application tasks,
 * then starts the scheduler.  http://www.freertos.org/a00102.html provides
 * more information on the standard demo tasks.
 *
 * In addition to a subset of the standard demo application tasks, main.c also
 * defines the following tasks:
 *
 * + A 'Print' task.  The print task is the only task permitted to access the
 * LCD - thus ensuring mutual exclusion and consistent access to the resource.
 * Other tasks do not access the LCD directly, but instead send the text they
 * wish to display to the print task.  The print task spends most of its time
 * blocked - only waking when a message is queued for display.
 *
 * + A 'Button handler' task.  The eval board contains a user push button that
 * is configured to generate interrupts.  The interrupt handler uses a
 * semaphore to wake the button handler task - demonstrating how the priority
 * mechanism can be used to defer interrupt processing to the task level.  The
 * button handler task sends a message both to the LCD (via the print task) and
 * the UART where it can be viewed using a dumb terminal (via the UART to USB
 * converter on the eval board).  NOTES:  The dumb terminal must be closed in
 * order to reflash the microcontroller.  A very basic interrupt driven UART
 * driver is used that does not use the FIFO.  19200 baud is used.
 *
 * + A 'check' task.  The check task only executes every five seconds but has a
 * high priority so is guaranteed to get processor time.  Its function is to
 * check that all the other tasks are still operational and that no errors have
 * been detected at any time.  If no errors have every been detected 'PASS' is
 * written to the display (via the print task) - if an error has ever been
 * detected the message is changed to 'FAIL'.  The position of the message is
 * changed for each write.
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
/*
 * Configure the processor and peripherals for this demo.
 */
static void prvSetupHardware( void );

/*
 * The 'check' task, as described at the top of this file.
 */
static void vCheckTask( void *pvParameters );

/*
 * The task that is woken by the ISR that processes GPIO interrupts originating
 * from the push button.
 */
static void vButtonHandlerTask( void *pvParameters );

/*
 * The task that controls access to the LCD.
 */
static void vPrintTask( void *pvParameter );

/* String that is transmitted on the UART. */
static char *cMessage = "Task woken by button interrupt! --- ";
static volatile char *pcNextChar;

/* The semaphore used to wake the button handler task from within the GPIO
interrupt handler. */
SemaphoreHandle_t xButtonSemaphore;

/* The queue used to send strings to the print task for display on the LCD. */
QueueHandle_t xPrintQueue;
#define T_MAX                		34
#define T_MIN               		18
static int temperature = 	26;
//TASKS
static void vSensorTask( void *pvParameters );
//-----
static uint32_t _dwRandNext=1 ;
uint32_t rand_number( void );
void intToAscii(int number, char *buffer, int bufferSize);
void GraphAxes(void);
/*--------------Main function----------------------*/

int main( void )
{
	/* Configure the clocks, UART and GPIO. */
	prvSetupHardware();

	/* Create the semaphore used to wake the button handler task from the GPIO
	ISR. */
	vSemaphoreCreateBinary( xButtonSemaphore );
	xSemaphoreTake( xButtonSemaphore, 0 );

	/* Create the queue used to pass message to vPrintTask. */
	xPrintQueue = xQueueCreate( mainQUEUE_SIZE, sizeof( char * ) );

	/* Start the standard demo tasks. */
	vStartIntegerMathTasks( tskIDLE_PRIORITY );
	vStartPolledQueueTasks( mainQUEUE_POLL_PRIORITY );
	vStartSemaphoreTasks( mainSEM_TEST_PRIORITY );
	vStartBlockingQueueTasks( mainBLOCK_Q_PRIORITY );

	/* Start the tasks defined within the file. */
	xTaskCreate( vSensorTask, "Sensor", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY+1, NULL );
	xTaskCreate( vPrintTask, "Print", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY + 1, NULL );

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

		//xQueueSend( xPrintQueue, &temperature, portMAX_DELAY );

		uxHighWaterMarkSensor = uxTaskGetStackHighWaterMark(NULL);
		if(uxHighWaterMarkSensor < 1){
			vTaskDelete(NULL);//stop task
		}
		if (xQueueSend(xPrintQueue, &temperature, mainCHECK_DELAY) != pdPASS)
		{
			OSRAMClear();
			OSRAMStringDraw("FULL", 0, 0);
			while (true){};
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

	// /* Setup the push button. */
	// SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    // GPIODirModeSet(GPIO_PORTC_BASE, mainPUSH_BUTTON, GPIO_DIR_MODE_IN);
	// GPIOIntTypeSet( GPIO_PORTC_BASE, mainPUSH_BUTTON,GPIO_FALLING_EDGE );
	// IntPrioritySet( INT_GPIOC, configKERNEL_INTERRUPT_PRIORITY );
	// GPIOPinIntEnable( GPIO_PORTC_BASE, mainPUSH_BUTTON );
	// IntEnable( INT_GPIOC );

	/* Enable the UART.  */
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	/* Set GPIO A0 and A1 as peripheral function.  They are used to output the
	UART signals. */
	GPIODirModeSet( GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_DIR_MODE_HW );

	/* Configure the UART for 8-N-1 operation. */
	UARTConfigSet( UART0_BASE, mainBAUD_RATE, UART_CONFIG_WLEN_8 | UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE );

	// /* We don't want to use the fifo.  This is for test purposes to generate
	// as many interrupts as possible. */
	// HWREG( UART0_BASE + UART_O_LCR_H ) &= ~mainFIFO_SET;

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

static void vButtonHandlerTask( void *pvParameters )
{
const char *pcInterruptMessage = "Int";

	for( ;; )
	{
		/* Wait for a GPIO interrupt to wake this task. */
		while( xSemaphoreTake( xButtonSemaphore, portMAX_DELAY ) != pdPASS );

		/* Start the Tx of the message on the UART. */
		UARTIntDisable( UART0_BASE, UART_INT_TX );
		{
			pcNextChar = cMessage;

			/* Send the first character. */
			if( !( HWREG( UART0_BASE + UART_O_FR ) & UART_FR_TXFF ) )
			{
				HWREG( UART0_BASE + UART_O_DR ) = *pcNextChar;
			}

			pcNextChar++;
		}
		UARTIntEnable(UART0_BASE, UART_INT_TX);

		/* Queue a message for the print task to display on the LCD. */
		xQueueSend( xPrintQueue, &pcInterruptMessage, portMAX_DELAY );

		/* Make sure we don't process bounces. */
		vTaskDelay( mainDEBOUNCE_DELAY );
		xSemaphoreTake( xButtonSemaphore, mainNO_DELAY );
	}
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

static void vPrintTask( void *pvParameters )
{
	char *pcMessage;
	uint8_t temp;
	uint8_t bufTmp[2]; //2bytes
	char *N_temp[4]; //4bytes to be safe(255'\0')
	
	int displayCounter = 20;

	UBaseType_t uxHighWaterMark;
	uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
	if(uxHighWaterMark < 1)
		while (true);

	OSRAMClear();
	for( ;; )
	{
		/* Wait for a message to arrive. */
		xQueueReceive( xPrintQueue, &temp, SENSOR_TIME );

		/* Write the message to the LCD. */
		intToAscii(temp, N_temp, 3);
		OSRAMStringDraw("T:", 0, 0);
		OSRAMStringDraw(N_temp, 9, 0);
		OSRAMStringDraw("N:", 0, 1); //next line lcd
		OSRAMStringDraw("1", 8, 1);
		GraphAxes();


		uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
		if(uxHighWaterMark < 1)
		while (true);
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
    while (number > 0 && index < bufferSize - 1)
    {
        buffer[index++] = '0' + (number % 10); //obtain the last digit
        number /= 10;
    }

    // Add the null
    buffer[index] = '\0';

    // Reorder the digits
    for (int i = 0; i < index / 2; i++)
    {
        char temp = buffer[i];
        buffer[i] = buffer[index - i - 1];
        buffer[index - i - 1] = temp;
    }
}
void GraphAxes(void)
{
	int x_start = 20;
	unsigned char yaxis[] = {0xFF, 0xFF};
	OSRAMImageDraw(yaxis, 20, 0, 1, 2);
	uint8_t xaxis[] = {0x00, 0x80};
	OSRAMImageDraw(xaxis, x_start+1, 0, 1, 2);

	for (int i = x_start + 1; i < 96; i++)
	{
		OSRAMImageDraw(xaxis, i, 0, 1, 2);
	}
	if (x_start < 96)
		x_start++;
	else
	{
		x_start = 20;
	}
}
/*-----------------------------------------------------------*/
