/**
 * This is template for main module created by New Kinetis SDK 2.x Project Wizard. Enjoy!
 *  Modified on: Nov 21 2017
 *      By: Enrique Galan
 *
 *  The purpose of this practice is to demonstrate the use of
 *      - Tasks
 *      - Alarms
 *      - ISR Handler
 *      - Mailboxes
 **/

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "fsl_port.h"
#include "fsl_gpio.h"

#include "rtos.h"

rtos_task_handle_t task_1;
rtos_task_handle_t task_2;
rtos_task_handle_t task_3;
rtos_task_handle_t task_4;
rtos_task_handle_t task_5;
rtos_alarm_handle_t alarm_1;
rtos_alarm_handle_t alarm_2;
/*!
 * @brief Application entry point.
 */

void dummy_task1(void)
{
	uint8_t counter = 0;
	for(;;)
	{
		PRINTF("IN TASK 1: %i +++++++++++++++\r\n",counter);
		counter++;
		rtos_delay(8000);
	}
}

void dummy_task2(void)
{
	uint8_t counter = 0;
	for(;;)
	{
		PRINTF("IN TASK 2: %i ***************\r\n",counter);
		counter++;
		rtos_delay(1000);
	}
}

void dummy_task3(void)
{
	uint8_t counter = 0;
	for(;;)
	{
		PRINTF("IN TASK 3: %i ---------------\r\n",counter);
		counter++;
		rtos_delay(16000);
	}
}

void dummy_task4(void)
{
    uint8_t counter = 0;
    for(;;)
    {
        PRINTF("IN TASK 4: %i ###############\r\n",counter);
        rtos_suspend_task();
        counter++;
    }
}

void alarm_task_1(void)
{
    uint8_t counter = 0;
    for(;;)
    {
        PRINTF("ALARM 1 Triggered: %d /*/*/*/*/*/* resetting...\r\n",counter);
        rtos_suspend_task();
        counter++;
    }
}

void alarm_task_2(void)
{
    PRINTF("ALARM 2 Triggered:########### resetting...\r\n");
}

/* Interrupts definitions */
void PORTC_IRQHandler()
{
    PORT_ClearPinsInterruptFlags( PORTC, 1<<6 );
    PRINTF("Button pressed!!!!!\r\n");
    rtos_activate_alarm(alarm_2);
    _change_clk_freq();
}

int main(void)
{
  BOARD_InitPins();
  BOARD_BootClockRUN();
  BOARD_InitDebugConsole();

  setup_interrupt_button();
  enable_button_interrupt();

  task_1 = rtos_create_task(dummy_task1,1,kAutoStart);
  task_2 = rtos_create_task(dummy_task2,2,kAutoStart);
  task_3 = rtos_create_task(dummy_task3,3,kAutoStart);
  task_4 = rtos_create_task(dummy_task4,5,kStartSuspended);
  task_5 = rtos_create_task(alarm_task_1,4,kStartSuspended);
  alarm_1 = rtos_create_alarm(4500, aActivateTask, task_5, rContinuous);
  alarm_2 = rtos_create_alarm(3000, aSuspended, task_4,rOnetime);
  rtos_start_scheduler();

  for(;;)
  {
    __asm("NOP");
  }
}
