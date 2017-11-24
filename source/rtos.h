/*
 * rtos.h
 *
 *  Created on: Oct 21, 2017
 *      Author: aldana
 *  Modified on: Nov 21 2017
 *      By: Enrique Galan
 */

#ifndef SOURCE_RTOS_H_
#define SOURCE_RTOS_H_

#include "rtos_config.h"
#include "stdint.h"


/**********************************************************************************/
// RTOS types
/**********************************************************************************/

typedef enum {kAutoStart, kStartSuspended} rtos_autostart_e;
typedef enum {aActivateTask, aSuspended, aCallback} alarm_action_e;
typedef enum {rContinuous, rOnetime} alarm_recurrency_e;

typedef int8_t rtos_task_handle_t;
typedef int8_t rtos_alarm_handle_t;
typedef uint64_t rtos_tick_t;

uint64_t RTOS_TIC_PERIOD_IN_US = 1000;
/**********************************************************************************/
// RTOS API
/**********************************************************************************/

/**
 * RTOS initialization function
 */
void rtos_start_scheduler(void);

/**
 * RTOS task creation
 */
rtos_task_handle_t rtos_create_task(void (*task_body)(), uint8_t priority, uint8_t autostart);

/**
 * RTOS task suspension
 */
void rtos_suspend_task(void);

/**
 * RTOS task activation
 */
void rtos_activate_task(rtos_task_handle_t task);

/**
 * returns RTOS global clock
 */
rtos_tick_t rtos_get_clock(void);

/**
 * RTOS delay
 */
void rtos_delay(rtos_tick_t ticks);

/**
 * RTOS alarm creation
 */
rtos_alarm_handle_t rtos_create_alarm(uint32_t set_counter, uint8_t action, uint8_t task, uint8_t recurrency);

/**
 * RTOS create_alarm
 */
void rtos_activate_alarm_task(rtos_alarm_handle_t alarm_index);

/**
 * RTOS activate alarm
 */
void rtos_activate_alarm(rtos_alarm_handle_t alarm_index);

/**
 * RTOS deactivate alarm
 */
void rtos_deactivate_alarm(rtos_alarm_handle_t alarm_index);

/**
 * RTOS decrease alarm counter
 */
void rtos_decrease_alarm(void);

/**
 * Setup of button for interrupt example
 */
void setup_interrupt_button (void);

/**
 * Setup the interrupt button and enable it
 */
void enable_button_interrupt(void);

/**
 * _rtos function to activate a task thru a button interrupt
 */
void rtos_button_interrupt(rtos_task_handle_t task);

/**
 * _rtos function to change clk frequency
 */
void _change_clk_freq(void);
#endif /* SOURCE_RTOS_H_ */
