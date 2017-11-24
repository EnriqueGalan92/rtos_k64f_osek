/*
 * rtos.c
 *
 *  Created on: Oct 21, 2017
 *      Author: aldana
 *  Modified on: Nov 21 2017
 *      By: Enrique Galan
 */


#include "rtos.h"
#include "rtos_config.h"
#include "clock_config.h"

#ifdef RTOS_ENABLE_IS_ALIVE
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_debug_console.h"

#endif
/**********************************************************************************/
// Module defines
/**********************************************************************************/

#define _FORCE_INLINE 	__attribute__((always_inline)) inline

#define _STACK_FRAME_SIZE			8
#define _STACK_LR_OFFSET			2
#define _STACK_PSR_OFFSET			1
#define _STACK_PSR_DEFAULT			0x01000000

/**********************************************************************************/
// IS ALIVE definitions
/**********************************************************************************/

#ifdef RTOS_ENABLE_IS_ALIVE
#define _CAT_STRING(x,y)  		x##y
#define _alive_GPIO(x)			_CAT_STRING(GPIO,x)
#define _alive_PORT(x)			_CAT_STRING(PORT,x)
#define _alive_CLOCK(x)			_CAT_STRING(kCLOCK_Port,x)
static void _init_is_alive(void);
static void _refresh_is_alive(void);
#endif

/**********************************************************************************/
// Interrupt macros
/**********************************************************************************/

#define INIT_SYSTICK	SysTick->CTRL =SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk
#define INVOKE_PENDSV	SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk

/**********************************************************************************/
// Type definitions
/**********************************************************************************/

typedef enum {S_READY=0, S_RUNNING, S_WAITING, S_SUSPENDED } _task_state_e;
typedef enum {kFromISR = 0, kFromNormalExec} _task_switch_type_e;


typedef struct{
	uint8_t priority;
	_task_state_e state;
	uint32_t *sp;
	void (*task_body)();
	rtos_tick_t local_tick;
	uint32_t stack[RTOS_STACK_SIZE];
}rtos_tcb_t;

typedef struct{
    uint32_t counter;
    uint32_t set_counter;
    alarm_action_e action;
    rtos_task_handle_t task;
    alarm_recurrency_e recurrency;
}rtos_alarm_t;

/**********************************************************************************/
// Port Configurations
/**********************************************************************************/
port_pin_config_t config_sw = {
    kPORT_PullDisable,
    kPORT_FastSlewRate,
    kPORT_PassiveFilterDisable,
    kPORT_OpenDrainDisable,
    kPORT_LowDriveStrength,
    kPORT_MuxAsGpio,
    kPORT_UnlockRegister,
};

// Output pin configuration
gpio_pin_config_t led_config_input =
{
    kGPIO_DigitalInput,
    1,
};

/**********************************************************************************/
// Global (static) task list
/**********************************************************************************/

static struct
{
	uint8_t nTasks;
	rtos_task_handle_t current_task;
	rtos_task_handle_t next_task;
	rtos_tcb_t tasks[RTOS_MAX_NUMBER_OF_TASKS+1];
	rtos_tick_t global_tick;
}_task_list = {0};

/**********************************************************************************/
// Global (static) alarm list
/**********************************************************************************/
static struct
{
    uint8_t nAlarms;
    rtos_alarm_t alarms [RTOS_MAX_NUMBER_OF_ALARMS+1];
}_alarm_list = {0};
/**********************************************************************************/
// Local methods prototypes
/**********************************************************************************/

static void _reload_systick(void);
static void _scheduler(_task_switch_type_e type);
static void _activate_waiting_tasks();
_FORCE_INLINE static void _dispatcher(_task_switch_type_e type);
static void _idle_task(void);

/**********************************************************************************/
// API implementation
/**********************************************************************************/

void rtos_start_scheduler(void)
{
#ifdef RTOS_ENABLE_IS_ALIVE
	_init_is_alive();
#endif
	_task_list.global_tick = 0;
	_task_list.current_task = -1;
	rtos_create_task(_idle_task,0,kAutoStart);
	INIT_SYSTICK;
	_reload_systick();
	for(;;);
}


rtos_task_handle_t rtos_create_task(void (*task_body)(),uint8_t priority, uint8_t autostart)
{
	rtos_task_handle_t retval;
	if( RTOS_MAX_NUMBER_OF_TASKS-1 <_task_list.nTasks )
	{
		retval = -1;
	}
	else
	{
		_task_list.tasks[_task_list.nTasks].state = autostart == kAutoStart? S_READY: S_SUSPENDED;
		_task_list.tasks[_task_list.nTasks].priority = priority;
		_task_list.tasks[_task_list.nTasks].sp = &(_task_list.tasks[_task_list.nTasks].stack[RTOS_STACK_SIZE-1]) - _STACK_FRAME_SIZE;
		_task_list.tasks[_task_list.nTasks].stack[RTOS_STACK_SIZE - _STACK_LR_OFFSET] = (uint32_t)task_body;
		_task_list.tasks[_task_list.nTasks].stack[RTOS_STACK_SIZE - _STACK_PSR_OFFSET] = _STACK_PSR_DEFAULT;
		_task_list.tasks[_task_list.nTasks].local_tick = 0;
		_task_list.nTasks++;
		retval = _task_list.nTasks - 1;
	}
	return retval;
}

rtos_tick_t rtos_get_clock(void)
{
	return _task_list.global_tick;
}

void rtos_delay(rtos_tick_t ticks)
{
	_task_list.tasks[_task_list.current_task].state = S_WAITING;
	_task_list.tasks[_task_list.current_task].local_tick = ticks;
	_scheduler(kFromNormalExec);
}

void rtos_suspend_task(void)
{
	_task_list.tasks[_task_list.current_task].state=S_SUSPENDED;
	_scheduler(kFromNormalExec);
}


void rtos_activate_task(rtos_task_handle_t task)
{
	_task_list.tasks[task].state=S_READY;
	_scheduler(kFromNormalExec);
}

/**********************************************************************************/
// ALARM handling
/**********************************************************************************/

rtos_alarm_handle_t rtos_create_alarm(uint32_t set_counter, uint8_t action, uint8_t task, uint8_t recurrency)
{
    rtos_alarm_handle_t retval;
    if(RTOS_MAX_NUMBER_OF_ALARMS-1 < _alarm_list.nAlarms)
    {
        retval = -1;
    }
    else
    {
        _alarm_list.alarms[_alarm_list.nAlarms].action = action;
        _alarm_list.alarms[_alarm_list.nAlarms].counter = set_counter;
        _alarm_list.alarms[_alarm_list.nAlarms].set_counter = set_counter;
        _alarm_list.alarms[_alarm_list.nAlarms].task = task;
        _alarm_list.alarms[_alarm_list.nAlarms].recurrency = recurrency;
        _alarm_list.nAlarms++;
        retval = _alarm_list.nAlarms - 1;
    }
    return retval;
}

void rtos_decrease_alarm(void)
{
    uint8_t alarm_index;
    for (alarm_index = 0; alarm_index < _alarm_list.nAlarms; alarm_index++)
    {
        if ( aActivateTask == _alarm_list.alarms[alarm_index].action )
        {
            _alarm_list.alarms[alarm_index].counter--;
            if (_alarm_list.alarms[alarm_index].counter == 0)
            {
                rtos_activate_alarm_task((rtos_alarm_handle_t)alarm_index);
                _alarm_list.alarms[alarm_index].counter =
                        _alarm_list.alarms[alarm_index].set_counter;
                if (rOnetime == _alarm_list.alarms[alarm_index].recurrency)
                {
                    _alarm_list.alarms[alarm_index].action = aSuspended;
                }
            }
        }
    }
}

void rtos_activate_alarm_task(rtos_alarm_handle_t alarm_index){
    rtos_task_handle_t task;

    if (_alarm_list.alarms[alarm_index].action == aActivateTask)
    {
        task = _alarm_list.alarms[alarm_index].task;
        _task_list.tasks[task].state=S_READY;
        PRINTF("ALARM ACTIVATED FOR TASK %d\n\r",task+1);
    }
}

void rtos_deactivate_alarm(rtos_alarm_handle_t alarm_index)
{
    _alarm_list.alarms[alarm_index].action = aSuspended;
}

void rtos_activate_alarm(rtos_alarm_handle_t alarm_index)
{
    _alarm_list.alarms[alarm_index].action = aActivateTask;
}

/**********************************************************************************/
// Local methods implementation
/**********************************************************************************/

static void _reload_systick(void)
{
	SysTick->LOAD = USEC_TO_COUNT(RTOS_TIC_PERIOD_IN_US,CLOCK_GetCoreSysClkFreq());
	SysTick->VAL = 0;
}

static void _scheduler(_task_switch_type_e type)
{
	uint8_t task_index;
	int8_t highest = -1;
	uint8_t next_task_handle=_task_list.nTasks;
	for(task_index = 0 ; task_index < _task_list.nTasks ; task_index++)
	{
		if(_task_list.tasks[task_index].priority>highest && (S_READY == _task_list.tasks[task_index].state || S_RUNNING == _task_list.tasks[task_index].state))
		{
			highest = _task_list.tasks[task_index].priority>highest;
			next_task_handle = task_index;
		}
	}
	_task_list.next_task = next_task_handle;
	if(_task_list.next_task != _task_list.current_task)
	{
		_dispatcher(type);
	}
}

_FORCE_INLINE static void _dispatcher(_task_switch_type_e type)
{
	static uint8_t first = 1;
	register uint32_t r0 asm("r0"); (void)r0;
	if(!first)
	{
		asm("mov r0, r7");
		_task_list.tasks[_task_list.current_task].sp = (uint32_t*)r0;
		_task_list.tasks[_task_list.current_task].sp-= kFromNormalExec==type?_STACK_FRAME_SIZE + 1:-(_STACK_FRAME_SIZE-1);
	}
	else
	{
		first = 0;
	}
	INVOKE_PENDSV;
}

static void _activate_waiting_tasks()
{
	uint8_t task_index;
	for(task_index = 0 ; task_index < _task_list.nTasks ; task_index++)
	{
		if(S_WAITING == _task_list.tasks[task_index].state)
		{
			_task_list.tasks[task_index].local_tick--;
			if(0 == _task_list.tasks[task_index].local_tick)
			{
				_task_list.tasks[task_index].state=S_READY;
			}
		}
	}
}

/**********************************************************************************/
// IDLE TASK
/**********************************************************************************/

static void _idle_task(void)
{
	for(;;)
	{
		asm("nop");
	}
}

/**********************************************************************************/
// ISR implementation
/**********************************************************************************/

void SysTick_Handler(void)
{
	_task_list.global_tick++;
#ifdef RTOS_ENABLE_IS_ALIVE
	_refresh_is_alive();
#endif
	_activate_waiting_tasks();
	_reload_systick();
	rtos_decrease_alarm();
	_scheduler(kFromISR);
}

void PendSV_Handler(void)
{
	register int32_t r0 asm("r0"); (void)r0;
	SCB->ICSR |= SCB_ICSR_PENDSVCLR_Msk;
	_task_list.current_task = _task_list.next_task;
	_task_list.tasks[_task_list.current_task].state = S_RUNNING;
	r0 = (int32_t)_task_list.tasks[_task_list.current_task].sp;
	asm("mov r7,r0");
}

/**********************************************************************************/
// ISR Interrupt button implementation
/**********************************************************************************/
void setup_interrupt_button (void)
{
    CLOCK_EnableClock(kCLOCK_PortC);
    PORT_SetPinConfig(PORTC, 6, &config_sw);
    GPIO_PinInit(GPIOC, 6, &led_config_input);
}

void enable_button_interrupt (void)
{
    NVIC_EnableIRQ(PORTC_IRQn);
    NVIC_SetPriority(PORTC_IRQn, 5);
    PORT_SetPinInterruptConfig(PORTC, 6, kPORT_InterruptFallingEdge);
}

void rtos_button_interrupt(rtos_task_handle_t task)
{
    _task_list.tasks[task].state=S_READY;
    PRINTF("BUTTON ACTIVATED TASK %d\n\r",task+1);
}
/**********************************************************************************/
// IS ALIVE SIGNAL IMPLEMENTATION
/**********************************************************************************/


#ifdef RTOS_ENABLE_IS_ALIVE
static void _init_is_alive(void)
{
	gpio_pin_config_t gpio_config =
	{
			kGPIO_DigitalOutput,
			1,
	};

	port_pin_config_t port_config = {
			kPORT_PullDisable,
			kPORT_FastSlewRate,
			kPORT_PassiveFilterDisable,
			kPORT_OpenDrainDisable,
			kPORT_LowDriveStrength,
			kPORT_MuxAsGpio,
			kPORT_UnlockRegister,
	};
	CLOCK_EnableClock(_alive_CLOCK(RTOS_IS_ALIVE_PORT));
	PORT_SetPinConfig(_alive_PORT(RTOS_IS_ALIVE_PORT), RTOS_IS_ALIVE_PIN, &port_config);
	GPIO_PinInit(_alive_GPIO(RTOS_IS_ALIVE_PORT), RTOS_IS_ALIVE_PIN, &gpio_config);
}

static void _refresh_is_alive(void)
{
	static uint8_t state = 0;
	static uint32_t count = 0;
	SysTick->LOAD =USEC_TO_COUNT(RTOS_TIC_PERIOD_IN_US,CLOCK_GetCoreSysClkFreq());
	SysTick->VAL = 0;
	if(RTOS_IS_ALIVE_PERIOD_IN_US/RTOS_TIC_PERIOD_IN_US - 1 == count)
	{
		GPIO_WritePinOutput(_alive_GPIO(RTOS_IS_ALIVE_PORT), RTOS_IS_ALIVE_PIN,state);
		state = state == 0 ? 1 : 0;
		count = 0;
	}
	else
	{
		count++;
	}
}

/**********************************************************************************/
// CLOCK FREQUENCY CHANGE IMPLEMENTATION
/**********************************************************************************/
void _change_clk_freq(void)
{
    static uint8_t clock_switch = 0;
    clock_switch ++;
    if (3 == clock_switch)
    {
        clock_switch = 0;
    }
    switch (clock_switch)
    {
        case 0:
            RTOS_TIC_PERIOD_IN_US = 1000;
            break;
        case 1:
            RTOS_TIC_PERIOD_IN_US = 2000;
            break;
        case 2:
            RTOS_TIC_PERIOD_IN_US = 4000;
            break;
        default:
            RTOS_TIC_PERIOD_IN_US = 10000;
            break;
    }
}
#endif
