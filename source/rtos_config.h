/*
 * rtos_config.h
 *
 *  Created on: Oct 21, 2017
 *      Author: aldana
 *  Modified on: Nov 21 2017
 *      By: Enrique Galan
 */

#ifndef SOURCE_RTOS_CONFIG_H_
#define SOURCE_RTOS_CONFIG_H_

#include "stdint.h"


/**********************************************************************************/
// RTOS functional parameters
/**********************************************************************************/
//#define RTOS_TIC_PERIOD_IN_US 		(1000)

#define RTOS_STACK_SIZE				(100)

#define RTOS_MAX_NUMBER_OF_TASKS	(10)

#define RTOS_MAX_NUMBER_OF_ALARMS   (5)

/**********************************************************************************/
// RTOS IS ALIVE SIGNAL PARAMETERS
/**********************************************************************************/
#define RTOS_ENABLE_IS_ALIVE
#ifdef RTOS_ENABLE_IS_ALIVE
#define RTOS_IS_ALIVE_PORT			E
#define RTOS_IS_ALIVE_PIN			26
#define RTOS_IS_ALIVE_PERIOD_IN_US  (500000)
#endif

extern uint64_t RTOS_TIC_PERIOD_IN_US;


#endif /* SOURCE_RTOS_CONFIG_H_ */
