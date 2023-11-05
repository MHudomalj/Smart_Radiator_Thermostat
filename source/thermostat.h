/*
 * thermostat.h
 *
 *  Created on: 3 Okt 2023
 *      Author: MHudo
 */

#ifndef SOURCE_THERMOSTAT_H_
#define SOURCE_THERMOSTAT_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// Thermostat task priority, stack size and queue length
#define THERMOSTAT_TASK_PRIORITY	(2)
#define THERMOSTAT_TASK_STACK_SIZE	(1024 * 1)
#define THERMOSTAT_QUEUE_LENGTH		(10u)
// Motor task priority and stack size
#define MOTOR_TASK_PRIORITY		(2)
#define MOTOR_TASK_STACK_SIZE	(1024 * 1)

// Voltage when to detect motor stall condition (determined from the ADC uV measurements)
#define MOTOR_STALL_VOLTAGE 1330000		//1.33V

/* Thermistor Constants */
/** Resistance of the reference resistor */
#define SENSOR_THERM_R_REF                         (float)(10000)
/** Beta constant of the (NCP18XH103F03RB) thermistor (3380 Kelvin).See the
 * thermistor datasheet for more details. */
#define SENSOR_THERM_B_CONST                       (float)(3380)
/** Resistance of the thermistor is 10K at 25 degrees C (from datasheet)
 * Therefore R0 = 10000 Ohm, and T0 = 298.15 Kelvin, which gives
 * R_INFINITY = R0 e^(-B_CONSTANT / T0) = 0.1192855 */
#define SENSOR_THERM_R_INFINITY                    (float)(0.1192855)

// Thermistor pins
#define THERM_GND P10_0
#define THERM_VDD P10_3
#define THERM_OUT_1 P10_1
// Motor control pins
#define MOT_1 P8_0
#define MOT_2 P9_2
#define MOT_3 P9_1
#define MOT_4 P9_4
// Motor stall pin
#define MOT_ADC P10_5

// Thermostat events
typedef enum{
	THERMOSTAT_MODE,
	THERMOSTAT_TEMP_SET,
	THERMOSTAT_TEMP_TIMEOUT,
} thermostat_evt_t;

// Thermostat modes
typedef enum{
	MODE_OFF,
	MODE_HEAT
} thermostat_mode_t;

// Thermostat queue message structure
typedef struct{
	thermostat_evt_t evt;
	union{
		float temp_set;
		thermostat_mode_t mode;
	} data;
} thermostat_queue_t;

// Thermostat queue
extern QueueHandle_t thermostat_queue;
// Thermostat task
void thermostat_task();

#endif /* SOURCE_THERMOSTAT_H_ */




