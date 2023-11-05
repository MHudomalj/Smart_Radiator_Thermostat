/*
 * thermostat.c
 *
 *  Created on: 3 Okt 2023
 *      Author: MHudo
 */

#include "thermostat.h"
#include <stdio.h>
#include "cyhal.h"
#include "cybsp.h"
#include "timers.h"
#include "cyhal_gpio.h"
#include "cyhal_adc.h"
#include "mtb_thermistor_ntc_gpio.h"
#include "subscriber_task.h"
#include "mqtt_task.h"
#include "mqtt_client_config.h"

cyhal_adc_t adc;
cyhal_adc_channel_t motor_channel;
mtb_thermistor_ntc_gpio_t thermistor;
mtb_thermistor_ntc_gpio_cfg_t thermistor_cfg = {
    .r_ref = SENSOR_THERM_R_REF,
    .b_const = SENSOR_THERM_B_CONST,
    .r_infinity = SENSOR_THERM_R_INFINITY,
};

TimerHandle_t temp_measure_timer_h;
QueueHandle_t thermostat_queue;
QueueHandle_t motor_queue;

cy_mqtt_publish_info_t publish =
{
    .qos = (cy_mqtt_qos_t) MQTT_MESSAGES_QOS,
    .retain = false,
    .dup = false
};

// Enable motor rotation in forward or reverse direction
void motor_drive(bool forward){
	if(forward){
	    cyhal_gpio_write(MOT_4, 0u);
	    cyhal_gpio_write(MOT_2, 1u);
	}
	else{
	    cyhal_gpio_write(MOT_1, 0u);
	    cyhal_gpio_write(MOT_3, 1u);
	}
}

// Stop motor
void motor_stop(){
    cyhal_gpio_write(MOT_1, 1u);
    cyhal_gpio_write(MOT_2, 0u);
    cyhal_gpio_write(MOT_3, 0u);
	cyhal_gpio_write(MOT_4, 1u);
}

// Fully open the valve
uint32_t valve_open(){
	int32_t voltage;
	uint32_t valve_time = 0;
	printf("Opening valve.\n");
	motor_drive(1);
	while(1){
		vTaskDelay(pdMS_TO_TICKS(100));
		voltage = cyhal_adc_read_uv(&motor_channel);
		//printf("Motor stall voltage %d\n", voltage);
		valve_time += 100;
	    if(voltage < MOTOR_STALL_VOLTAGE){
	    	printf("Motor opening time %ums\n", (uint)valve_time);
	    	break;
	    }
	}
	motor_stop();
	printf("Valve opened.\n");
	return valve_time;
}

// Fully close the valve
uint32_t valve_close(){
	int32_t voltage;
	uint32_t valve_time = 0;
	printf("Closing valve.\n");
	motor_drive(0);
	while(1){
		vTaskDelay(pdMS_TO_TICKS(100));
		voltage = cyhal_adc_read_uv(&motor_channel);
		//printf("Motor stall voltage %d\n", voltage);
		valve_time += 100;
	    if(voltage < MOTOR_STALL_VOLTAGE){
	    	printf("Motor closing time %ums\n", (uint)valve_time);
	    	break;
	    }
	}
	motor_stop();
	printf("Valve closed.\n");
	return valve_time;
}

// Motor task that initializes the motor signals and calibrates it on startup and sets the desired valve position
void motor_task(){
	cy_rslt_t result;

	// Initialize motor pin signals
    cyhal_gpio_init(MOT_1, CYHAL_GPIO_DIR_OUTPUT,
    		CYHAL_GPIO_DRIVE_STRONG, 1);
    cyhal_gpio_init(MOT_2, CYHAL_GPIO_DIR_OUTPUT,
    		CYHAL_GPIO_DRIVE_STRONG, 0);
    cyhal_gpio_init(MOT_3, CYHAL_GPIO_DIR_OUTPUT,
    		CYHAL_GPIO_DRIVE_STRONG, 0);
    cyhal_gpio_init(MOT_4, CYHAL_GPIO_DIR_OUTPUT,
    		CYHAL_GPIO_DRIVE_STRONG, 1);

    // Initialize motor stall detection signal with ADC
	#if (CYHAL_API_VERSION >= 2)
	static const cyhal_adc_channel_config_t DEFAULT_CHAN_CONFIG =
		{ .enable_averaging = false, .min_acquisition_ns = 10u, .enabled = true };
	result = cyhal_adc_channel_init_diff(&motor_channel, &adc, MOT_ADC, CYHAL_ADC_VNEG,
										 &DEFAULT_CHAN_CONFIG);
	#else // HAL API version 1
	result = cyhal_adc_channel_init(&motor_channel, motor_adc, MOT_ADC);
	#endif
	CY_ASSERT(result == CY_RSLT_SUCCESS);

	// Open and close the valve and time the duration that can be used for linear valve control
	uint32_t valve_time = 0;
	valve_time = valve_open();
	valve_time = valve_close();

	uint8_t power;
	while(1){
		if(pdTRUE == xQueueReceive(motor_queue, &power, portMAX_DELAY)){
			switch(power){
			case 0:
				valve_time = valve_close();
				break;
			case 100:
				valve_time = valve_open();
				break;
			default:
				//for linear control
				break;
			}
		}
	}
}

// Timer callback for temperature measurement. Notifies the thermostat task.
void temp_measure_callback(TimerHandle_t xTimer){
    thermostat_queue_t message;
    message.evt = THERMOSTAT_TEMP_TIMEOUT;
    if(pdTRUE != xQueueSend(thermostat_queue, &message, 0)){
    	printf("Thermostat message queue full\n");
    }
}

// Control of the valve based on the mode, target temperature and current temperature.
// Only basic control is implemented.
bool thermostat_control(thermostat_mode_t mode, uint8_t * valve_level, float target_temp, float current_temp){
	bool update = false;
	uint8_t valve_level_next = 0;
	switch(mode){
	case MODE_OFF:
		if(valve_level_next != *valve_level){
			*valve_level = valve_level_next;
			if(pdTRUE != xQueueOverwrite(motor_queue, valve_level)){
				printf("Motor message queue problem.\n");
			}
			update = true;
		}
		break;
	case MODE_HEAT:
		if(target_temp < current_temp){
			valve_level_next = 0;
		}else{
			valve_level_next = 100;
		}
		if(valve_level_next != *valve_level){
			*valve_level = valve_level_next;
			if(pdTRUE != xQueueOverwrite(motor_queue, valve_level)){
				printf("Motor message queue problem.\n");
			}
			update = true;
		}
		break;
	default:
		break;
	}
	return update;
}

// Thermostat task that handles settings from MQTT and measures the temperature.
void thermostat_task(){
	cy_rslt_t result;

	// Create thermostat task queue.
	thermostat_queue = xQueueCreate(THERMOSTAT_QUEUE_LENGTH, sizeof(thermostat_queue_t));
	if(thermostat_queue == NULL){
		printf("Thermostat queue not created.\n");
		return;
	}

	// Create motor queue.
	motor_queue = xQueueCreate(1, sizeof(uint8_t));
	if(motor_queue == NULL){
		printf("Motor queue not created.\n");
		return;
	}

    // Intialize adc for thermistor measurements.
    result = cyhal_adc_init(&adc, THERM_OUT_1, NULL);
    // Initialize thermistor
    result = mtb_thermistor_ntc_gpio_init(&thermistor, &adc,
        THERM_VDD, THERM_GND, THERM_OUT_1,
        &thermistor_cfg, MTB_THERMISTOR_NTC_WIRING_VIN_R_NTC_GND);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    // Create temperature timeout task on which to sample new ADC temperature measurement.
	temp_measure_timer_h = xTimerCreate("TEMP timer", pdMS_TO_TICKS(30000), pdTRUE, NULL, temp_measure_callback);
	if(temp_measure_timer_h == NULL){
		printf("Temperature timer not created.\n");
		return;
	}

	// Start temperature measurements
	if(pdTRUE != xTimerStart(temp_measure_timer_h, 0)){
		printf("Temperature timer not started.\n");
		return;
	}

	// Create motor task
	if(pdTRUE != xTaskCreate(motor_task, "Motor task", THERMOSTAT_TASK_STACK_SIZE,
                NULL, THERMOSTAT_TASK_PRIORITY, NULL)){
		printf("Motor task not created.\n");
		return;
	}

	// Received message
	thermostat_queue_t message;
	// String buffer for sending with MQTT messages
	char str[10];

	// Initial thermostat states
	thermostat_mode_t mode = MODE_OFF;
	uint8_t valve_level = 0;
	float target_temp = 21.0;
	float current_temp = 21.0;

	while(1){
		// Receive and parse the thermostat queue messages
		if(pdTRUE == xQueueReceive(thermostat_queue, &message, portMAX_DELAY)){
			bool pub = false;
			switch(message.evt){
			case THERMOSTAT_TEMP_TIMEOUT:
				current_temp = mtb_thermistor_ntc_gpio_get_temp(&thermistor);
			    printf("Temperature = %fC\n", current_temp);
			    sprintf(str,"%.1f", current_temp);
                publish.topic = MQTT_CURRENT_TEMPERATURE_TOPIC;
                publish.topic_len = (sizeof(MQTT_CURRENT_TEMPERATURE_TOPIC) - 1);
                publish.payload = str;
                publish.payload_len = strlen(str);
                pub = true;
				break;
			case THERMOSTAT_TEMP_SET:
				target_temp = message.data.temp_set;
				printf("Target temperature set to %.1f\n", target_temp);
				sprintf(str,"%.1f", target_temp);
                publish.topic = MQTT_TEMPERATURE_STATE_TOPIC;
                publish.topic_len = (sizeof(MQTT_TEMPERATURE_STATE_TOPIC) - 1);
                publish.payload = str;
                publish.payload_len = strlen(str);
                pub = true;
				break;
			case THERMOSTAT_MODE:
        		switch(message.data.mode){
				case MODE_OFF:
					mode = message.data.mode;
					printf("Mode changed to off.\n");
                    publish.topic = MQTT_MODE_STATE_TOPIC;
                    publish.topic_len = (sizeof(MQTT_MODE_STATE_TOPIC) - 1);
                    publish.payload = MQTT_MODE_OFF_MESSAGE;
                    publish.payload_len = (sizeof(MQTT_MODE_OFF_MESSAGE)-1);
                    pub = true;
					break;
				case MODE_HEAT:
					mode = message.data.mode;
					printf("Mode changed to heat.\n");
                    publish.topic = MQTT_MODE_STATE_TOPIC;
                    publish.topic_len = (sizeof(MQTT_MODE_STATE_TOPIC) - 1);
                    publish.payload = MQTT_MODE_HEAT_MESSAGE;
                    publish.payload_len = (sizeof(MQTT_MODE_HEAT_MESSAGE)-1);
                    pub = true;
					break;
				default:
					printf("Wrong thermostat message type.\n");
					break;
        		}
				break;
			default:
				printf("Wrong mode.\n");
				break;
			}
			// Send to MQTT topics for the updated state of the thermostat.
    		if(pub){
				if (CY_RSLT_SUCCESS != cy_mqtt_publish(mqtt_connection, &publish)){
					printf("Publisher: MQTT Publish failed with error 0x%0X.\n\n", (int)result);
					mqtt_task_cmd_t mqtt_task_cmd = HANDLE_MQTT_PUBLISH_FAILURE;
					if(pdTRUE != xQueueSend(mqtt_task_q, &mqtt_task_cmd, 0)){
						printf("MQTT queue full\n");
					}
				}
    		}
    		// Update thermostat control and send messages about the action on MQTT.
    		bool update = thermostat_control(mode, &valve_level, target_temp, current_temp);
    		if(update){
                publish.topic = MQTT_ACTION_TOPIC;
                publish.topic_len = (sizeof(MQTT_ACTION_TOPIC) - 1);
                if(valve_level == 100){
					publish.payload = MQTT_ACTION_HEATING_MESSAGE;
					publish.payload_len = (sizeof(MQTT_ACTION_HEATING_MESSAGE)-1);
                }
                else{
					publish.payload = MQTT_ACTION_OFF_MESSAGE;
					publish.payload_len = (sizeof(MQTT_ACTION_OFF_MESSAGE)-1);
                }
				if (CY_RSLT_SUCCESS != cy_mqtt_publish(mqtt_connection, &publish)){
					printf("Publisher: MQTT Publish failed with error 0x%0X.\n\n", (int)result);
					mqtt_task_cmd_t mqtt_task_cmd = HANDLE_MQTT_PUBLISH_FAILURE;
					if(pdTRUE != xQueueSend(mqtt_task_q, &mqtt_task_cmd, 0)){
						printf("MQTT queue full\n");
					}
				}
    		}
		}
	}
}



