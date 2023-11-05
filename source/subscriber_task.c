/******************************************************************************
* File Name:   subscriber_task.c
*
* Description: This file contains the task that subscribes to the topics.
*
*******************************************************************************
* Copyright 2020-2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include <stdbool.h>
#include <stdio.h>

#include "cyhal.h"
#include "cybsp.h"
#include "string.h"
#include "FreeRTOS.h"

/* Task header files */
#include "subscriber_task.h"
#include "mqtt_task.h"
#include "thermostat.h"

/* Configuration file for MQTT client */
#include "mqtt_client_config.h"

/* Middleware libraries */
#include "cy_mqtt_api.h"
#include "cy_retarget_io.h"

/******************************************************************************
* Macros
******************************************************************************/
/* Maximum number of retries for MQTT subscribe operation */
#define MAX_SUBSCRIBE_RETRIES                   (3u)

/* Time interval in milliseconds between MQTT subscribe retries. */
#define MQTT_SUBSCRIBE_RETRY_INTERVAL_MS        (1000)

/* The number of MQTT topics to be subscribed to. */
#define SUBSCRIPTION_COUNT                      (1)

/* Queue length of a message queue that is used to communicate with the 
 * subscriber task.
 */
#define SUBSCRIBER_TASK_QUEUE_LENGTH            (10u)

/******************************************************************************
* Global Variables
*******************************************************************************/
/* Task handle for this task. */
TaskHandle_t subscriber_task_handle;

/* Handle of the queue holding the commands for the subscriber task */
QueueHandle_t subscriber_task_q;

static cy_mqtt_subscribe_info_t subscribe_mode =
{
    .qos = (cy_mqtt_qos_t) MQTT_MESSAGES_QOS,
    .topic = MQTT_MODE_COMMAND_TOPIC,
    .topic_len = (sizeof(MQTT_MODE_COMMAND_TOPIC) - 1)
};

static cy_mqtt_subscribe_info_t subscribe_temp =
{
    .qos = (cy_mqtt_qos_t) MQTT_MESSAGES_QOS,
    .topic = MQTT_TARGET_TEMPERATURE_TOPIC,
    .topic_len = (sizeof(MQTT_TARGET_TEMPERATURE_TOPIC) - 1)
};

/******************************************************************************
* Function Prototypes
*******************************************************************************/
static void subscribe_to_topic(cy_mqtt_subscribe_info_t * topic);
static void unsubscribe_from_topic(cy_mqtt_subscribe_info_t * topic);
void print_heap_usage(char *msg);

/******************************************************************************
 * Function Name: subscriber_task
 ******************************************************************************
 * Summary:
 *  Task that sets up subscriptions.
 *
 * Parameters:
 *  void *pvParameters : Task parameter defined during task creation (unused)
 *
 * Return:
 *  void
 *
 ******************************************************************************/
void subscriber_task(void *pvParameters)
{
    subscriber_data_t subscriber_q_data;

    /* To avoid compiler warnings */
    (void) pvParameters;

    /* Create a message queue to communicate with other tasks and callbacks. */
    // There is a bug in the MQTT example where this is after the subscription to the topics.
    // The error can happen when subscription is done on topic with persistent message which
    // can be transmitted before the queue for subscriber data is created and so the program crashes.
    subscriber_task_q = xQueueCreate(SUBSCRIBER_TASK_QUEUE_LENGTH, sizeof(subscriber_data_t));

    /* Subscribe to the specified MQTT topic. */
    subscribe_to_topic(&subscribe_mode);
    subscribe_to_topic(&subscribe_temp);

    while (true)
    {
        /* Wait for commands from other tasks and callbacks. */
        if (pdTRUE == xQueueReceive(subscriber_task_q, &subscriber_q_data, portMAX_DELAY))
        {
            switch(subscriber_q_data.cmd)
            {
                case SUBSCRIBE_TO_TOPIC:
                {
                    subscribe_to_topic(&subscribe_mode);
                    subscribe_to_topic(&subscribe_temp);
                    break;
                }

                case UNSUBSCRIBE_FROM_TOPIC:
                {
                    unsubscribe_from_topic(&subscribe_mode);
                    unsubscribe_from_topic(&subscribe_temp);
                    break;
                }

                case UPDATE_DEVICE_STATE:
                {
                    print_heap_usage("subscriber_task");
                    break;
                }
            }
        }
    }
}

/******************************************************************************
 * Function Name: subscribe_to_topic
 ******************************************************************************
 * Summary:
 *  Function that subscribes to the MQTT topics
 *
 * Parameters:
 *  cy_mqtt_subscribe_info_t * topic : topic on which to subscribe to
 *
 * Return:
 *  void
 *
 ******************************************************************************/
static void subscribe_to_topic(cy_mqtt_subscribe_info_t * topic)
{
    /* Status variable */
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* Command to the MQTT client task */
    mqtt_task_cmd_t mqtt_task_cmd;

    /* Subscribe with the configured parameters. */
    for (uint32_t retry_count = 0; retry_count < MAX_SUBSCRIBE_RETRIES; retry_count++)
    {
        result = cy_mqtt_subscribe(mqtt_connection, topic, SUBSCRIPTION_COUNT);
        if (result == CY_RSLT_SUCCESS)
        {
            printf("\nMQTT client subscribed to the topic '%.*s' successfully.\n", 
            		topic->topic_len, topic->topic);
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(MQTT_SUBSCRIBE_RETRY_INTERVAL_MS));
    }

    if (result != CY_RSLT_SUCCESS)
    {
        printf("\nMQTT Subscribe failed with error 0x%0X after %d retries...\n\n", 
               (int)result, MAX_SUBSCRIBE_RETRIES);

        /* Notify the MQTT client task about the subscription failure */
        mqtt_task_cmd = HANDLE_MQTT_SUBSCRIBE_FAILURE;
        xQueueSend(mqtt_task_q, &mqtt_task_cmd, portMAX_DELAY);
    }
}

/******************************************************************************
 * Function Name: mqtt_subscription_callback
 ******************************************************************************
 * Summary:
 *  Callback to handle incoming MQTT messages. This callback prints the 
 *  contents of the incoming message.
 *
 * Parameters:
 *  cy_mqtt_publish_info_t *received_msg_info : Information structure of the 
 *                                              received MQTT message
 *
 * Return:
 *  void
 *
 ******************************************************************************/
void mqtt_subscription_callback(cy_mqtt_publish_info_t *received_msg_info)
{
	/* Received MQTT message */
	const char *received_topic = received_msg_info->topic;
	int received_topic_len = received_msg_info->topic_len;
	const char *received_msg = received_msg_info->payload;
	int received_msg_len = received_msg_info->payload_len;

	// Print the message content.
	printf("  \nSubsciber: Incoming MQTT message received:\n"
		"    Publish topic name: %.*s\n"
		"    Publish QoS: %d\n"
		"    Publish payload: %.*s\n",
		received_topic_len, received_topic,
		(int) received_msg_info->qos,
		received_msg_len, received_msg);

	bool send = false;
	thermostat_queue_t message;

	/* Parse received MQTT message. */
	if ((strlen(MQTT_MODE_COMMAND_TOPIC) == received_topic_len) &&
		(strncmp(MQTT_MODE_COMMAND_TOPIC, received_topic, received_topic_len) == 0)){
		message.evt = THERMOSTAT_MODE;
		if((strlen(MQTT_MODE_OFF_MESSAGE) == received_msg_len) &&
			(strncmp(MQTT_MODE_OFF_MESSAGE, received_msg, received_msg_len) == 0)){
			message.data.mode = MODE_OFF;
			send = true;
		}
		else if((strlen(MQTT_MODE_HEAT_MESSAGE) == received_msg_len) &&
			(strncmp(MQTT_MODE_HEAT_MESSAGE, received_msg, received_msg_len) == 0)){
			message.data.mode = MODE_HEAT;
			send = true;
		}
		else{
			printf("  Subscriber: Received MQTT message is not valid!\n");
		}
	}
	else if((strlen(MQTT_TARGET_TEMPERATURE_TOPIC) == received_topic_len) &&
			(strncmp(MQTT_TARGET_TEMPERATURE_TOPIC, received_topic, received_topic_len) == 0)){
		message.evt = THERMOSTAT_TEMP_SET;
		// Convert string to float.
		bool float_idx = false;
		float temperature = 0;
		uint32_t temperature_int = 0;
		uint32_t decade = 1;
		for(int32_t i=received_msg_len-1; i>-1; i--){
			if(received_msg[i]>='0' && received_msg[i]<='9'){
				temperature_int += (received_msg[i]-'0')*decade;
				decade *= 10;
			}
			else if(received_msg[i]=='.'){
				temperature = (float)temperature_int/decade;
				temperature_int = 0;
				decade = 1;
				float_idx = true;
			}
			else{
				break;
			}
		}
		if(float_idx){
			temperature += (float)temperature_int;
			message.data.temp_set = temperature;
			send = true;
		}
		else{
			printf("  Subscriber: Received MQTT message is not valid!\n");
		}
	}
	else{
		printf("  Subscriber: Received MQTT message topic is not parsed!\n");
	}

	print_heap_usage("MQTT subscription callback");

	/* Send the data to thermostat task queue */
	if(send){
		if(pdTRUE != xQueueSend(thermostat_queue, &message, portMAX_DELAY)){
			printf("Thermostat queue full.\n");
		}
	}
}

/******************************************************************************
 * Function Name: unsubscribe_from_topic
 ******************************************************************************
 * Summary:
 *  Function that unsubscribes from the topic.
 *
 * Parameters:
 *  cy_mqtt_subscribe_info_t * topic : topic from which to unsubscribe
 *
 * Return:
 *  void 
 *
 ******************************************************************************/
static void unsubscribe_from_topic(cy_mqtt_subscribe_info_t * topic)
{
    cy_rslt_t result = cy_mqtt_unsubscribe(mqtt_connection, 
                                           (cy_mqtt_unsubscribe_info_t *) topic,
                                           SUBSCRIPTION_COUNT);

    if (result != CY_RSLT_SUCCESS)
    {
        printf("MQTT Unsubscribe operation failed with error 0x%0X!\n", (int)result);
    }
}

/* [] END OF FILE */
