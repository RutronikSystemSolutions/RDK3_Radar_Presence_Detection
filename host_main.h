/*
 * host_main.h
 *
 *  Created on: 24 Mar 2023
 *      Author: jorda
 */

#ifndef HOST_MAIN_H_
#define HOST_MAIN_H_

#include <stdint.h>

#include "notification_fabric.h"
#include "linked_list.h"

#define BLE_CMD_PARAM_MAX_SIZE 32
#define BLE_ACK_MAX_SIZE 32

#define BLE_MODE_CONFIGURATION	1
#define BLE_MODE_PUSH_DATA 		2

typedef struct
{
	uint8_t command;
	uint8_t parameters[BLE_CMD_PARAM_MAX_SIZE];
} ble_cmt_t;


typedef struct
{
	uint8_t notification_enabled;			/**< Store if notification on the characteristic are enabled or not. The app must activate them */

	uint8_t cmd_to_process;					/**< Store if a command is ready to be processed or not */
	ble_cmt_t cmd;							/**< Store the command to be processed (once available) */

	uint8_t ack_to_send;					/**< Store if an ACK has to be send */
	uint16_t ack_len;						/**< Length of the ACK packet */
	uint8_t ack_content[BLE_ACK_MAX_SIZE];	/**< Content of the ACK packet */

	uint8_t notification_to_send;			/**< Something to send? */
	notification_t* notification;

	linked_list_t notification_list;

	uint8_t mode;							/**< Store the actual configuration mode */
} host_main_t;

void Ble_Init();

int host_main_do();

int host_main_add_notification(notification_t* notification);

int host_main_is_ready_for_notification();

#endif /* HOST_MAIN_H_ */
