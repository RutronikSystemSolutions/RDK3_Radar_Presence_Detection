/*
 * notification_fabric.h
 *
 *  Created on: 28 Mar 2023
 *      Author: jorda
 */

#ifndef NOTIFICATION_FABRIC_H_
#define NOTIFICATION_FABRIC_H_

#include <stdint.h>

typedef struct
{
	uint8_t length;
	uint8_t* data;
} notification_t;

void notification_fabric_free_notification(notification_t* notification);

#define NOTIFICATION_RADAR_PRESENCE_TYPE_INFORMATION 3


notification_t* notification_fabric_create_for_battery_monitor(uint16_t voltage, uint8_t charge_status, uint8_t charge_fault, uint8_t dio_status);

/**
 * @param [in] type
 * 	0: Macro presence
 * 	1: Micro presence
 * 	2: Absence
 * 	3: Information
 */
notification_t* notification_fabric_create_for_radar_presence(uint32_t timestamp, float distance, uint8_t type, float maxmacro, float maxmicro, float thresholdmacro, float thresholdmicro);

#endif /* NOTIFICATION_FABRIC_H_ */
