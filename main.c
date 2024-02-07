/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the RDK3 Radar Presence Detection
*               Application for ModusToolbox.
*
* Related Document: See README.md
*
*
*  Created on: 2023-07-21
*  Company: Rutronik Elektronische Bauelemente GmbH
*  Address:
*  Author: ROJ030, GDR
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
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
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*
* Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
* including the software is for testing purposes only and,
* because it has limited functions and limited resilience, is not suitable
* for permanent use under real conditions. If the evaluation board is
* nevertheless used under real conditions, this is done at one’s responsibility;
* any liability of Rutronik is insofar excluded
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "hal/hal_i2c.h"

#include "host_main.h"
#include "rutronik_app.h"
#include "battery_booster.h"

/**
 * Timer used to enable periodic call to the Bluetooth stack
 */
static cyhal_timer_t bluetooth_timer;

/**
 * Used to signal to the main task that an update of the Bluetooth stack has to be done
 */
static bool ble_update_pending = false;

/**
 * @brief Interrupt service routine called when the timer reaches the compare value
 */
static void bluetooth_timer_isr(void *callback_arg, cyhal_timer_event_t event)
{
	(void) callback_arg;
	(void) event;
	ble_update_pending = true;
}

/**
 * @brief Initializes the timer used to update the Bluetooth stack periodically
 *
 * At the moment, an update is done every 2ms (frequency = 500 Hz)
 *
 * @retval CY_RSLT_SUCCESS success
 * @retval != CY_RSLT_SUCCESS something wrongs happened
 */
static cy_rslt_t bluetooth_timer_init(void)
{
	const uint32_t timer_frequency_hz = 10000;
	const uint32_t isr_frequency_hz = 500; // every 2ms -> ISR
	const uint8_t priority = 6;
	cyhal_timer_cfg_t configuration;
	cy_rslt_t result = CY_RSLT_SUCCESS;

	configuration.compare_value = 0;
	configuration.period = (timer_frequency_hz / isr_frequency_hz);
	configuration.direction = CYHAL_TIMER_DIR_UP;
	configuration.is_compare = false;
	configuration.is_continuous = true;
	configuration.value = 0;

	result = cyhal_timer_init(&bluetooth_timer, NC, NULL);
	if (result != CY_RSLT_SUCCESS) return result;

	result = cyhal_timer_configure(&bluetooth_timer, &configuration);
	if (result != CY_RSLT_SUCCESS) return result;

	result = cyhal_timer_set_frequency(&bluetooth_timer, timer_frequency_hz);
	if (result != CY_RSLT_SUCCESS) return result;

	cyhal_timer_register_callback(&bluetooth_timer, bluetooth_timer_isr, NULL);

	cyhal_timer_enable_event(&bluetooth_timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT, priority, true);

	result = cyhal_timer_start(&bluetooth_timer);
	return result;
}

/**
 * @brief Initializes the LEDs of the RDK3 board
 */
static cy_rslt_t init_leds()
{
	cy_rslt_t result = CY_RSLT_SUCCESS;

	result = cyhal_gpio_init(LED1, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
	result |= cyhal_gpio_init(LED2, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
	result |= cyhal_gpio_init(LED3, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

	if (result != CY_RSLT_SUCCESS) return result;

	// Set all LEDs to OFF
	cyhal_gpio_write((cyhal_gpio_t)LED1, CYBSP_LED_STATE_OFF);
	cyhal_gpio_write((cyhal_gpio_t)LED2, CYBSP_LED_STATE_OFF);
	cyhal_gpio_write((cyhal_gpio_t)LED3, CYBSP_LED_STATE_OFF);

	return CY_RSLT_SUCCESS;
}

/**
 * @brief Init and enable the battery charger controller
 */
static cy_rslt_t init_battery_charger_control()
{
	cy_rslt_t result = cyhal_gpio_init(CHR_DIS, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, false);

	if (result != CY_RSLT_SUCCESS) return result;

	cyhal_gpio_write((cyhal_gpio_t)CHR_DIS, false); // Charger ON

	return CY_RSLT_SUCCESS;
}

/**
 * @brief Main function
 */
int main(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
    	// Board init failed. Stop program execution
        CY_ASSERT(0);
    }

    // Enable global interrupts.
    __enable_irq();

    // Initialize retarget-io to use the debug UART port
    result = cy_retarget_io_init(KITPROG_TX, KITPROG_RX, CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {
    	// retarget-io init failed. Stop program execution
        CY_ASSERT(0);
    }

    printf("------------------------- \r\n");
    printf("Starting RDK3 application - RADAR PRESENCE DETECTION \r\n");
    printf("------------------------- \r\n");

    // Initialize the User LEDs
    result = init_leds();
    if (result != CY_RSLT_SUCCESS)
	{
		printf("Error during initialization of the LEDs : %lu \r\n", result);
		CY_ASSERT(0);
	}

    /*Initialise I2C*/
    result = hal_i2c_init() ;
    if( result != CY_RSLT_SUCCESS)
    {
		printf("Error during initialization of the I2C peripheral: %lu \r\n", result);
		CY_ASSERT(0);
    }
    /* Li-ION/Li-Po battery Booster Initialisation */
    if(!batt_boost_ctrl_init())
    {
    	printf("Battery power supply failure.\r\n");
    	CY_ASSERT(0);
    }

    // Initialize timer (Bluetooth stack update rate)
    result = bluetooth_timer_init();
    if (result != CY_RSLT_SUCCESS)
	{
		printf("Error during initialization of the timer: %lu \r\n", result);
		CY_ASSERT(0);
	}

    // Charger control
    result = init_battery_charger_control();
    if (result != CY_RSLT_SUCCESS)
	{
		printf("Error during initialization of the battery charger: %lu \r\n", result);
		CY_ASSERT(0);
	}

    // Initializes the bluetooth stack
    Ble_Init();

    // Initialize Rutronik main app
    rutronik_app_init();

    // Main loop
    for (;;)
    {
    	if (ble_update_pending)
    	{
    		ble_update_pending = false;
    		host_main_do();
    	}

    	rutronik_app_do();
    }
}

/* [] END OF FILE */
