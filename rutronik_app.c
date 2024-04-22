/*
 * rutronik_app.c
 *
 *  Created on: 20 Jul 2023
 *      Author: jorda
 */

#include "rutronik_app.h"
#include "host_main.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "cyhal_spi.h"
#include "cycfg_pins.h"
#include "cyhal_system.h"

#include "hal/hal_i2c.h"
#include "dio59020/dio59020.h"
#include "battery_monitor/battery_monitor.h"

#include "xensiv_bgt60trxx_mtb.h"
#define XENSIV_BGT60TRXX_CONF_IMPL
#include "radar_settings.h"
#include "xensiv_radar_presence.h"

#define NUM_SAMPLES_PER_FRAME               (XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS *\
                                             XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME *\
                                             XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP)

#define NUM_SAMPLES_PER_CHIRP				XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP

#define SPI_FREQUENCY						15000000

/**
 * Internal structure used to retain run-time data
 */
typedef struct
{
	bool radar_values_available;
	uint64_t timestamp;
	uint8_t frame_counter;
	uint8_t battery_prescaler;
} rutronik_app_t;
static rutronik_app_t app;

/**
 * Handle to the SPI communication block. Enables to communicate over SPI with the BGT60TR13C IC.
 */
static cyhal_spi_t spi_obj;

/**
 * Handle to the BGT60TR13C IC. Enables to configure it and to read out its values.
 */
static xensiv_bgt60trxx_mtb_t sensor;

/**
 * Handle to the presence detection algorithm
 */
static xensiv_radar_presence_handle_t presence_handle;

/**
 * Configuration of the presence detection algorithm
 */
static const xensiv_radar_presence_config_t default_config =
{
    .bandwidth                         = XENSIV_BGT60TRXX_CONF_END_FREQ_HZ - XENSIV_BGT60TRXX_CONF_START_FREQ_HZ,
    .num_samples_per_chirp             = XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP,
    .micro_fft_decimation_enabled      = false,
    .micro_fft_size                    = 128,
    .macro_threshold                   = 0.5f,
    .micro_threshold                   = 12.5f,
    .min_range_bin                     = 1,
    .max_range_bin                     = 6,
    .macro_compare_interval_ms         = 250,
    .macro_movement_validity_ms        = 1000,
    .micro_movement_validity_ms        = 4000,
    .macro_movement_confirmations      = 0,
    .macro_trigger_range               = 1,
    .mode                              = XENSIV_RADAR_PRESENCE_MODE_MICRO_IF_MACRO,
    .macro_fft_bandpass_filter_enabled = false,
    .micro_movement_compare_idx       = 5
};

/**
 * Allocate enough memory for the radar data frame.
 * Remark: we are using global variables because the computation using global variables is faster
 */

/**
 * @var samples Store the raw ADC samples to be read from the FIFO
 */
static uint16_t samples[NUM_SAMPLES_PER_FRAME];

/**
 * @var frame Store the ADC samples converted into float and scaled between 0 and 1
 */
static float32_t frame[NUM_SAMPLES_PER_FRAME];

/**
 * @brief Initializes the SPI communication with the radar sensor
 *
 * @retval 0 Success
 * @retval -1 Error occurred during initialization
 * @retval -2 Cannot change the frequency
 */
static int init_spi()
{
	/* Set the SPI Clock to maximum */
	uint32_t spi_freq = cy_PeriClkFreqHz / 4;

	if (cyhal_spi_init(&spi_obj,
			ARDU_MOSI,
			ARDU_MISO,
			ARDU_CLK,
			NC,
			NULL,
			8,
			CYHAL_SPI_MODE_00_MSB,
			false) != CY_RSLT_SUCCESS)
	{
		printf("ERROR: cyhal_spi_init failed\n");
		return -1;
	}

	// Set the data rate to spi_freq Mbps
	if (cyhal_spi_set_frequency(&spi_obj, spi_freq) != CY_RSLT_SUCCESS)
	{
		printf("ERROR: cyhal_spi_set_frequency failed\n");
		return -2;
	}

	return 0;
}

/**
 * @brief Interrupt handler that will be called when a new frame is available in the FIFO of the BGT60TR13C IC
 * The frame needs then to be read using the function xensiv_bgt60trxx_get_fifo_data
 */
#if defined(CYHAL_API_VERSION) && (CYHAL_API_VERSION >= 2)
void xensiv_bgt60trxx_mtb_interrupt_handler(void *args, cyhal_gpio_event_t event)
#else
void xensiv_bgt60trxx_mtb_interrupt_handler(void *args, cyhal_gpio_irq_event_t event)
#endif
{
    CY_UNUSED_PARAMETER(args);
    CY_UNUSED_PARAMETER(event);
    app.radar_values_available = true;
}

/**
 * @brief Initializes the radar with a measurement configuration and start the measurement
 *
 * After calling this function, the radar IC BGT60TR13C will start to generate data frames at the configured time interval
 *
 * @retval 0: Success
 * @retval != 0: Something wrong happened
 */
static int init_radar()
{
	cy_rslt_t result = CY_RSLT_SUCCESS;

	if (init_spi() != 0) return -1;

	/*Initialize BGT60TR13C Power Control pin*/
    result = cyhal_gpio_init(ARDU_IO3, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, true); /*Turn it ON*/

    if (result != CY_RSLT_SUCCESS) return -2;

    /*Initialize NJR4652F2S2 POWER pin*/
    result = cyhal_gpio_init(ARDU_IO7, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_OPENDRAINDRIVESLOW, false); /*Keep it OFF*/
    if (result != CY_RSLT_SUCCESS) return -2;

    /*Must wait at least 1ms until the BGT60TR13C sensor power supply gets to nominal value*/
    CyDelay(10);

    result = xensiv_bgt60trxx_mtb_init(&sensor,
                                       &spi_obj,
									   ARDU_CS,
									   ARDU_IO4,
                                       register_list,
                                       XENSIV_BGT60TRXX_CONF_NUM_REGS);
    if (result != CY_RSLT_SUCCESS) return -3;

    // The sensor will generate an interrupt once the sensor FIFO level is NUM_SAMPLES_PER_FRAME
    result = xensiv_bgt60trxx_mtb_interrupt_init(&sensor,
    		NUM_SAMPLES_PER_FRAME,
			ARDU_IO6,
			CYHAL_ISR_PRIORITY_DEFAULT,
			xensiv_bgt60trxx_mtb_interrupt_handler,
			NULL);
    if (result != CY_RSLT_SUCCESS) return -4;

    if (xensiv_bgt60trxx_start_frame(&sensor.dev, true) != XENSIV_BGT60TRXX_STATUS_OK) return -6;

	return 0;
}

/**
 * @brief Callback being called by the presence detection algorithm when a presence (or absence) has been detected
 *
 * The callback will be called within the context call of the function xensiv_radar_presence_process_frame
 */
void presence_detection_cb(xensiv_radar_presence_handle_t handle,
                           const xensiv_radar_presence_event_t* event,
                           void *data)
{
    (void)handle;
    (void)data;
    float distance = 0;
    distance = xensiv_radar_presence_get_bin_length(presence_handle) * (float) event->range_bin;
    host_main_add_notification(
    		notification_fabric_create_for_radar_presence(event->timestamp, distance, event->state, 0, 0, default_config.macro_threshold, default_config.micro_threshold));

    switch(event->state)
    {
		case XENSIV_RADAR_PRESENCE_STATE_MACRO_PRESENCE:
			printf("Macro presence detected. Timestamp: %lu. Distance: %.2f m\r\n", event->timestamp, distance);
			break;
		case XENSIV_RADAR_PRESENCE_STATE_MICRO_PRESENCE:
			printf("Micro presence detected. Timestamp: %lu. Distance: %.2f m\r\n", event->timestamp, distance);
			break;
		case XENSIV_RADAR_PRESENCE_STATE_ABSENCE:
			printf("Absence detected.\r\n");
			break;
    }
}

/**
 * @brief Initializes the presence detection algorithm
 *
 * @retval 0 Success
 * @retval -1 Something wrong happened
 */
static int init_presence_detection()
{
	xensiv_radar_presence_set_malloc_free(malloc, free);

	if (xensiv_radar_presence_alloc(&presence_handle, &default_config) != 0)
	{
		printf("xensiv_radar_presence_alloc error. \r\n");
		return -1;
	}

	xensiv_radar_presence_set_callback(presence_handle, presence_detection_cb, NULL);

	return 0;
}

static int init_battery_monitoring()
{
	dio59020_init(hal_i2c_read_register, hal_i2c_write_register);
	battery_monitor_init();
	return 0;
}

void rutronik_app_init()
{
	app.radar_values_available = false;
	app.frame_counter = 0;
	app.timestamp = 0;
	app.battery_prescaler = 0;


	int retval = init_radar();
	if (retval != 0)
	{
		printf("init_radar returns: %d \r\n", retval);
		CY_ASSERT(0);
	}

	retval = init_presence_detection();
	if (retval != 0)
	{
		printf("init_presence_detection returns: %d \r\n", retval);
		CY_ASSERT(0);
	}

	retval = init_battery_monitoring();
	if (retval != 0)
	{
		printf("init_battery_monitoring returns: %d \r\n", retval);
		CY_ASSERT(0);
	}
}

void rutronik_app_do()
{
	static const float repetition_msec = XENSIV_BGT60TRXX_CONF_FRAME_REPETITION_TIME_S * 1000.f;
	static const uint8_t app_info_update_rate = 3;

	if (app.radar_values_available == false) return;
	app.radar_values_available = false;

	cyhal_gpio_toggle(LED1);

	// Get the FIFO data
	int32_t retval = xensiv_bgt60trxx_get_fifo_data(&sensor.dev, samples, NUM_SAMPLES_PER_FRAME);
	if (retval == XENSIV_BGT60TRXX_STATUS_OK)
	{
		// Need to convert the uint16_t samples into float32_t
		// because xensiv_radar_presence_process_frame expects a float array
		uint16_t *u16_ptr = &samples[0];
		float32_t *f_ptr = &frame[0];
		for (int32_t i = 0; i < NUM_SAMPLES_PER_FRAME; ++i)
		{
			*f_ptr++ = ((float32_t)(*u16_ptr++) / 4096.0F);
		}

		// Process the frame (presence detection)
		if (xensiv_radar_presence_process_frame(presence_handle, frame, app.timestamp) != XENSIV_RADAR_PRESENCE_OK)
		{
			printf("xensiv_radar_presence_process_frame error.\r\n");
		}

		// Send information to the app
		if (app.frame_counter == app_info_update_rate)
		{
			float macromax = 0;
			int macromaxindex = 0;
			xensiv_radar_presence_get_max_macro(presence_handle, &macromax, &macromaxindex);

			float micromax = 0;
			int micromaxindex = 0;
			xensiv_radar_presence_get_max_micro(presence_handle, &micromax, &micromaxindex);

			host_main_add_notification(
					notification_fabric_create_for_radar_presence(0, 0, NOTIFICATION_RADAR_PRESENCE_TYPE_INFORMATION, macromax, micromax, default_config.macro_threshold, default_config.micro_threshold));

			app.frame_counter  = 0;
		}
		else
		{
			app.frame_counter ++;
		}
	}
	else
	{
		printf("Error while getting the FIFO data %ld \r\n", retval);
		// An error occurred when reading the FIFO
		// Restart the frame generation
		xensiv_bgt60trxx_start_frame(&sensor.dev, false);
		xensiv_bgt60trxx_start_frame(&sensor.dev, true);
	}

	if (app.battery_prescaler == 0)
	{
		// Get the battery voltage
		uint16_t battery_voltage = battery_monitor_get_voltage_mv();

		charge_stat_t charge_stat;
		dio_get_status(&charge_stat);

		chrg_fault_t chrg_fault;
		dio_get_fault(&chrg_fault);

		uint8_t dio_status;
		dio_monitor_read_raw(&dio_status);

		host_main_add_notification(
				notification_fabric_create_for_battery_monitor(battery_voltage, (uint8_t) charge_stat, (uint8_t) chrg_fault, dio_status));
	}
	app.battery_prescaler++;
	if (app.battery_prescaler >= 10) app.battery_prescaler = 0;

	// Update the timestamp
	app.timestamp += (uint64_t) repetition_msec;
}

