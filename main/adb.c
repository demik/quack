/*
 *  adb.c
 *  quack
 *
 *  Created by Michel DEPEIGE on 7/01/2020.
 *  Copyright (c) 2020 Michel DEPEIGE.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program (see the file COPYING); if not, write to the
 * Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA
 *
 */

#include <stdio.h>
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/rmt.h"

#include "adb.h"
#include "led.h"
#include "gpio.h"

/* globals */
rmt_config_t adb_rmt_rx = RMT_DEFAULT_CONFIG_RX(GPIO_ADB, RMT_RX_CHANNEL);
extern TaskHandle_t t_green, t_blue, t_yellow, t_red;

/* static defines */
static uint16_t	adb_rx_mouse();
static bool adb_rx_tlt(void);
static void adb_tx_as(void);
static void	adb_tx_one(void);
static void	adb_tx_zero(void);

/* functions */

void	adb_init(void) {
	/* initialise */
	gpio_set_level(GPIO_ADB, 1);
	adb_tx_reset();

	/* avoir console flood when installing/uninstalling RMT driver */
	esp_log_level_set("intr_alloc", ESP_LOG_INFO);

	/* init RMT RX driver with default values for ADB  */
	adb_rmt_rx.rx_config.filter_en = true;
	adb_rmt_rx.rx_config.filter_ticks_thresh = 10;
	adb_rmt_rx.rx_config.idle_threshold = 100;
	rmt_config(&adb_rmt_rx);

	/* If jumper is set, switch to ADB host mode */
	if (gpio_get_level(GPIO_ADBSRC) == 0)
		xTaskCreatePinnedToCore(&adb_task_host, "ADB_HOST", 6 * 1024, NULL, tskADB_PRIORITY, NULL, 1);
	else
		xTaskCreatePinnedToCore(&adb_task_mouse, "ADB_MOUSE", 6 * 1024, NULL, tskADB_PRIORITY, NULL, 1);
}

void	adb_task_host(void *pvParameters) {
	uint16_t	data;

	if (gpio_get_level(GPIO_BTOFF) == 0)
		xTaskNotify(t_green, LED_ON, eSetValueWithOverwrite);
	ESP_LOGI("ADB", "ADB host started");

	/* poll the mouse like a maniac. It will answer only if there is user input */
	ESP_ERROR_CHECK(rmt_driver_install(RMT_RX_CHANNEL, 200, 0));

	while (1) {
		vTaskDelay(20 / portTICK_PERIOD_MS);
		adb_tx_cmd(ADB_MOUSE|ADB_TALK|ADB_REG0);
		data = adb_rx_mouse();
	}
}

void	adb_task_mouse(void *pvParameters) {

}

int dur( uint32_t level, uint32_t duration ) {
	if ( level == 0 ) { return duration; }
	else { return -1.0 * duration; }
}

static uint16_t	adb_rx_mouse() {
	RingbufHandle_t rb = NULL;
	rmt_item32_t* items = NULL;
	size_t rx_size = 0;
	size_t i;

	rmt_get_ringbuf_handle(RMT_RX_CHANNEL, &rb);
	assert(rb != NULL);
	rmt_rx_start(RMT_RX_CHANNEL, true);
	items = (rmt_item32_t*) xRingbufferReceive(rb, &rx_size, pdMS_TO_TICKS(10));
	printf( ">>> %i\n", rx_size);
	rmt_rx_stop(RMT_RX_CHANNEL);

	if (rx_size > (1 * sizeof(rmt_item32_t)))
		xTaskNotify(t_yellow, LED_ONCE, eSetValueWithOverwrite);
	else
		return 0;
	for (i = 0; i < (rx_size / sizeof(rmt_item32_t)); i++) {
		printf("%d:%dus %d:%dus\n", (items+i)->level0, (items+i)->duration0, (items+i)->level1, (items+i)->duration1);
	}
	vRingbufferReturnItem(rb, (void*) items);

	return 0;
}

static inline void	adb_tx_as() {
	/* send attention (800 µs low) + sync (70 µs high) */
	gpio_set_level(GPIO_ADB, 0);
	ets_delay_us(800-1);
	gpio_set_level(GPIO_ADB, 1);
	ets_delay_us(80-1);
}

void	adb_tx_cmd(unsigned char cmd) {
	gpio_set_direction(GPIO_ADB, GPIO_MODE_OUTPUT);

	adb_tx_as();

	/* send actual byte (unrolled loop) */
	cmd & 0x80 ? adb_tx_one() : adb_tx_zero();
	cmd & 0x40 ? adb_tx_one() : adb_tx_zero();
	cmd & 0x20 ? adb_tx_one() : adb_tx_zero();
	cmd & 0x10 ? adb_tx_one() : adb_tx_zero();
	cmd & 0x08 ? adb_tx_one() : adb_tx_zero();
	cmd & 0x04 ? adb_tx_one() : adb_tx_zero();
	cmd & 0x02 ? adb_tx_one() : adb_tx_zero();
	cmd & 0x01 ? adb_tx_one() : adb_tx_zero();

	/* stop bit */
	adb_tx_zero();
	gpio_set_direction(GPIO_ADB, GPIO_MODE_INPUT);
	gpio_set_pull_mode(GPIO_ADB, GPIO_PULLUP_ONLY);
}

static inline void	adb_tx_one() {
	/* values from AN591 Datasheet minus the estimated call to ets_delay_us */
	gpio_set_level(GPIO_ADB, 0);
	ets_delay_us(ADB_1_LOW - 1);
	gpio_set_level(GPIO_ADB, 1);
	ets_delay_us(ADB_1_HIGH - 1);
}

void	adb_tx_reset() {
	/*
	 * ADB spec says reset signal low for 3ms ±30%
	 * Note that the ADB Desktop Bus Mouse G5431 uses 4ms when plugged
	 * ADB mouse init in <70ms, but lets wait 500ms to be sure
	 */

	gpio_set_level(GPIO_ADB, 0);
	ets_delay_us(ADB_RESET);
	gpio_set_level(GPIO_ADB, 1);
	ets_delay_us(500);
}

static inline void	adb_tx_zero() {
	/* values from AN591 Datasheet minus the estimated call to ets_delay_us */
	gpio_set_level(GPIO_ADB, 0);
	ets_delay_us(ADB_0_LOW - 1);
	gpio_set_level(GPIO_ADB, 1);
	ets_delay_us(ADB_0_HIGH - 1);
}
