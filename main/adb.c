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
#include "esp_system.h"
#include "esp_spi_flash.h"

#include "adb.h"
#include "gpio.h"

/* static defines */
static void adb_tx_async(void);
static void	adb_tx_one(void);
static void	adb_tx_zero(void);

/* functions */

void	adb_init(void) {
	/* If jumper is set, switch to ADB host mode */
	if (gpio_get_level(GPIO_ADBSRC) == 0)
		xTaskCreatePinnedToCore(&adb_task_host, "ADB_HOST", 6 * 1024, NULL, 4, NULL, 1);
	else
		xTaskCreatePinnedToCore(&adb_task_mouse, "ADB_MOUSE", 6 * 1024, NULL, 4, NULL, 1);

	/* initialise */
}

void	adb_task_host(void *pvParameters) {
	/* wait a little bit for H to set up, otherwise devices will not see the reset command */
	gpio_set_level(GPIO_ADB, 1);
	vTaskDelay(20 / portTICK_PERIOD_MS);
	adb_tx_reset();

	adb_tx_cmd(ADB_MOUSE|ADB_TALK|ADB_REG0);

	while (1) {
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

void	adb_task_mouse(void *pvParameters) {

}

static inline void	adb_tx_async() {
	/* send attention (800 µS low) + sync (70 µS high) */
	gpio_set_level(GPIO_ADB, 0);
	ets_delay_us(800);
	gpio_set_level(GPIO_ADB, 1);
	ets_delay_us(70);
}

void	adb_tx_cmd(unsigned char cmd) {
	adb_tx_async();

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
}

static inline void	adb_tx_one() {
	/* values from AN591 Datasheet minus the estimated call to ets_delay_us */
	gpio_set_level(GPIO_ADB, 0);
	ets_delay_us(30);
	gpio_set_level(GPIO_ADB, 1);
	ets_delay_us(60);
}

void	adb_tx_reset() {
	/*
	 * ADB spec says reset signal low for 3ms ±30%
	 * Note that the ADB Desktop Bus Mouse G5431 uses 4ms when plugged
	 */

	gpio_set_level(GPIO_ADB, 0);
	ets_delay_us(3000);
	gpio_set_level(GPIO_ADB, 1);
	ets_delay_us(1000);
}

static inline void	adb_tx_zero() {
	/* values from AN591 Datasheet minus the estimated call to ets_delay_us */
	gpio_set_level(GPIO_ADB, 0);
	ets_delay_us(60);
	gpio_set_level(GPIO_ADB, 1);
	ets_delay_us(30);
}
