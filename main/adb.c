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

void	adb_init(void) {
	gpio_set_pull_mode(GPIO_ADB, GPIO_PULLUP_ONLY);

	/* If jumper is set, switch to ADB host mode */
	if (gpio_get_level(GPIO_ADBSRC) == 0)
		xTaskCreatePinnedToCore(&adb_task_host, "ADB_HOST", 6 * 1024, NULL, 4, NULL, 1);
	else
		xTaskCreatePinnedToCore(&adb_task_mouse, "ADB_MOUSE", 6 * 1024, NULL, 4, NULL, 1);
}

void	adb_task_host(void *pvParameters) {
	adb_send_reset();

	while (1) {
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

void	adb_task_mouse(void *pvParameters) {

}

void	adb_send_reset() {
	gpio_set_level(GPIO_ADB, 0);
	vTaskDelay(3 / portTICK_PERIOD_MS);
	gpio_set_level(GPIO_ADB, 1);
	vTaskDelay(1 / portTICK_PERIOD_MS);
}
