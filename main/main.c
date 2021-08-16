/*
 *  main.c
 *  quack
 *
 *  Created by Michel DEPEIGE on 14/12/2020.
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
#include "esp_hidh.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

#include "adb.h"
#include "blue.h"
#include "led.h"
#include "gpio.h"
#include "quad.h"

/* global TAG for ESP_LOG */
static const char* TAG = "quack";

void app_main(void)
{
	/* Print chip information */
	esp_chip_info_t chip_info;
	esp_chip_info(&chip_info);
	ESP_LOGI(TAG, "This is %s chip with %d CPU cores, WiFi%s%s, "
			"revision %d, %dMB %s flash",
			CONFIG_IDF_TARGET,
			chip_info.cores,
			(chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
			(chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "",
			chip_info.revision,
			spi_flash_get_chip_size() / (1024 * 1024),
			(chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

	ESP_LOGI(TAG, "Minimum free heap size: %d bytes", esp_get_minimum_free_heap_size());
	ESP_LOGI(TAG, "");
	ESP_LOGI(TAG, "\\_o< \\_o< \\_o< \\_O<");
	ESP_LOGI(TAG, "");

	gpio_init();
	led_init();
	quad_init();
	adb_init();

	/*
	 * Check /BTOFF and enable bluetooth if needed
	 * of bluetooth is disabled, enable quadrature outputs
	 */
	if (gpio_get_level(GPIO_BTOFF) == 1)
		blue_init();
	else
		gpio_output_enable();

	/* put LED error ON if no inputs (/BTOFF and no /ADBSRC) */
	if (gpio_get_level(GPIO_BTOFF) == 0 && gpio_get_level(GPIO_ADBSRC) == 1) {
		ESP_LOGE(TAG, "Bluetooth is off and ADB is NOT in host mode!");
		xTaskNotify(t_red, LED_ON, eSetValueWithOverwrite);
	}
}
