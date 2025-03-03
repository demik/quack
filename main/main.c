/*
 *  main.c
 *  quack
 *
 *  Created by Michel DEPEIGE on 14/12/2020.
 *  Copyright (c) 2020 Michel DEPEIGE.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the Apache License, Version 2.0 (the "License");
 * You may obtain a copy of the License at:
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <stdio.h>
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_flash.h"
#include "esp_hidh.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_gap_bt_api.h"

#include "adb.h"
#include "blue.h"
#include "led.h"
#include "gpio.h"
#include "quad.h"

/* global TAG for ESP_LOG */
static const char* TAG = "quack";

void app_main(void)
{
	uint32_t flash_size;
	esp_flash_get_size(NULL, &flash_size);

	/* Print chip information */
	esp_chip_info_t chip_info;
	esp_chip_info(&chip_info);
	ESP_LOGI(TAG, "This is %s chip with %d CPU cores, WiFi%s%s, "
			"revision %d, %" PRIu32 "MB %s flash",
			CONFIG_IDF_TARGET,
			chip_info.cores,
			(chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
			(chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "",
			chip_info.revision, flash_size / (1024 * 1024),
			(chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

	ESP_LOGI(TAG, "Minimum free heap size: %" PRIu32 " bytes", esp_get_minimum_free_heap_size());
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
		gpio_transceiver_enable();

	/* put LED error ON if no inputs (/BTOFF and no /ADBSRC) */
	if (gpio_get_level(GPIO_BTOFF) == 0 && gpio_get_level(GPIO_ADBSRC) == 1) {
		ESP_LOGE(TAG, "Bluetooth is off and ADB is NOT in host mode!");
		xTaskNotify(t_red, LED_ON, eSetValueWithOverwrite);
	}
}
