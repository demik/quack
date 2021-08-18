/*
 *  gpio.c
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
#include "esp_system.h"
#include "esp_spi_flash.h"

#include "gpio.h"

void	gpio_init(void) {
	/* LEDs */
	gpio_reset_pin(GPIO_GREENLED);
	gpio_reset_pin(GPIO_BLUELED);
	gpio_reset_pin(GPIO_YELLOWLED);
	gpio_reset_pin(GPIO_REDLED);
	gpio_set_direction(GPIO_GREENLED, GPIO_MODE_OUTPUT);
	gpio_set_direction(GPIO_BLUELED, GPIO_MODE_OUTPUT);
	gpio_set_direction(GPIO_YELLOWLED, GPIO_MODE_OUTPUT);
	gpio_set_direction(GPIO_REDLED, GPIO_MODE_OUTPUT);

	/* External chips */
	gpio_reset_pin(GPIO_DIR);
	gpio_reset_pin(GPIO_OE);
	gpio_set_direction(GPIO_DIR, GPIO_MODE_OUTPUT);
	gpio_set_direction(GPIO_OE, GPIO_MODE_OUTPUT);

	/* Modes */
	gpio_reset_pin(GPIO_ADBSRC);
	gpio_reset_pin(GPIO_BTOFF);
	gpio_set_direction(GPIO_ADBSRC, GPIO_MODE_INPUT);
	gpio_set_direction(GPIO_BTOFF, GPIO_MODE_INPUT);

	/* ADB */
	gpio_set_direction(GPIO_ADB, GPIO_MODE_INPUT);
	gpio_set_level(GPIO_DIR, 0);

	/* Quadrature mouse */
	gpio_reset_pin(GPIO_CLICK);
	gpio_reset_pin(GPIO_QX1);
	gpio_reset_pin(GPIO_QX2);
	gpio_reset_pin(GPIO_QY1);
	gpio_reset_pin(GPIO_QY2);
	gpio_set_pull_mode(GPIO_CLICK, GPIO_PULLUP_ONLY);
	gpio_set_pull_mode(GPIO_QX1, GPIO_PULLUP_ONLY);
	gpio_set_pull_mode(GPIO_QX2, GPIO_PULLUP_ONLY);
	gpio_set_pull_mode(GPIO_QY1, GPIO_PULLUP_ONLY);
	gpio_set_pull_mode(GPIO_QY2, GPIO_PULLUP_ONLY);
	gpio_set_direction(GPIO_CLICK, GPIO_MODE_OUTPUT);
	gpio_set_direction(GPIO_QX1, GPIO_MODE_OUTPUT);
	gpio_set_direction(GPIO_QX2, GPIO_MODE_OUTPUT);
	gpio_set_direction(GPIO_QY1, GPIO_MODE_OUTPUT);
	gpio_set_direction(GPIO_QY2, GPIO_MODE_OUTPUT);
}

void    gpio_output_disable(void) {
	gpio_set_level(GPIO_OE, 1);
}

void    gpio_output_enable(void) {
	gpio_set_level(GPIO_OE, 0);
}
