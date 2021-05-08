/*
 *  gpio.c
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
