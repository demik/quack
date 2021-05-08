/*
 *  quad.c
 *  quack
 *
 *  Created by Michel DEPEIGE on 17/04/2021.
 *  Copyright (c) 2021 Michel DEPEIGE.
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
#include "esp_err.h"
#include "esp_log.h"
#include "driver/timer.h"
#include "esp_timer.h"

#include "quad.h"
#include "gpio.h"

/* global variables for tasks handles */
TaskHandle_t t_click, t_qx, t_qy;
esp_timer_handle_t quad_qx, quad_qy;

/* static functions */
static void	IRAM_ATTR quad_timer(void* arg);

/* phases */
const bool q1[] = {true, false, false, true};
const bool q2[] = {true, true, false, false};

/* init all leds, blink everything once and start background tasks */
void	quad_init(void) {
	esp_timer_create_args_t args;

	/* create quadrature tasks */
	xTaskCreate(quad_click, "CLICK", 1024, NULL, tskIDLE_PRIORITY + 1, &t_click);
	xTaskCreate(quad_move_x, "QX", 4 * 1024, NULL, tskIDLE_PRIORITY + 1, &t_qx);
	xTaskCreate(quad_move_y, "QY", 4 * 1024, NULL, tskIDLE_PRIORITY + 1, &t_qy);

	printf(">>> T_QX %x\n", (unsigned int)t_qx);
	printf(">>> T_QY %x\n", (unsigned int)t_qy);

	/* click is active low */
	gpio_set_level(GPIO_CLICK, 1);

	/* init quadrature position */
	gpio_set_level(GPIO_QX1, q1[0]);
	gpio_set_level(GPIO_QX2, q2[0]);
	gpio_set_level(GPIO_QY1, q1[0]);
	gpio_set_level(GPIO_QY2, q2[0]);

	/* create timers for quadrature phases */
	args.callback = &quad_timer;
	args.arg = t_qx;
	args.name = "quad_qx";
	ESP_ERROR_CHECK(esp_timer_create(&args, &quad_qx));
	args.callback = &quad_timer;
	args.arg = t_qy;
	args.name = "quad_qy";
	ESP_ERROR_CHECK(esp_timer_create(&args, &quad_qy));

	ESP_LOGI("quad", "Quadrature tasks started on core %d", xPortGetCoreID());
}

void	quad_click(void *pvParameters) {
	unsigned int click = 0;

	(void)pvParameters;

	while (true) {
		xTaskNotifyWait(0, 0, &click, portMAX_DELAY);
		switch (click)
		{
			case true:
				gpio_set_level(GPIO_CLICK, 0);
				break;
			case false:
				gpio_set_level(GPIO_CLICK, 1);
				break;
			default:
				break;
		}
	}
}

void	quad_move_x(void *pvParameters) {
	unsigned int value = 0;
	int	move = 0;
	uint8_t i = 0;

	(void)pvParameters;

	while (true) {
		xTaskNotifyWait(0, 0, &value, portMAX_DELAY);
		assert(value != 0);
		move = (signed)value;

		if (move > 0) {
			while (move--) {
				i--;
				gpio_set_level(GPIO_QX1, q1[i % 4]);
				gpio_set_level(GPIO_QX2, q2[i % 4]);
				ESP_ERROR_CHECK(esp_timer_start_once(quad_qx, QUAD_INTERVAL));
				vTaskSuspend(NULL);
			}
		}
		else {
			while (move++) {
				i++;
				gpio_set_level(GPIO_QX1, q1[i % 4]);
				gpio_set_level(GPIO_QX2, q2[i % 4]);
				ESP_ERROR_CHECK(esp_timer_start_once(quad_qx, QUAD_INTERVAL));
				vTaskSuspend(NULL);
			}
		}
	}
}

void	quad_move_y(void *pvParameters) {
	unsigned int value = 0;
	int	move = 0;
	uint8_t i = 0;

	(void)pvParameters;

	while (true) {
		xTaskNotifyWait(0, 0, &value, portMAX_DELAY);
		assert(value != 0);
		move = (signed)value;

		if (move > 0) {
			while (move--) {
				i++;
				gpio_set_level(GPIO_QY1, q1[i % 4]);
				gpio_set_level(GPIO_QY2, q2[i % 4]);
				ESP_ERROR_CHECK(esp_timer_start_once(quad_qy, QUAD_INTERVAL));
				vTaskSuspend(NULL);
			}
		}
		else {
			while (move++) {
				i--;
				gpio_set_level(GPIO_QY1, q1[i % 4]);
				gpio_set_level(GPIO_QY2, q2[i % 4]);
				ESP_ERROR_CHECK(esp_timer_start_once(quad_qy, QUAD_INTERVAL));
				vTaskSuspend(NULL);
			}
		}
	}
}

/* simple ISR function. Resume task that called the oneshot timer */
static void IRAM_ATTR quad_timer(void* arg) {
	BaseType_t xYieldRequired = pdFALSE;
	xYieldRequired = xTaskResumeFromISR(arg);

	/* switch context of needed */
	if( xYieldRequired == pdTRUE )
		portYIELD_FROM_ISR();
}
