/*
 *  quad.c
 *  quack
 *
 *  Created by Michel DEPEIGE on 17/04/2021.
 *  Copyright (c) 2021 Michel DEPEIGE.
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
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "quad.h"
#include "gpio.h"

/* global variables for tasks handles */
TaskHandle_t t_click, t_qx, t_qy;
esp_timer_handle_t quad_qx, quad_qy;

QueueHandle_t q_qx, q_qy;

/* ISR functions */
static void	IRAM_ATTR quad_isr(void* arg);

/* phases */
const bool q1[] = {true, false, false, true};
const bool q2[] = {true, true, false, false};

/* init all leds, blink everything once and start background tasks */
void	quad_init(void) {
	esp_timer_create_args_t args;

	/* create quadrature queues */
	q_qx = xQueueCreate(4, sizeof(int8_t));
	q_qy = xQueueCreate(4, sizeof(int8_t));

	/* create quadrature tasks */
	xTaskCreate(quad_click, "CLICK", 1024, NULL, tskIDLE_PRIORITY + 1, &t_click);
	xTaskCreate(quad_move_x, "QX", 4 * 1024, NULL, tskIDLE_PRIORITY + 1, &t_qx);
	xTaskCreate(quad_move_y, "QY", 4 * 1024, NULL, tskIDLE_PRIORITY + 1, &t_qy);

	/* click is active low */
	gpio_set_level(GPIO_CLICK, 1);

	/* init quadrature position */
	gpio_set_level(GPIO_QX1, q1[0]);
	gpio_set_level(GPIO_QX2, q2[0]);
	gpio_set_level(GPIO_QY1, q1[0]);
	gpio_set_level(GPIO_QY2, q2[0]);

	/* create timers for quadrature phases */
	args.callback = quad_isr;
	args.arg = t_qx;
	args.name = "quad_qx";
	args.dispatch_method = ESP_TIMER_ISR;
	ESP_ERROR_CHECK(esp_timer_create(&args, &quad_qx));
	args.callback = quad_isr;
	args.arg = t_qy;
	args.name = "quad_qy";
	args.dispatch_method = ESP_TIMER_ISR;
	ESP_ERROR_CHECK(esp_timer_create(&args, &quad_qy));

	ESP_LOGI("quad", "Quadrature tasks started on core %d", xPortGetCoreID());
}


void	quad_click(void *pvParameters) {
	long unsigned int click = 0;

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

void IRAM_ATTR quad_move_x(void *pvParameters) {
	int8_t move = 0;
	uint8_t i = 0;

	(void)pvParameters;

	while (true) {
		xQueueReceive(q_qx, &move, portMAX_DELAY);
		assert(move != 0);

		if (move > 0) {
			while (move--) {
				i--;
				gpio_set_level(GPIO_QX1, q1[i % 4]);
				gpio_set_level(GPIO_QX2, q2[i % 4]);
				ESP_ERROR_CHECK(esp_timer_start_once(quad_qx, QUAD_INTERVAL));
				ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
			}
		}
		else {
			while (move++) {
				i++;
				gpio_set_level(GPIO_QX1, q1[i % 4]);
				gpio_set_level(GPIO_QX2, q2[i % 4]);
				ESP_ERROR_CHECK(esp_timer_start_once(quad_qx, QUAD_INTERVAL));
				ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
			}
		}
	}
}

void IRAM_ATTR quad_move_y(void *pvParameters) {
	int8_t move = 0;
	uint8_t i = 0;

	(void)pvParameters;

	while (true) {
		xQueueReceive(q_qy, &move, portMAX_DELAY);
		assert(move != 0);

		if (move > 0) {
			while (move--) {
				i++;
				gpio_set_level(GPIO_QY1, q1[i % 4]);
				gpio_set_level(GPIO_QY2, q2[i % 4]);
				ESP_ERROR_CHECK(esp_timer_start_once(quad_qy, QUAD_INTERVAL));
				ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
			}
		}
		else {
			while (move++) {
				i--;
				gpio_set_level(GPIO_QY1, q1[i % 4]);
				gpio_set_level(GPIO_QY2, q2[i % 4]);
				ESP_ERROR_CHECK(esp_timer_start_once(quad_qy, QUAD_INTERVAL));
				ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
			}
		}
	}
}

/* simple ISR function. Resume the task that called the oneshot timer */
static void IRAM_ATTR quad_isr(void* arg) {
	BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR(arg, &pxHigherPriorityTaskWoken);

	/* switch context of needed */
	if( pxHigherPriorityTaskWoken == pdTRUE )
		portYIELD_FROM_ISR();
}
