/*
 *  led.c
 *  quack
 *
 *  Created by Michel DEPEIGE on 15/12/2020.
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
#include "driver/ledc.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

#include "led.h"
#include "gpio.h"

/* defines */
#define TAG "LED"

/* global variables for tasks handles */
TaskHandle_t    t_dispatch, t_green, t_blue, t_yellow, t_red;

static ledc_timer_config_t ledc_timer = {
	.duty_resolution = LEDC_TIMER_12_BIT,	// resolution of PWM duty
	.freq_hz = 4000,						// frequency of PWM signal
	.speed_mode = LEDC_HIGH_SPEED_MODE,		// timer mode
	.timer_num = LEDC_TIMER_0,				// timer index
	.clk_cfg = LEDC_AUTO_CLK,				// Auto select the source clock
};

/*
 * LEDC channels configuration
 * channel number is: GPIO number - 20. This allow to get the hardware
 * channel number from the hardware GPIO number
 * example: GREEN = GPIO 21 = CHANNEL 21 - 20 = CHANNEL 1
 * others channels are set to an unused GPIO pin (5) but not registered
 */

static ledc_channel_config_t ledc_channel[] = {
	{
		.channel    = LEDC_CHANNEL_0,
		.duty       = 0,
		.gpio_num   = GPIO_NULL,
		.speed_mode = LEDC_HIGH_SPEED_MODE,
		.hpoint     = 0,
		.timer_sel  = LEDC_TIMER_0,
	},
	{
		.channel    = LEDC_CHANNEL_1,
		.duty       = 0,
		.gpio_num   = GPIO_GREENLED,
		.speed_mode = LEDC_HIGH_SPEED_MODE,
		.hpoint     = 0,
		.timer_sel  = LEDC_TIMER_0,
	},
	{
		.channel    = LEDC_CHANNEL_2,
		.duty       = 0,
		.gpio_num   = GPIO_NULL,
		.speed_mode = LEDC_HIGH_SPEED_MODE,
		.hpoint     = 0,
		.timer_sel  = LEDC_TIMER_0,
	},
	{
		.channel    = LEDC_CHANNEL_3,
		.duty       = 0,
		.gpio_num   = GPIO_NULL,
		.speed_mode = LEDC_HIGH_SPEED_MODE,
		.hpoint     = 0,
		.timer_sel  = LEDC_TIMER_0,
	},
	{
		.channel    = LEDC_CHANNEL_4,
		.duty       = 0,
		.gpio_num   = GPIO_NULL,
		.speed_mode = LEDC_HIGH_SPEED_MODE,
		.hpoint     = 0,
		.timer_sel  = LEDC_TIMER_0,
	},
	{
		.channel    = LEDC_CHANNEL_5,
		.duty       = 0,
		.gpio_num   = GPIO_BLUELED,
		.speed_mode = LEDC_HIGH_SPEED_MODE,
		.hpoint     = 0,
		.timer_sel  = LEDC_TIMER_0,
	},
	{
		.channel    = LEDC_CHANNEL_6,
		.duty       = 0,
		.gpio_num   = GPIO_YELLOWLED,
		.speed_mode = LEDC_HIGH_SPEED_MODE,
		.hpoint     = 0,
		.timer_sel  = LEDC_TIMER_0,
	},
	{
		.channel    = LEDC_CHANNEL_7,
		.duty       = 0,
		.gpio_num   = GPIO_REDLED,
		.speed_mode = LEDC_HIGH_SPEED_MODE,
		.hpoint     = 0,
		.timer_sel  = LEDC_TIMER_0,
	}
};

/* could be inside led_init but just used to allocate IRQs to CPU 1 instead of 0 */
void	led_dispatch(void *pvParameters)
{
	ledc_fade_func_install(0);

	/* avoit cur duty + target spam on console */
	esp_log_level_set("ledc", ESP_LOG_INFO);

	xTaskCreatePinnedToCore(led_task, led_gpio_name(GPIO_GREENLED), 2 * 1024, (void *) GPIO_GREENLED, tskIDLE_PRIORITY, &t_green, 1);
	xTaskCreatePinnedToCore(led_task, led_gpio_name(GPIO_BLUELED), 2 * 1024, (void *) GPIO_BLUELED, tskIDLE_PRIORITY, &t_blue, 1);
	xTaskCreatePinnedToCore(led_task, led_gpio_name(GPIO_YELLOWLED), 2 * 1024, (void *) GPIO_YELLOWLED, tskIDLE_PRIORITY, &t_yellow, 1);
	xTaskCreatePinnedToCore(led_task, led_gpio_name(GPIO_REDLED), 2 * 1024, (void *) GPIO_REDLED, tskIDLE_PRIORITY, &t_red, 1);

	/* blink green led fast (we are booting...) */
	xTaskNotify(t_green, LED_FAST, eSetValueWithOverwrite);

	ESP_LOGI(TAG, "tasks dispatch finished on core %d", xPortGetCoreID());
	vTaskDelete(t_dispatch);
}

const char	*led_gpio_name(uint8_t id) {
	uint8_t i = 0;

	static const led_names_t led_names[] = {
		{ .num = GPIO_GREENLED, .name = "GREEN" },
		{ .num = GPIO_BLUELED, .name = "BLUE" },
		{ .num = GPIO_YELLOWLED, .name = "YELLOW" },
		{ .num = GPIO_REDLED, .name = "RED" },
		{ .num = 0, .name = NULL}
	};

	configASSERT(id < 28);

	while (led_names[i].name != NULL) {
		if (led_names[i].num == id)
			return led_names[i].name;
		i++;
	}

	return "NA";
}

void	led_init(void) {
	/* blink allds LEDs once */
	gpio_set_level(GPIO_GREENLED, 1);
	gpio_set_level(GPIO_BLUELED, 1);
	gpio_set_level(GPIO_YELLOWLED, 1);
	gpio_set_level(GPIO_REDLED, 1);
	vTaskDelay(125 / portTICK_PERIOD_MS);
	gpio_set_level(GPIO_GREENLED, 0);
	gpio_set_level(GPIO_BLUELED, 0);
	gpio_set_level(GPIO_YELLOWLED, 0);
	gpio_set_level(GPIO_REDLED, 0);

	ledc_timer_config(&ledc_timer);
	ledc_channel_config(&ledc_channel[GPIO_GREENLED - 20]);
	ledc_channel_config(&ledc_channel[GPIO_BLUELED - 20]);
	ledc_channel_config(&ledc_channel[GPIO_YELLOWLED - 20]);
	ledc_channel_config(&ledc_channel[GPIO_REDLED - 20]);
	xTaskCreatePinnedToCore(led_dispatch, "DISPATCH", 2 * 1024, NULL, tskIDLE_PRIORITY, &t_dispatch, 1);
}

void	led_task(void *pvParameters) {
	unsigned int color = (unsigned int)pvParameters;
	unsigned int mode = LED_OFF;
	TickType_t wait = portMAX_DELAY;

	/* start only if there is a led specified */
	configASSERT(((uint32_t) pvParameters) > 0);

	ESP_LOGI(TAG, "led task %s started on core %d", led_gpio_name(color), xPortGetCoreID());

	while (1) {
		xTaskNotifyWait(0, 0, &mode, wait);
		switch (mode)
		{
			case LED_OFF:
				ledc_set_duty(LED_SPEED(color), LED_CHANNEL(color), 0);
				ledc_update_duty(LED_SPEED(color), LED_CHANNEL(color));
				wait = portMAX_DELAY;
				break;
			case LED_ON:
				ledc_set_duty(LED_SPEED(color), LED_CHANNEL(color), LED_DUTY);
				ledc_update_duty(LED_SPEED(color), LED_CHANNEL(color));
				wait = portMAX_DELAY;
				break;
			case LED_ONCE:
				ledc_set_duty(LED_SPEED(color), LED_CHANNEL(color), LED_DUTY);
				ledc_update_duty(LED_SPEED(color), LED_CHANNEL(color));
				vTaskDelay(40 / portTICK_PERIOD_MS);
				ledc_set_duty(LED_SPEED(color), LED_CHANNEL(color), 0);
				ledc_update_duty(LED_SPEED(color), LED_CHANNEL(color));
				wait = portMAX_DELAY;
				break;
			case LED_FAST:
				ledc_set_duty(LED_SPEED(color), LED_CHANNEL(color), LED_DUTY);
				ledc_update_duty(LED_SPEED(color), LED_CHANNEL(color));
				vTaskDelay(40 / portTICK_PERIOD_MS);
				ledc_set_duty(LED_SPEED(color), LED_CHANNEL(color), 0);
				ledc_update_duty(LED_SPEED(color), LED_CHANNEL(color));
				wait = 40 / portTICK_PERIOD_MS;
				break;
			case LED_SLOW:
				ledc_set_fade_with_time(LED_SPEED(color), LED_CHANNEL(color), LED_DUTY, 400);
				ledc_fade_start(LED_SPEED(color), LED_CHANNEL(color), LEDC_FADE_NO_WAIT);
				vTaskDelay(500 / portTICK_PERIOD_MS);
				ledc_set_fade_with_time(LED_SPEED(color), LED_CHANNEL(color), 0, 400);
				ledc_fade_start(LED_SPEED(color), LED_CHANNEL(color), LEDC_FADE_NO_WAIT);
				wait = 500 / portTICK_PERIOD_MS;
				break;
			default:
				break;
		}
	}

	gpio_set_level(color, 1);
}
