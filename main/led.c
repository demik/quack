/*
 *  led.c
 *  quack
 *
 *  Created by Michel DEPEIGE on 15/12/2020.
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

#include "led.h"
#include "gpio.h"

/* global variables for tasks handles */
TaskHandle_t    t_green, t_blue, t_yellow, t_red;

/* init all leds, blink everything once and start background tasks */
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

	xTaskCreate(led_task, "GREEN", 1024, (void *) GPIO_GREENLED, tskIDLE_PRIORITY, &t_green);
    xTaskCreate(led_task, "BLUE", 1024, (void *) GPIO_BLUELED, tskIDLE_PRIORITY, &t_blue);
    xTaskCreate(led_task, "YELLOW", 1024, (void *) GPIO_YELLOWLED, tskIDLE_PRIORITY, &t_yellow);
    xTaskCreate(led_task, "RED", 1024, (void *) GPIO_REDLED, tskIDLE_PRIORITY, &t_red);
    
    /* blink green led fast (we are booting...) */
    xTaskNotify(t_green, LED_FAST, eSetValueWithOverwrite);
}

void	led_task(void *pvParameters) {
    unsigned int color = (unsigned int)pvParameters;
    unsigned int mode = LED_OFF;
    TickType_t wait = portMAX_DELAY;

    /* start only if there is a led specified */
    configASSERT(((uint32_t) pvParameters) > 0);
   
	while (1) {
        xTaskNotifyWait(0, 0, &mode, wait);
        switch (mode)
        {
            case LED_OFF:
                gpio_set_level(color, 0);
                wait = portMAX_DELAY;
                break;
            case LED_ON:
                gpio_set_level(color, 1);
                wait = portMAX_DELAY;
                break;
            case LED_ONCE:
                gpio_set_level(color, 1);
                vTaskDelay(40 / portTICK_PERIOD_MS);
                gpio_set_level(color, 0);
                vTaskDelay(40 / portTICK_PERIOD_MS);
                wait = portMAX_DELAY;
                break;
            case LED_FAST:
                gpio_set_level(color, 1);
                vTaskDelay(40 / portTICK_PERIOD_MS);
                gpio_set_level(color, 0);
                wait = 40 / portTICK_PERIOD_MS;
                break;
            case LED_SLOW:
                gpio_set_level(color, 1);
                vTaskDelay(500 / portTICK_PERIOD_MS);
                gpio_set_level(color, 0);
                wait = 500 / portTICK_PERIOD_MS;
                break;
            default:
                break;
        }
	}
    
    gpio_set_level(color, 1);
}
