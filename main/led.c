/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
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

void	led_init(void) {
	BaseType_t	r;
	TaskHandle_t	green, blue, yellow, red;

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

	r = xTaskCreate(led_task, "GREEN", 1024, (void *) GPIO_GREENLED, tskIDLE_PRIORITY, &green);
}

void	led_task(void *pvParameters) {
	while (1) {
		gpio_set_level(GPIO_GREENLED, 1);
		vTaskDelay(75 / portTICK_PERIOD_MS);
		gpio_set_level(GPIO_GREENLED, 0);
		vTaskDelay(75 / portTICK_PERIOD_MS);
	}
}
