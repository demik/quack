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

#include "gpio.h"

void	gpio_init(void) {
	gpio_reset_pin(GREENLED_GPIO);
	gpio_set_direction(GREENLED_GPIO, GPIO_MODE_OUTPUT);

	/* do a half second blink for debug mode */
	gpio_set_level(GREENLED_GPIO, 1);
	vTaskDelay(500 / portTICK_PERIOD_MS);
	gpio_set_level(GREENLED_GPIO, 0);
	vTaskDelay(500 / portTICK_PERIOD_MS);
}
