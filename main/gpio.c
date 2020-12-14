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
	/* LEDs */
	gpio_reset_pin(GPIO_GREENLED);
	gpio_reset_pin(GPIO_BLUELED);
	gpio_reset_pin(GPIO_YELLOWLED);
	gpio_reset_pin(GPIO_REDLED);
	gpio_set_direction(GPIO_GREENLED, GPIO_MODE_OUTPUT);
	gpio_set_direction(GPIO_BLUELED, GPIO_MODE_OUTPUT);
	gpio_set_direction(GPIO_YELLOWLED, GPIO_MODE_OUTPUT);
	gpio_set_direction(GPIO_REDLED, GPIO_MODE_OUTPUT);
}
