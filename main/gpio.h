/*
 *  gpio.h
 *  quack
 *
 *  Created by Michel DEPEIGE on 13/09/2020.
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

#ifndef GPIO_H
#define GPIO_H

/* prototypes */
void	gpio_init(void);
void    gpio_transceiver_disable(void);
void    gpio_transceiver_enable(void);

/* GPIO pins definitions
 *
 * pin layout is compatible with ESP32-CAM module for prototyping
 * LEDs aren't avaible on ESP32-CAM but DEBUG LED (Camera connector)
 * 
 * on ESP32-CAM the following pins are already used:
 * - CSI_MCLK (GPIO0) 
 * - U0TXD (GPIO1)
 * - U0TXD (GPIO3)
 * - FLASH (GPIO4)
 *
 * on ESP32-PICO-D4, the following pins are used for connecting the
 * embedded flash:
 * - CLK (GPIO6)
 * - SD0 (GPIO7)
 * - SD1 (GPIO8)
 * - CMD (GPIO11)
 * - GPIO16
 * - GPIO17
 */

#define GPIO_ADB		4
#define GPIO_CLICK		2
#define GPIO_QX1		12
#define GPIO_QX2		13
#define GPIO_QY1		14
#define GPIO_QY2		15

#define GPIO_DIR		19
#define GPIO_OE			22
#define GPIO_ADBSRC		32
#define GPIO_BTOFF		33

#define GPIO_GREENLED	21
#define GPIO_BLUELED	25
#define GPIO_YELLOWLED	26
#define GPIO_REDLED		27

#define GPIO_NULL		5

#endif

