/*
 *  gpio.h
 *  quack
 *
 *  Created by Michel DEPEIGE on 13/09/2020.
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

#ifndef GPIO_H
#define GPIO_H

/* prototypes */
void	gpio_init(void);

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

#define ADB_GPIO	4
#define CLICK_HPIO	2
#define QX1_GPIO	12
#define QX2_GPIO	13
#define QY1_GPIO	14
#define QY2_GPIO	15

#define GREENLED_GPIO	21
#define BLUELED_GPIO	25
#define YELLOWLED_GPIO	26
#define REDLED_GPIO	27
#endif

