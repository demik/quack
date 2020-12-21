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
void    gpio_output_disable(void);
void    gpio_output_enable(void);

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

#define GPIO_ADB	    4
#define GPIO_CLICK	    2
#define GPIO_QX1	    12
#define GPIO_QX2	    13
#define GPIO_QY1	    14
#define GPIO_QY2        15

#define GPIO_OE         22
#define GPIO_ADBSRC     32
#define GPIO_BTOFF      33

#define GPIO_GREENLED	21
#define GPIO_BLUELED	25
#define GPIO_YELLOWLED	26
#define GPIO_REDLED	    27

#endif

