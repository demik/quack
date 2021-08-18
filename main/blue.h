/*
 *  blue.h
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

#ifndef BLUE_H
#define BLUE_H

/* defines */
#define BLUE_SCAN_DURATION 6
#define BLUE_SNIP "---8<------------------------------------------"

/* button bitmasks */
#define	BLUE_BUTTON_N 0
#define	BLUE_BUTTON_1 (1 << 0)
#define	BLUE_BUTTON_2 (1 << 1)
#define	BLUE_BUTTON_3 (1 << 2)
#define BLUE_BUTTON_E 0xF8		/* shouldn't happen */

/* prototypes */
void	blue_init(void);
void	blue_h_input(esp_hidh_dev_t *dev, uint8_t  *data, uint16_t length);
void    blue_h_close(esp_hidh_event_data_t *p);
void    blue_h_open(esp_hidh_event_data_t *p);
void    blue_scan(void *pvParameters);

/* global variables for tasks handles */
extern TaskHandle_t t_green, t_blue, t_yellow, t_red;

#endif

