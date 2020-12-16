/*
 *  led.h
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

#ifndef LED_H
#define LED_H

/* prototypes */
void	led_init(void);
void    led_task(void *pvParameters);

/* defines */
#define LED_OFF		(1 << 0)
#define LED_ON		(1 << 1)
#define LED_ONCE	(1 << 2)
#define LED_SLOW	(1 << 3)
#define LED_FAST	(1 << 4)

#endif

