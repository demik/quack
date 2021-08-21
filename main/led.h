/*
 *  led.h
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

#ifndef LED_H
#define LED_H

/* prototypes */
void		led_dispatch(void *pvParameters);
const char	*led_gpio_name(uint8_t id);
void		led_init(void);
void    	led_task(void *pvParameters);

/* structures */
typedef struct led_names_s {
	const uint8_t num;
	const char *name;
} led_names_t;

/* defines */
#define LED_OFF		(1 << 0)
#define LED_ON		(1 << 1)
#define LED_ONCE	(1 << 2)
#define LED_SLOW	(1 << 3)
#define LED_FAST	(1 << 4)

#define LED_DUTY	4096

#define LED_CHANNEL(c)	ledc_channel[c - 20].channel
#define LED_SPEED(c)	ledc_channel[c - 20].speed_mode

#endif

