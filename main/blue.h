/*
 *  blue.h
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
#define	BLUE_BUTTON_E 0xF8		/* shouldn't happen */

/* prototypes */
void	blue_adb2hid(void *pvParameters);
void	blue_init(void);
uint8_t blue_get_pin_code(esp_bd_addr_t bda, esp_bt_pin_code_t pin);
void	blue_h_boot(uint8_t  *data, uint16_t length);
void	blue_h_input(esp_hidh_dev_t *dev, uint16_t id, uint8_t  *data, uint16_t length);
void blue_h_close(esp_hidh_event_data_t *p);
void blue_h_open(esp_hidh_event_data_t *p);
void blue_scan(void *pvParameters);
void	blue_set_boot_protocol(esp_hidh_dev_t *dev);

/* global variables for tasks handles */
extern TaskHandle_t t_green, t_blue, t_yellow, t_red;

#endif

