/*
 *  wii.h
 *  quack
 *
 *  Created by Michel DEPEIGE on 11/08/2024.
 *  Copyright (c) 2024 Michel DEPEIGE.
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

#ifndef WII_H
#define WII_H

/* prototypes */
uint8_t *wii_generate_pin(esp_bt_pin_code_t pin);
void	wii_init(void);
void	wii_input(esp_hidh_dev_t *dev, uint16_t id, uint8_t  *data, uint16_t length);
bool	wii_is_nintendo_bda(esp_bd_addr_t bda);
bool wii_is_wiimote(uint16_t vid, uint16_t pid);
void	wii_open(const esp_bd_addr_t bda);
void wii_show_calibration(uint8_t  *data);

/* defines */
#define WII_VID	0x057e
#define WII_PID_OLD	0x0306
#define WII_PID_NEW	0x0330

#define WII_PIN_STR		"0x%02x%02x%02x%02x%02x%02x"
#define WII_PIN_HEX(pin)	pin[0], pin[1], pin[2], pin[3], pin[4], pin[5]

#define WII_THRESHOLD	0.1f
#define WII_ACCELERATION	-10.0f /* coonvert gravity to pixels */
#define WII_INTERVAL	(1000 / 60) /* pad update interval, about 60 Hz */
#define WII_THROTTLE	20

/* core buttons. LX6 is wrong endian */
#define WII_BUTTON_NONE	0x0000
#define WII_BUTTON_LEFT	0x0001
#define WII_BUTTON_RIGHT	0x0002
#define WII_BUTTON_DOWN	0x0004
#define WII_BUTTON_UP	0x0008
#define WII_BUTTON_PLUS	0x0010
#define WII_BUTTON_TWO	0x0100
#define WII_BUTTON_ONE	0x0200
#define WII_BUTTON_B	0x0400
#define WII_BUTTON_A	0x0800
#define WII_BUTTON_MINUS	0x1000
#define WII_BUTTON_HOME	0x8000

/* LEDs + Rumble */
#define WII_LED1	0x10
#define WII_LED2	0x20
#define WII_LED3	0x40
#define WII_LED4	0x80

/* for quadrature speed feedback */
#define WII_SPEED_NONE	0x00
#define WII_SPEED_LOW	WII_LED1
#define WII_SPEED_MEDIUM	WII_LED2
#define WII_SPEED_HIGH	WII_LED3

/* handled reports */
#define WII_REPORT_RUMBLE	0x10
#define WII_REPORT_LEDS		0x11
#define WII_REPORT_MODE		0x12
#define WII_REPORT_SPEAKER_E	0x14
#define WII_REPORT_REQUEST	0x15
#define WII_REPORT_WRITE_REG	0x16
#define WII_REPORT_READ_REG	0x17
#define WII_REPORT_SPEAKER_D	0x18
#define WII_REPORT_SPEAKER_M	0x19
#define WII_REPORT_STATUS	0x20
#define WII_REPORT_READ_INPUT	0x21
#define WII_REPORT_ACK		0x22
#define WII_REPORT_CORE_ONLY	0x30
#define WII_REPORT_CORE_ACCEL	0x31

#define WII_REPORT_INPUT	WII_REPORT_ACK|WII_REPORT_CORE_ONLY

/* structures */
typedef struct wii_accel_s {
	uint16_t x;
	uint16_t y;
	uint16_t z;
} wii_accel_t;

#endif
