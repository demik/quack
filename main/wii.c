/*
 *  wii.c
 *  quack
 *
 *  Created by Michel DEPEIGE on 11/08/2024.
 *  Copyright (c) 2020-2024 Michel DEPEIGE.
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

#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#if !CONFIG_BT_NIMBLE_ENABLED
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#endif
#include "esp_hid_common.h"
#include "esp_hidh.h"
#include "esp_hidh_api.h"

#include "wii.h"
#include "led.h"
#include "gpio.h"

/* defines */
#define TAG "WII"

/* globals */
TaskHandle_t t_wii_dpad;
extern TaskHandle_t t_click;
extern TaskHandle_t t_yellow, t_red;
extern QueueHandle_t q_qx, q_qy;

QueueHandle_t q_wii;

uint8_t		wii_speed;
wii_accel_t	wii_calibration_0g;
wii_accel_t	wii_calibration_1g;

/* static defines */
static void wii_accelerometer(esp_hidh_dev_t *dev, uint8_t *data);
static void wii_change_mode(esp_hidh_dev_t *dev);
static void wii_change_speed(esp_hidh_dev_t *dev, uint16_t buttons);
static void wii_handle_button(uint16_t buttons);
static void wii_mute_state(const esp_bd_addr_t bda, bool state);
static void wii_rumble_state(const esp_bd_addr_t bda, bool state);
static void wii_speaker_state(const esp_bd_addr_t bda, bool state);

/* functions */
static void wii_accelerometer(esp_hidh_dev_t *dev, uint8_t *data) {
	const uint8_t *bda = NULL;
	static uint8_t p = 0;				/* used for calibration pass */
	uint8_t report[3] = {0};
	esp_err_t ret;

	float x, y, z;
	static float average_x, average_y, average_z;
	static uint16_t sum_x, sum_y, sum_z;
	static int8_t out;					/* used for quadrature messages */
	wii_accel_t a = {0, 0, 0};

	configASSERT(data != NULL);
	configASSERT(dev != NULL);
	bda = esp_hidh_dev_bda_get(dev);
	if (bda == NULL)
		return;
	/* extract accelerator data. see https://wiibrew.org/wiki/Wiimote#Accelerometer */
	a.x = (data[2] << 2) | ((data[0] >> 5) & 0x3);
     a.y = (data[3] << 2) | ((data[1] >> 4) & 0x2);
     a.z = (data[4] << 2) | ((data[1] >> 5) & 0x2);

	/* generate acceleration values */
	x = ((float)a.x - (float)wii_calibration_0g.x) / (float)wii_calibration_1g.x;
	y = ((float)a.y - (float)wii_calibration_0g.y) / (float)wii_calibration_1g.y;
	z = ((float)a.z - (float)wii_calibration_0g.z) / (float)wii_calibration_1g.z;

	/*
	 * at this point it's 2AM and the values read from the test Wiimote don't match
	 * the calibration values. we are limited since no IR / lightbar anyway so can't
	 * get yaw
	 *
	 * dunno if there is something wrong in the code or the Wiimote is decalibrated
	 *
	 * so from no on, we will mangle the values to do a Arkanoid or Space Invaders
	 * mode and call it a day (or a night)
	 *
	 * also ignore Y and Z from now on. Sorry Y and Z
	 * inspiration from https://www.nxp.com/docs/en/application-note/AN3397.pdf
	 */

	if (p == 0) {
		ESP_LOGD(TAG, "starting calibration…");

		sum_x = 0;
		sum_y = 0;
		sum_z = 0;
	}
	if (p < 255) {
		/*
		 * store MSB X, Y and Z values so the averages gets updated
		 * this will override values read from EEPROM but they don't seem useable
		 * for now (see above)
		 */

		/* store MSB X and Y values so the average can get calculated later */
		sum_x += a.x >> 2;
		sum_y += a.y >> 2;
		sum_z += a.z >> 2;
		p++;

		/* got enough samples, let's find averages */
		if (p == 255) {
			average_x = ((float)((sum_x << 2) / 255) - (float)wii_calibration_0g.x);
			average_x /= (float)wii_calibration_1g.x;
			average_y = ((float)((sum_y << 2) / 255) - (float)wii_calibration_0g.y);
			average_y /= (float)wii_calibration_1g.y;
			average_z = ((float)((sum_z << 2) / 255) - (float)wii_calibration_0g.z);
			average_z /= (float)wii_calibration_1g.z;

			/* do a quick rumble as user notification */
			wii_rumble_state(bda, true);
			vTaskDelay(100 / portTICK_PERIOD_MS);
			wii_rumble_state(bda, false);
			report[0] = WII_REPORT_LEDS;
			report[1] = WII_LED4;
			ret = esp_bt_hid_host_send_data((uint8_t *)bda, report, 2);
			if (ret)
				ESP_LOGE(TAG, "cannot set LEDs state, send error: %x", ret);
			vTaskDelay(WII_THROTTLE / portTICK_PERIOD_MS);
			ESP_LOGD(TAG, "\t … X: %f, Y: %f, Z: %f", average_x, average_y, average_z);

			/* set to events only after calibration */
			report[0] = WII_REPORT_MODE;
			report[1] = 0x00;	/* events only report mode */
			report[2] = 0x31;	/* core buttons + accelerometer */
			ret = esp_bt_hid_host_send_data((uint8_t *)bda, report, 3);
			if (ret)
				ESP_LOGE(TAG, "cannot set mode, send error: %x", ret);
		}
		return;
	}

	/* just avoid  may be used uninitialized messages */
	(void)y;
	(void)z;

	/*
	 * do a mechanical filtering window, assuming the Wiimote is doing the low pass
	 * filtering itself since main report rate is about 30 Hz
	 * count idle passes to avoid drift, disregard half of what we did
	 * previously. this is a crap. Now I understand why Motion Plus exists
	 * this will move the mouse on the x axis, speed proportional to the x axis tilt
	 */
	if ((average_x - x) < -WII_THRESHOLD || (average_x - x) > WII_THRESHOLD) {
		/* cap value to not overflow 8 bit */
		out = (average_x - x) * WII_ACCELERATION * wii_speed;
		if (((average_x - x) * WII_ACCELERATION * wii_speed) < -120)
			out = -120;
		if (((average_x - x) * WII_ACCELERATION * wii_speed) > 120)
			out = 120;
	}
	else {
		out = out >> 1;
	}
	//ESP_LOGD(TAG, "X %f %f :: %i", (average_x - x), (average_x - x) * WII_ACCELERATION, out);

	if (out)
		xQueueSendToBack(q_qx, &out, 0);
}

/*
 * this will switch between the default report (buttons only) and the more advanced
 * mode with accelerometer data. We don't handle others advanced modes nor extensions
 */
static void wii_change_mode(esp_hidh_dev_t *dev) {
	static bool accel = 0;
	const uint8_t *bda = NULL;
	uint8_t report[3] = {0};
	esp_err_t ret;

	configASSERT(dev != NULL);
	bda = esp_hidh_dev_bda_get(dev);
	if (bda == NULL)
		return;

	accel = !accel;
	ESP_LOGD(TAG, "changing mode, setting accelerometer status to %x", accel);
	report[0] = WII_REPORT_MODE;
	if (accel) {
		report[1] = 0x04;	/* continuous report mode */
		report[2] = 0x31;	/* core buttons + accelerometer */
	}
	else {
		report[1] = 0x00;	/* events only report mode */
		report[2] = 0x30;	/* core buttons only */
	}
	ret = esp_bt_hid_host_send_data((uint8_t *)bda, report, 3);
	if (ret)
		ESP_LOGE(TAG, "cannot set mode, send error: %x", ret);

	wii_rumble_state(bda, true);
	vTaskDelay(100 / portTICK_PERIOD_MS);
	wii_rumble_state(bda, false);

	/* disable speed LEDs on accelerator mode (or restore in default mode) */
	report[0] = WII_REPORT_LEDS;
	if (accel)
		report[1] = WII_LED4;
	else
		report[1] = wii_speed | WII_LED4;
	ret = esp_bt_hid_host_send_data((uint8_t *)bda, report, 2);
	if (ret)
		ESP_LOGE(TAG, "cannot set LEDs state, send error: %x", ret);
}

static void wii_change_speed(esp_hidh_dev_t *dev, uint16_t buttons) {
	const uint8_t *bda = NULL;
	uint8_t report[2] = {0};
	esp_err_t ret;

	configASSERT(dev != NULL);
	bda = esp_hidh_dev_bda_get(dev);
	if (bda == NULL)
		return;

	/* user facerolling the wiimote, ignore */
	if ((buttons & WII_BUTTON_PLUS) && (buttons & WII_BUTTON_MINUS))
		return;

	if (buttons & WII_BUTTON_MINUS) {
		switch (wii_speed) {
			case WII_SPEED_NONE:
			case WII_SPEED_LOW:
				return;
			case WII_SPEED_MEDIUM:
				wii_speed = WII_SPEED_LOW;
				break;
			case WII_SPEED_HIGH:
				wii_speed = WII_SPEED_MEDIUM;
			default:
				break;
		}
	}

	if (buttons & WII_BUTTON_PLUS) {
		switch (wii_speed) {
			case WII_SPEED_NONE:
				return;
			case WII_SPEED_LOW:
				wii_speed = WII_SPEED_MEDIUM;
				break;
			case WII_SPEED_MEDIUM:
				wii_speed = WII_SPEED_HIGH;
				break;
			case WII_SPEED_HIGH:
				return;
			default:
				break;
		}
	}

	/* set LEDs for new speed */
	report[0] = WII_REPORT_LEDS;
	report[1] = wii_speed | WII_LED4;
	ret = esp_bt_hid_host_send_data((uint8_t *)bda, report, 2);
	if (ret)
		ESP_LOGE(TAG, "cannot set LEDs state, send error: %x", ret);
}

/*
 * this will enable the speaker, play the boot chime and then disable the speaker
 * this is blocking for about 400-600ms
 */
void wii_chime(const esp_bd_addr_t bda) {
	uint8_t report[22];
	esp_err_t ret;
	uint8_t *pos;

	/* dual note bleepbleepX2 extracted from wii.aiff */
	const unsigned char chime[] = {
		0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0x00, 0xff,
		0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x01, 0xfc,
		0xfd, 0xff, 0xfd, 0xf6, 0x09, 0x18, 0x02, 0x05, 0x17, 0x02,
		0xcb, 0xd8, 0x18, 0x3c, 0x13, 0xcf, 0xc8, 0x05, 0x40, 0x29,
		0xe0, 0xbe, 0xf0, 0x32, 0x34, 0xf0, 0xc1, 0xe1, 0x2a, 0x3c,
		0x02, 0xc3, 0xd0, 0x19, 0x42, 0x18, 0xcf, 0xc3, 0x04, 0x40,
		0x29, 0xde, 0xbb, 0xef, 0x3a, 0x3c, 0xf0, 0xb8, 0xda, 0x2b,
		0x45, 0x09, 0xc1, 0xc8, 0x15, 0x40, 0x1c, 0xd7, 0xc2, 0xff,
		0x43, 0x1d, 0xdc, 0xdd, 0xe2, 0x09, 0x49, 0x16, 0xb6, 0xdf,
		0x39, 0x1c, 0xcb, 0xeb, 0x35, 0x0f, 0xc9, 0xf1, 0x3b, 0x0c,
		0xc3, 0xf6, 0x3c, 0x05, 0xc3, 0xfd, 0x3b, 0x00, 0xc5, 0x03,
		0x3b, 0xf8, 0xc2, 0x0a, 0x3e, 0xf2, 0xc0, 0x10, 0x3f, 0xed,
		0xc1, 0x15, 0x3d, 0xe8, 0xc3, 0x1a, 0x3a, 0xe2, 0xc6, 0x20,
		0x38, 0xdc, 0xc9, 0x25, 0x34, 0xd8, 0xcd, 0x2a, 0x30, 0xd3,
		0xd1, 0x2e, 0x2c, 0xcf, 0xd5, 0x32, 0x27, 0xcb, 0xda, 0x36,
		0x23, 0xc8, 0xdf, 0x39, 0x1d, 0xc5, 0xe5, 0x3c, 0x18, 0xc2,
		0xea, 0x3e, 0x12, 0xc1, 0xf0, 0x3f, 0x0c, 0xbf, 0xf6, 0x40,
		0x06, 0xbe, 0xfc, 0x41, 0x00, 0xbe, 0x02, 0x41, 0xfa, 0xbf,
		0x08, 0x40, 0xf4, 0xbf, 0x0e, 0x3f, 0xee, 0xc1, 0x14, 0x3d,
		0xe8, 0xc3, 0x1a, 0x3b, 0xe3, 0xc5, 0x1f, 0x38, 0xde, 0xc9,
		0x24, 0x35, 0xd8, 0xcc, 0x29, 0x31, 0xd4, 0xd0, 0x2d, 0x2d,
		0xcf, 0xd4, 0x32, 0x28, 0xcc, 0xd9, 0x35, 0x23, 0xc8, 0xde,
		0x39, 0x1e, 0xc5, 0xe4, 0x3b, 0x19, 0xc3, 0xe9, 0x3d, 0x12,
		0xc1, 0xef, 0x3e, 0x0d, 0xc0, 0xf5, 0x3f, 0x07, 0xbf, 0xfb,
		0x40, 0x01, 0xc0, 0x01, 0x3e, 0xfb, 0xc1, 0x06, 0x3b, 0xf6,
		0xc5, 0x0b, 0x37, 0xf2, 0xcc, 0x12, 0x2e, 0xf1, 0xda, 0x0b,
		0x0c, 0xf3, 0x12, 0x13, 0x00, 0x09, 0x18, 0xf5, 0xc6, 0xe5,
		0x25, 0x3a, 0x02, 0xc7, 0xd1, 0x17, 0x43, 0x1a, 0xd1, 0xc3,
		0x01, 0x3b, 0x28, 0xe0, 0xc1, 0xf2, 0x35, 0x34, 0xf0, 0xbe,
		0xde, 0x28, 0x3f, 0x06, 0xc4, 0xcd, 0x16, 0x42, 0x1a, 0xcf,
		0xc0, 0x02, 0x43, 0x2f, 0xdd, 0xb8, 0xec, 0x39, 0x3e, 0xf6,
		0xba, 0xd7, 0x26, 0x3f, 0x0c, 0xcc, 0xca, 0x13, 0x45, 0x0a,
		0xd8, 0xdf, 0xe5, 0x1b, 0x4a, 0xfc, 0xb2, 0xf7, 0x3f, 0x07,
		0xc6, 0x00, 0x37, 0xfb, 0xc6, 0x06, 0x3c, 0xf5, 0xc3, 0x0c,
		0x3b, 0xef, 0xc6, 0x12, 0x38, 0xeb, 0xc9, 0x18, 0x35, 0xe3,
		0xc9, 0x1f, 0x36, 0xdd, 0xc9, 0x25, 0x34, 0xd8, 0xcd, 0x29,
		0x31, 0xd3, 0xd0, 0x2e, 0x2c, 0xcf, 0xd5, 0x32, 0x28, 0xcb,
		0xda, 0x36, 0x23, 0xc8, 0xdf, 0x39, 0x1e, 0xc5, 0xe4, 0x3b,
		0x18, 0xc2, 0xea, 0x3d, 0x12, 0xc1, 0xf0, 0x3f, 0x0d, 0xbf,
		0xf5, 0x40, 0x07, 0xbe, 0xfc, 0x41, 0x00, 0xbe, 0x01, 0x41,
		0xfa, 0xbf, 0x08, 0x40, 0xf4, 0xbf, 0x0d, 0x3f, 0xef, 0xc1,
		0x13, 0x3d, 0xe9, 0xc3, 0x19, 0x3b, 0xe3, 0xc5, 0x1f, 0x38,
		0xde, 0xc8, 0x24, 0x35, 0xd9, 0xcc, 0x29, 0x31, 0xd4, 0xd0,
		0x2d, 0x2d, 0xd0, 0xd4, 0x31, 0x29, 0xcc, 0xd9, 0x35, 0x24,
		0xc8, 0xde, 0x38, 0x1f, 0xc5, 0xe3, 0x3b, 0x19, 0xc3, 0xe9,
		0x3d, 0x13, 0xc1, 0xef, 0x3f, 0x0d, 0xbf, 0xf4, 0x40, 0x08,
		0xbe, 0xfb, 0x41, 0x01, 0xbe, 0x01, 0x40, 0xfb, 0xbf, 0x07,
		0x3f, 0xf5, 0xc0, 0x0d, 0x3e, 0xef, 0xc1, 0x13, 0x3c, 0xea,
		0xc5, 0x18, 0x39, 0xe5, 0xc8, 0x1b, 0x35, 0xe2, 0xcd, 0x1f,
		0x2f, 0xdf, 0xd7, 0x22, 0x25, 0xe4, 0xe3, 0x11, 0x18, 0xfa,
		0xef, 0x04, 0x07, 0xfe, 0xfd, 0x01, 0x00, 0xff, 0xff, 0xff,
		0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff
	};

	/*
	 * do the initialisation sequence for the speaker:
	 * - enable speaker
	 * - mute speaker
	 * - setup registers & voodoo
	 * - unmute speaker
	 */
	configASSERT(bda != NULL);
	wii_speaker_state(bda, true);
	wii_mute_state(bda, true);

	vTaskDelay(WII_THROTTLE / portTICK_PERIOD_MS);
	memset(report, 0, 22);
	report[0] = WII_REPORT_WRITE_REG;
	report[1] = 0x04;
	report[2] = 0xa2;
	report[3] = 0x00;
	report[4] = 0x09;
	report[5] = 1;
	report[6] = 0x01;
	ret = esp_bt_hid_host_send_data((uint8_t *)bda, report, 22);
	if (ret)
		ESP_LOGE(TAG, "cannot set register, send error: %x", ret);

	vTaskDelay(WII_THROTTLE / portTICK_PERIOD_MS);
	memset(report, 0, 22);
	report[0] = WII_REPORT_WRITE_REG;
	report[1] = 0x04;
	report[2] = 0xa2;
	report[3] = 0x00;
	report[4] = 0x01;
	report[5] = 1;
	report[6] = 0x08;
	ret = esp_bt_hid_host_send_data((uint8_t *)bda, report, 22);
	if (ret)
		ESP_LOGE(TAG, "cannot set register, send error: %x", ret);

	vTaskDelay(WII_THROTTLE / portTICK_PERIOD_MS);
	memset(report, 0, 22);
	report[0] = WII_REPORT_WRITE_REG;
	report[1] = 0x04;
	report[2] = 0xa2;
	report[3] = 0x00;
	report[4] = 0x01;
	report[5] = 7;
	report[6] = 0x00;
	report[7] = 0x40;	/* set PCM mode */
	report[8] = 0xe0;	/* set 1 kHz sample rate */
	report[9] = 0x2e;	/* set 1 kHz sample rate */
	report[10] = 0xe0;	/* increase default volume */
	report[11] = 0x00;
	report[12] = 0x00;
	ret = esp_bt_hid_host_send_data((uint8_t *)bda, report, 22);
	if (ret)
		ESP_LOGE(TAG, "cannot set register, send error: %x", ret);

	vTaskDelay(WII_THROTTLE / portTICK_PERIOD_MS);
	memset(report, 0, 22);
	report[0] = WII_REPORT_WRITE_REG;
	report[1] = 0x04;
	report[2] = 0xa2;
	report[3] = 0x00;
	report[4] = 0x08;
	report[5] = 1;
	report[6] = 0x01;
	ret = esp_bt_hid_host_send_data((uint8_t *)bda, report, 22);
	if (ret)
		ESP_LOGE(TAG, "cannot set register, send error: %x", ret);

	vTaskDelay(WII_THROTTLE / portTICK_PERIOD_MS);
	wii_mute_state(bda, false);
	vTaskDelay(WII_THROTTLE / portTICK_PERIOD_MS);
	memset(report, 0, 22);
	report[0] = WII_REPORT_SPEAKER_D;
	report[1] = 20 << 3; /* 20 bytes each round (20 8bits samples) */
	for (pos = (uint8_t *)chime ; pos < (chime + sizeof(chime)) ; pos += 20) {
		memcpy(report + 2 * sizeof(uint8_t), pos, 20);
		ret = esp_bt_hid_host_send_data((uint8_t *)bda, report, 22);
		if (ret)
			ESP_LOGE(TAG, "cannot stream PCM data, send error: %x", ret);
		/* max 1kHz 8bit PCM is at best what we can stream */
		vTaskDelay(WII_THROTTLE / portTICK_PERIOD_MS);
	}
	wii_speaker_state(bda, false);
}

/* this function will generate movement notifications while a button is keep pressed */
void	wii_dpad(void *pvParameters) {
	uint16_t buttons = 0;
	uint32_t notify = 0;
	int8_t x, y;
	TickType_t delay = portMAX_DELAY;

	(void)pvParameters;

	x = 0;
	y = 0;

	while (true) {
		if (xTaskNotifyWait(0, 0, &notify, delay) == pdTRUE) {
			/* we received a status change */
			buttons = (uint16_t)notify;

			if (! buttons) {
				delay = portMAX_DELAY;
				continue;
			}

			delay = WII_INTERVAL / portTICK_PERIOD_MS;
		}

		/* ignore facerolling */
		if ((buttons & WII_BUTTON_UP) && (buttons & WII_BUTTON_DOWN))
			continue;
		if ((buttons & WII_BUTTON_LEFT) && (buttons & WII_BUTTON_RIGHT))
			continue;

		if (buttons & WII_BUTTON_LEFT) {
			x = -((wii_speed >> 4) * 4);
			xQueueSendToBack(q_qx, &x, 0);
		}
		if (buttons & WII_BUTTON_RIGHT) {
			x = ((wii_speed >> 4) * 4);
			xQueueSendToBack(q_qx, &x, 0);
		}
		if (buttons & WII_BUTTON_UP) {
			y = -((wii_speed >> 4) * 4);
			xQueueSendToBack(q_qy, &y, 0);
		}
		if (buttons & WII_BUTTON_DOWN) {
			y = ((wii_speed >> 4) * 4);
			xQueueSendToBack(q_qy, &y, 0);
		}
	}
}

/*
 * generate the PIN code needed for pairing. Info from https://wiibrew.org/wiki/Wiimote
 * If connecting by holding down the 1+2 buttons, the PIN is the bluetooth address of
 * the wiimote backwards, if connecting by pressing the "sync" button on the back of
 * the wiimote, then the PIN is the bluetooth address of the host backwards.
 *
 * we will implement the last option
 */
uint8_t	*wii_generate_pin(esp_bt_pin_code_t pin) {
	uint8_t *mac;

	configASSERT(pin != NULL);
	mac = (uint8_t *)esp_bt_dev_get_address();
	if (mac == NULL)
		return NULL;

	pin[0] = mac[5];
	pin[1] = mac[4];
	pin[2] = mac[3];
	pin[3] = mac[2];
	pin[4] = mac[1];
	pin[5] = mac[0];
	ESP_LOGV(TAG, "Bluetooth MAC: " ESP_BD_ADDR_STR ", PIN code: " WII_PIN_STR,
		    ESP_BD_ADDR_HEX(mac), WII_PIN_HEX(pin));

	return mac;
}

/*
 * similar to blue_handle_button
 * this will only handle buttons that are used for clicks
 * buttons that are used for movement are handled by the wii_dpad task
 */

static void	wii_handle_button(uint16_t buttons) {
	static bool	locked = false;
	static bool	releasable = true;
	static bool	status = WII_BUTTON_NONE;	// Keep state

	buttons = buttons & (WII_BUTTON_A | WII_BUTTON_B);

	if (status && buttons)
		return ;
	if (status == WII_BUTTON_NONE && buttons == WII_BUTTON_NONE && !locked)
		return ;
	if (buttons == WII_BUTTON_A && locked && !releasable)
		return ;
	if (status == WII_BUTTON_NONE && buttons == WII_BUTTON_NONE && locked) {
		releasable = true;
		return ;
	}

	/* release button */
	if (status && buttons == WII_BUTTON_NONE && !locked) {
		xTaskNotify(t_click, 0, eSetValueWithOverwrite);
		status = 0;
		ESP_LOGD(TAG, "button released");
		return ;
	}

	/* stick button on right click */
	if (status == WII_BUTTON_NONE && buttons == WII_BUTTON_A && !locked) {
		xTaskNotify(t_click, 1, eSetValueWithOverwrite);
		locked = true;
		releasable = false;
		ESP_LOGD(TAG, "button locked");
		return ;
	}

	/* press button (simple click) */
	if (status == WII_BUTTON_NONE && buttons) {
		xTaskNotify(t_click, 1, eSetValueWithOverwrite);
		locked = false;
		status = 1;
		ESP_LOGD(TAG, "button pressed");
		return ;
	}
}

void	wii_init(void) {
	/* put default values into calibration data */
	memset(&wii_calibration_0g, 0, sizeof(wii_accel_t));
	memset(&wii_calibration_1g, 0, sizeof(wii_accel_t));

	xTaskCreate(wii_dpad, "WII_DPAD", 1024, NULL, tskIDLE_PRIORITY + 1, &t_wii_dpad);
}

void	wii_input(esp_hidh_dev_t *dev, uint16_t id, uint8_t *data, uint16_t length) {
	uint16_t buttons = 0;
	static bool changed = 0;

	/* do some basic sanity checks then ignore every report we cannot handle */
	if (id == WII_REPORT_CORE_ONLY && length != 2) {
		xTaskNotify(t_red, LED_ONCE, eSetValueWithOverwrite);
		return ;
	}
	if (id == WII_REPORT_ACK && length != 4) {
		xTaskNotify(t_red, LED_ONCE, eSetValueWithOverwrite);
		return ;
	}
	if (id == WII_REPORT_CORE_ACCEL && length != 5) {
		xTaskNotify(t_red, LED_ONCE, eSetValueWithOverwrite);
		return ;
	}
	if (id == WII_REPORT_STATUS && length != 6) {
		xTaskNotify(t_red, LED_ONCE, eSetValueWithOverwrite);
		return ;
	}
	if (id == WII_REPORT_READ_INPUT && length != 21) {
		xTaskNotify(t_red, LED_ONCE, eSetValueWithOverwrite);
		return ;
	}

	if (id != WII_REPORT_ACK && id != WII_REPORT_CORE_ONLY &&
	    id != WII_REPORT_STATUS && id != WII_REPORT_CORE_ACCEL &&
	    id != WII_REPORT_READ_INPUT) {
		ESP_LOGW(TAG, "Unknown report id 0x%x received (size: %i)", id, length);
		return;
	}

	/*
	 * handle error codes or specific stuff from the wiimote
	 * we can continue to handle core buttons as usual later
	 */
	switch (id) {
		case WII_REPORT_STATUS:
			/* format is BB BB LF 00 00 VV (buttons, flags, battery) */
			ESP_LOGD(TAG, "status: 0x%x%x 0x%x %i", data[0], data[1], data[2], data[5]);
			break;
		case WII_REPORT_READ_INPUT:
			/* assume a calibration read, since it's the only read request done */
			wii_show_calibration(data);
			break;
		case WII_REPORT_ACK:
			/* format is BB BB RR EE (buttons, report number, error code) */
			if (data[3]) {
				xTaskNotify(t_red, LED_ONCE, eSetValueWithOverwrite);
				ESP_LOGE(TAG, "error with report id 0x%x, code %x", data[2], data[3]);
			}
			break;
		case WII_REPORT_CORE_ACCEL:
			wii_accelerometer(dev, data);
			break;
		default:
			break;
	}

	buttons = (data[1] << 8) | data[0];
	/* changing speed is disabled in accelerometer mode */
	if ((buttons & WII_BUTTON_PLUS || buttons & WII_BUTTON_MINUS) && length == 2)
		wii_change_speed(dev, buttons);

	/* swap modes (enable / disable accelerometer) */
	if (buttons & WII_BUTTON_HOME) {
		if (! changed) {
			wii_change_mode(dev);
			changed = true;
		}
	}
	if (changed && !(buttons & WII_BUTTON_HOME))
		changed = false;

	/* do the buttons things */
	wii_handle_button(buttons);
	xTaskNotify(t_wii_dpad, (uint32_t)buttons, eSetValueWithOverwrite);
}

/*
 * this will compare the provided BDA to a list of known Nintendo list
 * return true if the provided BDA is a known prefix or fakse otherwise
 */
bool wii_is_nintendo_bda(esp_bd_addr_t bda) {
	uint8_t	i = 0;

	/* known "Nintendo Co., Ltd." BDA prefixes as of 08/2024 */
	const esp_bd_addr_t nintendo_bda_list[] = {
		{0x00, 0x09, 0xbf, 0x00, 0x00, 0x00}, /* Start of Nintendo BDAs */
		{0x00, 0x16, 0x56, 0x00, 0x00, 0x00},
		{0x00, 0x17, 0xab, 0x00, 0x00, 0x00},
		{0x00, 0x19, 0x1d, 0x00, 0x00, 0x00},
		{0x00, 0x19, 0xfd, 0x00, 0x00, 0x00},
		{0x00, 0x1a, 0xe9, 0x00, 0x00, 0x00},
		{0x00, 0x1b, 0x7a, 0x00, 0x00, 0x00},
		{0x00, 0x1b, 0xea, 0x00, 0x00, 0x00},
		{0x00, 0x1c, 0xbe, 0x00, 0x00, 0x00},
		{0x00, 0x1d, 0xbc, 0x00, 0x00, 0x00},
		{0x00, 0x1e, 0x35, 0x00, 0x00, 0x00},
		{0x00, 0x1e, 0xa9, 0x00, 0x00, 0x00},
		{0x00, 0x1f, 0x32, 0x00, 0x00, 0x00},
		{0x00, 0x1f, 0xc5, 0x00, 0x00, 0x00},
		{0x00, 0x21, 0x47, 0x00, 0x00, 0x00},
		{0x00, 0x21, 0xbd, 0x00, 0x00, 0x00},
		{0x00, 0x22, 0x4c, 0x00, 0x00, 0x00},
		{0x00, 0x22, 0xaa, 0x00, 0x00, 0x00},
		{0x00, 0x22, 0xd7, 0x00, 0x00, 0x00},
		{0x00, 0x23, 0x31, 0x00, 0x00, 0x00},
		{0x00, 0x23, 0xcc, 0x00, 0x00, 0x00},
		{0x00, 0x24, 0x1e, 0x00, 0x00, 0x00},
		{0x00, 0x24, 0x44, 0x00, 0x00, 0x00},
		{0x00, 0x24, 0xf3, 0x00, 0x00, 0x00},
		{0x00, 0x25, 0xa0, 0x00, 0x00, 0x00},
		{0x00, 0x26, 0x59, 0x00, 0x00, 0x00},
		{0x00, 0x27, 0x09, 0x00, 0x00, 0x00},
		{0x18, 0x2a, 0x7b, 0x00, 0x00, 0x00},
		{0x2c, 0x10, 0xc1, 0x00, 0x00, 0x00},
		{0x34, 0xaf, 0x2c, 0x00, 0x00, 0x00},
		{0x40, 0xd2, 0x8a, 0x00, 0x00, 0x00},
		{0x40, 0xf4, 0x07, 0x00, 0x00, 0x00},
		{0x58, 0xbd, 0xa3, 0x00, 0x00, 0x00},
		{0x78, 0xa2, 0xa0, 0x00, 0x00, 0x00},
		{0x7c, 0xbb, 0x8a, 0x00, 0x00, 0x00},
		{0x8c, 0x56, 0xc5, 0x00, 0x00, 0x00},
		{0x8c, 0xcd, 0xe8, 0x00, 0x00, 0x00},
		{0x9c, 0xe6, 0x35, 0x00, 0x00, 0x00},
		{0xa4, 0x5c, 0x27, 0x00, 0x00, 0x00},
		{0xa4, 0xc0, 0xe1, 0x00, 0x00, 0x00},
		{0xb8, 0xae, 0x6e, 0x00, 0x00, 0x00},
		{0xcc, 0x9e, 0x00, 0x00, 0x00, 0x00},
		{0xcc, 0xfb, 0x65, 0x00, 0x00, 0x00},
		{0xd8, 0x6b, 0xf7, 0x00, 0x00, 0x00},
		{0xe0, 0x0c, 0x7f, 0x00, 0x00, 0x00},
		{0xe0, 0xe7, 0x51, 0x00, 0x00, 0x00},
		{0xe8, 0x4e, 0xce, 0x00, 0x00, 0x00}, /* End of Nintendo BDAs */
		{0x08, 0x03, 0x64, 0x00, 0x00, 0x00}, /* PDP Rock Candy compatible Wiimotes */
		{0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
	};

	while (nintendo_bda_list[i][0] || nintendo_bda_list[i][1] || nintendo_bda_list[i][2]) {
		if (nintendo_bda_list[i][0] == bda[0] &&
		    nintendo_bda_list[i][1] == bda[1] &&
		    nintendo_bda_list[i][2] == bda[2]) {
			return true;
		  }

		  i++;
	}

	return false;
}

/* this will tell us if the device is a Wiimote. PID + VID provided by the HID stack */
bool wii_is_wiimote(uint16_t vid, uint16_t pid) {
	if (vid != WII_VID)
		return false;
	if (pid != WII_PID_OLD && pid != WII_PID_NEW)
		return false;

	return true;
}

/* this is called every time a Wiimote is connected */
void wii_open(const esp_bd_addr_t bda) {
	uint8_t report[7] = {0};
	esp_err_t ret;

	configASSERT(bda != NULL);
	/* enable LED4 on connect */
	report[0] = WII_REPORT_LEDS;
	report[1] = WII_LED4;
	ret = esp_bt_hid_host_send_data((uint8_t *)bda, report, 2);
	if (ret)
		ESP_LOGE(TAG, "cannot set LED4 state, send error: %x", ret);

	/* set initial LEDs state */
	wii_speed = 0;

	/* stream the boot chime to the wii mote. this will also wake up the rumble */
	wii_chime(bda);
	wii_rumble_state(bda, true);
	vTaskDelay(400 / portTICK_PERIOD_MS);
	wii_speed = WII_SPEED_MEDIUM;
	wii_rumble_state(bda, false);

	/* do a read to fetch accelerometer calibration data from both spaces */
	report[0] = WII_REPORT_READ_REG;
	report[1] = 0x00;	/* EEPROM space */
	report[2] = 0x00;   /* read from 0x000016 */
	report[3] = 0x00;   /* read from 0x000016 */
	report[4] = 0x16;   /* read from 0x000016 */
	report[5] = 0x00;   /* read 10 bytes */
	report[6] = 0x0a;   /* read 10 bytes */
	ret = esp_bt_hid_host_send_data((uint8_t *)bda, report, 7);
	if (ret)
		ESP_LOGE(TAG, "cannot read accelerometer calibration, send error: %x", ret);
	report[0] = WII_REPORT_READ_REG;
	report[1] = 0x00;	/* EEPROM space */
	report[2] = 0x00;   /* read from 0x000020 */
	report[3] = 0x00;   /* read from 0x000020 */
	report[4] = 0x20;   /* read from 0x000020 */
	report[5] = 0x00;   /* read 10 bytes */
	report[6] = 0x0a;   /* read 10 bytes */
	ret = esp_bt_hid_host_send_data((uint8_t *)bda, report, 7);
	if (ret)
		ESP_LOGE(TAG, "cannot read accelerometer calibration, send error: %x", ret);
}

/* mute the speaker on the Wiimote. this will clear the rumble ! */
static void wii_mute_state(const esp_bd_addr_t bda, bool state) {
	uint8_t report[2] = {0};
	esp_err_t ret;

	configASSERT(bda != NULL);
	ESP_LOGV(TAG, "setting mute to %x", state);
	report[0] = WII_REPORT_SPEAKER_M;
	if (state)
		report[1] = 0x04;
	else
		report[1] = 0x0;
	ret = esp_bt_hid_host_send_data((uint8_t *)bda, report, 2);
	if (ret)
		ESP_LOGE(TAG, "cannot set mute state, send error: %x", ret);
}

/* enable the rumble on the Wiimote. this will clear the LEDs so keep the LEDs state */
static void wii_rumble_state(const esp_bd_addr_t bda, bool state) {
	uint8_t report[2] = {0};
	esp_err_t ret;

	if (state) {
		report[0] = WII_REPORT_RUMBLE;
		report[1] = 0x1 | WII_LED4;
	} else {
		report[0] = WII_REPORT_LEDS;
		report[1] = wii_speed | WII_LED4;
	}
	ret = esp_bt_hid_host_send_data((uint8_t *)bda, report, 2);
	if (ret)
		ESP_LOGE(TAG, "cannot set rumble state, send error: %x", ret);
}

/* enable the speaker on the Wiimote. this will clear the rumble ! */
static void wii_speaker_state(const esp_bd_addr_t bda, bool state) {
	uint8_t report[2] = {0};
	esp_err_t ret;

	configASSERT(bda != NULL);
	ESP_LOGV(TAG, "setting speaker state to %x", state);
	report[0] = WII_REPORT_SPEAKER_E;
	if (state)
		report[1] = 0x04;
	else
		report[1] = 0x0;
	ret = esp_bt_hid_host_send_data((uint8_t *)bda, report, 2);
	if (ret)
		ESP_LOGE(TAG, "cannot set speaker state, send error: %x", ret);
}

/* display calibration data. do not save it for now since we are calibrating ourselves */
void	wii_show_calibration(uint8_t *data) {
	uint8_t	i;
	uint8_t	checksum;

	configASSERT(data != NULL);

	ESP_LOGD(TAG, "EEPROM read from 0x%02x%02x", data[3], data[4]);
	/* skip the header */
	data += 5;
	wii_calibration_0g.x = ((data[0] << 2) | ((data[3] >> 4) & 3));
	wii_calibration_0g.y = ((data[1] << 2) | ((data[3] >> 2) & 3));
	wii_calibration_0g.z = ((data[2] << 2) | (data[3] & 3));

	wii_calibration_1g.x = (((data[4] << 2) | ((data[7] >> 4) & 3)) - wii_calibration_0g.x);
	wii_calibration_1g.y = (((data[5] << 2) | ((data[7] >> 2) & 3)) - wii_calibration_0g.y);
	wii_calibration_1g.z = (((data[6] << 2) | (data[7] & 3)) - wii_calibration_0g.z);

	ESP_LOGD(TAG, "Wiimote calibration data: 0G (%i %i %i) 1G (%i %i %i)…",
		     wii_calibration_0g.x, wii_calibration_0g.y, wii_calibration_0g.z,
		     wii_calibration_1g.x, wii_calibration_1g.y, wii_calibration_1g.z);

	checksum = 0;
	for (i = 0 ; i < 9 ; i++)
		checksum += data[i];
	checksum += 0x55;
	ESP_LOGD(TAG, "\t… checksum read: %02X, calculated: %02X", data[9], checksum);
	if (data[9] != checksum)
		xTaskNotify(t_red, LED_ONCE, eSetValueWithOverwrite);
}
