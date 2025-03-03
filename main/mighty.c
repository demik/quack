/*
 *  mighty.c
 *  mighty
 *
 *  Created by Michel DEPEIGE on 03/03/2025.
 *  Copyright (c) 2025 Michel DEPEIGE.
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

#include "blue.h"
#include "led.h"
#include "mighty.h"

/* functions */

/*
 * this function will decode a mighty frame and convert it for quadrature tasks
 *
 * Byte | Bits | Description
 * -----+------+---------------------------------------------------------------
 * 0    | 0    | Button 1 (left)
 * 0    | 1    | Button 2 (right)
 * 0    | 2    | Button 3 (middle)
 * 0    | 3-7  | unused
 * 1    | 0-7  | X Displacement
 * 2    | 0-7  | Y Displacement
 * 3    | 0-7  | Ball X Axis
 * 4    | 0-7  | Ball Y Axis
 * 5    | 0-7  | pressure sensor. Follow roughly (0)
 *
 * Original HID dump
 * +-------------------------------------------------+
 * | 05 01 09 02 a1 01 85 02 05 09 19 01 29 04 15 00 |
 * | 09 01 a1 00 15 81 25 7f 09 30 09 31 75 08 95 02 |
 * | 81 06 05 0c 0a 38 02 75 08 95 01 81 06 05 01 09 |
 * | 38 75 08 95 01 81 06 c0 05 ff 09 c0 75 08 95 01 |
 * | 81 02 05 06 09 20 85 47 15 00 25 64 75 08 95 01 |
 * | b1 a2 c0                                        |
 * +-------------------------------------------------+
 */

void	mighty_input(esp_hidh_dev_t *dev, uint16_t id, uint8_t *data, uint16_t length) {
	/*
	 * Mighty Mouse reports are exactly 6 bytes. Check for size in case of random error
	 * The button checks and handling follow the boot protocol
	 *
	 * Since the first 3 bytes are the same as boot, let's handle some errors here
	 * and call the boot decoder with a truncated report (ignore pressure and ball)
	 */

	if (length != 6) {
		xTaskNotify(t_red, LED_ONCE, eSetValueWithOverwrite);
		return;
	}
	if (data[3] || data[4]) {
		xTaskNotify(t_red, LED_ONCE, eSetValueWithOverwrite);
	}

	blue_h_boot(data, 3);
}

/*
 * this will tell us if the device is a supported Mighty Mouse
 * PID + VID provided by the HID stack
 */
bool mighty_is_mm(uint16_t vid, uint16_t pid) {
	if (vid != MIGHTY_VID)
		return false;
	if (pid != MIGHTY_PID)
		return false;

	return true;
}