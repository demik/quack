/*
 *  blue.c
 *  quack
 *
 *  Created by Michel DEPEIGE on 18/07/2021.
 *  Copyright (c) 2021 Michel DEPEIGE.
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
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#if CONFIG_BT_BLE_ENABLED
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#endif
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_timer.h"

#include "esp_hid_common.h"
#include "esp_hidd.h"
#include "esp_hidh.h"
#include "esp_hid_gap.h"

#include "adb.h"
#include "blue.h"
#include "gpio.h"
#include "led.h"
#include "m4848.h"
#include "mighty.h"
#include "wii.h"

/* debug tag */
static const char *TAG = "blue";

/* globals */
TaskHandle_t t_adb2hid;
extern TaskHandle_t t_click;
static esp_hidd_dev_t *hid_dev = NULL;
extern QueueHandle_t q_qx, q_qy;
uint8_t	blue_pointers;	// mouse count

/* private functions */
static void blue_d_connect(void);
static void blue_d_disconnect(esp_hidd_event_data_t *dev);
static void blue_d_init(void);
static void blue_d_start(void);
void blue_h_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data);
static void blue_h_init(void);
static esp_hid_report_item_t blue_h_ri_find(esp_hidh_dev_t *d, esp_hid_usage_t u, uint8_t t, uint8_t p);
static void	blue_handle_button(uint8_t buttons);
static esp_hid_raw_report_map_t	*blue_hid_rm_get(esp_hidh_dev_t *dev);
void blue_set_boot_protocol(esp_hidh_dev_t *dev);
static bool blue_support_boot(esp_hidh_dev_t *dev);

/* direct calls to esp_hid_* */
void bt_gap_event_handler(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);

/* Device specific functions blue_d_* */
static void blue_d_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
	esp_hidd_event_t event = (esp_hidd_event_t)id;
	esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;

	switch (event) {
		case ESP_HIDD_START_EVENT: {
			blue_d_start();
			break;
		}
		case ESP_HIDD_CONNECT_EVENT: {
			blue_d_connect();
			break;
		}
		case ESP_HIDD_PROTOCOL_MODE_EVENT: {
			ESP_LOGI(TAG, "PROTOCOL MODE[%u]: %s", param->protocol_mode.map_index, param->protocol_mode.protocol_mode ? "REPORT" : "BOOT");
			break;
		}
		case ESP_HIDD_CONTROL_EVENT: {
			ESP_LOGI(TAG, "CONTROL[%u]: %sSUSPEND", param->control.map_index, param->control.control ? "EXIT_" : "");
			break;
		}
		case ESP_HIDD_OUTPUT_EVENT: {
			ESP_LOGI(TAG, "OUTPUT[%u]: %8s ID: %2u, Len: %d, Data:", param->output.map_index, esp_hid_usage_str(param->output.usage), param->output.report_id, param->output.length);
			ESP_LOG_BUFFER_HEX(TAG, param->output.data, param->output.length);
			break;
		}
		case ESP_HIDD_FEATURE_EVENT: {
			ESP_LOGI(TAG, "FEATURE[%u]: %8s ID: %2u, Len: %d, Data:", param->feature.map_index, esp_hid_usage_str(param->feature.usage), param->feature.report_id, param->feature.length);
			ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
			break;
		}
		case ESP_HIDD_DISCONNECT_EVENT: {
			blue_d_disconnect(param);
			break;
		}
		case ESP_HIDD_STOP_EVENT: {
			ESP_LOGI(TAG, "STOP");
			break;
		}
		default:
			break;
	}
	return;
}

static void blue_d_connect() {
	ESP_LOGI(TAG, "Host connected");
	xTaskNotify(t_blue, LED_ON, eSetValueWithOverwrite);
}

static void blue_d_disconnect(esp_hidd_event_data_t *dev) {
	ESP_LOGI(TAG, "Host disconnected, reason: %s",
			 esp_hid_disconnect_reason_str(esp_hidd_dev_transport_get(dev->disconnect.dev), dev->disconnect.reason));
#if CONFIG_BT_BLE_ENABLED
	esp_hid_ble_gap_adv_start();
#endif
	xTaskNotify(t_blue, LED_SLOW, eSetValueWithOverwrite);
}

static void blue_d_init() {
	esp_err_t	ret;

	if ((ret = esp_bt_gap_register_callback(bt_gap_event_handler)) != ESP_OK) {
		ESP_LOGE(TAG, "BT GAP register callback failed: %d", ret);
		return;
	}

	ESP_ERROR_CHECK(esp_hidd_dev_init(&m4848_config, ESP_HID_TRANSPORT_BT, blue_d_callback, &hid_dev));
	xTaskCreate(blue_adb2hid, "ADB2BT", 2 * 1024, NULL, tskIDLE_PRIORITY + 1, &t_adb2hid);
}

/*
 * Called when the ESP HIS stack is started
 * Start advertising our Bluetooth stuff
 */

static void blue_d_start()
{
	ESP_LOGD(TAG, "Bluetooth stack started");
	xTaskNotify(t_blue, LED_SLOW, eSetValueWithOverwrite);
#if CONFIG_BT_BLE_ENABLED
	esp_hid_ble_gap_adv_start();
#endif
}

/*
 * Called by the ADB stack from adb_task_host on mouse activity
 * Convert the 16bit ADB data to a 3 bytes HID INPUT REPORT matching the m4848
 *
 * The format is also BOOT compatible so don't bother to check what mode we are in
 */

void	blue_adb2hid(void *pvParameters) {
	uint16_t data = 0;
	uint8_t buffer[3] = {0, 0, 0};
	int8_t move = 0;
	long unsigned int tmp;

	ESP_LOGD(TAG, "ADB2BT started on core %d", xPortGetCoreID());

	while (true) {
		xTaskNotifyWait(0, 0, &tmp, portMAX_DELAY);
		data = (uint16_t)tmp;

		if (! esp_hidd_dev_connected(hid_dev))
			continue;

		/* button pressed */
		if (! (data & ADB_CMP_B1) || (! (data & ADB_CMP_B2)))
			buffer[0] = (1<<0);
		else
			buffer[0] = 0;

		/*
		 * cast negative signed 7 bits to signed 8 bits
		 * then apply a slight acceleration (x^1.2)
		 */

		move = (data & ADB_CMP_MX) >> 0;
		if (move & 0x40) {
			move &= ~0x40;
			move |= 0x80;
			move += 64;
			move *= -1;
			buffer[1] = m4848_accel[move];
			buffer[1] = buffer[1] * -1;
		}
		else {
			buffer[1] = m4848_accel[move];
		}

		move = (data & ADB_CMP_MY) >> 8;
		if (move & 0x40) {
			move &= ~0x40;
			move |= 0x80;
			move += 64;
			move *= -1;
			buffer[2] = m4848_accel[move];
			buffer[2] = buffer[2] * -1;
		}
		else {
			buffer[2] = m4848_accel[move];
		}

		esp_hidd_dev_input_set(hid_dev, 1, 0, buffer, 3);
	}
}

/*
 * this function try to guess wich pin code to use when asked for one
 * it just harass the different drivers and get a pin code from the driver
 */
uint8_t blue_get_pin_code(esp_bd_addr_t bda, esp_bt_pin_code_t pin)
{
	if (wii_is_nintendo_bda(bda)) {
		wii_generate_pin(pin);
		ESP_LOGD(TAG, "Nintendo device asking for pin, trying with pin " WII_PIN_STR,
			    WII_PIN_HEX(pin));
		return 6;
	}

	/* let's try with default pin */
	ESP_LOGI(TAG, "no match found, trying with default pin code: 1234");
	pin[0] = '1';
	pin[0] = '2';
	pin[0] = '3';
	pin[0] = '4';

	return 4;
}

/************************************
 * Host specific functions blue_h_* *
 ************************************/

/*
 * this function will decode a boot protocol frame and convert it for quadrature tasks
 * data arg follows HID Mouse boot protocol format
 *
 * Byte | Bits | Description
 * -----+------+---------------------------------------------------------------
 * 0    | 0    | Button 1 (left)
 * 0    | 1    | Button 2 (right)
 * 0    | 2    | Button 3 (middle)
 * 0    | 3-7  | Device specific, usually unused (0)
 * 1    | 0-7  | X Displacement
 * 2    | 0-7  | Y Displacement
 * 3+   | 0-7  | Device specific, usually 3 is scroll wheel, usually unused (0)
 */

void blue_h_boot(uint8_t *data, uint16_t length) {
	uint8_t buttons;
	uint8_t	i;
	int8_t x, y;

	buttons = data[0];

	/*
	 * A friend of mine did this for a beer, it helps a little with high DPI mouses
	 * This is a precalculated table that looks like a squared arctan()
	 */

	const unsigned char hid2quad[] = {
		0x01, 0x01, 0x02, 0x02, 0x03, 0x04, 0x05, 0x06,
		0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0E, 0x0F,
		0x11, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19,
		0x1A, 0x1C, 0x1D, 0x1E, 0x20, 0x21, 0x22, 0x23,
		0x24, 0x25, 0x26, 0x26, 0x27, 0x27, 0x28, 0x28
	};

	/*
	 * Do some checks before parsing data. We sould't get anything wrong in theory,
	 * but we sometimes to get shit from either mouse or bluetooth stack
	 *
	 * Stack issue: https://github.com/espressif/esp-idf/issues/6985
	 */

	if (length < 3) {
		xTaskNotify(t_red, LED_ONCE, eSetValueWithOverwrite);
		return;
	}
	if (buttons & BLUE_BUTTON_E) {
		xTaskNotify(t_red, LED_ONCE, eSetValueWithOverwrite);
		return;
	}

	/*
	 * we also want to ignore anything that contains scroll wheel & co for now
	 * statistically theses are corrupted packets
	 */

	for (i = 3; i < length; i++) {
		if (data[i]) {
			xTaskNotify(t_red, LED_ONCE, eSetValueWithOverwrite);
			return;
		}
	}

	blue_handle_button(buttons);

	/*
	 * this try to help avoiding issues with high DPI mouses:
	 * - reduce bluetooth movement speed a little bit before 40 pixels per poll
	 * - go slower after 40 pixels per poll just to avoid too big jumps
	 */

	x = data[1];
	y = data[2];

	if (x != 0) {
		if (x < 0) {
			if (x < -40)
				x = ((x + 40) / 2) - 40;
			else
				x = -1 * hid2quad[(x * -1) - 1];
		}
		else {
			if (x > 40)
				x = ((x - 40) / 2) + 40;
			else
				x = hid2quad[x - 1];
		}
		xQueueSendToBack(q_qx, &x, 0);
	}
	if (y != 0) {
		if (y < 0) {
			if (y < -40)
				y = ((y + 40) / 2) - 40;
			else
				y = -1 * hid2quad[(y * -1) - 1];
		}
		else {
			if (y > 40)
				y = ((y - 40) / 2) + 40;
			else
				y = hid2quad[y - 1];
		}
		xQueueSendToBack(q_qy, &y, 0);
	}
}

void blue_h_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
	esp_hidh_event_t event = (esp_hidh_event_t)id;
	esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;

	const uint8_t *bda = NULL;

	/* union described in include/esp_hidh.h */
	switch (event) {
		case ESP_HIDH_OPEN_EVENT: {
			esp_hidh_dev_dump(param->open.dev, stdout);
			blue_h_open(param);
			break;
		}
		case ESP_HIDH_BATTERY_EVENT: {
			bda = esp_hidh_dev_bda_get(param->battery.dev);
			ESP_LOGI(TAG, ESP_BD_ADDR_STR " battery: %d%%", ESP_BD_ADDR_HEX(bda), param->battery.level);
			break;
		}
		case ESP_HIDH_INPUT_EVENT: {
			//ESP_LOG_BUFFER_HEX(TAG, param->input.data, param->input.length);
			xTaskNotify(t_yellow, LED_ONCE, eSetValueWithOverwrite);
			blue_h_input(param->input.dev, param->input.report_id, param->input.data, param->input.length);
			break;
					   }
		case ESP_HIDH_FEATURE_EVENT: {
			bda = esp_hidh_dev_bda_get(param->feature.dev);
							 ESP_LOGI(TAG, ESP_BD_ADDR_STR " FEATURE: %8s, MAP: %2u, ID: %3u, Len: %d", ESP_BD_ADDR_HEX(bda), esp_hid_usage_str(param->feature.usage), param->feature.map_index, param->feature.report_id, param->feature.length);
							 ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
							 break;
						 }
		case ESP_HIDH_CLOSE_EVENT: {
			blue_h_close(param);
			break;
		}
		default:
			ESP_LOGI(TAG, "Unknwown event: %d", event);
	}
}

void blue_h_close(esp_hidh_event_data_t *p) {
	const uint8_t *bda = NULL;

	configASSERT(p != NULL);
	bda = esp_hidh_dev_bda_get(p->close.dev);
	ESP_LOGI(TAG, "closed connection with device " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(bda));
	esp_hidh_dev_free(p->close.dev);

	blue_pointers--;
	if (! blue_pointers)
		xTaskNotify(t_blue, LED_SLOW, eSetValueWithOverwrite);
}

/*
 * this function dispatch the input message to the correct driver
 * the detection is based on Product ID + Vendor ID lists that each driver provides
 * if no specific driver claims that combinaison, try with the generic boot driver
 */
void blue_h_input(esp_hidh_dev_t *dev, uint16_t id, uint8_t *data, uint16_t length) {
     uint16_t vid = esp_hidh_dev_vendor_id_get(dev);
     uint16_t pid = esp_hidh_dev_product_id_get(dev);

	if (mighty_is_mm(vid, pid)) {
		mighty_input(dev, id, data, length);
		return;
	}
	if (wii_is_wiimote(vid, pid)) {
		wii_input(dev, id, data, length);
		return;
	}

	blue_h_boot(data, length);
}

static void blue_h_init(void) {
	esp_hidh_config_t config = {
		.callback = blue_h_callback,
		.event_stack_size = 4096
	};

	ESP_ERROR_CHECK(esp_hidh_init(&config));

	blue_pointers = 0;
	xTaskCreatePinnedToCore(blue_scan, "blue_scan", 6 * 1024, NULL, 2, NULL, 0);

	/* bluetooth stack ready at this point, initialise wii stuff there */
	wii_init();
}

static void	blue_handle_button(uint8_t buttons) {
	static bool	locked = false;
	static bool	releasable = true;
	static bool	status = BLUE_BUTTON_N;	// Keep state

	buttons = buttons & (BLUE_BUTTON_1 | BLUE_BUTTON_2 | BLUE_BUTTON_3);

	if (status && buttons)
		return ;
	if (status == BLUE_BUTTON_N && buttons == BLUE_BUTTON_N && !locked)
		return ;
	if (buttons == BLUE_BUTTON_2 && locked && !releasable)
		return ;
	if (status == BLUE_BUTTON_N && buttons == BLUE_BUTTON_N && locked) {
		releasable = true;
		return ;
	}

	/* release button */
	if (status && buttons == BLUE_BUTTON_N && !locked) {
		xTaskNotify(t_click, 0, eSetValueWithOverwrite);
		status = 0;
		ESP_LOGD(TAG, "button released");
		return ;
	}

	/* stick button on right click */
	if (status == BLUE_BUTTON_N && buttons == BLUE_BUTTON_2 && !locked) {
		xTaskNotify(t_click, 1, eSetValueWithOverwrite);
		locked = true;
		releasable = false;
		ESP_LOGD(TAG, "button locked");
		return ;
	}

	/* press button (simple click) */
	if (status == BLUE_BUTTON_N && buttons) {
		xTaskNotify(t_click, 1, eSetValueWithOverwrite);
		locked = false;
		status = 1;
		ESP_LOGD(TAG, "button pressed");
		return ;
	}
}

/*
 * Bluetooth common init: init module and various stuff
 * Host or Device specific inits go in blue_d_init() or blue_h_init()
 */
void blue_init(void)
{
	esp_err_t	ret;

	ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}

	ESP_LOGD(TAG, "Starting Bluetooth init on core %d", xPortGetCoreID());
	ESP_ERROR_CHECK(ret);
	ESP_ERROR_CHECK(esp_hid_gap_init(ESP_BT_MODE_CLASSIC_BT));

	esp_log_level_set("event", ESP_LOG_INFO);
	/* complains about wrong data len on BOOT mode and CCONTROL */
	esp_log_level_set("BT_HIDH", ESP_LOG_VERBOSE);

	/*
	 * at this point, everything but bluetooth is started.
	 * put green steady, start blinking blue and keep scanning until a device is found
	 *
	 * starting IDF 5.2.2, sometimes the first xTaskNotify() call may be lost. why ?
	 */

	xTaskNotify(t_green, LED_ON, eSetValueWithOverwrite);
	xTaskNotify(t_green, LED_ON, eSetValueWithOverwrite);
	xTaskNotify(t_blue, LED_FAST, eSetValueWithOverwrite);

	if (adb_is_host())
		blue_d_init();
	else
		blue_h_init();
}

void blue_h_open(esp_hidh_event_data_t *p) {
	const uint8_t *bda = NULL;
	bool	generic = true;

	configASSERT(p != NULL);
	uint16_t vid = esp_hidh_dev_vendor_id_get(p->open.dev);
     uint16_t pid = esp_hidh_dev_product_id_get(p->open.dev);
	bda = esp_hidh_dev_bda_get(p->open.dev);
	ESP_LOGI(TAG, "opened connection with " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(bda));

	if (wii_is_wiimote(vid, pid)) {
		wii_open(bda);
		generic = false;
	}

	/* Dump various info on console for generic devices */
	if (generic) {
		blue_hid_rm_get(p->open.dev);
		blue_set_boot_protocol(p->open.dev);
		if (blue_support_boot(p->open.dev) == false) {
			xTaskNotify(t_red, LED_ONCE, eSetValueWithOverwrite);
			ESP_LOGE(TAG, "Mouse does not support boot protocol !");
		}
	}

	blue_pointers++;
	xTaskNotify(t_blue, LED_ON, eSetValueWithOverwrite);
	gpio_transceiver_enable();
}

/* get specific report from report map matching specified usage + type + protocol */
static esp_hid_report_item_t blue_h_ri_find(esp_hidh_dev_t *d, esp_hid_usage_t u, uint8_t t, uint8_t p) {
	size_t num_reports = 0;
	esp_hid_report_item_t *reports;
	esp_hid_report_item_t search;

	configASSERT(d != NULL);
	configASSERT(p <= ESP_HID_REPORT_TYPE_FEATURE);
	configASSERT(t > 0);

	esp_hidh_dev_reports_get(d, &num_reports, &reports);
	memset(&search, 0, sizeof(esp_hid_report_item_t));

	for (uint8_t i = 0; i < num_reports; i++) {
		if (reports[i].protocol_mode == p && reports[i].report_type == t && reports[i].usage == u) {
			memcpy(&search, &reports[i], sizeof(esp_hid_report_item_t));
			break;
		}
	}

	free(reports);
	if (search.value_len == 0) {
		ESP_LOGW(TAG, "%s %s %s not found",
				 esp_hid_usage_str(u),
				 esp_hid_report_type_str(t),
				 esp_hid_protocol_mode_str(p));
	}
	else {
		ESP_LOGD(TAG, "%s %s %s is id %i size %i byte(s)",
				 esp_hid_usage_str(u),
				 esp_hid_report_type_str(t),
				 esp_hid_protocol_mode_str(p),
				 search.report_id,
				 search.value_len);
	}

	return search;
}

/* get report map raw pointer. Also dump it on JTAG */
static esp_hid_raw_report_map_t *blue_hid_rm_get(esp_hidh_dev_t *dev) {
	size_t num_maps = 0;
	esp_hid_raw_report_map_t *maps;

	configASSERT(dev != NULL);
	ESP_LOGI(TAG, BLUE_SNIP);
	esp_hidh_dev_report_maps_get(dev, &num_maps, &maps);
	ESP_LOG_BUFFER_HEX(TAG, maps[0].data, maps[0].len);
	ESP_LOGI(TAG, BLUE_SNIP);

	/* looking at 4.2 SDK source, seems there is always only one map */
	configASSERT(num_maps == 1);
	return maps;
}

void blue_scan(void *pvParameters) {
	size_t len = 0;
	esp_hid_scan_result_t *mouse = NULL;
	esp_hid_scan_result_t *results = NULL;
	esp_hid_scan_result_t *wii = NULL;

	ESP_LOGI(TAG, "starting scan on core %d…", xPortGetCoreID());
	esp_hid_scan(BLUE_SCAN_DURATION, &len, &results);
	ESP_LOGI(TAG, "scan returned %u result(s)", len);

	/* don't put the slow blink is a device reconnected while scanning */
	if (blue_pointers == 0)
		xTaskNotify(t_blue, LED_SLOW, eSetValueWithOverwrite);

	if (len) {
		esp_hid_scan_result_t *r = results;

		/*
		 * as of v1.4.5, esp_hid_gap will print detected devices in console (handle_bt_device_result())
		 * just look for bluetooth classic devices that may be supported
		 */
		while (r) {
			if (r->transport == ESP_HID_TRANSPORT_BT &&
			    strcmp("PERIPHERAL", esp_hid_cod_major_str(r->bt.cod.major)) == 0) {

				/* look for something that look like a mouse… */
				if (r->bt.cod.minor & ESP_HID_COD_MIN_MOUSE)
					mouse = r;

				/* … or a wiimote (COD minor 1 is JOYSTICK) */
				if (r->bt.cod.minor == 1 && wii_is_nintendo_bda(r->bda))
					wii = r;
			}
			r = r->next;
		}
	}

	/* try to connect to the last candidate found */
	if (mouse)
#if CONFIG_BT_BLE_ENABLED
		esp_hidh_dev_open(mouse->bda, mouse->transport, mouse->ble.addr_type);
#else
		esp_hidh_dev_open(mouse->bda, mouse->transport, 0);
#endif
	if (wii)
		esp_hidh_dev_open(wii->bda, wii->transport, 0);

	if (len && (!mouse && !wii)) {
		ESP_LOGI(TAG, "%i devices found, but no mouse or wiimote detected", len);
		esp_hid_scan_results_free(results);
	}
	if (len == 0) {
		ESP_LOGI(TAG, "no new mouse or wiimote detected");
	}

	vTaskDelete(NULL);
}

void blue_set_boot_protocol(esp_hidh_dev_t *dev) {
	configASSERT(dev != NULL);

	ESP_LOGI(TAG, "switching " ESP_BD_ADDR_STR " to protocol mode boot", ESP_BD_ADDR_HEX(esp_hidh_dev_bda_get(dev)));
	esp_hidh_dev_set_protocol(dev, ESP_HID_PROTOCOL_MODE_BOOT);
}

static bool blue_support_boot(esp_hidh_dev_t *dev) {
	esp_hid_report_item_t	mib;

	configASSERT(dev != NULL);
	mib = blue_h_ri_find(dev, ESP_HID_USAGE_MOUSE, ESP_HID_REPORT_TYPE_INPUT, ESP_HID_PROTOCOL_MODE_BOOT);
	if (mib.value_len == 0)
		return false;
	return true;
}

