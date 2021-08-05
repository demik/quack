/* This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this software is
   distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"

#include "esp_hidh.h"
#include "esp_hid_gap.h"

#include "blue.h"
#include "gpio.h"
#include "led.h"
#include "m4848.h"

/* debug tag */
static const char *TAG = "blue";

/* globals */
extern TaskHandle_t t_click, t_qx, t_qy;

/* private functions */
static void	blue_handle_button(uint8_t buttons);
static esp_hid_raw_report_map_t	*blue_hid_rm_get(esp_hidh_dev_t *dev);
static esp_hid_report_item_t	blue_hid_ri_find(esp_hidh_dev_t *d, esp_hid_usage_t u, uint8_t t, uint8_t p);
static void blue_set_boot_protocol(esp_hidh_dev_t *dev);
static bool blue_support_boot_protocol(esp_hidh_dev_t *dev);

/* direct calls to bluedroid */
extern void BTA_HhSetProtoMode(uint8_t handle, uint8_t t_type);

void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
	esp_hidh_event_t event = (esp_hidh_event_t)id;
	esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;

    const uint8_t *bda = NULL;

	/*
     * esp_hidh_event_data_t = union
     *
     * struct {
     *     esp_hidh_dev_t *dev;     // HID Remote bluetooth device
     * } open;
     *
     * struct {
     *     esp_hidh_dev_t *dev;     // HID Remote bluetooth device
     *     int reason;              // Reason why the connection was closed. BLE Only
     * } close;
     *
     * struct {
     *     esp_hidh_dev_t *dev;     // HID Remote bluetooth device
     *     uint8_t level;           // Battery Level (0-100%)
     * } battery;
     *
     * struct {
     *     esp_hidh_dev_t *dev;     // HID Remote bluetooth device
     *     esp_hid_usage_t usage;   // HID report usage
     *     uint16_t report_id;      // HID report index
     *     uint16_t length;         // HID data length
     *     uint8_t  *data;          // The pointer to the HID data
     *     uint8_t map_index;       // HID report map index
     * } input;
     *
     * struct {
     *     esp_hidh_dev_t *dev;     // HID Remote bluetooth device
     *     esp_hid_usage_t usage;   // HID report usage
     *     uint16_t report_id;      // HID report index
     *     uint16_t length;         // HID data length
     *     uint8_t  *data;          // The pointer to the HID data
     *     uint8_t map_index;       // HID report map index
     * } feature;
     */

	switch (event) {
        case ESP_HIDH_OPEN_EVENT: {
			esp_hidh_dev_dump(param->open.dev, stdout);
            blue_open(param);
			break;
		}
		case ESP_HIDH_BATTERY_EVENT: {
			bda = esp_hidh_dev_bda_get(param->battery.dev);
			ESP_LOGI(TAG, ESP_BD_ADDR_STR " battery: %d%%", ESP_BD_ADDR_HEX(bda), param->battery.level);
			break;
		}
		case ESP_HIDH_INPUT_EVENT: {
            bda = esp_hidh_dev_bda_get(param->input.dev);
						   //ESP_LOGD(TAG, ESP_BD_ADDR_STR " INPUT: %8s, MAP: %2u, ID: %3u, Len: %d, Data:", ESP_BD_ADDR_HEX(bda), esp_hid_usage_str(param->input.usage), param->input.map_index, param->input.report_id, param->input.length);
						   //ESP_LOG_BUFFER_HEX(TAG, param->input.data, param->input.length);
                           xTaskNotify(t_yellow, LED_ONCE, eSetValueWithOverwrite);
			blue_input(param->input.dev, param->input.data, param->input.length);
			break;
					   }
		case ESP_HIDH_FEATURE_EVENT: {
            bda = esp_hidh_dev_bda_get(param->feature.dev);
						     ESP_LOGI(TAG, ESP_BD_ADDR_STR " FEATURE: %8s, MAP: %2u, ID: %3u, Len: %d", ESP_BD_ADDR_HEX(bda), esp_hid_usage_str(param->feature.usage), param->feature.map_index, param->feature.report_id, param->feature.length);
						     ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
						     break;
					     }
		case ESP_HIDH_CLOSE_EVENT: {
			blue_close(param);
			break;
		}
		default:
			ESP_LOGI(TAG, "Unknwown event: %d", event);
	}
}

void blue_close(esp_hidh_event_data_t *p) {
	const uint8_t *bda = NULL;

	configASSERT(p != NULL);
	bda = esp_hidh_dev_bda_get(p->close.dev);
	ESP_LOGI(TAG, "closed connection with device " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(bda));
	esp_hidh_dev_free(p->close.dev);

	xTaskNotify(t_blue, LED_SLOW, eSetValueWithOverwrite);
}

static void	blue_handle_button(uint8_t buttons) {
	static bool	status = BLUE_BUTTON_N;	// Keep state

	buttons = buttons & (BLUE_BUTTON_1 | BLUE_BUTTON_2 | BLUE_BUTTON_3);

	if (status && buttons)
		return ;
	if (status == BLUE_BUTTON_N && buttons == BLUE_BUTTON_N)
		return ;

	/* release button */
	if (status && buttons == BLUE_BUTTON_N) {
		xTaskNotify(t_click, 0, eSetValueWithOverwrite);
		status = 0;
		ESP_LOGD(TAG, "button released");
		return ;
	}

	/* press button */
	if (status == BLUE_BUTTON_N && buttons) {
		xTaskNotify(t_click, 1, eSetValueWithOverwrite);
		status = 1;
		ESP_LOGD(TAG, "button pressed");
		return ;
	}

}

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
	ESP_ERROR_CHECK(esp_hid_gap_init(ESP_BT_MODE_BTDM));
	ESP_ERROR_CHECK(esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler));

	esp_hidh_config_t config = {
		.callback = hidh_callback,
		.event_stack_size = 4096
	};

	ESP_ERROR_CHECK(esp_hidh_init(&config));
	esp_log_level_set("event", ESP_LOG_INFO);

	/* complains about wrong data len on BOOT mode and CCONTROL */
	esp_log_level_set("BT_HIDH", ESP_LOG_ERROR);

    /*
     * at this point, everything but bluetooth is started.
     * put green steady, start blinking blue and keep scanning until a device is found
     */

    xTaskNotify(t_green, LED_ON, eSetValueWithOverwrite);
    xTaskNotify(t_blue, LED_FAST, eSetValueWithOverwrite);
	xTaskCreatePinnedToCore(blue_scan, "blue_scan", 6 * 1024, NULL, 2, NULL, 0);
}

/*
 * data follows HID Mouse boot protocol format
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

void blue_input(esp_hidh_dev_t *dev, uint8_t *data, uint16_t length) {
	uint8_t buttons;
	uint8_t	i;
	int8_t x, y;

	buttons = data[0];

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

	/* quadrature use 7 bits x/y offsets, bluetooth Boot use 8 bits */
	x = data[1];
	y = data[2];

	if (x != 0) {
		if (x == 1 || x == -1)
			xTaskNotify(t_qx, x, eSetValueWithOverwrite);
		else
			xTaskNotify(t_qx, x / 2, eSetValueWithOverwrite);
	}
	if (y != 0) {
		if (y == 1 || y == -1)
			xTaskNotify(t_qy, y, eSetValueWithOverwrite);
		else
			xTaskNotify(t_qy, y / 2, eSetValueWithOverwrite);
	}
}

void blue_open(esp_hidh_event_data_t *p) {
    const uint8_t *bda = NULL;

	esp_hid_report_item_t	mir;	/* MOUSE INPUT REPORT */

	configASSERT(p != NULL);
    bda = esp_hidh_dev_bda_get(p->open.dev);
    ESP_LOGI(TAG, "opened connection with " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(bda));

	/* search for MIR section */
	esp_hid_raw_report_map_t *maps;
	maps = blue_hid_rm_get(p->open.dev);

	blue_set_boot_protocol(p->open.dev);
	mir = blue_hid_ri_find(p->open.dev, ESP_HID_USAGE_MOUSE, ESP_HID_REPORT_TYPE_INPUT, ESP_HID_PROTOCOL_MODE_REPORT);

    xTaskNotify(t_blue, LED_ON, eSetValueWithOverwrite);
    gpio_output_enable();
}

/* get specific report from report map matching specified usage + type + protocol */
static esp_hid_report_item_t blue_hid_ri_find(esp_hidh_dev_t *d, esp_hid_usage_t u, uint8_t t, uint8_t p) {
	size_t num_reports = 0;
	esp_hid_report_item_t *reports;
	esp_hid_report_item_t search;

	configASSERT(d != NULL);
	configASSERT(p <= ESP_HID_REPORT_TYPE_FEATURE);
	configASSERT(t > 0);

	esp_hidh_dev_reports_get(d, &num_reports, &reports);
	memset(&search, 0, sizeof(esp_hid_report_item_t));

	for (uint8_t i = 0; i < num_reports; i++) {
		if (reports[i].protocol_mode == p && reports[i].report_type == t &&	reports[i].usage == u) {
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

    ESP_LOGI(TAG, "starting scan on core %d…", xPortGetCoreID());
    esp_hid_scan(BLUE_SCAN_DURATION, &len, &results);
    ESP_LOGI(TAG, "scan returned %u result(s)", len);
	xTaskNotify(t_blue, LED_SLOW, eSetValueWithOverwrite);

    if (len) {
        esp_hid_scan_result_t *r = results;

        while (r) {
            ESP_LOGI(TAG, "found %s %s device: " ESP_BD_ADDR_STR ", RSSI: %d, NAME: %s",
					 (r->transport == ESP_HID_TRANSPORT_BLE) ? "BLE" : "BT",
					 esp_hid_cod_major_str(r->bt.cod.major),
					 ESP_BD_ADDR_HEX(r->bda), r->rssi, r->name ? r->name : "");

			/* search for something that looks like Bluetooth Classic mouse */
            if (r->transport == ESP_HID_TRANSPORT_BT &&
				strcmp("PERIPHERAL", esp_hid_cod_major_str(r->bt.cod.major)) == 0
				&& (r->bt.cod.minor & ESP_HID_COD_MIN_MOUSE)) {
                    mouse = r;
            }
            r = r->next;
        }

        /* try to connect to the last candidate found */
        if (mouse)
            esp_hidh_dev_open(mouse->bda, mouse->transport, mouse->ble.addr_type);
        else
            ESP_LOGI(TAG, "devices found but no mouse detected");

        esp_hid_scan_results_free(results);
    }

    vTaskDelete(NULL);
}

static void blue_set_boot_protocol(esp_hidh_dev_t *dev) {
	configASSERT(dev != NULL);

	/*
	 * /!\ Disclaimer /!\
	 * This is ugly. We are accessing directly bluedroid and need the hidden handle to do that
	 * Extract it from a private esp_hidh_dev_s struct and call BTA_HhSetProtoMode directly.
	 */

	struct decoy_dev_s {
		struct esp_hidh_dev_s   *next;

		esp_hid_device_config_t config;
		esp_hid_usage_t         usage;
		esp_hid_transport_t     transport;
		bool                    connected;
		bool                    opened;
		int                     status;

		size_t                  reports_len;
		void                    *reports;

		void                    *tmp;
		size_t                  tmp_len;

		xSemaphoreHandle        semaphore;

		esp_err_t               (*close)        (esp_hidh_dev_t *dev);
		esp_err_t               (*report_write) (esp_hidh_dev_t *dev, size_t map_index, size_t report_id, int report_type, uint8_t *data, size_t len);
		esp_err_t               (*report_read)  (esp_hidh_dev_t *dev, size_t map_index, size_t report_id, int report_type, size_t max_length, uint8_t *value, size_t *value_len);
		void                    (*dump)         (esp_hidh_dev_t *dev, FILE *fp);

		uint8_t padding;		// for some reason, bda is off by one byte...
		esp_bd_addr_t bda;

		struct {
			esp_bt_cod_t cod;
			int handle;
			uint8_t sub_class;
			uint8_t app_id;
			uint16_t attr_mask;
		} bt;
		TAILQ_ENTRY(esp_hidh_dev_s) devices;
	};

	struct decoy_dev_s *pass_that_handle;
	pass_that_handle = (struct decoy_dev_s *)dev;

	ESP_LOGI(TAG, "switching " ESP_BD_ADDR_STR " (%i) to protocol mode boot" ,
			 pass_that_handle->bt.handle, ESP_BD_ADDR_HEX(pass_that_handle->bda));

	//ESP_LOG_BUFFER_HEX(TAG, dev, sizeof(struct decoy));
	/* bluedroid/bta/include/bta_hh_api.h */
	BTA_HhSetProtoMode(pass_that_handle->bt.handle, 0x01);
}

static bool blue_support_boot_protocol(esp_hidh_dev_t *dev) {
	configASSERT(dev != NULL);
}

