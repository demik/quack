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

#define	BTMOUSE_BUTTON1	(1 << 0)
#define	BTMOUSE_BUTTON2	(1 << 1)
#define	BTMOUSE_BUTTON3	(1 << 2)
#define BTMOUSE_BUTTONE 248

/* debug tag */
static const char *TAG = "blue";

/* private functions */
static esp_hid_raw_report_map_t	*blue_hid_rm_get(esp_hidh_dev_t *dev);
static esp_hid_report_item_t	blue_hid_ri_find(esp_hidh_dev_t *d, esp_hid_usage_t u, uint8_t t, uint8_t p);

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

	char	click;
	short	x, y;

	switch (event) {
        case ESP_HIDH_OPEN_EVENT: {
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
						   ESP_LOGD(TAG, ESP_BD_ADDR_STR " INPUT: %8s, MAP: %2u, ID: %3u, Len: %d, Data:", ESP_BD_ADDR_HEX(bda), esp_hid_usage_str(param->input.usage), param->input.map_index, param->input.report_id, param->input.length);
						   ESP_LOG_BUFFER_HEX(TAG, param->input.data, param->input.length);
                           xTaskNotify(t_yellow, LED_ONCE, eSetValueWithOverwrite);
						   memcpy (&click, param->input.data, sizeof(uint8_t));
						   click = click & (BTMOUSE_BUTTON1 | BTMOUSE_BUTTON2 | BTMOUSE_BUTTON3);
						   if (click)
							   ESP_LOGI(TAG, "CLICK: %d", click);
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

void blue_init(void)
{
	esp_err_t	ret;

	ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}

	ESP_LOGD(TAG, "Starting Bluetooth init");
	ESP_ERROR_CHECK(ret);
	ESP_ERROR_CHECK(esp_hid_gap_init(ESP_BT_MODE_BTDM));
	ESP_ERROR_CHECK(esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler));

	esp_hidh_config_t config = {
		.callback = hidh_callback,
		.event_stack_size = 4096
	};

	ESP_ERROR_CHECK(esp_hidh_init(&config));
	esp_log_level_set("event", ESP_LOG_INFO);

    /*
     * at this point, everything but bluetooth is started.
     * put green steady, start blinking blue and keep scanning until a device is found
     */

    xTaskNotify(t_green, LED_ON, eSetValueWithOverwrite);
    xTaskNotify(t_blue, LED_FAST, eSetValueWithOverwrite);
	xTaskCreatePinnedToCore(blue_scan, "blue_scan", 6 * 1024, NULL, 2, NULL, 0);
}

void blue_close(esp_hidh_event_data_t *p) {
	const uint8_t *bda = NULL;

	configASSERT(p != NULL);
	bda = esp_hidh_dev_bda_get(p->close.dev);
	ESP_LOGI(TAG, "closed connection with device " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(bda));
	esp_hidh_dev_free(p->close.dev);

	xTaskNotify(t_blue, LED_SLOW, eSetValueWithOverwrite);
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
