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

static const char *TAG = "blue";

#define	BTMOUSE_BUTTON1	(1 << 0)
#define	BTMOUSE_BUTTON2	(1 << 1)
#define	BTMOUSE_BUTTON3	(1 << 2)

void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
	esp_hidh_event_t event = (esp_hidh_event_t)id;
	esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;

	/*
	 * esp_hidh_event_data_t: 
	 * struct {
	 * esp_hidh_dev_t *dev;		HID Remote bluetooth device
	 * esp_hid_usage_t usage;	HID report usage
	 * uint16_t report_id;		HID report index
	 * uint16_t length;		HID data length
	 * uint8_t  *data;		The pointer to the HID data
	 * uint8_t map_index;		HID report map index
	 * } input;
	 */

	char	click;
	short	x, y;

	switch (event) {
		case ESP_HIDH_OPEN_EVENT: {
			const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
			if (strlen((char *)param->open.dev) > 0)
				ESP_LOGI(TAG, "opened connection with device: " ESP_BD_ADDR_STR " named %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->open.dev));
			else
				ESP_LOGI(TAG, "opened connection with device: " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(bda));
			//esp_hidh_dev_dump(param->open.dev, stdout);
			break;
		}
		case ESP_HIDH_BATTERY_EVENT: {
			const uint8_t *bda = esp_hidh_dev_bda_get(param->battery.dev);
			ESP_LOGI(TAG, ESP_BD_ADDR_STR " BATTERY: %d%%", ESP_BD_ADDR_HEX(bda), param->battery.level);
			break;
		}
		case ESP_HIDH_INPUT_EVENT: {
						   const uint8_t *bda = esp_hidh_dev_bda_get(param->input.dev);
						   ESP_LOGD(TAG, ESP_BD_ADDR_STR " INPUT: %8s, MAP: %2u, ID: %3u, Len: %d, Data:", ESP_BD_ADDR_HEX(bda), esp_hid_usage_str(param->input.usage), param->input.map_index, param->input.report_id, param->input.length);
						   ESP_LOG_BUFFER_HEX(TAG, param->input.data, param->input.length);
						   memcpy (&click, param->input.data, sizeof(uint8_t));
						   click = click & (BTMOUSE_BUTTON1 | BTMOUSE_BUTTON2 | BTMOUSE_BUTTON3);
						   if (click)
							   ESP_LOGI(TAG, "CLICK: %d", click);
							   break;
					   }
		case ESP_HIDH_FEATURE_EVENT: {
						     const uint8_t *bda = esp_hidh_dev_bda_get(param->feature.dev);
						     ESP_LOGI(TAG, ESP_BD_ADDR_STR " FEATURE: %8s, MAP: %2u, ID: %3u, Len: %d", ESP_BD_ADDR_HEX(bda), esp_hid_usage_str(param->feature.usage), param->feature.map_index, param->feature.report_id, param->feature.length);
						     ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
						     break;
					     }
		case ESP_HIDH_CLOSE_EVENT: {
			const uint8_t *bda = esp_hidh_dev_bda_get(param->close.dev);
			ESP_LOGI(TAG, ESP_BD_ADDR_STR " CLOSE: '%s' %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->close.dev), esp_hid_disconnect_reason_str(esp_hidh_dev_transport_get(param->close.dev), param->close.reason));
			esp_hidh_dev_free(param->close.dev);
			break;
		}
		default:
			ESP_LOGI(TAG, "EVENT: %d", event);
	}
}

#define SCAN_DURATION_SECONDS 6

void hid_demo_task(void *pvParameters)
{
	size_t len = 0;
    esp_hid_scan_result_t *mouse = NULL;
	esp_hid_scan_result_t *results = NULL;

	ESP_LOGI(TAG, "starting scan on core %d…", xPortGetCoreID());
	esp_hid_scan(SCAN_DURATION_SECONDS, &len, &results);
	ESP_LOGI(TAG, "scan returned %u result(s)", len);

	if (len) {
		esp_hid_scan_result_t *r = results;
		while (r) {
            ESP_LOGI(TAG, "found %s device: " ESP_BD_ADDR_STR ", RSSI: %d, NAME: %s",
                 (r->transport == ESP_HID_TRANSPORT_BLE) ? "BLE" : "BT",
                 ESP_BD_ADDR_HEX(r->bda), r->rssi, r->name ? r->name : "");
        
			if (r->transport == ESP_HID_TRANSPORT_BLE) {
				printf("APPEARANCE: 0x%04x, ", r->ble.appearance);
				printf("ADDR_TYPE: '%s', ", ble_addr_type_str(r->ble.addr_type));
			} else {
                if (strcmp("PERIPHERAL", esp_hid_cod_major_str(r->bt.cod.major)) == 0
                    && (r->bt.cod.minor & ESP_HID_COD_MIN_MOUSE)) {
                    ESP_LOGI(TAG, "found generic mouse");
                    mouse = r;
                }
			}
			r = r->next;
		}
		if (mouse) {
            // try to connect to the last mouse found
            esp_hidh_dev_open(mouse->bda, mouse->transport, mouse->ble.addr_type);
		}
        else {
            ESP_LOGI(TAG, "devices found but no mouse detected");
        }

		esp_hid_scan_results_free(results);
	}

	vTaskDelete(NULL);
}

void blue_init(void)
{
	esp_err_t	ret;

	ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);
	ESP_ERROR_CHECK(esp_hid_gap_init(ESP_BT_MODE_BTDM));
	ESP_ERROR_CHECK(esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler));

	esp_hidh_config_t config = {
		.callback = hidh_callback,
	};

	ESP_ERROR_CHECK( esp_hidh_init(&config) );

	/* keep scanning until a device is found */
	xTaskCreatePinnedToCore(&hid_demo_task, "hid_task", 6 * 1024, NULL, 2, NULL, 0);
}
