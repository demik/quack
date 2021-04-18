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

    /*
     * at this point, everything but bluetooth is started.
     * put green steady, start blinking blue and keep scanning until a device is found
     */
    
    xTaskNotify(t_green, LED_ON, eSetValueWithOverwrite);
    xTaskNotify(t_blue, LED_SLOW, eSetValueWithOverwrite);
	xTaskCreatePinnedToCore(blue_scan, "blue_scan", 6 * 1024, NULL, 2, NULL, 0);
}

void blue_close(esp_hidh_event_data_t *p) {
	const uint8_t *bda = NULL;

	configASSERT(p != NULL);
	bda = esp_hidh_dev_bda_get(p->close.dev);
	ESP_LOGI(TAG, "closed connection with device " ESP_BD_ADDR_STR ", reason: %s", ESP_BD_ADDR_HEX(bda),
				 esp_hid_disconnect_reason_str(esp_hidh_dev_transport_get(p->close.dev), p->close.reason));
	esp_hidh_dev_free(p->close.dev);
	xTaskNotify(t_blue, LED_SLOW, eSetValueWithOverwrite);
}

void blue_open(esp_hidh_event_data_t *p) {
    const uint8_t *bda = NULL;

	configASSERT(p != NULL);
    bda = esp_hidh_dev_bda_get(p->open.dev);
    ESP_LOGI(TAG, "opened connection with " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(bda));

    //fprintf(fp, "BDA:" ESP_BD_ADDR_STR ", Status: %s, Connected: %s, Handle: %d, Usage: %s\n", ESP_BD_ADDR_HEX(dev->bda), s_bta_hh_status_names[dev->status], dev->connected ? "YES" : "NO", dev->bt.handle, esp_hid_usage_str(dev->usage));

    //ESP_LOGI(TAG, "opened connection with "
    // ESP_LOGI(TAG, "PID: 0x%04x, VID: 0x%04x, VERSION: 0x%04x\n", p->open.dev->config.product_id, p->open.dev->config.vendor_id, p->open.dev->config.version);
    //fprintf(fp, "Report Map Length: %d\n", dev->config.report_maps[0].len);

    //esp_hidh_dev_report_t *report = dev->reports;
    //while (report) {
    //    fprintf(fp, "  %8s %7s %6s, ID: %3u, Length: %3u\n",
    //           esp_hid_usage_str(report->usage), esp_hid_report_type_str(report->report_type), esp_hid_protocol_mode_str(report->protocol_mode),
    //           report->report_id, report->value_len);
    //    report = report->next;
    //}

    // (p->open.dev, stdout);
    xTaskNotify(t_blue, LED_ON, eSetValueWithOverwrite);
    gpio_output_enable();
}

void blue_scan(void *pvParameters) {
    size_t len = 0;
    esp_hid_scan_result_t *mouse = NULL;
    esp_hid_scan_result_t *results = NULL;

    ESP_LOGI(TAG, "starting scan on core %d…", xPortGetCoreID());
    esp_hid_scan(BLUE_SCAN_DURATION, &len, &results);
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

        // try to connect to the last mouse found
        if (mouse)
            esp_hidh_dev_open(mouse->bda, mouse->transport, mouse->ble.addr_type);
        else
            ESP_LOGI(TAG, "devices found but no mouse detected");

        esp_hid_scan_results_free(results);
    }

    vTaskDelete(NULL);
}
