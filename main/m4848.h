/*
 *  m4848.h
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

#ifndef M4848_H
#define M4848_H

/*
 * Dunno what this is for... The ESP-IDF stack freeze without it.
 * See similar issue at: https://www.esp32.com/viewtopic.php?t=22661&p=81652
 */

const unsigned char m4848_APIRM[] = {
	0x06, 0x00, 0xFF,  // Usage Page (Vendor Defined 0xFF00)
	0x0A, 0x00, 0x01,  // Usage (0x0100)
	0xA1, 0x01,        // Collection (Application)
	0x85, 0x01,        //   Report ID (1)
	0x15, 0x00,        //   Logical Minimum (0)
	0x26, 0xFF, 0x00,  //   Logical Maximum (255)
	0x75, 0x08,        //   Report Size (8)
	0x95, 0x08,        //   Report Count (8)
	0x09, 0x01,        //   Usage (0x01)
	0x82, 0x02, 0x01,  //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Buffered Bytes)
	0x95, 0x08,        //   Report Count (8)
	0x09, 0x02,        //   Usage (0x02)
	0xB2, 0x02, 0x01,  //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile,Buffered Bytes)
	0x95, 0x08,        //   Report Count (8)
	0x09, 0x03,        //   Usage (0x03)
	0x91, 0x02,        //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
	0xC0,              // End Collection

	// 38 bytes
};

/*
 * This is dumped from an Apple USB "Hockey Puck" Mouse
 * https://en.wikipedia.org/wiki/Apple_USB_Mouse
 * We will use this HID Report Descriptor as it's the closer to an ADB Mouse
 */

const unsigned char m4848_HIDDD[] = {
	0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
	0x09, 0x02,        // Usage (Mouse)
	0xA1, 0x01,        // Collection (Application)
	0x05, 0x09,        //   Usage Page (Button)
	0x19, 0x01,        //   Usage Minimum (0x01)
	0x29, 0x01,        //   Usage Maximum (0x01)
	0x15, 0x00,        //   Logical Minimum (0)
	0x25, 0x01,        //   Logical Maximum (1)
	0x95, 0x01,        //   Report Count (1)
	0x75, 0x01,        //   Report Size (1)
	0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
	0x95, 0x01,        //   Report Count (1)
	0x75, 0x07,        //   Report Size (7)
	0x81, 0x03,        //   Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
	0x05, 0x01,        //   Usage Page (Generic Desktop Ctrls)
	0x09, 0x01,        //   Usage (Pointer)
	0xA1, 0x00,        //   Collection (Physical)
	0x09, 0x30,        //     Usage (X)
	0x09, 0x31,        //     Usage (Y)
	0x15, 0x81,        //     Logical Minimum (-127)
	0x25, 0x7F,        //     Logical Maximum (127)
	0x75, 0x08,        //     Report Size (8)
	0x95, 0x02,        //     Report Count (2)
	0x81, 0x06,        //     Input (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null Position)
	0xC0,              //   End Collection
	0xC0,              // End Collection

	// 50 bytes
};

static esp_hid_raw_report_map_t m4848_report_maps[] = {
	{
		.data = m4848_APIRM,
		.len = sizeof(m4848_APIRM)
	},
	{
		.data = m4848_HIDDD,
		.len = sizeof(m4848_HIDDD)
	}
};

static esp_hid_device_config_t m4848_config = {
	.vendor_id          = 0x05ac,	// Apple, Inc.
	.product_id         = 0x0301,	// USB Mouse [Mitsumi, M4848]
	.version            = 0x0104,
	.device_name        = "Quack ADB Mouse",
	.manufacturer_name  = "68kmla",
	.serial_number      = "0",
	.report_maps        = m4848_report_maps,
	.report_maps_len    = 2
};

/* precalculed table for 0-64 values to 0-127 with a slight acceleration of x^1.2 */
const unsigned char m4848_accel[] = {
	0x00, 0x01, 0x02, 0x03, 0x05, 0x06, 0x08, 0x0A,
	0x0C, 0x0D, 0x0F, 0x11, 0x13, 0x15, 0x17, 0x19,
	0x1B, 0x1D, 0x20, 0x22, 0x24, 0x26, 0x28, 0x2B,
	0x2D, 0x2F, 0x31, 0x34, 0x36, 0x38, 0x3B, 0x3D,
	0x3F, 0x42, 0x44, 0x47, 0x49, 0x4C, 0x4E, 0x51,
	0x53, 0x56, 0x58, 0x5B, 0x5D, 0x60, 0x62, 0x65,
	0x68, 0x6A, 0x6D, 0x6F, 0x72, 0x75, 0x77, 0x7A,
	0x7D, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F
};

#endif
