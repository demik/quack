/*
 *  m4848.h
 *  quack
 *
 *  Created by Michel DEPEIGE on 18/05/2021.
 *  Copyright (c) 2021 Michel DEPEIGE.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program (see the file COPYING); if not, write to the
 * Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA
 *
 */

#ifndef M4848_H
#define M4848_H

/*
 * This is dumped from an Apple USB "Hockey Puck" Mouse
 *
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
		.data = m4848_HIDDD,
		.len = sizeof(m4848_HIDDD)
	}
};

static esp_hid_device_config_t m4848_config = {
	.vendor_id          = 0x05ac,	// Apple, Inc.
	.product_id         = 0x0301,	// USB Mouse [Mitsumi, M4848]
	.version            = 0x0104,
	.device_name        = "Quack Mouse Adapter",
	.manufacturer_name  = "68kmla",
	.serial_number      = "0",
	.report_maps        = m4848_report_maps,
	.report_maps_len    = 1
};

#endif
