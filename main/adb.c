/*
 *  adb.c
 *  quack
 *
 *  Created by Michel DEPEIGE on 7/01/2020.
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
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_private/periph_ctrl.h"
#include "driver/rmt.h"
#include "soc/periph_defs.h"
#include "soc/rmt_reg.h"

#include "adb.h"
#include "led.h"
#include "gpio.h"

/* defines */
#define TAG "ADB"

/* globals */
rmt_config_t adb_rmt_rx = RMT_DEFAULT_CONFIG_RX(GPIO_ADB, RMT_RX_CHANNEL);
extern TaskHandle_t t_adb2hid;
extern TaskHandle_t t_green, t_blue, t_yellow, t_red;
extern TaskHandle_t t_click;
extern QueueHandle_t q_qx, q_qy;

/* static defines */
static void	adb_handle_button(bool action);
static void	adb_rmt_reset(void);
static bool	adb_rx_isone(rmt_item32_t cell);
static bool	adb_rx_isstop(rmt_item32_t cell);
static bool	adb_rx_iszero(rmt_item32_t cell);
static uint16_t	IRAM_ATTR adb_rx_mouse();
static void	adb_rx_setup(void);
static void	adb_scan_macaj(void);
static void	adb_tx_as(void);
static void	adb_tx_one(void);
static void	adb_tx_setup(void);
static void	adb_tx_stop(void);
static void	adb_tx_zero(void);

/* functions */
static void	adb_handle_button(bool action) {
	static bool	status = ADB_B_UP;	// Keep state. Click is active low

	if (status == ADB_B_UP && action == ADB_B_UP)
		return ;
	if (status == ADB_B_DOWN && action == ADB_B_DOWN)
		return ;

	/* release button */
	if (status == ADB_B_DOWN && action == ADB_B_UP) {
		xTaskNotify(t_click, 0, eSetValueWithOverwrite);
		status = ADB_B_UP;
		ESP_LOGD(TAG, "button released");
		return ;
	}

	/* press button */
	if (status == ADB_B_UP && action == ADB_B_DOWN) {
		xTaskNotify(t_click, 1, eSetValueWithOverwrite);
		status = ADB_B_DOWN;
		ESP_LOGD(TAG, "button pressed");
		return ;
	}
}

void	adb_init(void) {
	/* initialise */
	gpio_set_level(GPIO_ADB, 1);
	adb_tx_reset();

	/* avoid console flood when installing/uninstalling RMT driver */
	esp_log_level_set("intr_alloc", ESP_LOG_INFO);

	/* init RMT RX driver with default values for ADB  */
	adb_rmt_rx.mem_block_num = 4;
	adb_rmt_rx.rx_config.filter_en = true;
	adb_rmt_rx.rx_config.filter_ticks_thresh = 10;
	adb_rmt_rx.rx_config.idle_threshold = 100;
	rmt_config(&adb_rmt_rx);

	/* If jumper is set, switch to ADB host mode */
	if (adb_is_host())
		xTaskCreatePinnedToCore(adb_task_host, "ADB_HOST", 6 * 1024, NULL, tskADB_PRIORITY, NULL, 1);
	else
		xTaskCreatePinnedToCore(adb_task_idle, "ADB_IDLE", 6 * 1024, NULL, tskADB_PRIORITY, NULL, 1);
}

inline bool	adb_is_host(void) {
	if (gpio_get_level(GPIO_ADBSRC) == 0)
		return true;
	else
		return false;
}

/*
 * Probe ADB mouse. Also switch the mouse to a better mode if avaible
 *
 * 1. check if there is someting at address $3, if not: retry
 * 2. try to switch to CLASSIC2 protocol at temporary address $9
 * 3. if CLASSIC2 is successfull, move device back to $3
 */

void adb_probe(void) {
	uint16_t	register3;

	ESP_LOGI(TAG, "Probing for mouse…");
	xTaskNotify(t_yellow, LED_SLOW, eSetValueWithOverwrite);

	/* for some reason, RMT misses the first exchange sometimes. Flush the device should give it time */
	adb_tx_cmd(ADB_MOUSE|ADB_FLUSH);
	adb_rx_mouse();
	vTaskDelay(12 / portTICK_PERIOD_MS);

	while (true) {

		adb_tx_cmd(ADB_MOUSE|ADB_TALK|ADB_REG3);
		register3 = adb_rx_mouse();
		ESP_LOGD("ADB", "Device $3 register3: %04x", register3);

		if (register3 && (register3 & ADB_H_ALL) == ADB_H_ERR)
			ESP_LOGE(TAG, "Mouse failed self init test");

		/*
		 * try to unglue composite devices (Kensington)
		 * the idea is to move detected devices to $9, and check $3 again
		 * if there is something again at $3, it may be a composite device, or the user
		 * plugged two devices.
		 * if there is nothing anymore, move back $9 to $3
		 *
		 * we will handle that further down the line by checking the handler
		 */
		if (register3 & ADB_H_ALL) {
			ESP_LOGI(TAG, "\tMoving $3 to $9 to check for composite devices");
			adb_tx_listen(ADB_MOUSE|ADB_LISTEN|ADB_REG3, (ADB_TMP<<4)|ADB_H_MOVE);
			vTaskDelay(7 / portTICK_PERIOD_MS);
			ESP_LOGI(TAG, "\tChecking $3 again…");
			adb_tx_cmd(ADB_MOUSE|ADB_TALK|ADB_REG3);
			register3 = adb_rx_mouse();
			vTaskDelay(7 / portTICK_PERIOD_MS);
			if (register3) {
				adb_tx_cmd(ADB_TMP|ADB_TALK|ADB_REG3);
				register3 = adb_rx_mouse();
				ESP_LOGI(TAG, "\t… something was there, $9 handler: $%02x", register3 & ADB_H_ALL);
				xTaskNotify(t_red, LED_ONCE, eSetValueWithOverwrite);
			}
			else {
				ESP_LOGI(TAG, "\t… nothing, moving $9 back to $3");
				adb_tx_listen(ADB_TMP|ADB_LISTEN|ADB_REG3, (ADB_MOUSE<<4)|ADB_H_MOVE);
			}

			/* restore register3 from $3 for handler detection */
			vTaskDelay(7 / portTICK_PERIOD_MS);
			adb_tx_cmd(ADB_MOUSE|ADB_TALK|ADB_REG3);
			register3 = adb_rx_mouse();
		}

		/* Accept all known handlers */
		if (((register3 & ADB_H_ALL) == ADB_H_C100) || ((register3 & ADB_H_ALL) == ADB_H_C200) ||
			((register3 & ADB_H_ALL) == ADB_H_MTRC) || ((register3 & ADB_H_ALL) == ADB_H_KSGT) ||
			((register3 & ADB_H_ALL) == ADB_H_MACJ)) {
			ESP_LOGI(TAG, "… detected mouse at $3");

			break;
		}

		/* try to find other misc devices */
		if (!register3)
			adb_scan_macaj();

		vTaskDelay(500 / portTICK_PERIOD_MS);
	}

	/* try to switch to 200cpi mode. (0x6000) == Exceptional event + Service request enable */
	vTaskDelay(7 / portTICK_PERIOD_MS);
	if ((register3 & ADB_H_ALL) == ADB_H_C100) {
		adb_tx_listen(ADB_MOUSE|ADB_LISTEN|ADB_REG3, 0x6000|(ADB_MOUSE<<4)|ADB_H_C200);
		vTaskDelay(7 / portTICK_PERIOD_MS);
		adb_tx_cmd(ADB_MOUSE|ADB_TALK|ADB_REG3);
		register3 = adb_rx_mouse();
	}

	switch (register3 & ADB_H_ALL) {
		case ADB_H_C100:
			ESP_LOGD(TAG, "Mouse running at 100cpi");
			break;
		case ADB_H_C200:
			ESP_LOGD(TAG, "Mouse running at 200cpi");
			break;
		case ADB_H_MACJ:
			ESP_LOGD(TAG, "MacAlly Joystick running at default cpi");
			break;
		case ADB_H_MTRC:
			ESP_LOGD(TAG, "MacTRAC running at default cpi");
			break;
		case ADB_H_KSGT:
			ESP_LOGD(TAG, "Kensington detected, switching to standard device");
			adb_tx_listen(ADB_MOUSE|ADB_LISTEN|ADB_REG3, (ADB_KML1<<4)|ADB_H_MOVE);
			adb_tx_listen(ADB_TMP|ADB_LISTEN|ADB_REG3, (ADB_MOUSE<<4)|ADB_H_MOVE);
			adb_tx_listen(ADB_MOUSE|ADB_LISTEN|ADB_REG3, 0x6000|(ADB_MOUSE<<4)|ADB_H_C200);
			ESP_LOGD(TAG, "Kensington running at 200cpi");
			break;
		default:
			ESP_LOGE(TAG, "Mouse running with unknow handler: %x", register3 & ADB_H_ALL);
	}

	xTaskNotify(t_yellow, LED_OFF, eSetValueWithOverwrite);
}

/*
 * sometimes the RMT device get stuck with a full RX buffer. On the Quack Mouse adapter,
 * it prevends reading from the ADB if the receive buffer has been filled
 *
 * Somewhat a hack, but resetting the module sometimes help.
 */

static void	adb_rmt_reset() {
	periph_module_reset(PERIPH_RMT_MODULE);
	rmt_config(&adb_rmt_rx);
	xTaskNotify(t_red, LED_ONCE, eSetValueWithOverwrite);

	gpio_set_level(GPIO_ADB, 1);
	adb_tx_reset();
}

/*
 * MacAlly Joystick uses address $5 instead of the classic $3
 * First check that it's in mouse mode emulation then switch it to $3
 * We should then be able to use the base code path
 *
 * https://github.com/lampmerchant/tashnotes/blob/main/macintosh/adb/protocols/macally_joystick.md
 */

static void	adb_scan_macaj() {
	uint16_t	register1;

	/* the joystick will not move if we don't don't talk to register 3 first */
	vTaskDelay(7 / portTICK_PERIOD_MS);
	adb_tx_cmd(ADB_MACAJ|ADB_TALK|ADB_REG3);
	adb_rx_mouse();
	vTaskDelay(7 / portTICK_PERIOD_MS);
	adb_tx_cmd(ADB_MACAJ|ADB_TALK|ADB_REG1);
	register1 = adb_rx_mouse();

	if (register1 == 0xAAAA) {
		ESP_LOGD(TAG, "MacAlly Joystick in joystick mode detected, switching modes");
		adb_tx_listen(ADB_MACAJ|ADB_LISTEN|ADB_REG1, 0xAAAA);
		vTaskDelay(7 / portTICK_PERIOD_MS);
		adb_tx_cmd(ADB_MACAJ|ADB_TALK|ADB_REG1);
		register1 = adb_rx_mouse();
	}
	if (register1 == 0x5555) {
		ESP_LOGD(TAG, "MacAlly Joystick in mouse mode detected, moving it to $3");
		adb_tx_listen(ADB_MACAJ|ADB_LISTEN|ADB_REG3, (ADB_MOUSE<<4)|ADB_H_MOVE);
	}
}

void	adb_task_host(void *pvParameters) {
	/* Classic Apple Mouse Protocol is 16 bits long */
	uint16_t	data;
	uint8_t	last = 0;
	int8_t	move = 0;
	uint8_t	state = ADB_S_PROBE;

     /*
      * wait a little for the LED tasks to start on the other core
      * starting IDF 5.2.2, we need this. Looks like the init is now too fast on app core
      */
     vTaskDelay(200 / portTICK_PERIOD_MS);

	/* put green led to steady if BT is disabled. Otherwise BT init will do it */
	if (gpio_get_level(GPIO_BTOFF) == 0)
		xTaskNotify(t_green, LED_ON, eSetValueWithOverwrite);
	ESP_LOGI(TAG, "host started on core %d", xPortGetCoreID());

	/* poll the mouse like a maniac. It will answer only if there is user input */
	ESP_ERROR_CHECK(rmt_driver_install(RMT_RX_CHANNEL, 256, 0));

	while (true) {
		/* Should give us a polling rate between 80-90 Hz */
		vTaskDelay(7 / portTICK_PERIOD_MS);

		if (state == ADB_S_PROBE) {
			adb_probe();
			state = ADM_S_POLL;
			continue;
		}

		adb_tx_cmd(ADB_MOUSE|ADB_TALK|ADB_REG0);
		data = adb_rx_mouse();

		if (data) {
			last = 0;

			/* split data for quad and (maybe) bluetooth */
			if (gpio_get_level(GPIO_BTOFF) == 1 && t_adb2hid != NULL)
				xTaskNotify(t_adb2hid, data, eSetValueWithOverwrite);

			/* click is active low */
			if (! (data & ADB_CMP_B1) || (! (data & ADB_CMP_B2)))
				adb_handle_button(ADB_B_DOWN);
			else
				adb_handle_button(ADB_B_UP);

			/* cast negative signed 7 bits to signed 8 bits */
			move = (data & ADB_CMP_MX) >> 0;
			if (move & 0x40) {
				move &= ~0x40;
				move |= 0x80;
				move += 64;
			}

			if (move)
				xQueueSendToBack(q_qx, &move, 0);

			move = (data & ADB_CMP_MY) >> 8;
			if (move & 0x40) {
				move &= ~0x40;
				move |= 0x80;
				move += 64;
			}

			if (move)
				xQueueSendToBack(q_qy, &move, 0);
		}
		else {
			last++;
			if (last == 0xff) {
				adb_tx_cmd(ADB_MOUSE|ADB_TALK|ADB_REG3);
				data = adb_rx_mouse();
				if (!data) {
					if (state == ADB_S_KEEP)
						state = ADB_S_PROBE;
					else
						state = ADB_S_KEEP;
				}
				ESP_LOGD("ADB", "Check mouse presence %04x", data);
			}
		}
	}
}

/*
 * this do nothing since device mode doesn't work
 * put level converter on input mode with a pull up to avoid oscillations
 * if plugged into an ADB Bus by mistake.
 */

void	adb_task_idle(void *pvParameters) {
	ESP_LOGI(TAG, "idle started on core %d", xPortGetCoreID());

	/* RMT is not installed via rmt_driver_install() so no need to uninstall */
	adb_rx_setup();
	ESP_ERROR_CHECK(gpio_pullup_en(GPIO_ADB));

	/*
	 * We sould exit here but for some reason core 1 panic with
	 * IllegalInstruction if we do... Dunno what's going on here either.
	 * let's suspend the task instead
	 */

	ESP_LOGI(TAG, "idle suspending itself on core %d", xPortGetCoreID());
	vTaskSuspend(NULL);
}

/*
 * some ADB mouses are using cells as short as 24µs+49µs, try to get that...
 * spec allow ±30% variance
 *
 * units are in µs
 */

static bool adb_rx_isone(rmt_item32_t cell) {
	if (cell.level0 == 0 && (cell.duration0 >= 22 && cell.duration0 <= 44) &&
		cell.level1 == 1 && (cell.duration1 >= 46 && cell.duration1 <= 86))
		return true;
	return false;
}

static bool adb_rx_isstop(rmt_item32_t cell) {
	/* high part of the well is lengh 0 because of RMT timeout (100µs+) */
	if (cell.level0 == 0 && (cell.duration0 >= 46 && cell.duration0 <= 86) &&
		cell.level1 == 1 && cell.duration1 == 0)
		return true;
	return false;
}

static bool adb_rx_iszero(rmt_item32_t cell) {
	if (cell.level0 == 0 && (cell.duration0 >= 46 && cell.duration0 <= 86) &&
		cell.level1 == 1 && (cell.duration1 >= 22 && cell.duration1 <= 44))
		return true;
	return false;
}

static uint16_t	IRAM_ATTR adb_rx_mouse() {
	uint32_t rmt_status;
	uint16_t data = 0;
	RingbufHandle_t rb = NULL;
	rmt_item32_t* items = NULL;
	size_t rx_size = 0;
	size_t i;

	rmt_get_ringbuf_handle(RMT_RX_CHANNEL, &rb);
	configASSERT(rb != NULL);
	rmt_rx_start(RMT_RX_CHANNEL, true);
	items = (rmt_item32_t*)xRingbufferReceive(rb, &rx_size, pdMS_TO_TICKS(8));
	rmt_get_status(RMT_RX_CHANNEL, &rmt_status);
	if (((rmt_status >> RMT_STATE_CH0_S) & RMT_STATE_CH0_V) == 4)
		adb_rmt_reset();
	rmt_rx_stop(RMT_RX_CHANNEL);

	if (items == NULL)
		return 0;

	/*
 	 * Mouse response size in bits is events / sizeof(rmt_item32_t) (4)
	 * start bit + 16 data bits + stop bit = 72 bytes in RMT buffer (18 bits received)
	 *
	 * Check start / stop bits and size.
	 * On real Macintoshes, ADB Manager does something similar
	 */

	switch (rx_size) {
		case 0:
			/* RMT giving back a buffer with a rx_size of 0… */
		case 4:
			/* single glitch or service request (keyboard), timeout/ignore */
			vRingbufferReturnItem(rb, (void*) items);
			return 0;
		case 72:
			xTaskNotify(t_yellow, LED_ONCE, eSetValueWithOverwrite);
			break;
		default:
			ESP_LOGD(TAG, "wrong size of %i bit(s)", rx_size / sizeof(rmt_item32_t));
			xTaskNotify(t_red, LED_ONCE, eSetValueWithOverwrite);
	}

	/* check start and stop bits */
	if (! adb_rx_isone(*(items+0)) && (! adb_rx_isstop(*(items+(rx_size / sizeof(rmt_item32_t) - 1))))) {
		xTaskNotify(t_red, LED_ONCE, eSetValueWithOverwrite);
		return 0;
	}

	/* rebuild our data with RMT buffer */
	for (i = 1; i < ((rx_size / sizeof(rmt_item32_t)) - 1); i++) {
		data <<= 1;

		/* check that every data is either one or zero */
		if ((! adb_rx_isone(*(items+i))) && (! adb_rx_iszero(*(items+i)))) {
			xTaskNotify(t_red, LED_ONCE, eSetValueWithOverwrite);
			return 0;
		}

		if (adb_rx_isone(*(items+i)))
			data |= 1;
	}
	vRingbufferReturnItem(rb, (void*) items);

	return data;
}

static inline void	adb_rx_setup() {
	gpio_set_level(GPIO_DIR, 0);
	gpio_set_direction(GPIO_ADB, GPIO_MODE_INPUT);
	gpio_pullup_dis(GPIO_ADB);
	gpio_pulldown_dis(GPIO_ADB);
}

static inline void	adb_tx_as() {
	/*
	 * send attention (800 µs low) + sync (65 µs high).
	 * AN591 mentions 70 µs which is a mistake, make some devices angry
	 */

	gpio_set_level(GPIO_ADB, 0);
	esp_rom_delay_us(800-1);
	gpio_set_level(GPIO_ADB, 1);
	esp_rom_delay_us(65-1);
}

void IRAM_ATTR adb_tx_cmd(unsigned char cmd) {
	adb_tx_setup();
	adb_tx_as();

	/* send command byte (unrolled loop) */
	cmd & 0x80 ? adb_tx_one() : adb_tx_zero();
	cmd & 0x40 ? adb_tx_one() : adb_tx_zero();
	cmd & 0x20 ? adb_tx_one() : adb_tx_zero();
	cmd & 0x10 ? adb_tx_one() : adb_tx_zero();
	cmd & 0x08 ? adb_tx_one() : adb_tx_zero();
	cmd & 0x04 ? adb_tx_one() : adb_tx_zero();
	cmd & 0x02 ? adb_tx_one() : adb_tx_zero();
	cmd & 0x01 ? adb_tx_one() : adb_tx_zero();

	/* stop bit */
	adb_tx_stop();
	adb_rx_setup();
}

void IRAM_ATTR adb_tx_data(uint16_t data) {
	adb_tx_setup();
	adb_tx_one();

	/* send data 2 bytes (unrolled loop) */
	data & 0x8000 ? adb_tx_one() : adb_tx_zero();
	data & 0x4000 ? adb_tx_one() : adb_tx_zero();
	data & 0x2000 ? adb_tx_one() : adb_tx_zero();
	data & 0x1000 ? adb_tx_one() : adb_tx_zero();
	data & 0x800 ? adb_tx_one() : adb_tx_zero();
	data & 0x400 ? adb_tx_one() : adb_tx_zero();
	data & 0x200 ? adb_tx_one() : adb_tx_zero();
	data & 0x100 ? adb_tx_one() : adb_tx_zero();
	data & 0x80 ? adb_tx_one() : adb_tx_zero();
	data & 0x40 ? adb_tx_one() : adb_tx_zero();
	data & 0x20 ? adb_tx_one() : adb_tx_zero();
	data & 0x10 ? adb_tx_one() : adb_tx_zero();
	data & 0x08 ? adb_tx_one() : adb_tx_zero();
	data & 0x04 ? adb_tx_one() : adb_tx_zero();
	data & 0x02 ? adb_tx_one() : adb_tx_zero();
	data & 0x01 ? adb_tx_one() : adb_tx_zero();

	/* stop bit */
	adb_tx_stop();
	adb_rx_setup();
}

void	adb_tx_listen(unsigned char cmd, uint16_t data) {
	adb_tx_cmd(cmd);

	/*
	 * Stop to start is between 160-240µS. Go for around 160 + time for GPIO setup
	 * values from AN591 Datasheet minus the estimated call to esp_rom_delay_us
	 */

	esp_rom_delay_us(160);
	adb_tx_data(data);
}

static inline void	adb_tx_one() {
	gpio_set_level(GPIO_ADB, 0);
	esp_rom_delay_us(ADB_1_LOW - 1);
	gpio_set_level(GPIO_ADB, 1);
	esp_rom_delay_us(ADB_1_HIGH - 1);
}

void	adb_tx_reset() {
	/*
	 * ADB spec says reset signal low for 3ms ±30%
	 * Note that the ADB Desktop Bus Mouse G5431 uses 4ms when plugged
	 * ADB mouse init in <70ms, but lets wait 500ms to be sure
	 */

	gpio_set_level(GPIO_ADB, 0);
	esp_rom_delay_us(ADB_RESET);
	gpio_set_level(GPIO_ADB, 1);
	esp_rom_delay_us(500);
}

static inline void	adb_tx_setup() {
	gpio_set_direction(GPIO_ADB, GPIO_MODE_OUTPUT);
	gpio_set_level(GPIO_DIR, 1);
}

static inline void	adb_tx_stop() {
	gpio_set_level(GPIO_ADB, 0);
	esp_rom_delay_us(ADB_S_LOW - 1);
	gpio_set_level(GPIO_ADB, 1);
	esp_rom_delay_us(ADB_S_HIGH - 1);
}

static inline void	adb_tx_zero() {
	gpio_set_level(GPIO_ADB, 0);
	esp_rom_delay_us(ADB_0_LOW - 1);
	gpio_set_level(GPIO_ADB, 1);
	esp_rom_delay_us(ADB_0_HIGH - 1);
}
