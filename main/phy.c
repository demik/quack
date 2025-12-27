/*
 *  phy.c
 *  quack
 *
 *  Created by Michel DEPEIGE on 27/12/2025.
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

#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_check.h>
#include <esp_clk_tree.h>
#include <freertos/FreeRTOS.h>
#include <freertos/ringbuf.h>
#include <hal/rmt_hal.h>
#include <hal/rmt_ll.h>
#include <soc/io_mux_reg.h>
#include <soc/rmt_periph.h>

#include "gpio.h"
#include "phy.h"
#include "sdkconfig.h"

/* defines */
static const char *TAG = "PHY(ADB)";

/* spinlock for protecting concurrent register-level access only */
#define PHY_ENTER_CRITICAL()  portENTER_CRITICAL_SAFE(&(rmt_contex.rmt_spinlock))
#define PHY_EXIT_CRITICAL()   portEXIT_CRITICAL_SAFE(&(rmt_contex.rmt_spinlock))

/* globals */
typedef struct {
	rmt_hal_context_t hal;
	_lock_t rmt_driver_isr_lock;
	/* Mutex lock for protecting concurrent register/unregister of RMT channels (ISR) */
	portMUX_TYPE rmt_spinlock;
	phy_isr_handle_t rmt_driver_intr_handle;
	/* Bitmask of installed drivers channels for protecting concurrent register/unregister of RMT channels (ISR) */
	uint8_t	rmt_driver_channels;
	bool		rmt_module_enabled;
} phy_contex_t;

static phy_contex_t rmt_contex = {
	.hal.regs = &RMT,
	.rmt_spinlock = portMUX_INITIALIZER_UNLOCKED,
	.rmt_driver_intr_handle = NULL,
	.rmt_driver_channels = 0,
	.rmt_module_enabled = false,
};

phy_config_t adb_rmt_rx;
RingbufHandle_t phy_rx_buf = NULL;
bool	  phy_driver_installed = false;

/* static defines */
static void phy_driver_isr(void *arg);
static esp_err_t phy_internal_config(rmt_dev_t *dev, const phy_config_t *phy_param);
static void phy_module_disable(void);
static void phy_module_enable(void);

/* externals… for private stuff */
extern esp_err_t gpio_func_sel(gpio_num_t gpio_num, uint32_t func);
extern esp_err_t esp_clk_tree_src_get_freq_hz();
extern esp_err_t esp_clk_tree_enable_src(soc_module_clk_t clk_src, bool enable);

/* or devices. RMTMEM address is declared in <target>.peripherals.ld */
extern phy_mem_t RMTMEM;

/* functions */
esp_err_t phy_config(void) {
	/* set RMT RX driver with values for ADB (others are the default legacy ones) */
	adb_rmt_rx.gpio_num = GPIO_ADB;
	adb_rmt_rx.clk_div = PHY_DEFAULT_CLK_DIV;
	adb_rmt_rx.mem_block_num = 4;
	adb_rmt_rx.flags = 0;
	adb_rmt_rx.rx_config.filter_en = true;
	adb_rmt_rx.rx_config.filter_ticks_thresh = 10;
	adb_rmt_rx.rx_config.idle_threshold = 100;

	phy_module_enable();

	ESP_RETURN_ON_ERROR(phy_set_gpio(adb_rmt_rx.gpio_num, adb_rmt_rx.flags & PHY_CHANNEL_FLAGS_INVERT_SIG), TAG, "set gpio for RMT driver failed");
	ESP_RETURN_ON_ERROR(phy_internal_config(&RMT, &adb_rmt_rx), TAG, "initialize RMT driver failed");

	return ESP_OK;
}

esp_err_t phy_driver_install(size_t rx_buf_size, int intr_alloc_flags) {
	esp_err_t err = ESP_OK;

	if (phy_driver_installed) {
		ESP_LOGD(TAG, "RMT driver already installed");
		return ESP_ERR_INVALID_STATE;
	}

#if CONFIG_RINGBUF_PLACE_ISR_FUNCTIONS_INTO_FLASH
	if (intr_alloc_flags & ESP_INTR_FLAG_IRAM) {
		ESP_LOGE(TAG, "ringbuf ISR functions in flash, but used in IRAM interrupt");
		return ESP_ERR_INVALID_ARG;
	}
#endif

	if (phy_rx_buf == NULL && rx_buf_size > 0)
		phy_rx_buf = xRingbufferCreate(rx_buf_size, RINGBUF_TYPE_NOSPLIT);

	_lock_acquire_recursive(&(rmt_contex.rmt_driver_isr_lock));
	if (rmt_contex.rmt_driver_channels == 0) {
		ESP_LOGD(TAG, "ISR registered");
		err = phy_isr_register(phy_driver_isr, &rmt_contex.hal, intr_alloc_flags, &(rmt_contex.rmt_driver_intr_handle));
	}
	if (err == ESP_OK)
		rmt_contex.rmt_driver_channels |= BIT(PHY_CHANNEL_NUM);
	_lock_release_recursive(&(rmt_contex.rmt_driver_isr_lock));

	phy_module_enable();
	rmt_hal_rx_channel_reset(&rmt_contex.hal, PHY_CHANNEL_NUM);

	return err;
}

static void IRAM_ATTR phy_driver_isr(void *arg) {
	uint32_t status = 0;
	phy_item32_t *addr = NULL;
	rmt_hal_context_t *hal = (rmt_hal_context_t *)arg;
	BaseType_t HPTaskAwoken = pdFALSE;

	/* this should not be needed (RX only driver) but your never know */
	status = rmt_ll_get_tx_end_interrupt_status(hal->regs);
	if (status) {
		ESP_DRAM_LOGE(TAG, "spurious end TX interrupt %x", status);
		rmt_ll_clear_interrupt_status(hal->regs, RMT_LL_EVENT_TX_DONE(PHY_CHANNEL_NUM));
	}
	status = rmt_ll_get_tx_thres_interrupt_status(hal->regs);
	if (status) {
		rmt_ll_clear_interrupt_status(hal->regs, RMT_LL_EVENT_TX_THRES(PHY_CHANNEL_NUM));
		ESP_DRAM_LOGE(TAG, "spurious threshold TX interrupt %x", status);
	}
	status = rmt_ll_get_tx_err_interrupt_status(hal->regs);
	if (status) {
		rmt_ll_clear_interrupt_status(hal->regs, RMT_LL_EVENT_TX_ERROR(PHY_CHANNEL_NUM));
		ESP_DRAM_LOGE(TAG, "spurious TX interrupt error %x", status);
	}

	/* RX end interrupt */
	status = rmt_ll_get_rx_end_interrupt_status(hal->regs);
	if (status) {
		rmt_ll_rx_enable(rmt_contex.hal.regs, PHY_CHANNEL_NUM, false);
		int item_len = rmt_ll_rx_get_memory_writer_offset(rmt_contex.hal.regs, PHY_CHANNEL_NUM);
		rmt_ll_rx_set_mem_owner(rmt_contex.hal.regs, PHY_CHANNEL_NUM, RMT_LL_MEM_OWNER_SW);
		if (phy_rx_buf) {
			addr = (phy_item32_t *)RMTMEM.chan[PHY_CHANNEL_NUM].data32;
			BaseType_t res = xRingbufferSendFromISR(phy_rx_buf, (void *)addr, item_len * 4, &HPTaskAwoken);
			if (res == pdFALSE)
				ESP_DRAM_LOGE(TAG, "RMT RX buffer is full");
		} else {
			ESP_DRAM_LOGE(TAG, "RMT RX buffer error");
		}

		rmt_ll_rx_reset_pointer(rmt_contex.hal.regs, PHY_CHANNEL_NUM);
		rmt_ll_rx_set_mem_owner(rmt_contex.hal.regs, PHY_CHANNEL_NUM, RMT_LL_MEM_OWNER_HW);
		rmt_ll_rx_enable(rmt_contex.hal.regs, PHY_CHANNEL_NUM, true);
		rmt_ll_clear_interrupt_status(hal->regs, RMT_LL_EVENT_RX_DONE(PHY_CHANNEL_NUM));
	}

	/* RMT RX error. It's probably fucked up */
	status = rmt_ll_get_rx_err_interrupt_status(hal->regs);
	if (status) {
		/* Reset the receiver's write/read addresses to prevent endless err interrupts. */
		rmt_ll_rx_reset_pointer(rmt_contex.hal.regs, PHY_CHANNEL_NUM);
		ESP_DRAM_LOGD(TAG, "RMT RX channel %d error", PHY_CHANNEL_NUM);
		ESP_DRAM_LOGD(TAG, "status: 0x%08x", rmt_ll_rx_get_status_word(rmt_contex.hal.regs, PHY_CHANNEL_NUM));
		rmt_ll_clear_interrupt_status(hal->regs, RMT_LL_EVENT_RX_ERROR(PHY_CHANNEL_NUM));
	}

	if (HPTaskAwoken == pdTRUE) {
		portYIELD_FROM_ISR();
	}
}

esp_err_t phy_driver_uninstall(void) {
	esp_err_t err = ESP_OK;

	PHY_ENTER_CRITICAL();
	rmt_ll_enable_interrupt(rmt_contex.hal.regs, RMT_LL_EVENT_RX_MASK(PHY_CHANNEL_NUM) | RMT_LL_EVENT_RX_ERROR(PHY_CHANNEL_NUM), false);
	PHY_EXIT_CRITICAL();

	_lock_acquire_recursive(&(rmt_contex.rmt_driver_isr_lock));
	rmt_contex.rmt_driver_channels &= ~BIT(PHY_CHANNEL_NUM);
	if (rmt_contex.rmt_driver_channels == 0 && rmt_contex.rmt_driver_intr_handle) {
		phy_module_disable();
		err = phy_isr_deregister(rmt_contex.rmt_driver_intr_handle);
		rmt_contex.rmt_driver_intr_handle = NULL;
	}
	_lock_release_recursive(&(rmt_contex.rmt_driver_isr_lock));

	if (phy_rx_buf) {
		vRingbufferDelete(phy_rx_buf);
		phy_rx_buf = NULL;
	}

	return err;
}

static esp_err_t phy_internal_config(rmt_dev_t *dev, const phy_config_t *phy_param) {
	uint8_t gpio_num = phy_param->gpio_num;
	uint8_t mem_cnt = phy_param->mem_block_num;
	uint8_t clk_div = phy_param->clk_div;
	uint32_t rmt_source_clk_hz;
	rmt_clock_source_t clk_src = RMT_BASECLK_DEFAULT;

	ESP_RETURN_ON_FALSE(mem_cnt <= SOC_RMT_CHANNELS_PER_GROUP && mem_cnt > 0, ESP_ERR_INVALID_ARG, TAG, PHY_MEM_CNT_ERROR_STR);
	ESP_RETURN_ON_FALSE(clk_div > 0, ESP_ERR_INVALID_ARG, TAG, PHY_CLK_DIV_ERROR_STR);

	PHY_ENTER_CRITICAL();
	rmt_ll_enable_mem_access_nonfifo(dev, true);
	esp_clk_tree_src_get_freq_hz((soc_module_clk_t)clk_src, ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &rmt_source_clk_hz);
	ESP_ERROR_CHECK(esp_clk_tree_enable_src((soc_module_clk_t)clk_src, true));
	rmt_ll_set_group_clock_src(dev, PHY_CHANNEL_NUM, clk_src, 1, 0, 0);
	rmt_ll_enable_group_clock(dev, true);
	PHY_EXIT_CRITICAL();

	ESP_LOGD(TAG, "phy_source_clk_hz: %"PRIu32, rmt_source_clk_hz);

	uint8_t filter_cnt = phy_param->rx_config.filter_ticks_thresh;
	uint16_t threshold = phy_param->rx_config.idle_threshold;

	PHY_ENTER_CRITICAL();
	rmt_ll_rx_set_channel_clock_div(dev, PHY_CHANNEL_NUM, clk_div);
	rmt_ll_rx_set_mem_blocks(dev, PHY_CHANNEL_NUM, mem_cnt);
	rmt_ll_rx_reset_pointer(dev, PHY_CHANNEL_NUM);
	rmt_ll_rx_set_mem_owner(dev, PHY_CHANNEL_NUM, RMT_LL_MEM_OWNER_HW);
	rmt_ll_rx_set_idle_thres(dev, PHY_CHANNEL_NUM, threshold);
	rmt_ll_rx_set_filter_thres(dev, PHY_CHANNEL_NUM, filter_cnt);
	rmt_ll_rx_enable_filter(dev, PHY_CHANNEL_NUM, phy_param->rx_config.filter_en);
	PHY_EXIT_CRITICAL();

	ESP_LOGD(TAG, "RMT RX Channel 0|GPIO %u|Sclk_Hz %"PRIu32"|Div %u|Threshold %u|Filter %u",
			gpio_num, rmt_source_clk_hz, clk_div, threshold, filter_cnt);

	return ESP_OK;
}

esp_err_t phy_isr_deregister(phy_isr_handle_t handle) {
    return esp_intr_free(handle);
}

esp_err_t phy_isr_register(void (*fn)(void *), void *arg, int intr_alloc_flags, intr_handle_t *handle) {
	ESP_RETURN_ON_FALSE(fn, ESP_ERR_INVALID_ARG, TAG, PHY_ADDR_ERROR_STR);
	ESP_RETURN_ON_FALSE(rmt_contex.rmt_driver_channels == 0, ESP_FAIL, TAG, "RMT driver installed, can not install ISR handler");

	return esp_intr_alloc(rmt_periph_signals.groups[0].irq, intr_alloc_flags, fn, arg, handle);
}

esp_err_t phy_get_ringbuf_handle(RingbufHandle_t *buf_handle) {
    *buf_handle = phy_rx_buf;
    return ESP_OK;
}

void phy_get_status(uint32_t *status) {
	PHY_ENTER_CRITICAL();
	*status = rmt_ll_rx_get_status_word(rmt_contex.hal.regs, PHY_CHANNEL_NUM);
	PHY_EXIT_CRITICAL();
}

/* disable RMT module */
static void phy_module_disable(void) {
	/* to suppress build errors about spinlock's __DECLARE_RCC_ATOMIC_ENV */
	int __DECLARE_RCC_ATOMIC_ENV __attribute__ ((unused));

	PHY_ENTER_CRITICAL();
	if (rmt_contex.rmt_module_enabled == true) {
		rmt_ll_mem_force_power_off(rmt_contex.hal.regs);
		rmt_ll_enable_bus_clock(0, false);
		rmt_contex.rmt_module_enabled = false;
	}
	PHY_EXIT_CRITICAL();
}

/* enable RMT module */
static void phy_module_enable(void) {
	/* to suppress build errors about spinlock's __DECLARE_RCC_ATOMIC_ENV */
	int __DECLARE_RCC_ATOMIC_ENV __attribute__ ((unused));

	PHY_ENTER_CRITICAL();
	if (rmt_contex.rmt_module_enabled == false) {
		rmt_ll_enable_bus_clock(0, true);
		rmt_ll_reset_register(0);
		rmt_ll_mem_power_by_pmu(rmt_contex.hal.regs);
		rmt_contex.rmt_module_enabled = true;
	}
	PHY_EXIT_CRITICAL();
}

esp_err_t phy_rx_start(bool rx_idx_rst) {
    PHY_ENTER_CRITICAL();
    rmt_ll_rx_enable(rmt_contex.hal.regs, PHY_CHANNEL_NUM, false);
    if (rx_idx_rst)
        rmt_ll_rx_reset_pointer(rmt_contex.hal.regs, PHY_CHANNEL_NUM);
    rmt_ll_clear_interrupt_status(rmt_contex.hal.regs, RMT_LL_EVENT_RX_DONE(PHY_CHANNEL_NUM));
    rmt_ll_enable_interrupt(rmt_contex.hal.regs, RMT_LL_EVENT_RX_DONE(PHY_CHANNEL_NUM), true);
    rmt_ll_rx_enable(rmt_contex.hal.regs, PHY_CHANNEL_NUM, true);
    PHY_EXIT_CRITICAL();
    return ESP_OK;
}

esp_err_t phy_rx_stop(void) {
    PHY_ENTER_CRITICAL();
    rmt_ll_enable_interrupt(rmt_contex.hal.regs, RMT_LL_EVENT_RX_DONE(PHY_CHANNEL_NUM), false);
    rmt_ll_rx_enable(rmt_contex.hal.regs, PHY_CHANNEL_NUM, false);
    rmt_ll_rx_reset_pointer(rmt_contex.hal.regs, PHY_CHANNEL_NUM);
    PHY_EXIT_CRITICAL();
    return ESP_OK;
}

/* set GPIO matrix and GPIO pin for channel 0 */
esp_err_t phy_set_gpio(gpio_num_t gpio_num, bool invert_signal) {
	ESP_RETURN_ON_FALSE(GPIO_IS_VALID_GPIO(gpio_num), ESP_ERR_INVALID_ARG, TAG, "PHY(ADB) invalid GPIO");

	gpio_func_sel(gpio_num, PIN_FUNC_GPIO);
	gpio_set_direction(gpio_num, GPIO_MODE_INPUT);
	esp_rom_gpio_connect_in_signal(gpio_num, rmt_periph_signals.groups[0].channels[PHY_CHANNEL_NUM].rx_sig, invert_signal);
	return ESP_OK;
}
