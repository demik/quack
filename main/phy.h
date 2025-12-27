/*
 *  phy.h
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

#ifndef PHY_H
#define PHY_H

/*
 * Note: theses comes mostly from the legacy RMT driver (pre ESP-IDK v5)
 * Since we are only doing RX on one channel and only on ES32, the driver and structures
 * have been pruned of useless (for us) features
 *
 * the naming scheme mostly follow the legacy driver (ex: rmt_config_t => phy_config_t)
 */

/* defines */
#define PHY_CHANNEL_NUM 0
#define PHY_CHANNEL_FLAGS_INVERT_SIG (1 << 1) /* Invert RMT signal */
#define PHY_ADDR_ERROR_STR "PHY(ADB) ADDRESS ERR"
#define PHY_CLK_DIV_ERROR_STR "PHY(ADB) CLK DIV ERR"
#define PHY_DEFAULT_CLK_DIV 80
#define PHY_MEM_CNT_ERROR_STR "PHY(ADB) MEM BLOCK NUM ERR"

/* structures */
typedef intr_handle_t phy_isr_handle_t;

/* RMT RX specific configure parameters (main structure below) */
typedef struct {
	uint16_t idle_threshold;     /* RMT RX idle threshold */
	uint8_t filter_ticks_thresh; /* RMT filter tick number */
	bool filter_en;              /* RMT receiver filter enable */
} phy_rx_config_t;

/* structure of RMT configure parameters */
typedef struct {
	gpio_num_t gpio_num;         /* RMT GPIO number */
	uint8_t clk_div;             /* RMT channel counter divider */
	uint8_t mem_block_num;       /* RMT memory block number */
	uint32_t flags;              /* RMT channel flags OR'd with RMT_CHANNEL_FLAGS_[*] */
	phy_rx_config_t rx_config;   /* RMT RX parameter */
} phy_config_t;

/* structure of data coming from the RMT device */
typedef struct {
	union {
		struct {
			uint32_t duration0 : 15; /* Duration of level0 */
			uint32_t level0 : 1;     /* Level of the first part */
			uint32_t duration1 : 15; /* Duration of level1 */
			uint32_t level1 : 1;     /* Level of the second part */
		};
		uint32_t val; /* Equivalent unsigned value for the RMT item */
	};
} phy_item32_t;

/* hardware memory layout */
typedef struct {
	struct {
		volatile phy_item32_t data32[SOC_RMT_MEM_WORDS_PER_CHANNEL];
	} chan[SOC_RMT_CHANNELS_PER_GROUP];
} phy_mem_t;

/* prototypes */
esp_err_t phy_config();
esp_err_t phy_driver_install(size_t rx_buf_size, int intr_alloc_flags);
esp_err_t phy_driver_uninstall(void);
esp_err_t phy_get_ringbuf_handle(RingbufHandle_t *buf_handle);
void		phy_get_status(uint32_t *status);
esp_err_t phy_isr_deregister(phy_isr_handle_t handle);
esp_err_t phy_isr_register(void (*fn)(void *), void *arg, int intr_alloc_flags, intr_handle_t *handle);
esp_err_t phy_rx_start(bool rx_idx_rst);
esp_err_t phy_rx_stop(void);
esp_err_t phy_set_gpio(gpio_num_t gpio_num, bool invert_signal);


#endif
