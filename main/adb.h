/*
 *  adb.h
 *  quack
 *
 *  Created by Michel DEPEIGE on 7/01/2021.
 *  Copyright (c) 2020 Michel DEPEIGE.
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

#ifndef ADB_H
#define ADB_H

/* prototypes */
void	adb_init(void);
bool	adb_is_host(void);
void    adb_task_host(void *pvParameters);
void    adb_task_idle(void *pvParameters);

void	adb_tx_cmd(unsigned char cmd);
void	adb_tx_data(uint16_t data);
void	adb_tx_listen(unsigned char cmd, uint16_t data);
void	adb_tx_reset(void);

/* defines */
#define RMT_RX_CHANNEL	RMT_CHANNEL_0
#define tskADB_PRIORITY 4

/* ADB timers */
#define	ADB_RESET	3000
#define ADB_0_LOW	65
#define ADB_0_HIGH	35
#define ADB_1_LOW	35
#define ADB_1_HIGH	65

/* Classic Apple Mouse Protocol bitmasks */
#define ADB_CMP_B1	(1<<15)
#define ADB_CMP_B2	(1<<7)
#define ADB_CMP_MX	(127<<0)
#define ADB_CMP_MY	(127<<8)

/* ADB commands values from 00591b.pdf page 16-17 */
#define ADB_TALK	0xC
#define ADB_LISTEN	0x8
#define ADB_FLUSH	0x1
#define ADB_REG0	0x0
#define ADB_REG3	0x3

/* Device addresses */
#define ADB_MOUSE	(3<<4)
#define ADB_KML1	(8<<4)	// disabled Kensington device
#define ADB_TMP	(9<<4)

/* Various stuff */
#define ADB_B_UP	0
#define ADB_B_DOWN	1

#define ADB_H_ALL	0xff	// Handlers bitmask
#define ADB_H_ERR	0x00	// Handler 0 (Self init error)
#define ADB_H_C100	0x01	// Handler 1 (Classic @ 100cpi)
#define ADB_H_C200	0x02	// Handler 2 (Classic @ 200cpi)
#define ADB_H_MTRC	0x2f	// Handler for MacTRAC 2.0
#define ADB_H_KSGT	0x32	// Handler for Kensington devices
#define ADB_H_MOVE	0xfe	// Move to another address

/* Host states */
#define ADB_S_PROBE		0x00
#define ADM_S_POLL		0x01
#define ADB_S_KEEP		0x08

#endif
