/*
 *  wii.h
 *  quack
 *
 *  Created by Michel DEPEIGE on 03/03/2025.
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

#pragma once

/* prototypes */
void	mighty_input(esp_hidh_dev_t *dev, uint16_t id, uint8_t  *data, uint16_t length);
bool mighty_is_mm(uint16_t vid, uint16_t pid);

/* defines */
#define MIGHTY_VID	0x05ac
#define MIGHTY_PID	0x030c

/* handled reports */
#define MIGHTY_REPORT_INPUT	0x02