/*
 *  quad.h
 *  quack
 *
 *  Created by Michel DEPEIGE on 17/01/2021.
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

#ifndef QUAD_H
#define QUAD_H

/* prototypes */
void		quad_init(void);
void		quad_click(void *pvParameters);
void		quad_move_x(void *pvParameters);
void		quad_move_y(void *pvParameters);

/* defines */
#define QUAD_INTERVAL 200

#endif

