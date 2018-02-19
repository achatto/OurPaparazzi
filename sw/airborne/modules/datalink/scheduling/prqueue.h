/*
 * Copyright (C) 2018 Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
 * Copyright (C) 2018 David Cerny
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/** \file modules/scheduling/prqueue.h
 *  \brief Priority queue for telemetry messages
 */
#ifndef PRQUEUE_H
#define PRQUEUE_H

#include "std.h"

#ifndef MAX_Q_SIZE
#define MAX_Q_SIZE 5
#endif

#ifndef TRANSPORT_PAYLOAD_LEN
#define TRANSPORT_PAYLOAD_LEN 256
#endif

typedef struct
{
  uint8_t priority;  // message priority
  float insertion_time;  // time of message insertion
  uint8_t payload[TRANSPORT_PAYLOAD_LEN];  // messgae payload
  uint8_t idx;  // current payload length

} msg_container_t;

typedef struct
{
  msg_container_t elements[MAX_Q_SIZE];
  uint8_t N;

} pqueue_t;

bool pq_isless(const msg_container_t a, const msg_container_t b);
void pq_init(pqueue_t *queue);
bool pq_push(pqueue_t *queue, const msg_container_t* element);
bool pq_push_over(pqueue_t *queue, const msg_container_t* element);
bool pq_getmax(pqueue_t *queue, msg_container_t* max_element);
uint8_t pq_get_min_index(pqueue_t *queue);
uint8_t pq_size(pqueue_t *queue);
bool pq_peek(pqueue_t *queue, msg_container_t *max_element);

#endif /* PRQUEUE_H */
