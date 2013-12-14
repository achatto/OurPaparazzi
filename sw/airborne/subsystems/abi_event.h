/*
 * Copyright (C) 2013 ENAC - Gautier Hattenberger
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file subsystems/abi_event.h
 *
 * ABI main event handler API
 *
 * Modules and subsystems can register event callbacks
 * Several implementations are possible
 * - the main static task scheduler checks if a flag is raised and call the callback
 * - raise an OS event when using a RTOS
 *
 */

#ifndef ABI_EVENT_H
#define ABI_EVENT_H

#include "subsystems/abi_common.h"

extern abi_event* abi_main_event_queue;

/** Register a main event callback
 *
 * @param ev pointer to an abi_event structure
 * @param cb callback function
 */
extern void abi_register_main_event(abi_event * ev, abi_callback cb);

/** Raise an event
 *
 * - raise an event flag for static scheduler
 * - send OS event for RTOS scheduler
 *
 * @param ev pointer to the abi_event to trigger
 */
extern void abi_raise_main_event(abi_event * ev);

/** Check and call events
 *
 * With static scheduler, events need to be checked
 * in order to call the callback function if an event
 * is raised
 *
 * Implementing and calling this function when using a RTOS
 * should not be needed
 */
extern void abi_main_event_check(void);


#endif /* ABI_EVENT_H */
