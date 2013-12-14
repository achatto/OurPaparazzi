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
 * @file subsystems/abi_event_static.c
 *
 * ABI main event handler API
 *
 * Implementation for static scheduler
 *
 */

#include "std.h"
#include "subsystems/abi_event.h"

abi_event* abi_main_event_queue;

/** Register a main event callback
 */
void abi_register_main_event(abi_event * ev, abi_callback cb) {
  ev->id = FALSE;
  ev->cb = cb;
  ABI_PREPEND(abi_main_event_queue, ev);
}

/** Raise an event
 */
void abi_raise_main_event(abi_event * ev) {
  ev->id = TRUE;
}

/** Check and call events
 */
void abi_main_event_check(void) {
  abi_event* e;
  ABI_FOREACH(abi_main_event_queue, e) {
    if (e->id) {
      e->id = FALSE;
      e->cb();
    }
  }
}
