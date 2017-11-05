/*
 * Copyright (C) 2017 Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
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

/** \file modules/datalink/spprz_dl.c
 *  \brief Datalink using secure PPRZ protocol
 */

#include "modules/datalink/spprz_dl.h"
#include "subsystems/datalink/datalink.h"
#include "mcu_periph/rng.h"

struct spprz_transport spprz_tp;

void spprz_dl_init(void)
{
  spprz_transport_init(&spprz_tp);
}

void spprz_dl_event(void)
{
  spprz_check_and_parse(&DOWNLINK_DEVICE.device, &spprz_tp, dl_buffer, &dl_msg_available);
  DlCheckAndParse(&DOWNLINK_DEVICE.device, &spprz_tp.trans_tx, dl_buffer, &dl_msg_available);
}

bool spprz_is_comm_status_ok(void) {
  // TODO: just a dummy for now
  return true;
}

void spprz_process_dl_msg(struct link_device *dev, struct transport_tx *trans, uint8_t *buf) {
  // TODO: just a dummy for now
  (void)dev;
  (void)trans;
  (void)buf;
}

// generate keys

// setters/getters for spprz struct
