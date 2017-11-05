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

/** \file modules/datalink/spprz_dl.h
 *  \brief Datalink using secure PPRZ protocol
 */

#ifndef SPPRZ_DL_H
#define SPPRZ_DL_H

#include "pprzlink/secure_pprz_transport.h"

#include "mcu_periph/uart.h"


/** PPRZ transport structure */
extern struct spprz_transport spprz_tp;

/** Init function */
extern void spprz_dl_init(void);

/** Datalink Event */
extern void spprz_dl_event(void);

/** Check if the status is OK and we can
 * process the messages */
extern bool spprz_is_comm_status_ok(void);

/** Process auxiliarry messages (such as key exchange)
 * before the proper secure channel is established
 */
extern void spprz_process_dl_msg(struct link_device *dev, struct transport_tx *trans, uint8_t *buf);

#endif /* SPPRZ_DL_H */

