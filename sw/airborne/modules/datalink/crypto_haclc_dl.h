/*
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** \file modules/datalink/crypto_haclc_dl.h
 *  \brief Datalink using HACLC Crypto over PPRZ protocol
 */

#ifndef CRYPTO_HACLC_DL_H
#define CRYPTO_HACLC_DL_H

#include "pprzlink/pprzlink_transport.h"
#include "pprzlink/pprz_transport.h"
#include "pprz_mutex.h" // mutex definitions FIXME only in C part ?

#include "mcu_periph/uart.h"
#if USE_USB_SERIAL
#include "mcu_periph/usb_serial.h"
#endif
#if USE_UDP
#include "mcu_periph/udp.h"
#endif

struct crypto_haclc_transport {
  // pprz encapsulation layer
  struct pprz_transport pprz_tp;

  // generic reception interface
  struct transport_rx trans_rx;

  // generic transmission interface
  struct transport_tx trans_tx;
  // buffered tx message
  uint8_t tx_msg[TRANSPORT_PAYLOAD_LEN];
  volatile uint8_t tx_msg_idx;
  bool packet_encrypted;
  PPRZ_MUTEX(mtx_tx); // mutex is a part of the transport FIXME only in C part ?
};


/** PPRZ transport structure */
extern struct crypto_haclc_transport crypto_haclc_tp;

/** Init function */
extern void crypto_haclc_dl_init(void);

/** Datalink Event */
extern void crypto_haclc_dl_event(void);

/** Parsing a frame data and copy the payload to the datalink buffer */
void crypto_haclc_check_and_parse(struct link_device *dev, struct crypto_haclc_transport *trans,
                          uint8_t *buf, bool *msg_available)

#endif /* CRYPTO_HACLC_DL_H */


