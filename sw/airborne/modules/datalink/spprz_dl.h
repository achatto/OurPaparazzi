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

typedef unsigned char ed25519_signature[64];
typedef unsigned char ed25519_public_key[32];
typedef unsigned char ed25519_secret_key[32];

struct gec_privkey {
    ed25519_secret_key priv;
    ed25519_public_key pub;
};

struct gec_pubkey {
    ed25519_public_key pub;
};

typedef enum {
  INIT,
  WAIT_MSG1,
  WAIT_MSG2,
  WAIT_MSG3,
  CRYPTO_OK,
} stage_t;

typedef enum {
    INITIATOR, RESPONDER, CLIENT, INVALID_PARTY
} party_t;

struct gec_sym_key {
    uint8_t  key[PPRZ_KEY_LEN];
    uint8_t  nonce[PPRZ_NONCE_LEN];
    uint32_t ctr;
};

// Intermediate data structure containing information relating to the stage of
// the STS protocol.
struct gec_sts_ctx {
    struct gec_pubkey theirPublicKey;
    struct gec_privkey myPrivateKey;
    uint8_t myPublicKey_ephemeral[PPRZ_KEY_LEN];
    uint8_t myPrivateKey_ephemeral[PPRZ_KEY_LEN];
    uint8_t theirPublicKey_ephemeral[PPRZ_KEY_LEN];
    uint8_t client_key_material[PPRZ_KEY_MATERIAL_LEN];
    stage_t protocol_stage;
    party_t party;
};


// Zero the protocol stage.  This is like reset_patner(), but the current
// parnter public key is retained.
void reset_ctx(struct gec_sts_ctx * ctx);

// Zero all fields, including the long term public and private keys.
void clear_ctx(struct gec_sts_ctx * ctx);


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
extern void spprz_process_sts_msg(struct link_device *dev, struct spprz_transport *trans, uint8_t *buf);

#endif /* SPPRZ_DL_H */

