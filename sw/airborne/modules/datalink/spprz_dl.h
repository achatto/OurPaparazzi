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

#define PPRZ_SIGN_LEN 64

typedef unsigned char ed25519_signature[64];

struct gec_privkey {
    uint8_t priv[PPRZ_KEY_LEN];;
    uint8_t pub[PPRZ_KEY_LEN];;
};

struct gec_pubkey {
    uint8_t pub[PPRZ_KEY_LEN];;
};

struct gec_sym_key {
    uint8_t  key[PPRZ_KEY_LEN];
    uint8_t  nonce[PPRZ_NONCE_LEN];
    uint32_t ctr;
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

typedef enum {
  P_AE, P_BE, SIG,
} gec_sts_msg_type_t;

typedef enum {
  ERROR_NONE,
  // RESPONDER ERRORS
  MSG1_TIMEOUT_ERROR,
  MSG1_ENCRYPT_ERROR,
  MSG3_TIMEOUT_ERROR,
  MSG3_DECRYPT_ERROR,
  MSG3_SIGNVERIFY_ERROR,
  // INITIATOR ERRORS
  MSG2_TIMEOUT_ERROR,
  MSG2_DECRYPT_ERROR,
  MSG2_SIGNVERIFY_ERROR,
  MSG3_ENCRYPT_ERROR
} sts_error_t;

// Intermediate data structure containing information relating to the stage of
// the STS protocol.
struct gec_sts_ctx {
    struct gec_pubkey theirPublicKey;
    struct gec_privkey myPrivateKey;
    struct gec_pubkey theirPublicKeyEphemeral;
    struct gec_privkey myPrivateKeyEphemeral;
    struct gec_sym_key theirSymmetricKey;
    struct gec_sym_key mySymmetricKey;
    stage_t protocol_stage;
    party_t party;
    sts_error_t last_error;
};


// Zero all fields, excluding the long term public and private keys.
void clear_ctx(struct gec_sts_ctx * ctx);

/** PPRZ transport structure */
extern struct spprz_transport spprz_tp;

/** Init function */
extern void spprz_dl_init(void);

/** Datalink Event */
extern void spprz_dl_event(void);

/** Process auxiliarry messages (such as key exchange)
 * before the proper secure channel is established
 */
void spprz_process_sts_msg(struct link_device *dev, struct spprz_transport *trans, uint8_t *buf);

void respond_sts(struct link_device *dev, struct spprz_transport *trans, uint8_t *buf);

void finish_sts(struct link_device *dev, struct spprz_transport *trans, uint8_t *buf);

void generate_ephemeral_keys(struct gec_privkey *sk);

void derive_key_material(struct gec_sts_ctx *ctx, uint8_t* z);

uint32_t gec_encrypt(struct gec_sym_key *k, uint8_t *ciphertext, uint8_t *plaintext, size_t len, uint8_t *mac);

#endif /* SPPRZ_DL_H */

