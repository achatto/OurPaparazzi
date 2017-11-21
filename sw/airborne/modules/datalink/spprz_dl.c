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
#include "generated/keys_uav.h"
#include "datalink/hacl-c/Ed25519.h"
#include "datalink/hacl-c/Curve25519.h"
#include "datalink/hacl-c/SHA_512.h"

#include <string.h> // for memcpy

struct spprz_transport spprz_tp;
struct gec_sts_ctx sts;


void spprz_dl_init(void)
{
  spprz_transport_init(&spprz_tp);

  // initialize keys
  clear_ctx(&sts);

  uint8_t theirPublicKey[PPRZ_KEY_LEN] = GCS_PUBLIC;
  memcpy(&sts.theirPublicKey, theirPublicKey, PPRZ_KEY_LEN);

  uint8_t myPublicKey[PPRZ_KEY_LEN] = UAV_PUBLIC;
  memcpy(&sts.myPrivateKey.pub, myPublicKey, PPRZ_KEY_LEN);

  uint8_t myPrivateKey[PPRZ_KEY_LEN] = UAV_PRIVATE;
  memcpy(&sts.myPrivateKey.priv, myPrivateKey, PPRZ_KEY_LEN);
}


/**
 * Generate private and public key pairs for future use.
 */
void generate_ephemeral_keys(struct gec_privkey *sk)
{
  for (uint8_t i = 0; i < (PPRZ_KEY_LEN / sizeof(uint32_t)); i++) {
    uint32_t tmp = rng_wait_and_get();
    sk->priv[i] = (uint8_t) tmp;
    sk->priv[i + 1] = (uint8_t) (tmp >> 8);
    sk->priv[i + 2] = (uint8_t) (tmp >> 16);
    sk->priv[i + 3] = (uint8_t) (tmp >> 24);
  }
  uint8_t basepoint[32] = {0};
  basepoint[0] = 9; // default basepoint
  Curve25519_crypto_scalarmult(sk->pub, sk->priv, basepoint);
}


/**
 * Derive key material for both sender and receiver
 */
void derive_key_material(struct gec_sts_ctx *ctx, uint8_t* z) {
  uint8_t tmp[PPRZ_KEY_LEN*2] = {0};
  uint8_t input[PPRZ_KEY_LEN+1] = {0};

  // Ka|| Sa = kdf(z,0)
  memcpy(input, z, PPRZ_KEY_LEN);
  input[PPRZ_KEY_LEN] = 0;
  SHA2_512_hash(tmp, input, sizeof(input));
  memcpy(ctx->theirSymmetricKey.key, tmp, PPRZ_KEY_LEN); // K_a
  memcpy(ctx->theirSymmetricKey.nonce, &tmp[PPRZ_KEY_LEN], PPRZ_NONCE_LEN); // S_a

  // Kb|| Sb = kdf(z,1)
  input[PPRZ_KEY_LEN] = 1;
  SHA2_512_hash(tmp, input, sizeof(input));
  memcpy(ctx->mySymmetricKey.key, tmp, PPRZ_KEY_LEN);  // K_b
  memcpy(ctx->mySymmetricKey.nonce, &tmp[PPRZ_KEY_LEN], PPRZ_NONCE_LEN);  // S_b

}


/**
 * 2. B generates ephemeral curve25519 key pair (Pbe, Qbe).
 * 3. B computes the shared secret: z = scalar_multiplication(Qbe, Pae)
 * 4. B uses the key derivation function kdf(z,1) to compute Kb || Sb,
 * kdf(z,0) to compute Ka || Sa, and kdf(z,2) to compute Kclient || Sclient.
 * 5. B computes the ed25519 signature: sig = signQb(Pbe || Pae)
 * 6. B computes and sends the message Pbe || Ekey=Kb,IV=Sb||zero(sig)
 */
void respond_sts(struct link_device *dev, struct spprz_transport *trans, uint8_t *buf) {
  // copy P_ae over
  memcpy(&sts.theirPublicKeyEphemeral.pub, DL_KEY_EXCHANGE_GCS_msg_data(buf), sizeof(struct gec_pubkey));

  // 2. B generates ephemeral curve25519 key pair (Pbe, Qbe).
  generate_ephemeral_keys(&sts.myPrivateKeyEphemeral);

  // 3. B computes the shared secret: z = scalar_multiplication(Qbe, Pae)
  uint8_t z[32] = {0};
  Curve25519_crypto_scalarmult(z, &sts.myPrivateKeyEphemeral.priv, &sts.theirPublicKeyEphemeral.pub);

  // 4. B uses the key derivation function kdf(z,1) to compute Kb || Sb,
  // kdf(z,0) to compute Ka || Sa, and kdf(z,2) to compute Kclient || Sclient.
  derive_key_material(&sts);

  // 5. B computes the ed25519 signature: sig = signQb(Pbe || Pae)
}


void finish_sts(struct link_device *dev, struct spprz_transport *trans, uint8_t *buf){

}

void spprz_dl_event(void)
{
  // check and parse incoming data
  spprz_check_and_parse(&DOWNLINK_DEVICE.device, &spprz_tp, dl_buffer, &dl_msg_available);

  if (dl_msg_available && (sts.protocol_stage != CRYPTO_OK)) {
    // process the unencrypted message
    spprz_process_sts_msg(&DOWNLINK_DEVICE.device, &spprz_tp, dl_buffer);
    dl_msg_available = false;
  }
  DlCheckAndParse(&DOWNLINK_DEVICE.device, &spprz_tp.trans_tx, dl_buffer, &dl_msg_available);
}

void spprz_process_sts_msg(struct link_device *dev, struct spprz_transport *trans, uint8_t *buf) {
  uint8_t sender_id = SenderIdOfPprzMsg(buf);
  uint8_t msg_id = IdOfPprzMsg(buf);


  if (sender_id != 0) {
    // process only messages from GCS
    return;
  }

  uint8_t msg_type = DL_KEY_EXCHANGE_GCS_msg_type(buf);

  switch (msg_id) {
    case DL_KEY_EXCHANGE_GCS:
      switch (sts.protocol_stage) {
        case WAIT_MSG1:
          if (msg_type == P_AE) {
            respond_sts(dev, trans, buf);
          }
          break;
        case WAIT_MSG3:
          if (msg_type == SIG) {
            finish_sts(dev, trans, buf);
          }
          break;
        default:
          // INIT, WAIT_MSG2, CRYPTO_OK
          // do nothing
          break;
      }
      break;
    default:
      // process only KEY_EXCHANGE for now
      break;
  }
}


void clear_ctx(struct gec_sts_ctx * ctx)
{
  memset(&ctx->theirPublicKeyEphemeral, 0, sizeof(struct gec_pubkey));
  memset(&ctx->myPrivateKeyEphemeral, 0, sizeof(struct gec_privkey));
  memset(&ctx->theirSymmetricKey, 0, sizeof(struct gec_sym_key));
  memset(&ctx->mySymmetricKey, 0, sizeof(struct gec_sym_key));
  ctx->protocol_stage = WAIT_MSG1;
  ctx->party = RESPONDER;
  ctx->last_error = ERROR_NONE;
}
