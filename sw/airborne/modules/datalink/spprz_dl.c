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
#include "datalink/hacl-c/SHA2_512.h"
#include "datalink/hacl-c/Chacha20Poly1305.h"

#include "subsystems/datalink/telemetry.h"

#include <string.h> // for memcpy

struct spprz_transport spprz_tp;
struct gec_sts_ctx sts;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_spprz_info(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_SPRRZ_STATUS(trans, dev, AC_ID,
      &sts.protocol_stage,
      &sts.last_error);

}

#endif


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

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SPRRZ_STATUS, send_spprz_info);
#endif
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
 * Decrypt a message, no AAD for now
 */
uint32_t gec_decrypt(struct gec_sym_key *k, uint8_t *plaintext, uint8_t *ciphertext, size_t len, uint8_t *mac){
  uint32_t res = Chacha20Poly1305_aead_decrypt(plaintext,
                                               ciphertext,
                                               len,
                                               mac,
                                               NULL,
                                               0,
                                               k->key,
                                               k->nonce);
  return res;
}

/**
 * Encrypt a message, no AAD for now
 */
uint32_t gec_encrypt(struct gec_sym_key *k, uint8_t *ciphertext, uint8_t *plaintext, size_t len, uint8_t *mac) {
  uint32_t res = Chacha20Poly1305_aead_encrypt(ciphertext,  // ciphertext
                                               mac,  // mac
                                               plaintext,  // plaintext
                                               len,  // plaintext len
                                               NULL,  // aad
                                               0,  // aad len
                                               k->key,  // key
                                               k->nonce);  // nonce
  return res;
}

/**
 * message1 = [ Pae (32 bytes) ]
 * 2. B generates ephemeral curve25519 key pair (Pbe, Qbe).
 * 3. B computes the shared secret: z = scalar_multiplication(Qbe, Pae)
 * 4. B uses the key derivation function kdf(z,1) to compute Kb || Sb,
 * kdf(z,0) to compute Ka || Sa, and kdf(z,2) to compute Kclient || Sclient.
 * 5. B computes the ed25519 signature: sig = signQb(Pbe || Pae)
 * 6. B computes and sends the message Pbe || Ekey=Kb,IV=Sb||zero(sig)
 *
 * Sends: message2 = [ Pbe (32 bytes) | Encrypted Signature (64 bytes) ] + MAC (16bytes)
 */
void respond_sts(struct link_device *dev, struct spprz_transport *trans, uint8_t *buf) {
  // TODO: check message length

  // copy P_ae over
  memcpy(&sts.theirPublicKeyEphemeral.pub, DL_KEY_EXCHANGE_GCS_msg_data(buf), sizeof(struct gec_pubkey));

  // 2. B generates ephemeral curve25519 key pair (Pbe, Qbe).
  generate_ephemeral_keys(&sts.myPrivateKeyEphemeral);

  // 3. B computes the shared secret: z = scalar_multiplication(Qbe, Pae)
  uint8_t z[32] = {0};
  Curve25519_crypto_scalarmult(z, sts.myPrivateKeyEphemeral.priv, sts.theirPublicKeyEphemeral.pub);

  // 4. B uses the key derivation function kdf(z,1) to compute Kb || Sb,
  // kdf(z,0) to compute Ka || Sa, and kdf(z,2) to compute Kclient || Sclient.
  derive_key_material(&sts, z);

  // 5. B computes the ed25519 signature: sig = signQb(Pbe || Pae)
  uint8_t sig[PPRZ_SIGN_LEN] = {0};
  uint8_t pbe_concat_p_ae[PPRZ_KEY_LEN*2] = {0};
  memcpy(pbe_concat_p_ae, &sts.myPrivateKeyEphemeral.pub, PPRZ_KEY_LEN);
  memcpy(&pbe_concat_p_ae[PPRZ_KEY_LEN], &sts.theirPublicKeyEphemeral.pub, PPRZ_KEY_LEN);
  Ed25519_sign(sig, sts.myPrivateKey.priv, pbe_concat_p_ae, PPRZ_KEY_LEN*2);

  // 6. B computes and sends the message Pbe || Ekey=Kb,IV=Sb||zero(sig)
  uint8_t msg_data[PPRZ_KEY_LEN + PPRZ_SIGN_LEN + PPRZ_MAC_LEN] = {0};
  memcpy(msg_data, &sts.myPrivateKeyEphemeral.pub, PPRZ_KEY_LEN);
  if (gec_encrypt(&sts.mySymmetricKey, &msg_data[PPRZ_KEY_LEN], sig, PPRZ_SIGN_LEN, &msg_data[PPRZ_KEY_LEN + PPRZ_SIGN_LEN])) {
    // log error here and return
    sts.last_error = MSG1_ENCRYPT_ERROR;
    return;
  }
  // all good, send message and increment status
  uint8_t msg_type = P_BE;
  uint8_t nb_msg_data = PPRZ_KEY_LEN + PPRZ_SIGN_LEN + PPRZ_MAC_LEN;
  pprz_msg_send_KEY_EXCHANGE_UAV(&trans->trans_tx, dev, AC_ID, &msg_type, nb_msg_data, msg_data);  // enqueue message
  spprz_send_plaintext(dev, trans);  // send plaintext

  // update protocol stage
  sts.protocol_stage = WAIT_MSG3;
  // TODO: add timeout
}

/**
 * Finalize STS exchange
 *
 * message3 = [ Encrypted Signature (64 bytes) ] + MAC(16 bytes) = Ekey=Ka,IV=Sa||zero(sig)
 * where sig = signQa(Pae || Pbe)
 * 13. B decrypts the message and verifies the signature.
 */
void finish_sts(struct link_device *dev __attribute__((__unused__)),
                struct spprz_transport *trans __attribute__((__unused__)),
                uint8_t *buf){
  // TODO: check message length

  // decrypt
  uint8_t sign[PPRZ_SIGN_LEN] = {0};
  if (gec_decrypt(&sts.theirSymmetricKey, sign, DL_KEY_EXCHANGE_GCS_msg_data(buf),
      PPRZ_SIGN_LEN, DL_KEY_EXCHANGE_GCS_msg_data(buf)+PPRZ_SIGN_LEN)) {
    // log error and return
    // MSG3 decrypt error
    sts.last_error = MSG3_DECRYPT_ERROR;
    return;
  }

  // verify
  uint8_t pbe_concat_p_ae[PPRZ_KEY_LEN*2] = {0};
  memcpy(pbe_concat_p_ae, &sts.myPrivateKeyEphemeral.pub, PPRZ_KEY_LEN);
  memcpy(&pbe_concat_p_ae[PPRZ_KEY_LEN], &sts.theirPublicKeyEphemeral.pub, PPRZ_KEY_LEN);
  if (!Ed25519_verify(sts.theirPublicKeyEphemeral.pub, pbe_concat_p_ae, PPRZ_SIGN_LEN, sign)) {
    // log error and return
    // MSG3 sign verify error
    sts.last_error = MSG3_SIGNVERIFY_ERROR;
  }

  // if everything went OK, proceed to CRYPTO_OK status
  sts.protocol_stage = CRYPTO_OK;
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
    // log an error
    return;
  }

  uint8_t msg_type = DL_KEY_EXCHANGE_GCS_msg_type(buf);

  switch (msg_id) {
    case DL_KEY_EXCHANGE_GCS:
      switch (sts.protocol_stage) {
        case WAIT_MSG1:
          if (msg_type == P_AE) {
            respond_sts(dev, trans, buf);
          } else {
            // log an error
            sts.last_error = UNEXPECTED_MSG_TYPE_ERROR;
          }
          break;
        case WAIT_MSG3:
          if (msg_type == SIG) {
            finish_sts(dev, trans, buf);
          } else {
            // log an error
            sts.last_error = UNEXPECTED_MSG_TYPE_ERROR;
          }
          break;
        default:
          // INIT, WAIT_MSG2, CRYPTO_OK
          // do nothing or log an error
          sts.last_error = UNEXPECTED_STS_STAGE_ERROR;
          break;
      }
      break;
    default:
      // process only KEY_EXCHANGE for now
      // log an error
      sts.last_error = UNEXPECTED_MSG_ERROR;
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
