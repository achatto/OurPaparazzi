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

  // generate ephemeral keys
  for (uint8_t i = 0; i < (PPRZ_KEY_LEN / sizeof(uint32_t)); i++) {
    uint32_t tmp = rng_wait_and_get();
    // Q_BE
    sts.myPrivateKey_ephemeral[i] = (uint8_t) tmp;
    sts.myPrivateKey_ephemeral[i + 1] = (uint8_t) (tmp >> 8);
    sts.myPrivateKey_ephemeral[i + 2] = (uint8_t) (tmp >> 16);
    sts.myPrivateKey_ephemeral[i + 3] = (uint8_t) (tmp >> 24);

    // P_BE
    tmp = rng_wait_and_get();
    sts.myPublicKey_ephemeral[i] = (uint8_t) tmp;
    sts.myPublicKey_ephemeral[i + 1] = (uint8_t) (tmp >> 8);
    sts.myPublicKey_ephemeral[i + 2] = (uint8_t) (tmp >> 16);
    sts.myPublicKey_ephemeral[i + 3] = (uint8_t) (tmp >> 24);
  }
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

bool spprz_is_comm_status_ok(void) {
  if (sts.protocol_stage == CRYPTO_OK) {
    return true;
  } else {
    return false;
  }
}

void spprz_process_sts_msg(struct link_device *dev, struct spprz_transport *trans, uint8_t *buf) {
  // TODO: just a dummy for now
  (void)dev;
  (void)trans;
  (void)buf;
}


void clear_ctx(struct gec_sts_ctx * ctx)
{
  memset(&ctx->theirPublicKey, 0, sizeof(struct gec_pubkey));
  memset(&ctx->myPrivateKey, 0, sizeof(struct gec_privkey));
  memset(&ctx->myPrivateKey_ephemeral, 0, PPRZ_KEY_LEN);
  memset(&ctx->theirPublicKey_ephemeral, 0, PPRZ_KEY_LEN);
  memset(ctx->client_key_material, 0, PPRZ_KEY_MATERIAL_LEN);
  ctx->protocol_stage = WAIT_MSG1;
  ctx->party = RESPONDER;
}

void reset_ctx(struct gec_sts_ctx * ctx)
{
  memset(&ctx->theirPublicKey, 0, sizeof(struct gec_pubkey));
  memset(&ctx->myPrivateKey, 0, sizeof(struct gec_privkey));
  memset(&ctx->myPrivateKey_ephemeral, 0, PPRZ_KEY_LEN);
  memset(&ctx->theirPublicKey_ephemeral, 0, PPRZ_KEY_LEN);
  memset(ctx->client_key_material, 0, PPRZ_KEY_MATERIAL_LEN);
  ctx->protocol_stage = WAIT_MSG1;
  ctx->party = RESPONDER;
}
