/*
 * Copyright (C) 2010  ENAC
 * Copyright (C) 2016  2016 Michal Podhradsky <http://github.com/podhrmic>
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
 *
 */

#ifndef CRYPTO_H
#define CRYPTO_H

#include "gec.h"
#include "gec-ke.h"
#include "std.h"

enum CryptoStatus {
  CRYPTO_WAITING_FOR_MSG1,
  CRYPTO_WAITING_FOR_MSG2,
  CRYPTO_OK,
  CRYPTO_ERROR,
};

struct crypto_status {
  bool connected;
  enum CryptoStatus status;
};

extern struct crypto_status crypto;

bool crypto_decrypt_and_authenticate(uint8_t *buf);
bool crypto_encrypt(uint8_t *buf);
bool crypto_key_exchange(uint8_t * buf);


#endif /* CRYPTO_H */

