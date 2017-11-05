/*
 * Copyright (C) 2017 Michal Podhradsky <mpodhradsky@galois.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** \file rng_arch.c
 *  \brief arch independent Random Number Generator API
 *
 */
#include "mcu_periph/rng.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/rng.h>

void rng_init(void) {
  rcc_periph_clock_enable(RCC_RNG);
  rng_enable();
}

void rng_deinit(void) {
  rng_disable();
  rcc_periph_clock_disable(RCC_RNG);
}

bool rng_get(uint32_t *rand_nr) {
  return rng_get_random(rand_nr);
}


