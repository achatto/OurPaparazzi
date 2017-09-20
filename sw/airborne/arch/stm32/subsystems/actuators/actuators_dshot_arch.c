/*
 * Copyright (C) 2010 The Paparazzi Team
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
 */

/** @file arch/stm32/subsystems/actuators/actuators_dualpwm_arch.c
 *  STM32 dual PWM servos handling.
 */

//VALID TIMERS IS TIM5 ON THE LISA/M

#include "subsystems/actuators/actuators_shared_arch.h"
#include "subsystems/actuators/actuators_dshot_arch.h"
#include "subsystems/actuators/actuators_dshot.h"

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

#include "mcu_periph/gpio_arch.h"



int32_t actuators_dshot_values[ACTUATORS_DSHOT_NB];

void actuators_dshot_arch_init(void) {

}

void actuators_dshot_commit(void) {

}

