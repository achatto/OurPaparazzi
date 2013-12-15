/*
 * Copyright (C) 2008-2011 The Paparazzi Team
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
 */

/** @file gps.c
 *  @brief Device independent GPS code
 *
 */

#include "subsystems/gps.h"

#include "led.h"

#define MSEC_PER_WEEK (1000*60*60*24*7)

struct GpsState * gps;

#if DOWNLINK
#include "subsystems/datalink/telemetry.h"

static void send_gps(void) {
  static uint8_t i;
  int16_t climb = -_gps->ned_vel.z;
  int16_t course = (DegOfRad(_gps->course)/((int32_t)1e6));
  DOWNLINK_SEND_GPS(DefaultChannel, DefaultDevice, &_gps->fix,
      &_gps->utm_pos.east, &_gps->utm_pos.north,
      &course, &_gps->hmsl, &_gps->gspeed, &climb,
      &_gps->week, &_gps->tow, &_gps->utm_pos.zone, &i);
  if ((_gps->fix != GPS_FIX_3D) && (i >= _gps->nb_channels)) i = 0;
  if (i >= _gps->nb_channels * 2) i = 0;
  if (i < _gps->nb_channels && ((_gps->fix != GPS_FIX_3D) || (_gps->svinfos[i].cno > 0))) {
    DOWNLINK_SEND_SVINFO(DefaultChannel, DefaultDevice, &i,
        &_gps->svinfos[i].svid, &_gps->svinfos[i].flags,
        &_gps->svinfos[i].qi, &_gps->svinfos[i].cno,
        &_gps->svinfos[i].elev, &_gps->svinfos[i].azim);
  }
  i++;
}

static void send_gps_int(void) {
  static uint8_t i;
  static uint8_t last_cnos[GPS_NB_CHANNELS];
  DOWNLINK_SEND_GPS_INT(DefaultChannel, DefaultDevice,
      &_gps->ecef_pos.x, &_gps->ecef_pos.y, &_gps->ecef_pos.z,
      &_gps->lla_pos.lat, &_gps->lla_pos.lon, &_gps->lla_pos.alt,
      &_gps->hmsl,
      &_gps->ecef_vel.x, &_gps->ecef_vel.y, &_gps->ecef_vel.z,
      &_gps->pacc, &_gps->sacc,
      &_gps->tow,
      &_gps->pdop,
      &_gps->num_sv,
      &_gps->fix);
  if (i == _gps->nb_channels) i = 0;
  if (i < _gps->nb_channels && _gps->svinfos[i].cno > 0 && _gps->svinfos[i].cno != last_cnos[i]) {
    DOWNLINK_SEND_SVINFO(DefaultChannel, DefaultDevice, &i,
        &_gps->svinfos[i].svid, &_gps->svinfos[i].flags,
        &_gps->svinfos[i].qi, &_gps->svinfos[i].cno,
        &_gps->svinfos[i].elev, &_gps->svinfos[i].azim);
    last_cnos[i] = _gps->svinfos[i].cno;
  }
  i++;
}

static void send_gps_lla(void) {
  uint8_t err = 0;
  int16_t climb = -_gps->ned_vel.z;
  int16_t course = (DegOfRad(_gps->course)/((int32_t)1e6));
  DOWNLINK_SEND_GPS_LLA(DefaultChannel, DefaultDevice,
      &_gps->lla_pos.lat, &_gps->lla_pos.lon, &_gps->lla_pos.alt,
      &course, &_gps->gspeed, &climb,
      &_gps->week, &_gps->tow,
      &_gps->fix, &err);
}

static void send_gps_sol(void) {
  DOWNLINK_SEND_GPS_SOL(DefaultChannel, DefaultDevice, &_gps->pacc, &_gps->sacc, &_gps->pdop, &_gps->num_sv);
}
#endif

void gps_init(void) {
  _gps = NULL;
#ifdef GPS_LED
  LED_OFF(GPS_LED);
#endif
#ifdef GPS_TYPE_H
  _gps = gps_impl_init();
#endif
  if (_gps == NULL) return; // Return if _gps is not pointing to a gps struct
  // force to GPS_FIX_NONE
  _gps->fix = GPS_FIX_NONE;
  _gps->cacc = 0;

#if DOWNLINK
  register_periodic_telemetry(DefaultPeriodic, "GPS", send_gps);
  register_periodic_telemetry(DefaultPeriodic, "GPS_INT", send_gps_int);
  register_periodic_telemetry(DefaultPeriodic, "GPS_LLA", send_gps_lla);
  register_periodic_telemetry(DefaultPeriodic, "GPS_SOL", send_gps_sol);
#endif
}

uint32_t gps_tow_from_sys_ticks(struct GpsTimeSync * gps_time, uint32_t sys_ticks)
{
  uint32_t clock_delta;
  uint32_t time_delta;
  uint32_t itow_now;

  if (sys_ticks < gps_time->t0_ticks) {
    clock_delta = (0xFFFFFFFF - sys_ticks) + gps_time->t0_ticks + 1;
  } else {
    clock_delta = sys_ticks - gps_time->t0_ticks;
  }

  time_delta = msec_of_sys_time_ticks(clock_delta);

  itow_now = gps_time->t0_tow + time_delta;
  if (itow_now > MSEC_PER_WEEK) {
    itow_now %= MSEC_PER_WEEK;
  }

  return itow_now;
}
