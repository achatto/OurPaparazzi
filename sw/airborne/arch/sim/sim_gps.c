/* OCaml binding to link the simulator to autopilot functions. */

#include <assert.h>
#include <math.h>
#include <inttypes.h>

/** From airborne/autopilot/ */
#include "generated/airframe.h"
#include "generated/flight_plan.h"
#include "subsystems/gps.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_geodetic_int.h"

// currently needed for nav_utm_zone0
#include "subsystems/navigation/common_nav.h"

#include <caml/mlvalues.h>


value sim_use_gps_pos(value x, value y, value z, value c, value a, value s, value cl, value t, value m, value lat, value lon) {
  _gps->fix = (Bool_val(m) ? 3 : 0);
  _gps->course = Double_val(c) * 1e7;
  _gps->hmsl = Double_val(a) * 1000.;
  _gps->gspeed = Double_val(s) * 100.;
  _gps->ned_vel.x = _gps->gspeed * cos(Double_val(c));
  _gps->ned_vel.y = _gps->gspeed * sin(Double_val(c));
  _gps->ned_vel.z = -Double_val(cl) * 100.;
  _gps->week = 0; // FIXME
  _gps->tow = Double_val(t) * 1000.;

  //TODO set alt above ellipsoid and hmsl

#ifdef GPS_USE_LATLONG
  struct LlaCoor_f lla_f;
  struct UtmCoor_f utm_f;
  lla_f.lat = Double_val(lat);
  lla_f.lon = Double_val(lon);
  lla_f.alt = Double_val(a);
  utm_f.zone = nav_utm_zone0;
  utm_of_lla_f(&utm_f, &lla_f);
  LLA_BFP_OF_REAL(_gps->lla_pos, lla_f);
  _gps->utm_pos.east = utm_f.east*100;
  _gps->utm_pos.north = utm_f.north*100;
  _gps->utm_pos.zone = nav_utm_zone0;
  x = y = z; /* Just to get rid of the "unused arg" warning */
  y = x;     /* Just to get rid of the "unused arg" warning */
#else // GPS_USE_LATLONG
  _gps->utm_pos.east = Int_val(x);
  _gps->utm_pos.north = Int_val(y);
  _gps->utm_pos.zone = Int_val(z);
  lat = lon; /* Just to get rid of the "unused arg" warning */
  lon = lat; /* Just to get rid of the "unused arg" warning */
#endif // GPS_USE_LATLONG


  /** Space vehicle info simulation */
  _gps->nb_channels = GPS_NB_CHANNELS;
  int i;
  static int time;
  time++;
  for(i = 0; i < 1/*_gps->nb_channels*/; i++) {
    _gps->svinfos[i].svid = 7 + i;
    _gps->svinfos[i].elev = (cos(((100*i)+time)/100.) + 1) * 45;
    _gps->svinfos[i].azim = (time/_gps->nb_channels + 50 * i) % 360;
    _gps->svinfos[i].cno = 40 + sin((time+i*10)/100.) * 10.;
    _gps->svinfos[i].flags = ((time/10) % (i+1) == 0 ? 0x00 : 0x01);
    _gps->svinfos[i].qi = (int)((time / 1000.) + i) % 8;
  }
  _gps->pdop = _gps->sacc = _gps->pacc = 500+200*sin(time/100.);
  _gps->num_sv = 7;

  //gps_verbose_downlink = !launch;
  //gps_downlink();

  gps_available = TRUE;

  return Val_unit;
}

/* Second binding required because number of args > 5 */
value sim_use_gps_pos_bytecode(value *a, int argn) {
  assert(argn == 11);
  return sim_use_gps_pos(a[0],a[1],a[2],a[3],a[4],a[5],a[6],a[7], a[8],a[9], a[10]);
}

void ubxsend_cfg_rst(uint16_t a __attribute__ ((unused)), uint8_t b __attribute__ ((unused))) {
  return;
}

