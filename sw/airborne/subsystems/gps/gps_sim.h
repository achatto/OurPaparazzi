#ifndef GPS_SIM_H
#define GPS_SIM_H

#include "std.h"

extern bool_t gps_available;

//extern void gps_feed_values(double utm_north, double utm_east, double utm_alt, double gspeed, double course, double climb);

//extern void gps_impl_init(void);

extern void GpsEvent(void);
#if 0
#define GpsEvent(_sol_available_callback) {     \
    if (gps_available) {                        \
      if (_gps->fix == GPS_FIX_3D) {            \
        _gps->last_fix_time = sys_time.nb_sec;  \
      }                                         \
      _sol_available_callback();                \
      gps_available = FALSE;                    \
    }                                           \
  }
#endif

#endif /* GPS_SIM_H */
