/*
 * Copyright (C) 2013 Felix Ruess <felix.ruess@gmail.com>
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

/**
 * @file subsystems/abi_sender_ids.h
 *
 * Convenience defines for ABI sender IDs.
 */

#ifndef ABI_SENDER_IDS_H
#define ABI_SENDER_IDS_H

/** default onboard baro */
#ifndef BARO_BOARD_SENDER_ID
#define BARO_BOARD_SENDER_ID 1
#endif

/*
 * IDs of baro modules that can be loaded
 */
#ifndef BARO_MS5611_SENDER_ID
#define BARO_MS5611_SENDER_ID 10
#endif

#ifndef BARO_AMSYS_SENDER_ID
#define BARO_AMSYS_SENDER_ID 11
#endif

#ifndef BARO_BMP_SENDER_ID
#define BARO_BMP_SENDER_ID 12
#endif

#ifndef BARO_ETS_SENDER_ID
#define BARO_ETS_SENDER_ID 13
#endif

#ifndef BARO_MS5534A_SENDER_ID
#define BARO_MS5534A_SENDER_ID 14
#endif

#ifndef BARO_HCA_SENDER_ID
#define BARO_HCA_SENDER_ID 15
#endif

#ifndef BARO_MPL3115_SENDER_ID
#define BARO_MPL3115_SENDER_ID 16
#endif

#ifndef BARO_SCP_SENDER_ID
#define BARO_SCP_SENDER_ID 17
#endif

#ifndef BARO_PBN_SENDER_ID
#define BARO_PBN_SENDER_ID 18
#endif

#ifndef BARO_SIM_SENDER_ID
#define BARO_SIM_SENDER_ID 19
#endif

/*
 * IDs of gps subsystems and modules
 */
#ifndef GPS_UBX_SENDER_ID
#define GPS_UBX_SENDER_ID 1
#endif

#ifndef GPS_NMEA_SENDER_ID
#define GPS_NMEA_SENDER_ID 2
#endif

#ifndef GPS_SIRF_SENDER_ID
#define GPS_SIRF_SENDER_ID 3
#endif

#ifndef GPS_MTK_SENDER_ID
#define GPS_MTK_SENDER_ID 4
#endif

#ifndef GPS_SKYTRAQ_SENDER_ID
#define GPS_SKYTRAQ_SENDER_ID 5
#endif

#ifndef GPS_SIRF_SENDER_ID
#define GPS_SIRF_SENDER_ID 6
#endif

#ifndef GPS_ARDRONE2_SENDER_ID
#define GPS_ARDRONE2_SENDER_ID 7
#endif

#ifndef GPS_SIM_SENDER_ID
#define GPS_SIM_SENDER_ID 8
#endif

#endif /* ABI_SENDER_IDS_H */
