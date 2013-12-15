# Hey Emacs, this is a -*- makefile -*-

# ARDrone 2 Flightrecorder GPS unit

GPS_LED ?= none

$(TARGET).CFLAGS += -DUSE_GPS
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/gps.c

ap.CFLAGS += -DUSE_GPS_ARDRONE2
ap.CFLAGS += -DGPS_NB_CHANNELS=12

ifneq ($(GPS_LED),none)
  ap.CFLAGS += -DGPS_LED=$(GPS_LED)
endif

ap.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_ardrone2.h\"
ap.srcs   += $(SRC_SUBSYSTEMS)/gps/gps_ardrone2.c

nps.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_sim.h\"
nps.srcs += $(SRC_SUBSYSTEMS)/gps/gps_sim_nps.c

