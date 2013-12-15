# Hey Emacs, this is a -*- makefile -*-

GPS_LED ?= none

$(TARGET).CFLAGS += -DUSE_GPS
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/gps.c

ap.CFLAGS += -DGPS_LINK=$(GPS_PORT)
ap.CFLAGS += -DUSE_$(GPS_PORT)
ap.CFLAGS += -D$(GPS_PORT)_BAUD=$(GPS_BAUD)

ifneq ($(GPS_LED),none)
  ap.CFLAGS += -DGPS_LED=$(GPS_LED)
endif

ap.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_skytraq.h\"
ap.srcs += $(SRC_SUBSYSTEMS)/gps/gps_skytraq.c

nps.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_sim_nps.h\"
nps.srcs += $(SRC_SUBSYSTEMS)/gps/gps_sim_nps.c
