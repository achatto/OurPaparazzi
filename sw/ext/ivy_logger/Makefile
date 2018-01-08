os=$(shell uname -s)

ifeq ($(os),Darwin)
    IVY_PATH := $(HOME)/src-ext/ivy-c-3.6/src
    CPPFLAGS = -MMD  -I$(IVY_PATH)
    DSO_EXT = .dylib
    LD = g++ -dynamiclib -single_module
    LDFLAGS = -L$(IVY_PATH) -L/sw/lib -framework OpenGL -framework AGL -framework Carbon
else
	LIB:= lib
    CPPFLAGS = -I/usr/local/include/Ivy
    DSO_EXT = .so
    LD = g++ -shared -fPIC
    #PREFIX=/usr/local
    ifndef PREFIX
        PREFIX=/usr
        LDFLAGS = -L$(PREFIX)/$(LIB)
    else
        LDFLAGS = -L$(PREFIX)/$(LIB)/ivy
    endif
endif

CASESENSITIVE = "yes"
ifeq ($(CASESENSITIVE), "no")
	PCRE_OPT=PCRE_CASELESS
	REGCOMP_OPT=REG_ICASE
else
	PCRE_OPT=0
	REGCOMP_OPT=REG_EXTENDED	
endif
REGEXP= -DUSE_PCRE_REGEX -DPCRE_OPT=$(PCRE_OPT)
PCREINC = $(shell pcre-config --cflags)


CXXFILES := Ivy.cxx IvyApplication.cxx
OBJECTS := $(CXXFILES:.cxx=.o)
DEPS := $(CXXFILES:.cxx=.d)

LD_QUICKSTART_INFO=
GCXXINCS= 

LIBIVY = libIvy

LIBIVY_STATIC = $(LIBIVY).a
LIBIVY_SHARED = $(LIBIVY)$(DSO_EXT)
LIBIVY_DEPLIBS = -livy

ifdef DEBUG
    CC= g++ -fPIC -g -Wall -pg $(CPPFLAGS)
    DBG=debug
else
    CC= g++ -fPIC -O2  -Wall $(CPPFLAGS)
    DBG=
endif

default: $(LIBIVY_SHARED) \
	 ivyLogger

%.o: %.cxx
	$(CC) -c $<

$(LIBIVY_STATIC) : $(OBJECTS)
	ar rv $(LIBIVY_STATIC) $(OBJECTS)
	ranlib $(LIBIVY_STATIC)

$(LIBIVY_SHARED) : $(OBJECTS)
	$(LD) -o $(LIBIVY_SHARED) $(OBJECTS) $(LDFLAGS) $(LIBIVY_DEPLIBS)

LLDLIBS = -L. -Wl,-rpath,. -L$(IVY_PATH)/src -Wl,-rpath,/usr/local/lib64 -L/usr/local/lib64

ivyLogger : IvyLogger.cxx
	g++  -g  $(CPPFLAGS) $(LLDLIBS) -std=c++11 -o $@  IvyLogger.cxx -lIvy -pthread

clean : 
	rm -f $(LIBIVY_STATIC) $(LIBIVY_SHARED) \
		$(OBJECTS) $(DEPS) \
		core *.o *.d *~ *.moc ivyLogger

-include *.d

