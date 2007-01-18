include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = "GRID_FASTSLAM"
MODULE_COMMENT = "Grid based Fast SLAM by D. Haehnel"

CFLAGS 	+= 	-Wno-shadow
IFLAGS 	+=	-I.
LFLAGS 	+= 	-llogtools -lreadlog -lglobal -lipc

CXXFLAGS += -Wno-non-virtual-dtor

QT3_DIR = /usr/lib/qt3
IFLAGS += -I$(QT3_DIR)/include

ifeq ($(PROCESSOR),x86_64)
LFLAGS += -L$(QT3_DIR)/lib64
else
LFLAGS += -L$(QT3_DIR)/lib
endif


LFLAGS += -lqt-mt `/usr/bin/Magick-config --libs`

LINK  = g++

GUI_FILES = 		./graphics-moc.cpp

ADD_CLEAN += $(GUI_FILES)

SOURCES =		read-ini.c line.c utils.c graphics.cpp main.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 

MAN_PAGES = 

TARGETS = 		fast-slam

# rules

depend:		$(GUI_FILES) 

fast-slam:	$(GUI_FILES) read-ini.o line.o utils.o graphics.o \
		graphics-moc.o main.o

include ../Makefile.rules
