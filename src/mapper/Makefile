include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = "SCAN_MATCHER"
MODULE_COMMENT = "Grid based Scan Matcher by D. Haehnel"

CFLAGS 	+= 	-Wno-shadow
IFLAGS 	+=	-I.
LFLAGS 	+= 	-lglobal -lipc -llogtools -lreadlog

CXXFLAGS += -Wno-non-virtual-dtor

QT3_DIR = /usr/lib/qt3
IFLAGS += -I$(QT3_DIR)/include

ifeq ($(PROCESSOR),x86_64)
LFLAGS += -L$(QT3_DIR)/lib64
else
LFLAGS += -L$(QT3_DIR)/lib
endif


LFLAGS += -lqt-mt

GUI_FILES = 		./gui.h ./gui-moc.cpp ./gui.cpp ./mapview-moc.cpp 

ADD_CLEAN += $(GUI_FILES)

SOURCES =		line.c map.c read-ini.c match-map.c global-map.c \
			write-map.c map2d.c ipc.c run_file.c run_online.c \
			main.cpp mapview.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 

MAN_PAGES = 

TARGETS = 		ml-mapper

# rules

depend:		$(GUI_FILES) 

ml-mapper:	$(GUI_FILES) line.o map.o read-ini.o match-map.o global-map.o \
		write-map.o map2d.o ipc.o run_file.o run_online.o \
		main.o mapview-moc.o mapview.o gui-moc.o gui.o

include ../Makefile.rules
