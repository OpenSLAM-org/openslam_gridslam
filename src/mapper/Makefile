1NAME  = mapper
CONF  = ../Makefile.conf
RULES = ../Makefile.rules

##
## System config variables
##

include $(CONF)

##
## Variables
##

DEPENDENCIES += *.c *.cpp *.h

CFLAGS      += -DVERBOSE

INCLUDES    += -I$(QT_DIR)/include

LIB_DIRS    += -L$(QT_DIR)/lib

MOC         = $(QT_DIR)/bin/moc

# SILENT =

##
## local build variables
##

GUI_DIR = .

GUI_PARTS := \
	$(GUI_DIR)/gui.h \
	$(GUI_DIR)/gui.cpp \
	$(GUI_DIR)/gui-moc.cpp \
	$(GUI_DIR)/gui-2.3-moc.cpp \
	$(GUI_DIR)/mapview-moc.cpp

ifeq ($(QT3),yes)
QT_OBJS := \
	$(OBJ_DIR)/gui-moc.o \
	$(OBJ_DIR)/gui.o
else
QT_OBJS := \
        $(OBJ_DIR)/gui-2.3-moc.o \
        $(OBJ_DIR)/gui-2.3.o
endif

OBJS := \
	$(QT_OBJS) \
	$(OBJ_DIR)/mapview.o \
	$(OBJ_DIR)/mapview-moc.o \
	$(OBJ_DIR)/line.o \
	$(OBJ_DIR)/map.o \
	$(OBJ_DIR)/read-ini.o \
	$(OBJ_DIR)/match-map.o \
	$(OBJ_DIR)/global-map.o \
	$(OBJ_DIR)/write-map.o \
	$(OBJ_DIR)/map2d.o \
	$(OBJ_DIR)/ipc.o \
	$(OBJ_DIR)/run_file.o \
	$(OBJ_DIR)/run_online.o \
	$(OBJ_DIR)/main.o

LIBS := $(EXTERN_LIBS) -lglobal -lipc -lqt -lutils -lm

##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

build:	$(OBJ_DIR) \
	$(BIN_DIR) \
	show-info \
	$(BIN_DIR)/mapper

export:
	$(SILENT) $(MKDIR) $(NAV_DIR)
	$(INSTALL_H) `pwd`/mapper_messages.h $(NAV_DIR)

CLEAN_FILES += $(GUI_PARTS)

clean:
	$(SILENT) $(RM) *.o *.a *~ core a.out obj obj_sun obj_solaris $(CLEAN_FILES)

distclean: clean
	$(SILENT) $(RM) $(DEPEND) Makefile.depend* $(DISTCLEAN_FILES)

##
## local rules
##

$(OBJ_DIR) $(LIB_DIR) $(BIN_DIR):
	$(SILENT) $(MKDIR) $@

show-info: 
	$(ECHO) "============================================="
	$(ECHO) "-> found QT version    " $(QT_VERSION)
	$(ECHO) "-> in directory        " $(QT_DIR)
	$(ECHO) "-> which is QT>=3.0.0  " $(QT3)
	$(ECHO) "============================================="

$(BIN_DIR)/mapper: $(OBJ_DIR) $(OBJS)
	$(ECHO) "    ->>- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJS) $(LIBS)

##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)


