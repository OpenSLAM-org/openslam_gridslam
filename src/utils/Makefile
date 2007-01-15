NAME = utils
CONF  = ../Makefile.conf
RULES = ../Makefile.rules

##
## System config variables
##

include $(CONF)

# CFLAGS += -DNO_CARMEN_CORE

##
## Variables
##

DEPENDENCIES += *.c

##
## local build variables
##

OBJS := $(OBJ_DIR)/utils.o \
	$(OBJ_DIR)/mesh-utils.o \
	$(OBJ_DIR)/color.o \
	$(OBJ_DIR)/quaternions.o \
	$(OBJ_DIR)/mdalloc.o \
	$(OBJ_DIR)/read-script.o \
	$(OBJ_DIR)/read-rec2d.o \
	$(OBJ_DIR)/parse-lines.o \
	$(OBJ_DIR)/read-rec3d.o \
	$(OBJ_DIR)/comp-rec3d.o \
	$(OBJ_DIR)/comp-tri3d.o \
	$(OBJ_DIR)/comp-trigo.o \
	$(OBJ_DIR)/intersec-planes.o \
	$(OBJ_DIR)/comp-hull.o \
	$(OBJ_DIR)/read-spf.o \
	$(OBJ_DIR)/correction.o \
	$(OBJ_DIR)/read-planes.o \
	$(OBJ_DIR)/rotation.o

##
## Compulsory rules
##

all:
		$(MAKE) export $(DEPEND)
		$(MAKE) build

build: 		$(OBJ_DIR) \
		$(LIB_DIR) \
		$(LIB_DIR)/libutils.a

export:
	$(ECHO) "    ---- Installing header files"
	$(SILENT) $(MKDIR) $(NAV_DIR)
	$(INSTALL_H) `pwd`/utils.h $(NAV_DIR)
	$(INSTALL_H) `pwd`/geometry.h $(NAV_DIR)

clean:	
	$(SILENT) $(RM) *.o *.a *~ core a.out obj obj_sun obj_solaris $(CLEAN_FILES)

distclean: clean
	$(SILENT) $(RM) $(DEPEND) Makefile.depend* \
		$(NAME)-src.tgz $(NAME)-bin.tgz $(DISTCLEAN_FILES)

##
## local rules
##

$(OBJ_DIR) $(LIB_DIR) $(BIN_DIR):
	$(SILENT) $(MKDIR) $@

$(LIB_DIR)/libutils.a: $(OBJS)
	$(SILENT) $(AR) $@ $(OBJS)
	$(SILENT) $(RANLIB) $@

$(LIB_DIR)/libkdtree.a: $(KD_OBJS)
	$(SILENT) $(AR) $@ $(KD_OBJS)
	$(SILENT) $(RANLIB) $@

##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)
