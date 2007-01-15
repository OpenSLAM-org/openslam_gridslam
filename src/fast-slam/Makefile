NAME  = transformer
CONF  = ../Makefile.conf
RULES = ../Makefile.rules

##
## System config variables
##

include $(CONF)

##
## Variables
##

DEPENDENCIES += *.cpp

INCLUDES    += -I$(QT_DIR)/include

LIB_DIRS    += -L$(QT_DIR)/lib

MOC         = $(QT_DIR)/bin/moc

# CXXFLAGS    += -DDEBUG

##
## local build variables
##

OBJS := \
	$(OBJ_DIR)/read-ini.o \
	$(OBJ_DIR)/line.o \
	$(OBJ_DIR)/utils.o \
	$(OBJ_DIR)/graphics.o \
	$(OBJ_DIR)/graphics-moc.o \
	$(OBJ_DIR)/main.o

LIBS := $(EXTERN_LIBS) -lqt -lutils -lm

##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

build:	$(OBJ_DIR) \
	$(BIN_DIR) \
	$(LIB_DIR) \
	$(BIN_DIR)/fast-slam

export:

clean:
	$(RM) *.o *.a *~ core a.out obj obj_sun obj_solaris $(CLEAN_FILES)

distclean: clean
	$(RM) $(DEPEND) Makefile.depend* \
		$(NAME)-src.tgz $(NAME)-bin.tgz $(DISTCLEAN_FILES)

##
## local rules
##

$(OBJ_DIR) $(LIB_DIR) $(BIN_DIR):
	$(SILENT) $(MKDIR) $@

$(BIN_DIR)/fast-slam: $(OBJ_DIR) $(OBJS) 
	$(ECHO) "    ->>- Creating $@"
		$(SILENT) $(CXX) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJS) $(LIBS)

##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)


