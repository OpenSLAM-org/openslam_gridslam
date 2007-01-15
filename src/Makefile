NAME  = navigator

CONF  = Makefile.conf
RULES = Makefile.rules
VARS  = Makefile.vars

include $(CONF)

TAR_BIN = include lib bin etc doc

PACKAGES = 	utils mapper fast-slam

##
## Corrupt modules, not compilable
##
## RTLClientServer (if no GL installed)
## multiNavigation (no Makefile)
##


RECURSE = $(SILENT) \
	echo "Considering $(PACKAGES)" ; \
	for i in $(PACKAGES) ; do \
		if [ -d $$i ] ; then \
			if ( cd $$i ; echo "    MODULE:"  $$i ; echo; \
				${MAKE} $@ ; ) then \
				echo ; echo ; \
			else \
				exit 1 ; \
			fi ; \
		fi \
	done


##
## Compulsory rules
##
## 'make linux' and 'make solaris' create OS specific modules AND
## common modules compilable and runnable on both OS types.
##
## 'make linux_only' and 'make solaris_only' create ONLY OS specific modules.
##

VARS = Makefile.vars

all:  config export $(DEPEND) build success

config:
	$(ECHO)
	$(ECHO) "       C O N F I G"
	$(ECHO)
	$(shell echo "# flags created by make config ..." > $(VARS))
	$(shell if [ -x $(GLIB_CONFIG) ]; then \
		echo "CFLAGS += -DUSE_GLIB" >> $(VARS); \
		echo "GLIB_CFLAGS     = `$(GLIB_CONFIG) --cflags`" >> $(VARS); \
		echo "GLIB_LIBS       = `$(GLIB_CONFIG) --libs`" >> $(VARS); \
	fi;)
	$(shell if [ -x $(GSL_CONFIG) ]; then \
		echo "CFLAGS += -DUSE_GSL" >> $(VARS); \
		echo "GSL_CFLAGS      = `$(GSL_CONFIG) --cflags`" >> $(VARS); \
		echo "GSL_LIBS        = `$(GSL_CONFIG) --libs`" >> $(VARS); \
	fi;)
	$(shell if [ -x $(GTS_CONFIG) ]; then \
		echo "CFLAGS += -DUSE_GTS" >> $(VARS); \
		echo "GTS_CFLAGS      = `$(GTS_CONFIG) --cflags`" >> $(VARS); \
		echo "GTS_LIBS        = `$(GTS_CONFIG) --libs`" >> $(VARS); \
	fi;)
	$(shell if [ -x $(MAGICK_CONFIG) ]; then \
		echo "CFLAGS += -DUSE_MAGICK" >> $(VARS); \
		echo "MAGICK_CFLAGS   = `$(MAGICK_CONFIG) --cflags `" >> $(VARS); \
		echo "MAGICK_CXXFLAGS = `$(MAGICK_CONFIG) --cppflags `" >> $(VARS); \
		echo "MAGICK_LIBS     = `$(MAGICK_CONFIG) --libs`" >> $(VARS); \
	fi;)

build:
	$(ECHO)
	$(ECHO) "       B U I L D"
	$(ECHO)
	$(RECURSE)

export:
	$(ECHO)
	$(ECHO) "       E X P O R T"
	$(ECHO)
	$(RECURSE)

clean:
	$(ECHO)
	$(ECHO) "       C L E A N"
	$(ECHO)
	$(RECURSE)
	$(RM) *~

distclean:
	$(ECHO)
	$(ECHO) "       D I S T     C L E A N"
	$(ECHO)
	$(RECURSE)
	$(RM) ../include ../lib* ../bin* ../etc ../log \
		$(NAME)-src.tgz $(NAME)-bin.tgz *~ Makefile.vars

dep $(DEPEND) depend: 
	$(ECHO)
	$(ECHO) "       D E P E N D"
	$(ECHO)
	$(RECURSE)

success:
	$(ECHO)
	$(ECHO) "Software successfully compiled."
