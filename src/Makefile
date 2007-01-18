SILENT = @
ECHO = @echo

PACKAGES = mapper fast-slam

RECURSE = $(SILENT) \
	for i in $(PACKAGES) xxxx ; do \
		if [ -d $$i ] ; then \
			if ! $(MAKE) -C $$i $@ ; then \
                                exit -1; \
                        fi; \
		fi \
	done

ifndef CARMEN_HOME
  $(warning )
  $(warning The CARMEN_HOME is not set.)
  $(warning )
  $(warning This package needs CARMEN. Please install CARMEN)
  $(warning and set the CARMEN_HOME environment variable to)
  $(warning the CARMEN directory.)
  $(warning )
  $(warning for more information about CARMEN)
  $(warning see http://carmen.sourceforge.net/)
  $(warning )
  $(error )
endif


all:	phase1 phase2

phase1:
	$(ECHO)
	$(ECHO) "       *****************"
	$(ECHO) "       L I B R A R I E S"
	$(ECHO) "       *****************"
	$(ECHO)
	$(RECURSE)

phase2:
	$(ECHO)
	$(ECHO) "       ***************"
	$(ECHO) "       B I N A R I E S"
	$(ECHO) "       ***************"
	$(ECHO)
	$(RECURSE)
	$(ECHO) "Done making binaries..."

clean:
	$(ECHO)
	$(ECHO) "       *********"
	$(ECHO) "       C L E A N"
	$(ECHO) "       *********"
	$(ECHO)
	$(RECURSE)

distclean:
	$(ECHO)
	$(ECHO) "       *****************"
	$(ECHO) "       D I S T C L E A N"
	$(ECHO) "       *****************"
	$(ECHO)
	$(RECURSE)
