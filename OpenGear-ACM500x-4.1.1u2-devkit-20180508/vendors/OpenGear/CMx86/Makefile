#
# We are just a CD with some extra scripts
# and a different kernel config
#

LOCAL_TARGET = cmx86_target

include ../CD/Makefile

.PHONY: cmx86_target
cmx86_target:
	$(ROMFSINST) -p 555 /bin/setup-hd
	$(ROMFSINST) -p 555 /bin/setup-flashkey

