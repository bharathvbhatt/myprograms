#!/bin/bash -x 
#
# A depmod wrapper used by the toplevel Makefile

if test $# -ne 3; then
	echo "Usage: $0 /sbin/depmod <kernelrelease> <symbolprefix>" >&2
	exit 1
fi
DEPMOD=$1
KERNELRELEASE=$2
SYMBOL_PREFIX=$3

if ! test -r System.map -a -x "$DEPMOD"; then
	exit 0
fi

set -- -ae -F System.map
if test -n "$INSTALL_MOD_PATH"; then
	set -- "$@" -b "$INSTALL_MOD_PATH/lib/modules"
fi

"$DEPMOD" "$@" $SYMBOL_PREFIX
ret=$?

exit $ret
