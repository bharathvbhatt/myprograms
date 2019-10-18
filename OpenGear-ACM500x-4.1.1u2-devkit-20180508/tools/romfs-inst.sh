#!/bin/sh
#
# A tool to simplify Makefiles that need to put something
# into the ROMFS
#
# Copyright (C) David McCullough, 2002,2003
#
#############################################################################
# Provide a default PATH setting to avoid potential problems...
PATH="/bin:/sbin:/usr/bin:/usr/sbin:/usr/local/bin:$PATH"

usage()
{
cat << !EOF >&2
$0: [options] [src...] dst
    -v          : output actions performed.
    -e env-var  : only take action if env-var is set to "y".
    -E env-var  : only take action if env-var is not set to "y".
    -o option   : only take action if option is set to "y".
    -O option   : only take action if option is not set to "y".
    -c          : process with cpp+cflags
    -p perms    : chmod style permissions for dst.
    -d          : make dst directory if it doesn't exist
    -S          : don't strip after installing
    -c          : process with cpp+cflags
    -a text     : append text to dst.
    -F file     : append contexts of given file to dst.
    -A pattern  : only append text/file if pattern doesn't exist in file
    -l link     : dst is a hard link to 'link'.
    -s sym-link : dst is a sym-link to 'sym-link'.
    -M          : install kernel module into dst subdir of module dir
    -r          : root directory of install (default ROMFSDIR)
    -f          : do not follow symlinks
    -R          : delete dst (ignore src)

    if "src" is not provided,  basename is run on dst to determine the
    source in the current directory.

    multiple -e and -o options are ANDed together.  To achieve an OR affect
    use a single -e/-o with 1 or more y/n/"" chars in the condition.

    if src is a directory,  everything in it is copied recursively to dst
    with special files removed (currently CVS and Subversion dirs).
!EOF
	exit 1
}

#############################################################################

setperm()
{
	rc=0
	# Always start with write access for the user so that files can be
	# updated/overwritten during make romfs
	chmod u+w "${ROMFSDIR}${dst}"
	if [ "$perm" ]
	then
		[ "$v" ] && echo "chmod ${perm} ${ROMFSDIR}${dst}"
		chmod ${perm} "${ROMFSDIR}${dst}"
		rc=$?
	fi
	return $rc
}

#############################################################################

maybe_strip()
{
	local dst=${1#$ROMFSDIR}
	local OBJCOPY=${STRIPTOOL%strip}objcopy

	if [ -n "$strip" ]; then
                # Prerequsities for making a foo.debug file for gdb:
                #   1. you have created the $DEBUGDIR directory
                #   2. a foo.debug file was not made by the package
                #      (unlikely, but it could happen!)
		if [ -d "$DEBUGDIR" -a ! -s "$1.debug" ] &&
		    "$OBJCOPY" --only-keep-debug \
			"$1" "$1.debug" 2>/dev/null
		then
		    # The debug symbols stripped from objects in the target
		    # staging area can be placed into a directory for GDB
		    # to find later. GDB will use various strategies
		    # to find these files. The normal mechanism is the
		    # debugdir approach, which happens when GDB opens an
		    # object file /path/to/objfile.so and it also looks for
		    # the companion file $DEBUGDIR/path/to/objfile.so.debug.
                    local buildid=
                    if [ -w "$DEBUGDIR/.build-id" ]; then
                        # XXX binutils not new enough for 'readelf -n'
                        buildid=$(file "$1" | sed -n \
                            -e 's/.*BuildID\[sha1\]=0x\([0-9a-f]*\).*/\1/p')
                    fi
                    if [ -z "$buildid" ]; then
                        # No build-id section; create a debuglink section
                        # for gdb
		        local base=$(basename "$1")
		        (cd $(dirname "$1") &&
		         "$OBJCOPY" --add-gnu-debuglink="$base".debug "$base")
		        install -p -D "$1.debug" \
			    "$DEBUGDIR$(readlink -f "$1.debug")"
                    else
                        # gdb will use the build-id
                        local bn=`echo $buildid | sed -e 's,^..,&/,'`.debug
                        install -p -D "$1.debug" "$DEBUGDIR/.build-id/$bn"
                    fi
		    rm "$1.debug"
		fi

		if ${STRIPTOOL} "$1" 2>/dev/null; then
		    ${STRIPTOOL} -R .comment -R .note "$1"
		fi

		case $1 in
		    *-gdb.py)
			# This file is purely for GDB to use; So we "strip"
			# it by moving it completely into $DEBUGDIR.
			echo "Moving debug script $1"
			install -p -D "$1" "$DEBUGDIR$(readlink -f "$1")"
			rm -f "$1"
			;;
		esac
	fi
} 2>&1

file_copy()
{
	rc=0
	if [ -d "${src}" ]
	then
		[ "$v" ] && echo "CopyDir ${src} ${ROMFSDIR}${dst}"
		(
			cd ${src} || return 1
			V=
			[ "$v" ] && V=v
			find . -print | grep -E -v '/CVS|/\.svn' | cpio -p${V}dum${follow} ${ROMFSDIR}${dst}
			rc=$?
			# And make sure these files are still writable
			find . -print | grep -E -v '/CVS|/\.svn' | ( cd ${ROMFSDIR}${dst}; xargs chmod u+w )
			setperm ${ROMFSDIR}${dst}
			find . -type f | grep -E -v '/CVS|/\.svn|\.ko$' | while read t; do
				maybe_strip ${ROMFSDIR}${dst}/$t
			done
		)
	else
		if [ -d ${ROMFSDIR}${dst} ]; then
			dstfile=${ROMFSDIR}${dst}/`basename ${src}`
		else
			dstfile=${ROMFSDIR}${dst}
		fi
		rm -f ${dstfile}
	    if [ -z "$follow" -a -h "${src}" ]; then
	        # copy symlinks as symlinks
		local target="$(readlink "${src}")"
		case $target in
		   /*) echo "ERROR: absolute link target" >&2;;
		esac
		[ "$v" ] && echo "ln -sT ${target} ${dstfile}"
		ln -sT "${target}" "${dstfile}"
	    else
		[ "$v" ] && echo "cp ${src} ${dstfile}"
		if [ ! -d "$IMAGEDIR" ]
		then
			mkdir -p $IMAGEDIR
		fi
		case "$src" in
			/*) echo "${src} ${dstfile}" ;;
			*)  echo "`pwd`/${src} ${dstfile}" ;;
		esac >> $IMAGEDIR/romfs-inst.log
		cp ${src} ${dstfile} && setperm ${dstfile}
		rc=$?
		if [ $rc -eq 0 ]; then
			maybe_strip ${dstfile}
		fi
	    fi
	fi
	return $rc
}

#############################################################################

file_append()
{
	touch ${ROMFSDIR}${dst} || return 1
	if [ -z "$appending_file_contents" -a -z "${pattern}" ] && grep -F "${src}" ${ROMFSDIR}${dst} > /dev/null
	then
		[ "$v" ] && echo "File entry already installed."
	elif [ "${pattern}" ] && egrep "${pattern}" ${ROMFSDIR}${dst} > /dev/null
	then
		[ "$v" ] && echo "File pattern already installed."
	else
		[ "$v" ] && echo "Installing entry into ${ROMFSDIR}${dst}."
		if [ -s ${ROMFSDIR}${dst} ] ; then
			# if file lacks a trailing new line, add it before appending the text
			if [ $(tail -n1 ${ROMFSDIR}${dst} | tr -d '\n' | wc -c) = $(tail -n1 ${ROMFSDIR}${dst} | wc -c) ] ; then
				echo "" >> ${ROMFSDIR}${dst} || return 1
			fi
		fi
		if [ -z "$appending_file_contents" ]
		then
			echo "${src}" >> ${ROMFSDIR}${dst} || return 1
		else
			cat "${src}" >> ${ROMFSDIR}${dst} || return 1
		fi
	fi
	setperm ${ROMFSDIR}${dst}
	return $?
}

#############################################################################

hard_link()
{
	rm -f ${ROMFSDIR}${dst}
	[ "$v" ] && echo "ln ${src} ${ROMFSDIR}${dst}"
	ln ${ROMFSDIR}${src} ${ROMFSDIR}${dst}
	return $?
}

#############################################################################

sym_link()
{
	rm -f ${ROMFSDIR}${dst}

	case ${src} in
	    /*)
		: "convert absolute symlink target to relative"

		# canonicalize
		local adst=$(readlink -m ${ROMFSDIR}${dst})  # link name
		local asrc=$(readlink -m ${ROMFSDIR}${src})  # target

		# remove common prefix /COMMON/ from adst, asrc
		adst=${adst#/} asrc=${asrc#/}
		while :; do
		    local d=${adst%%/*}/
		    case $asrc in
			$d*) adst=${adst#$d} asrc=${asrc#$d} ;;
			*)   break;;
		    esac
		done

		# convert leading dir components of the link path
		# into "../"s prepended on the target path:
		#   $adst        ->  $asrc
		#   XX/YY/LINK   ->  TDIR/TARGET
		#         LINK   ->  ../../TDIR/TARGET
		while :; do
		    case $adst in
			*/*) asrc=../$asrc adst=${adst#*/} ;;
			*)   break;;
		    esac
		done

		if [ "${asrc}" != "${src}" ]; then
		    echo "** converted symlink target ${src} to ${asrc}" >&2
		    src="${asrc}"
		fi

		;;
	    *)  : "symlink target is already relative"
		;;
	esac

	[ "$v" ] && echo "ln -s ${src} ${ROMFSDIR}${dst}"
	ln -sf ${src} ${ROMFSDIR}${dst}
	return $?
}

#############################################################################

cpp_file()
{
	set -x
	if [ -d ${ROMFSDIR}${dst} ]; then
		dstfile=${ROMFSDIR}${dst}/`basename ${src}`
	else
		dstfile=${ROMFSDIR}${dst}
	fi
	rm -f ${dstfile}
	[ "$v" ] && echo "${CROSS_COMPILE}cpp ${CFLAGS} -P < ${src} > ${dstfile}"
	${CROSS_COMPILE}cpp ${CFLAGS} -P < ${src} > ${dstfile}
	return $?
}

#############################################################################

rm_file()
{
	# if ROMFSDIR is not set, play it safe
	[ "${ROMFSDIR}" ] || return 1
	if [ -d "${ROMFSDIR}${dst}" ]; then
		echo "rm -rf ${ROMFSDIR}${dst}"
		rm -rf "${ROMFSDIR}${dst}"
	else
		echo "rm -f ${ROMFSDIR}${dst}"
		rm -f "${ROMFSDIR}${dst}"
	fi
	return $?
}

#############################################################################
#
# main program entry point
#

v=
option=y
noption=
pattern=
perm=
func=file_copy
mdir=
src=
dst=
strip=1
kernmod=
r=
follow=L
appending_file_contents=

while getopts 'dRfSMvce:E:o:O:A:p:a:F:l:s:r:' opt "$@"
do
	case "$opt" in
	v) v="1";                                  ;;
	d) mdir="1";                               ;;
	f) follow=;                                ;;
	S) strip=;                                 ;;
	M) kernmod="1";                            ;;
	o) option="$OPTARG";                       ;;
	O) noption="$OPTARG";                      ;;
	e) eval option=\"\$$OPTARG\";              ;;
	E) eval noption=\"\$$OPTARG\";             ;;
	p) perm="$OPTARG";                         ;;
	a) src="$OPTARG"; func=file_append;        ;;
	F) src="$OPTARG"; func=file_append;
		appending_file_contents=1;             ;;
	A) pattern="$OPTARG";                      ;;
	l) src="$OPTARG"; func=hard_link;          ;;
	s) src="$OPTARG"; func=sym_link;           ;;
	r) ROMFSDIR="$OPTARG"; r=1;                ;;
	c) func=cpp_file;                          ;;
	R) func=rm_file;                           ;;

	*)  break ;;
	esac
#
#	process option here to get an ANDing effect
#
	case "$option" in
	*[mMyY]*) # this gives OR effect, ie., nYn
		;;
	*)
		[ "$v" ] && echo "Condition not satisfied."
		exit 0
		;;
	esac

#
#	process negative options here to get an ANDing effect
#
	case "${noption:-n}" in
	*[nN]*) # this gives OR effect, ie., yNy
		;;
	*)
		[ "$v" ] && echo "Condition not satisfied."
		exit 0
		;;
	esac
done

if [ -z "$ROMFSDIR" -a -z "$r" ]
then
	echo "ROMFSDIR is not set" >&2
	usage
	exit 1
fi

if [ -z "$STRIPTOOL" ]
then
	STRIPTOOL=strip
fi	

if [ -z "$DEBUGDIR" ]
then
        # This is GDB's default 'debug-file-directory' setting
        DEBUGDIR=/usr/local/lib/debug
fi

shift `expr $OPTIND - 1`

if [ $# -eq 0 ]; then usage; fi

# remove the last argument, assigning it to $dst
n=$#
for arg; do
	shift
	n=$(expr $n - 1)
	if [ $n -eq 0 ]; then
		dst="$arg";
	else
		set -- "$@" "$arg"
	fi
done

case $# in
0)
	set -- "${src:-`basename "$dst"`}"
	;;
*)
	if [ ! -z "$src" ]
	then
		echo "Source file already provided" >&2
		exit 1
	fi
	;;
esac

if [ -n "$kernmod" ]; then
	strip=
	kerndir=${ROOTDIR}/${LINUXDIR}
	# could prob take from UTS headers as well ...
	kernver=$(cat ${kerndir}/include/config/kernel.release)
	dst="/lib/modules/${kernver}/${dst}"
fi

if [ "$mdir" -a ! -d "`dirname ${ROMFSDIR}${dst}`/." ]
then
	mkdir -p "`dirname ${ROMFSDIR}${dst}`/." || exit 1
fi

for src; do
	$func || exit 1
done

exit 0

#############################################################################
