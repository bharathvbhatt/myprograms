# Makefile to auto build tarball's with patches etc
# see Documentation/automake.txt for more info on using this file
# Copyright (C) 2009 David McCullough <davidm@uclinux.org>

.EXPORT_ALL_VARIABLES:

AT := $(if $(V:0=),,@)
ifndef STAGEDIR
STAGEDIR = $(ROOTDIR)
endif

BLD = build

#
# You can override the downloads cache dir in case you would prefer
# have all the files in tree or elsewhere.  The default is ~/.downloads
#
ifndef UC_DOWNLOADS
UC_DOWNLOADS = $(HOME)/.downloads
endif

#
# these config options apply to all things we build
#
CONFOPTS += \
    --prefix= \
    --sysconfdir=/etc/config

CONFOPTS += --with-vendor-name=$(CONFIG_VENDOR)
CONFOPTS += --with-vendor-name-short=$(CONFIG_VENDOR)
CONFOPTS += --with-os-name=uClinux-dist
CONFOPTS += --with-os-vendor=uClinux
CONFOPTS += --disable-static

#
# for uClibc without glibc compat
#
CONFOPTS += --enable-malloc0returnsnull
CONFOPTS += ac_cv_func_realloc_0_nonnull=yes

#
# make sure LD does the right thing during configure if invoked directly
#
LDFLAGS  += -Wl,-L$(STAGEDIR)/lib -Wl,-rpath-link -Wl,$(STAGEDIR)/lib
LDFLAGS  += -Wl,-L$(STAGEDIR)/lib64 -Wl,-rpath-link -Wl,$(STAGEDIR)/lib64

#
# where our pkgconfig stuff will go
#
PKG_CONFIG_PATH   = $(STAGEDIR)/lib/pkgconfig:$(STAGEDIR)/lib:$(STAGEDIR)/share/pkgconfig
PKG_CONFIG_LIBDIR = $(STAGEDIR)/lib/pkgconfig

#
# its generally a bad idea to fail on linker warnings for packages we
# aren't fully controlling,  though we could put this back conditionally
#
LDFLAGS := $(LDFLAGS:-Wl,--fatal-warnings=)

BUILD_CC = $(HOSTCC)
HOST_CC      = gcc
HOST_CXX     = g++
HOST_RANLIB  = $(shell which ranlib || echo :)
FLOCK        = $(shell which flock || echo : no flock)

#
# we use wget to download
#
WGET ?= wget $(if $(V:0=),,--quiet)
GIT ?= git
METHOD ?= default

#
# force using our special libtool
#
MAKEVARS     ?= LIBTOOL="$(LIBTOOL)"

#
# remap directories in a build to ensure that only uClinux-dist files
# are used
#
UCFRONT_MAPDIRS  = /lib/=$(STAGEDIR)/lib/
UCFRONT_MAPDIRS += /include/=$(STAGEDIR)/include/
ifneq ($(STAGEDIR),$(ROOTDIR))
UCFRONT_MAPDIRS += /usr/include=$(STAGEDIR)/usr/include
UCFRONT_MAPDIRS += /usr/lib=$(STAGEDIR)/usr/lib
else
UCFRONT_MAPDIRS += /usr/include=$(STAGEDIR)/include
UCFRONT_MAPDIRS += /usr/lib=$(STAGEDIR)/lib
endif

#
# some short cut config,  just set URL = blah
# and we work the rest out
#
ifeq ($(AUTOMAKE_y)$(AUTOMAKE_n)$(AUTOMAKE_),)
  ifdef URL
    ifndef PKG
      PKG := $(shell basename $(URL))
      PKG := $(PKG:.tar.gz=)
      PKG := $(PKG:.tar.bz2=)
      PKG := $(PKG:.tar.xz=)
      PKG := $(PKG:.tgz=)
      PKG := $(PKG:.tbz2=)
      PKG := $(PKG:.txz=)
      PKG := $(PKG:.zip=)
      PKG := $(PKG:.git=)
    endif
    AUTOMAKE_y += $(PKG)
  endif
endif

define pkg_def
  $(1)_URL ?= $$(URL)
  $(1)_METHOD ?= $$(METHOD)
  $(1)_INSTALL ?= $$(or $$(INSTALL),install)
  $(1)_NODEFCONF ?= $$(NODEFCONF)
  $(1)_AUTOCONF ?= $$(AUTOCONF)
  $(1)_VERSION ?= $$(VERSION)
  $(1)_SUBDIR ?= $$(SUBDIR)
  $(1)_ROMFS ?= $$(ROMFS)
  $(1)_CONFIGURE ?= $$(or $$(CONFIGURE),configure)
  $(1)_BUILDDIR ?= $$(or $$(BUILDDIR),$(1))
  $(1)_SRCDIR ?= $$(or $$(SRCDIR),$$($(1)_BUILDDIR))
  $(1)_DOWNLOADNAME ?= $$(or $$(DOWNLOADNAME),$$(notdir $$($(1)_URL)))
endef
$(foreach pkg,$(AUTOMAKE_y),$(eval $(call pkg_def,$(pkg))))

# put arguments on their own line
define oneline
$(1)
endef

# Backwards compat for patch command
PATCH = patch $(shell if patch --follow-symlinks < /dev/null > /dev/null 2>&1 ; then echo --follow-symlinks ; fi)

#
# allow some targets to be overridden
#
BUILDTARGET ?= built
INSTALLTARGET ?= installed
FINALTARGET ?= lndir
COMPLETETARGET ?= complete_target

#
# the main target
#
all: $(foreach pkg,$(AUTOMAKE_y),$(BLD)/$(pkg)-$(FINALTARGET)) $(COMPLETETARGET)

.PHONY: $(COMPLETETARGET)
$(COMPLETETARGET): $(foreach pkg,$(AUTOMAKE_y),$(BLD)/$(pkg)-$(FINALTARGET))

.PHONY: force_target
force_target:

.PRECIOUS: $(addprefix $(BLD)/,$(addsuffix -extracted,$(AUTOMAKE_y)))
.PRECIOUS: $(addprefix $(BLD)/,$(addsuffix -autoconf,$(AUTOMAKE_y)))
.PRECIOUS: $(addprefix $(BLD)/,$(addsuffix -configured,$(AUTOMAKE_y)))
.PRECIOUS: $(addprefix $(BLD)/,$(addsuffix -built,$(AUTOMAKE_y)))
.PRECIOUS: $(addprefix $(BLD)/,$(addsuffix -installed,$(AUTOMAKE_y)))
.PRECIOUS: $(addprefix $(BLD)/,$(addsuffix -patched,$(AUTOMAKE_y)))
.PRECIOUS: $(addprefix $(BLD)/,$(addsuffix -$(BUILDTARGET),$(AUTOMAKE_y)))
.PRECIOUS: $(addprefix $(BLD)/,$(addsuffix -$(INSTALLTARGET),$(AUTOMAKE_y)))

#
# download support for tarballs and the like from a URL
#
# The $(URL) is tried first, then each base from $(MIRROR_URLS) is
# used with the last component of $(URL). If $(FORCE_MIRROR_URLS) is
# defined, then they are tried before $(URL).
#

MIRROR_URLS ?= http://cvs.bne.opengear.com/downloads-mirror/
FORCE_MIRROR_URLS ?=

# verify the file is non-empty (and optionally matches a SHA1)
define do_verify_download # (url,filename)
  [ -s "$2" ] $(if $(SHA1SUM_$(notdir $1)),&& echo "$(SHA1SUM_$(notdir $1))  $2" | \
                                              sha1sum $(if $(V:0=),,--quiet) --check)
endef

# Check that the existing download file is valid, otherwise
# download it fresh from the first URLs that results in a valid
# file. This fragment acquires a lock on the file to avoid
# the case where multiple makes are downloading the same file.
# If the file exists and is unwritable, we just check that it
define do_download_and_verify # (1=file,2=urls)
	if [ -e $1 -a ! -w $1 ]; then \
	    exec 3<$1; \
	else \
	    exec 3>>$1; \
	fi; \
	if ! $(FLOCK) -s 3; then \
	  exit 1; \
	elif $(call do_verify_download,$1,$1) 2>/dev/null; then \
	  : already good; \
	elif $(FLOCK) -x 3 && $(call do_verify_download,$1,$1); then \
	  : it became good; \
	elif [ ! -w "$1" ]; then \
	  echo "$1: Can't replace this file" >&2; \
	  exit 1; \
	else \
	  rm -f $1; \
	  for url in $2; do \
	    echo "Downloading $$url ..."; \
	    if $(WGET) -O $1.partial "$$url" && \
	       $(call do_verify_download,$1,$1.partial); \
	    then \
	      cat $1.partial > $1; \
	      rm -f $1.partial; \
	      exit 0; \
	    else \
	      rm -f $1.partial; \
	    fi; \
	  done; \
	  false; \
	fi
endef

.PRECIOUS: $(UC_DOWNLOADS)/%

define download_dep_default # (pkg)

URLS_$$($(1)_DOWNLOADNAME) ?= \
   $$(addprefix $$(FORCE_MIRROR_URLS),$$(notdir $($(1)_URL))) \
   $($(1)_URL) \
   $$(addprefix $$(or $$(MIRROR_URLS_$$($(1)_DOWNLOADNAME)),$$(MIRROR_URLS)),$$(notdir $($(1)_URL)))

downloads/$$($(1)_DOWNLOADNAME):
	$(AT)mkdir -p "$$(@D)" $(UC_DOWNLOADS)
	$(AT)$$(call do_download_and_verify,$(UC_DOWNLOADS)/$$(@F),$$(URLS_$$(@F)))
	$(AT)ln -fs $(UC_DOWNLOADS)/$$(@F) "$$@"

$(BLD)/$(1)-extracted: downloads/$($(1)_DOWNLOADNAME)
	@echo "Extracting $$< ..."
	$(AT)rm -rf $(BLD)/$($(1)_SRCDIR)
	$(AT)mkdir -p $(BLD)
	$(AT)case "$$<" in \
	*zip) rm -rf $(BLD)/$(1); mkdir -p $(BLD)/$(1); pkg=`pwd`/$$<; (cd $(BLD)/$(1); unzip $$$$pkg);; \
	*bz2) bunzip2 < $$< | (cd $(BLD); tar xf -);; \
	*xz) unxz < $$< | (cd $(BLD); tar xf -);; \
	*) gunzip < $$< | (cd $(BLD); tar xf -);; \
	esac || exit 1
	$(AT)touch $$@

endef

#
# Support for remote GIT subdirectories of sorts, we do not remember master
# downloads as they change from load to load so just save it until the next
# clean.  The clone will save us downloading it all again.
#
define download_dep_gitarchive

.PRECIOUS: downloads/$(1)$(if $($(1)_VERSION),-$($(1)_VERSION)).tar
downloads/$(1)$(if $($(1)_VERSION),-$($(1)_VERSION)).tar:
	@echo "Downloading `basename $$@` ..."
	$(AT)[ -d "$(UC_DOWNLOADS)" ] || mkdir -p "$(UC_DOWNLOADS)"
	$(AT)if [ ! -f "$(UC_DOWNLOADS)/`basename $$@`" ]; then \
		echo "Updating/Cloning `basename $$($(1)_URL)` for archive ..."; \
		if [ -d "archive/`basename $$($(1)_URL)`" ]; then \
			(\
			cd "archive/`basename $$($(1)_URL)`"; \
			$(GIT) checkout -q master || exit 1; \
			$(GIT) pull || exit 1; \
			$(GIT) checkout -q $(if $($(1)_VERSION),$($(1)_VERSION),master) || exit 1; \
			) || exit 1; \
		else \
			$(GIT) clone $$($(1)_URL) archive/`basename $$($(1)_URL)` || exit 1; \
			(\
			cd "archive/`basename $$($(1)_URL)`"; \
			$(GIT) checkout -q $(if $($(1)_VERSION),$($(1)_VERSION),master) || exit 1; \
			) || exit 1; \
		fi ; \
		cd archive/`basename $$($(1)_URL)`; \
			$(GIT) archive -o $(UC_DOWNLOADS)/`basename $$@` HEAD:$(if $($(1)_SUBDIR),$($(1)_SUBDIR)/,) || exit 1; \
	fi
	$(AT)mkdir -p `dirname $$@`
	$(AT)if [ "$(if $($(1)_VERSION),$($(1)_VERSION),master)" = "master" ]; then \
			mv -f "$(UC_DOWNLOADS)/`basename $$@`" $$@ || exit 1; \
	elif [ -f "$(UC_DOWNLOADS)/`basename $$@`" ]; then \
			ln -fs "$(UC_DOWNLOADS)/`basename $$@`" $$@; \
	fi
	$(AT)if [ ! -f "$$@" ]; then \
		echo "Cannot find download for $$@" >&2 ; \
		exit 1; \
	fi

$(BLD)/$(1)-extracted: downloads/$(1)$(if $($(1)_VERSION),-$($(1)_VERSION)).tar
	@echo "Extracting $$< ..."
	$(AT)rm -rf $(BLD)/$(1)
	$(AT)mkdir -p $(BLD)/$(1)
	$(AT)cat < $$< | (cd $(BLD)/$(1); tar xf -); exit $?
	$(AT)touch $$@

endef

#
# Support for remote GIT repos
#
define download_dep_git

$(BLD)/$(1)-extracted:
	@echo "Cloning $(1) ..."
	$(AT)mkdir -p clone
	$(AT)if [ -d "clone/$(1)" ]; then \
		cd "clone/$(1)"; \
		$(GIT) checkout -q master; \
		$(GIT) pull || exit 1; \
	else \
		$(GIT) clone $$($(1)_URL) clone/$(1) || exit 1; \
	fi
	$(AT)cd "clone/$(1)" &&	$(GIT) checkout -q $(if $($(1)_VERSION),$($(1)_VERSION),master)
	$(AT)rm -rf $(BLD)/$(1)
	$(AT)mkdir -p $(BLD)/$(1)
	$(AT)cp -rlT clone/$(1) $(BLD)/$(1)
	$(AT)touch $$@

endef

$(foreach pkg,$(AUTOMAKE_y),$(eval $(call download_dep_$($(pkg)_METHOD),$(pkg))))

# Use NOPATCH=1 if you are editing patches with quilt in the build directory.
ifndef NOPATCH
$(foreach pkg,$(AUTOMAKE_y), $(eval $(call oneline,$(BLD)/$(pkg)-extracted: $(wildcard patches/series patches/$(pkg)*.patch) $(MAKEFILE_LIST))))
endif

*_BUILDDIR = $(BLD)/$(or $($*_BUILDDIR),$*)
*_SRCDIR = $(BLD)/$(or $($*_SRCDIR),$(SRCDIR))
QUILT ?= quilt

$(BLD)/%-patched: $(BLD)/%-extracted
	$(AT)if [ -d patches ] && $(QUILT) --version >/dev/null 2>&1; then \
		if [ -d $(*_SRCDIR)/patches ]; then \
		    mv $(*_SRCDIR)/patches $(*_SRCDIR)/patches~; \
		fi; \
		ln -sf $(abspath patches) $(*_SRCDIR)/; \
		if [ ! -s patches/series ]; then \
			echo "creating series file for quilt" >&2; \
			for i in patches/$**.patch; do \
			    [ ! -f $$i ] || basename $$i; \
			done > patches/series; \
		fi; \
		(cd $(*_SRCDIR) && $(QUILT) push -a); \
	elif [ -f patches/series ]; then \
		while read t; do \
			[ -f patches/$$t ] || continue; \
			echo "Patching $* with patches/$$t from series"; \
			case "$$t" in \
			*.gz) (cd $(BLD)/$($*_SRCDIR); gunzip | $(PATCH) -E -p1) < patches/$$t || exit 1;; \
			*)    (cd $(BLD)/$($*_SRCDIR); $(PATCH) -E -p1) < patches/$$t || exit 1;; \
			esac || exit 1; \
		done < patches/series || exit 1; \
	else \
		for t in patches/$**.patch*; do \
			[ -f "$$t" ] || continue; \
			echo "Patching $* with $$t"; \
			case "$$t" in \
			*.gz) (cd $(BLD)/$($*_SRCDIR); gunzip | $(PATCH) -E -p1) < $$t || exit 1 ;; \
			*)    (cd $(BLD)/$($*_SRCDIR); $(PATCH) -E -p1) < $$t || exit 1;; \
			esac || exit 1; \
		done || exit 1; \
	fi
	$(AT)touch $@

$(BLD)/%-autoconf: $(BLD)/%-patched
	@echo "Autoconf $* ..."
	$(if $($*_AUTOCONF),cd $(BLD)/$($*_SRCDIR); $($*_AUTOCONF))
	$(AT)touch $@

$(BLD)/%-configured: $(BLD)/%-autoconf
	@echo "Configure $* ..."
	$(AT)mkdir -p $(BLD)/$($*_BUILDDIR)
	$(AT)[ ! -f config.cache ] || cp config.cache $(BLD)/$($*_BUILDDIR)
	$(AT)cd $(BLD)/$($*_BUILDDIR); \
		if [ -f $($*_CONFIGURE) ]; then \
			$(CONFVARS) \
			$($*_CONFVARS) \
			CFLAGS="$(CFLAGS) $($*_CFLAGS)" \
			bash $($*_CONFIGURE) $(if $($*_NODEFCONF),,$(CONFIGURE_OPTS) $(CONFOPTS)) $($*_CONFOPTS); \
		fi
	$(AT)touch $@

$(BLD)/%-built: $(BLD)/%-configured $(if $(NOPATCH),force_target)
	@echo "Building $* ..."
	$(AT)$(BUILDVARS) \
	$($*_BUILDVARS) \
		$(MAKE) -C $(BLD)/$($*_BUILDDIR) $(MAKEVARS) $($*_MAKEVARS)
	$(AT)touch $@

$(BLD)/%-installed: $(BLD)/%-$(BUILDTARGET)
	@echo "Installing $* ..."
	$(AT)mkdir -p $(BLD)/$*-install
	$(AT)$(BUILDVARS) \
	$($*_BUILDVARS) \
	$($*_INSTALLVARS) \
		$(MAKE) -C $(BLD)/$($*_BUILDDIR) $($*_INSTALL) DESTDIR=`pwd`/$(BLD)/$*-install $(MAKEVARS) $($*_MAKEVARS)
	$(AT)touch $@

$(BLD)/%-lndir: $(BLD)/%-$(INSTALLTARGET) force_target
	@echo "Staging $* ..."
	$(AT)mkdir -p $(BLD)/$*-install
	$(AT)cd $(BLD)/$*-install; find . \( -type f -o -type l \) | egrep -v '.la$$' | \
	while read t; do \
		case "$$t" in \
		./bin/*) continue;; \
		./sbin/*) continue;; \
		esac; \
		rm -f $(STAGEDIR)/$$t; \
		mkdir -p "`dirname \"$(STAGEDIR)/$$t\"`"; \
		ln -s "`pwd`/$$t" "$(STAGEDIR)/$$t"; \
		echo "ln -s \"`pwd`/$$t\" \"$(STAGEDIR)/$$t\""; \
	done
	$(AT)touch $@

clean: $(foreach pkg,$(AUTOMAKE_y),$($(pkg)_CLEAN))
	rm -rf $(BLD) downloads

romfs_pre:
	@echo "Romfs pre-install ..."
	$(AT)for i in $(BLD)/*-install; do \
		echo "Installing `basename $$i -install` ..."; \
		[ ! -d $$i/usr ] || $(ROMFSINST) -v -f $$i/usr /usr; \
		[ ! -d $$i/bin ] || $(ROMFSINST) -v -f $$i/bin /bin; \
		[ ! -d $$i/sbin ] || $(ROMFSINST) -v -f $$i/sbin /sbin; \
		[ ! -d $$i/lib ] || $(ROMFSINST) -v -f $$i/lib /lib; \
		[ ! -d $$i/lib64 ] || $(ROMFSINST) -v -f $$i/lib64 /lib; \
		[ ! -d $$i/libexec ] || $(ROMFSINST) -v -f $$i/libexec /libexec; \
		rm -rf $$i/etc/default; \
		[ ! -d $$i/etc/config ] || mv $$i/etc/config $$i/etc/default; \
		[ ! -d $$i/etc ] || $(ROMFSINST) -v -f $$i/etc /etc; \
		[ ! -d $$i/share ] || $(ROMFSINST) -v -f $$i/share /share; \
	done
	rm -f $(ROMFSDIR)/lib/*.a
	rm -f $(ROMFSDIR)/lib/*.la

romfs_post: $(foreach pkg,$(AUTOMAKE_y),$($(pkg)_ROMFS))

romfs: $(if $(AUTOMAKE_ROMFS),$(AUTOMAKE_ROMFS),romfs_pre) romfs_post

