
PKG_NAME := gcc

PKG_VER  := 6.3.0
#PKG_VER  := 8.3.0

PKG_SOURCES := http://ftp.gnu.org/gnu/gcc/$(PKG_NAME)-$(PKG_VER)/$(PKG_NAME)-$(PKG_VER).tar.gz

PKG_MD5 := 6e5ea04789678f1250c1b30c4d9ec417
#PKG_MD5 := 9972f8c24c02ebcb5a342c1b30de69ff

PKG_PATCHES := patch.txt

include $(EXTBLD_LIB)

ifeq (arm,$(EMBOX_ARCH))
AUTOCONF_TARGET_TRIPLET := arm-none-eabi
else ifeq (sparc,$(EMBOX_ARCH))
AUTOCONF_TARGET_TRIPLET := sparc-elf
else ifeq (mips,$(EMBOX_ARCH))
AUTOCONF_TARGET_TRIPLET := mips-elf
else ifeq (x86,$(EMBOX_ARCH))
AUTOCONF_TARGET_TRIPLET := i386-pc-elf
else
$(error Unsupported architecture: $(EMBOX_ARCH))
endif

target_libgcc_cppflags := -DLIBGCC2_HAS_XF_MODE=0 \
	-DDO_GLOBAL_CTORS_BODY -DDO_GLOBAL_DTORS_BODY \
	-DNAME__MAIN -DSYMBOL__MAIN=__embox_register_eh_frame_once

EMBOX_DERIVED_CXXFLAGS = $(filter -g% -f% -m% -O% -G% -E% -Wa%,$(EMBOX_CXXFLAGS))
ifeq (arm,$(EMBOX_ARCH))
GCC_IMPORTED_CXXFLAGS = $(filter-out -std=% -fno-rtti,$(EMBOX_DERIVED_CXXFLAGS))
else
GCC_IMPORTED_CXXFLAGS = $(filter-out -std=%, $(EMBOX_DERIVED_CXXFLAGS))
endif

PKG_BUILD_DIR  := $(BUILD_DIR)/build

$(CONFIGURE) :
	mkdir -p $(PKG_BUILD_DIR) && cd $(PKG_BUILD_DIR) && ( \
		. $(EMBOX_GCC_ENV) && $(PKG_SOURCE_DIR)/configure \
			--target=$(AUTOCONF_TARGET_TRIPLET) \
			--with-cross-host=embox \
			--prefix=$(PKG_INSTALL_DIR) \
			--disable-multilib \
			--disable-shared \
			--disable-tls \
			--disable-nls \
			--disable-decimal-float \
			--disable-libffi \
			--disable-libgomp \
			--disable-libmudflap \
			--disable-libquadmath \
			--disable-libstdcxx-pch \
			--disable-libssp \
			--disable-bootstrap \
			--without-headers \
			--without-newlib \
			--enable-languages=c,c++ \
			--enable-soft-float \
			CC_FOR_TARGET=$${EMBOX_CROSS_COMPILE}gcc \
			CXX_FOR_TARGET=$${EMBOX_CROSS_COMPILE}g++ \
			CFLAGS_FOR_TARGET="$${EMBOX_IMPORTED_CPPFLAGS} $(target_libgcc_cppflags) $${EMBOX_IMPORTED_CFLAGS}" \
			CXXFLAGS_FOR_TARGET="$${EMBOX_IMPORTED_CPPFLAGS} $(GCC_IMPORTED_CXXFLAGS) -fpermissive" \
			LDFLAGS_FOR_TARGET="$${EMBOX_IMPORTED_LDFLAGS}"; \
	)
	touch $@

$(BUILD) :
	cd $(PKG_BUILD_DIR) && ( \
		$(MAKE) -j $(shell nproc) \
			all-target-libgcc all-target-libstdc++-v3; \
	)
	touch $@

$(INSTALL) :
	cd $(PKG_BUILD_DIR) && ( \
		$(MAKE) install-target-libgcc; \
		$(MAKE) install-target-libstdc++-v3; \
	)
	mkdir -p $(PKG_INSTALL_DIR)/libs
	cp $(PKG_INSTALL_DIR)/lib/gcc/$(AUTOCONF_TARGET_TRIPLET)/$(PKG_VER)/libgcc.a $(PKG_INSTALL_DIR)/libs
	cp $(PKG_INSTALL_DIR)/$(AUTOCONF_TARGET_TRIPLET)/lib/libstdc++.a $(PKG_INSTALL_DIR)/libs
	cp $(PKG_INSTALL_DIR)/$(AUTOCONF_TARGET_TRIPLET)/lib/libsupc++.a $(PKG_INSTALL_DIR)/libs
	# Rename directories from "arm-none-eabi" to "_target" to use them in Mybuild.
	find $(PKG_INSTALL_DIR) -type d -execdir rename 's/$(AUTOCONF_TARGET_TRIPLET)/_target/' '{}' \+
	touch $@
