# SPDX-License-Identifier: GPL-2.0

EXTRA_SYMBOLS += $(OUT_DIR)/../private/google-modules/trusty/Module.symvers

modules modules_install headers_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) \
	KBUILD_EXTRA_SYMBOLS="$(EXTRA_SYMBOLS)" $(@)

modules_install: headers_install

