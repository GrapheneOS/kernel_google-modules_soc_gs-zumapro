# SPDX-License-Identifier: GPL-2.0

ccflags-y += -I$(srctree)/drivers/usb/typec/tcpm

obj-$(CONFIG_TYPEC_FUSB307)		+= tcpci_fusb307.o slg46826.o
obj-$(CONFIG_USB_PSY)			+= usb_psy.o
obj-$(CONFIG_TYPEC_MAX77759)		+= tcpci_max77759.o max77759_helper.o
obj-$(CONFIG_TYPEC_MAX77759)		+= bc_max77759.o
obj-$(CONFIG_TYPEC_MAX77759)		+= max77759_contaminant.o
obj-$(CONFIG_TYPEC_COOLING_DEV)		+= usbc_cooling_dev.o
