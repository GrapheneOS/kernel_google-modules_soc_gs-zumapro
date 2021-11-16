# SPDX-License-Identifier: GPL-2.0
#
# Makefile for regulator drivers.
#

obj-$(CONFIG_REGULATOR_MAX77826_GS) += max77826-gs-regulator.o
obj-$(CONFIG_DRV_SAMSUNG_PMIC) += pmic_class.o
obj-$(CONFIG_REGULATOR_S2MPG10) += s2mpg10-regulator.o s2mpg10-powermeter.o
obj-$(CONFIG_REGULATOR_S2MPG11) += s2mpg11-regulator.o s2mpg11-powermeter.o
obj-$(CONFIG_REGULATOR_SLG51000_GS) += slg51000-regulator.o

ccflags-$(CONFIG_REGULATOR_DEBUG) += -DDEBUG
