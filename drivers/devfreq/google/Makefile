# Exynos DEVFREQ Drivers

ccflags-y := -DDYNAMIC_DEBUG_MODULE
ccflags-y += -I$(srctree)/drivers/devfreq
ccflags-y += -I$(srctree)/drivers/thermal

obj-$(CONFIG_ARM_EXYNOS_DEVFREQ)		+= exynos_devfreq.o
exynos_devfreq-objs				+= gs-devfreq.o governor_simpleinteractive.o
exynos_devfreq-$(CONFIG_EXYNOS_ALT_DVFS)	+= gs-ppc.o

obj-$(CONFIG_ARM_MEMLAT_MON)			+= arm-memlat-mon.o
obj-$(CONFIG_DEVFREQ_GOV_MEMLAT)		+= governor_memlat.o
obj-$(CONFIG_ARM_EXYNOS_DEVFREQ)		+= memlat-devfreq.o
