# SPDX-License-Identifier: GPL-2.0
# Pixel UFSHCD Makefile

obj-$(CONFIG_SCSI_UFS_EXYNOS_GS) += ufs-exynos-gs.o
ufs-exynos-gs-y += ufs-exynos.o ufs-exynos-dbg.o ufs-pixel.o
ufs-exynos-gs-$(CONFIG_SOC_GS101) += gs101/ufs-cal-if.o
ufs-exynos-gs-$(CONFIG_SCSI_UFS_CRYPTO) += ufs-pixel-crypto.o

ccflags-y += -I $(srctree)/drivers/scsi/ufs

