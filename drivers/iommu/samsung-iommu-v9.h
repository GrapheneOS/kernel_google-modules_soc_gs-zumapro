/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 */

#ifndef __SAMSUNG_IOMMU_V9_H
#define __SAMSUNG_IOMMU_V9_H

#include <linux/clk.h>
#include <linux/interrupt.h>

#define REG_MMU_VERSION				0x0034

#define MMU_VERSION_MAJOR(val)			((val) >> 12)
#define MMU_VERSION_MINOR(val)			(((val) >> 8) & 0xF)
#define MMU_VERSION_REVISION(val)		((val) & 0xFF)
#define MMU_VERSION_RAW(reg)			(((reg) >> 16) & 0xFFFF)

struct sysmmu_drvdata {
	struct iommu_device iommu;
	struct device *dev;
	void __iomem *sfrbase;
	struct clk *clk;
	u32 version;
};

#endif
