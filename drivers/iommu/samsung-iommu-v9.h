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

typedef u64 sysmmu_iova_t;
typedef u32 sysmmu_pte_t;

#define SECT_ORDER	20
#define LPAGE_ORDER	16
#define SPAGE_ORDER	12

#define SECT_SIZE	(1UL << SECT_ORDER)
#define LPAGE_SIZE	(1UL << LPAGE_ORDER)
#define SPAGE_SIZE	(1UL << SPAGE_ORDER)

#define VA_WIDTH_32BIT		0x0
#define VA_WIDTH_36BIT		0x1
#define NUM_LV1ENTRIES_32BIT	4096
#define NUM_LV1ENTRIES_36BIT	65536
#define NUM_LV2ENTRIES		(SECT_SIZE / SPAGE_SIZE)
#define LV1TABLE_SIZE_32BIT	(NUM_LV1ENTRIES_32BIT * sizeof(sysmmu_pte_t))
#define LV1TABLE_SIZE_36BIT	(NUM_LV1ENTRIES_36BIT * sizeof(sysmmu_pte_t))
#define LV2TABLE_SIZE		(NUM_LV2ENTRIES * sizeof(sysmmu_pte_t))

struct sysmmu_drvdata {
	struct list_head list;
	struct iommu_device iommu;
	struct device *dev;
	struct iommu_group *group;
	void __iomem *sfrbase;
	struct clk *clk;
	phys_addr_t pgtable;
	spinlock_t lock; /* protect atomic update to H/W status */
	u32 version;
	u32 va_width;
	int num_pmmu;
	int attached_count;
};

struct sysmmu_clientdata {
	struct device *dev;
	struct sysmmu_drvdata **sysmmus;
	int sysmmu_count;
};

#endif
