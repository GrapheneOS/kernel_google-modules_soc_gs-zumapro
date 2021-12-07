/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 */

#ifndef __SAMSUNG_IOMMU_V9_H
#define __SAMSUNG_IOMMU_V9_H

#include <linux/clk.h>
#include <linux/interrupt.h>

#define SYSMMU_VM_OFFSET	0x1000
#define SYSMMU_MASK_VMID	0x1

#define REG_MMU_CTRL				0x0000
#define REG_MMU_VERSION				0x0034
#define REG_MMU_CTRL_VM				0x8000
#define REG_MMU_CONTEXT0_CFG_FLPT_BASE_VM	0x8404
#define REG_MMU_CONTEXT0_CFG_ATTRIBUTE_VM	0x8408


#define MMU_VERSION_MAJOR(val)			((val) >> 12)
#define MMU_VERSION_MINOR(val)			(((val) >> 8) & 0xF)
#define MMU_VERSION_REVISION(val)		((val) & 0xFF)
#define MMU_VERSION_RAW(reg)			(((reg) >> 16) & 0xFFFF)
#define MMU_VM_ADDR(reg, idx)			((reg) + ((idx) * SYSMMU_VM_OFFSET))

#define MMU_CTRL_ENABLE			0x5
#define MMU_CTRL_DISABLE		0x0

#define MMU_CONTEXT0_CFG_FLPT_BASE_PPN(reg)	((reg) & 0xFFFFFF)

typedef u64 sysmmu_iova_t;
typedef u32 sysmmu_pte_t;

#define SECT_ORDER	20
#define LPAGE_ORDER	16
#define SPAGE_ORDER	12

#define SECT_SIZE	(1UL << SECT_ORDER)
#define LPAGE_SIZE	(1UL << LPAGE_ORDER)
#define SPAGE_SIZE	(1UL << SPAGE_ORDER)

#define SECT_MASK (~(SECT_SIZE - 1))
#define LPAGE_MASK (~(LPAGE_SIZE - 1))
#define SPAGE_MASK (~(SPAGE_SIZE - 1))

#define SECT_ENT_MASK	~((SECT_SIZE >> PG_ENT_SHIFT) - 1)
#define LPAGE_ENT_MASK	~((LPAGE_SIZE >> PG_ENT_SHIFT) - 1)
#define SPAGE_ENT_MASK	~((SPAGE_SIZE >> PG_ENT_SHIFT) - 1)

#define SPAGES_PER_LPAGE	(LPAGE_SIZE / SPAGE_SIZE)

#define VA_WIDTH_32BIT		0x0
#define VA_WIDTH_36BIT		0x1
#define NUM_LV1ENTRIES_32BIT	4096
#define NUM_LV1ENTRIES_36BIT	65536
#define NUM_LV2ENTRIES		(SECT_SIZE / SPAGE_SIZE)
#define LV1TABLE_SIZE_32BIT	(NUM_LV1ENTRIES_32BIT * sizeof(sysmmu_pte_t))
#define LV1TABLE_SIZE_36BIT	(NUM_LV1ENTRIES_36BIT * sizeof(sysmmu_pte_t))
#define LV2TABLE_SIZE		(NUM_LV2ENTRIES * sizeof(sysmmu_pte_t))

#define lv1ent_offset(iova) ((iova) >> SECT_ORDER)
#define lv2ent_offset(iova) (((iova) & ~SECT_MASK) >> SPAGE_ORDER)

#define FLPD_FLAG_MASK	7
#define SLPD_FLAG_MASK	3
#define UNMAPPED_FLAG	0
#define SLPD_FLAG	1
#define SECT_FLAG	2
#define LPAGE_FLAG	1
#define SPAGE_FLAG	2

#define PG_ENT_SHIFT	4

#define lv1ent_unmapped(sent)	((*(sent) & FLPD_FLAG_MASK) == UNMAPPED_FLAG)
#define lv1ent_page(sent)	((*(sent) & FLPD_FLAG_MASK) == SLPD_FLAG)
#define lv1ent_section(sent)	((*(sent) & FLPD_FLAG_MASK) == SECT_FLAG)
#define lv1ent_offset(iova)	((iova) >> SECT_ORDER)

#define lv2table_base(sent)	((phys_addr_t)(*(sent) & ~0x3FU) << PG_ENT_SHIFT)
#define lv2ent_unmapped(pent)	((*(pent) & SLPD_FLAG_MASK) == UNMAPPED_FLAG)
#define lv2ent_small(pent)	((*(pent) & SLPD_FLAG_MASK) == SPAGE_FLAG)
#define lv2ent_large(pent)	((*(pent) & SLPD_FLAG_MASK) == LPAGE_FLAG)
#define lv2ent_offset(iova)	(((iova) & ~SECT_MASK) >> SPAGE_ORDER)

#define	PGBASE_TO_PHYS(pgent)	((phys_addr_t)(pgent) << PG_ENT_SHIFT)
#define ENT_TO_PHYS(ent)	((phys_addr_t)(*(ent)))
#define section_phys(sent)	PGBASE_TO_PHYS(ENT_TO_PHYS(sent) & SECT_ENT_MASK)
#define section_offs(iova)	((iova) & (SECT_SIZE - 1))
#define lpage_phys(pent)	PGBASE_TO_PHYS(ENT_TO_PHYS(pent) & LPAGE_ENT_MASK)
#define lpage_offs(iova)	((iova) & (LPAGE_SIZE - 1))
#define spage_phys(pent)	PGBASE_TO_PHYS(ENT_TO_PHYS(pent) & SPAGE_ENT_MASK)
#define spage_offs(iova)	((iova) & (SPAGE_SIZE - 1))

static inline sysmmu_pte_t *page_entry(sysmmu_pte_t *sent, sysmmu_iova_t iova)
{
	return (sysmmu_pte_t *)(phys_to_virt(lv2table_base(sent))) + lv2ent_offset(iova);
}

static inline sysmmu_pte_t *section_entry(sysmmu_pte_t *pgtable, sysmmu_iova_t iova)
{
	return pgtable + lv1ent_offset(iova);
}

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
	u32 vmid_mask;
	int max_vm;
	int num_pmmu;
	int attached_count;
};

struct sysmmu_clientdata {
	struct device *dev;
	struct sysmmu_drvdata **sysmmus;
	struct device_link **dev_link;
	int sysmmu_count;
};

#endif
