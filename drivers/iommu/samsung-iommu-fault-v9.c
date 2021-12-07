// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 */

#define pr_fmt(fmt) "sysmmu: " fmt

#include <linux/smc.h>
#include <linux/arm-smccc.h>
#include "samsung-iommu-v9.h"

#define SYSMMU_FAULT_PTW_ACCESS		0
#define SYSMMU_FAULT_PAGE		1
#define SYSMMU_FAULT_ACCESS		2
#define SYSMMU_FAULT_CONTEXT		3
#define SYSMMU_FAULT_UNKNOWN		4
#define SYSMMU_FAULTS_NUM		(SYSMMU_FAULT_UNKNOWN + 1)

#define REG_MMU_FAULT_STATUS_VM		0x8060
#define REG_MMU_FAULT_CLEAR_VM		0x8064
#define REG_MMU_FAULT_VA_VM		0x8070
#define REG_MMU_FAULT_INFO0_VM		0x8074
#define REG_MMU_FAULT_INFO1_VM		0x8078
#define REG_MMU_FAULT_INFO2_VM		0x807C

#define REG_MMU_FAULT_RW_MASK			GENMASK(20, 20)
#define IS_READ_FAULT(x)			(((x) & REG_MMU_FAULT_RW_MASK) == 0)

#define MMU_FAULT_INFO_VA_36(reg)		(((reg) >> 21) & 0x1)
#define MMU_FAULT_INFO_VA_HIGH(reg)		(((reg) & 0x3C00000) << 10)

#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)
#define SMC_DRM_SEC_SMMU_INFO          (0x820020D0)
/* secure SysMMU SFR access */
enum sec_sysmmu_sfr_access_t {
	SEC_SMMU_SFR_READ,
	SEC_SMMU_SFR_WRITE,
};

#define is_secure_info_fail(x)		((((x) >> 16) & 0xffff) == 0xdead)
static inline u32 read_sec_info(unsigned int addr)
{
	struct arm_smccc_res res;

	arm_smccc_smc(SMC_DRM_SEC_SMMU_INFO,
		      (unsigned long)addr, 0, SEC_SMMU_SFR_READ, 0, 0, 0, 0,
		      &res);
	if (is_secure_info_fail(res.a0))
		pr_err("Invalid value returned, %#lx\n", res.a0);

	return (u32)res.a0;
}
#else
static inline u32 read_sec_info(unsigned int addr)
{
	return 0xdead;
}
#endif

static char *sysmmu_fault_name[SYSMMU_FAULTS_NUM] = {
	"PTW ACCESS FAULT",
	"PAGE FAULT",
	"ACCESS FAULT",
	"CONTEXT FAULT",
	"UNKNOWN FAULT"
};

static int sysmmu_fault_type[SYSMMU_FAULTS_NUM] = {
	IOMMU_FAULT_REASON_WALK_EABT,
	IOMMU_FAULT_REASON_PTE_FETCH,
	IOMMU_FAULT_REASON_ACCESS,
	IOMMU_FAULT_REASON_PASID_FETCH,
	IOMMU_FAULT_REASON_UNKNOWN,
};

struct samsung_sysmmu_fault_info {
	struct sysmmu_drvdata *drvdata;
	struct iommu_fault_event event;
};

static inline u32 __sysmmu_get_intr_status(struct sysmmu_drvdata *data, bool is_secure, int *vmid)
{
	int i;
	u32 val = 0x0;

	for (i = 0; i < data->max_vm; i++) {
		if (!(data->vmid_mask & (1 << i)))
			continue;

		if (is_secure)
			val = read_sec_info(MMU_VM_ADDR(data->secure_base +
					    REG_MMU_FAULT_STATUS_VM, i));
		else
			val = readl_relaxed(MMU_VM_ADDR(data->sfrbase +
					    REG_MMU_FAULT_STATUS_VM, i));

		if (val & 0xF) {
			*vmid = i;
			break;
		}
	}

	return val;
}

static inline u64 __sysmmu_get_fault_address(struct sysmmu_drvdata *data, bool is_secure, int vmid)
{
	u64 va = 0x0;
	u32 val;

	if (is_secure) {
		va = read_sec_info(MMU_VM_ADDR(data->secure_base + REG_MMU_FAULT_VA_VM, vmid));
		val = read_sec_info(MMU_VM_ADDR(data->secure_base + REG_MMU_FAULT_INFO0_VM, vmid));
	} else {
		va = readl_relaxed(MMU_VM_ADDR(data->sfrbase + REG_MMU_FAULT_VA_VM, vmid));
		val = readl_relaxed(MMU_VM_ADDR(data->sfrbase + REG_MMU_FAULT_INFO0_VM, vmid));
	}

	if (MMU_FAULT_INFO_VA_36(val))
		va += MMU_FAULT_INFO_VA_HIGH(val);

	return va;
}

static void sysmmu_show_secure_fault_information(struct sysmmu_drvdata *drvdata, int intr_type,
						 unsigned long fault_addr, int vmid)
{
	const char *port_name = NULL;
	unsigned int info0, info1, info2;
	phys_addr_t pgtable;
	unsigned int sfrbase = drvdata->secure_base;

	pgtable = read_sec_info(MMU_VM_ADDR(sfrbase + REG_MMU_CONTEXT0_CFG_FLPT_BASE_VM, vmid));
	pgtable <<= PAGE_SHIFT;

	info0 = read_sec_info(MMU_VM_ADDR(sfrbase + REG_MMU_FAULT_INFO0_VM, vmid));
	info1 = read_sec_info(MMU_VM_ADDR(sfrbase + REG_MMU_FAULT_INFO1_VM, vmid));
	info2 = read_sec_info(MMU_VM_ADDR(sfrbase + REG_MMU_FAULT_INFO2_VM, vmid));

	of_property_read_string(drvdata->dev->of_node, "port-name", &port_name);

	pr_crit("----------------------------------------------------------\n");
	pr_crit("From [%s], SysMMU %s %s at %#010lx (page table @ %pa)\n",
		port_name ? port_name : dev_name(drvdata->dev),
		IS_READ_FAULT(info0) ? "READ" : "WRITE",
		sysmmu_fault_name[intr_type], fault_addr, &pgtable);

	if (intr_type == SYSMMU_FAULT_UNKNOWN) {
		pr_crit("The fault is not caused by this System MMU.\n");
		pr_crit("Please check IRQ and SFR base address.\n");
		goto finish;
	}

	if (intr_type == SYSMMU_FAULT_CONTEXT) {
		pr_crit("Context fault\n");
		goto finish;
	}

	pr_crit("ASID: %#x, Burst LEN: %#x, AXI ID: %#x, PMMU ID: %#x, STREAM ID: %#x\n",
		info0 & 0xFFFF, (info0 >> 16) & 0xF, info1, (info2 >> 24) & 0xFF,
		info2 & 0xFFFFFF);

	if (!pfn_valid(pgtable >> PAGE_SHIFT)) {
		pr_crit("Page table base is not in a valid memory region\n");
		pgtable = 0;
	}

	if (intr_type == SYSMMU_FAULT_PTW_ACCESS) {
		pr_crit("System MMU has failed to access page table\n");
		pgtable = 0;
	}

	info0 = MMU_VERSION_RAW(read_sec_info(sfrbase + REG_MMU_VERSION));

	pr_crit("ADDR: %#x, MMU_CTRL: %#010x, PT_BASE: %#010x\n",
		sfrbase,
		read_sec_info(sfrbase + REG_MMU_CTRL),
		read_sec_info(MMU_VM_ADDR(sfrbase + REG_MMU_CONTEXT0_CFG_FLPT_BASE_VM, vmid)));
	pr_crit("VERSION %d.%d.%d, MMU_STATUS: %#010x\n",
		MMU_VERSION_MAJOR(info0), MMU_VERSION_MINOR(info0), MMU_VERSION_REVISION(info0),
		read_sec_info(sfrbase + REG_MMU_STATUS));

finish:
	pr_crit("----------------------------------------------------------\n");
}

static void sysmmu_show_fault_info_simple(struct sysmmu_drvdata *drvdata, int intr_type,
					  unsigned long fault_addr, phys_addr_t *pt, int vmid)
{
	const char *port_name = NULL;
	u32 info0, info1, info2;
	phys_addr_t pgtable;

	pgtable = readl_relaxed(MMU_VM_ADDR(drvdata->sfrbase + REG_MMU_CONTEXT0_CFG_FLPT_BASE_VM,
					    vmid));
	pgtable <<= PAGE_SHIFT;

	info0 = readl_relaxed(MMU_VM_ADDR(drvdata->sfrbase + REG_MMU_FAULT_INFO0_VM, vmid));
	info1 = readl_relaxed(MMU_VM_ADDR(drvdata->sfrbase + REG_MMU_FAULT_INFO1_VM, vmid));
	info2 = readl_relaxed(MMU_VM_ADDR(drvdata->sfrbase + REG_MMU_FAULT_INFO2_VM, vmid));

	of_property_read_string(drvdata->dev->of_node, "port-name", &port_name);

	pr_crit("From [%s], SysMMU %s %s at %#010lx (pgtable @ %pa\n",
		port_name ? port_name : dev_name(drvdata->dev),
		IS_READ_FAULT(info0) ? "READ" : "WRITE",
		sysmmu_fault_name[intr_type], fault_addr, &pgtable);

	pr_crit("ASID: %#x, Burst LEN: %#x, AXI ID: %#x, PMMU ID: %#x, STREAM ID: %#x\n",
		info0 & 0xFFFF, (info0 >> 16) & 0xF, info1, (info2 >> 24) & 0xFF,
		info2 & 0xFFFFFF);
	if (pt)
		*pt = pgtable;
}

static void sysmmu_show_fault_information(struct sysmmu_drvdata *drvdata, int intr_type,
					  unsigned long fault_addr, int vmid)
{
	phys_addr_t pgtable;

	pr_crit("----------------------------------------------------------\n");
	sysmmu_show_fault_info_simple(drvdata, intr_type, fault_addr, &pgtable, vmid);

	if (intr_type == SYSMMU_FAULT_UNKNOWN) {
		pr_crit("The fault is not caused by this System MMU.\n");
		pr_crit("Please check IRQ and SFR base address.\n");
		goto finish;
	}

	if (intr_type == SYSMMU_FAULT_CONTEXT) {
		pr_crit("Context fault\n");
		goto finish;
	}

	if (pgtable != drvdata->pgtable)
		pr_crit("Page table base of driver: %p\n", &drvdata->pgtable);

	if (!pfn_valid(pgtable >> PAGE_SHIFT)) {
		pr_crit("Page table base is not in a valid memory region\n");
		pgtable = 0;
	} else {
		sysmmu_pte_t *ent;

		ent = section_entry(phys_to_virt(pgtable), fault_addr);
		pr_crit("Lv1 entry: %#010x\n", *ent);

		if (lv1ent_page(ent)) {
			ent = page_entry(ent, fault_addr);
			pr_crit("Lv2 entry: %#010x\n", *ent);
		}
	}

	if (intr_type == SYSMMU_FAULT_PTW_ACCESS) {
		pr_crit("System MMU has failed to access page table\n");
		pgtable = 0;
	}

finish:
	pr_crit("----------------------------------------------------------\n");
}

static void sysmmu_get_interrupt_info(struct sysmmu_drvdata *data,
				      int *intr_type, unsigned long *addr,
				      int *vmid, bool is_secure)
{
	*intr_type = ffs(__sysmmu_get_intr_status(data, is_secure, vmid));
	*addr = __sysmmu_get_fault_address(data, is_secure, *vmid);
}

static void sysmmu_clear_interrupt(struct sysmmu_drvdata *data, int *vmid)
{
	u32 val = __sysmmu_get_intr_status(data, false, vmid);

	writel(val, MMU_VM_ADDR(data->sfrbase + REG_MMU_FAULT_CLEAR_VM, *vmid));
}

irqreturn_t samsung_sysmmu_irq(int irq, void *dev_id)
{
	int itype, vmid;
	unsigned long addr;
	struct sysmmu_drvdata *drvdata = dev_id;
	bool is_secure = (irq == drvdata->secure_irq);

	dev_info(drvdata->dev, "[%s] interrupt (%d) happened\n",
		 is_secure ? "Secure" : "Non-secure", irq);

	if (drvdata->async_fault_mode)
		return IRQ_WAKE_THREAD;

	sysmmu_get_interrupt_info(drvdata, &itype, &addr, &vmid, is_secure);
	if (is_secure)
		sysmmu_show_secure_fault_information(drvdata, itype, addr, vmid);
	else
		sysmmu_show_fault_information(drvdata, itype, addr, vmid);

	return IRQ_WAKE_THREAD;
}

static int samsung_sysmmu_fault_notifier(struct device *dev, void *data)
{
	struct samsung_sysmmu_fault_info *fi;
	struct sysmmu_clientdata *client;
	struct sysmmu_drvdata *drvdata;
	int i, ret, result = 0;

	fi = (struct samsung_sysmmu_fault_info *)data;
	drvdata = fi->drvdata;

	client = (struct sysmmu_clientdata *)dev_iommu_priv_get(dev);

	for (i = 0; i < client->sysmmu_count; i++) {
		if (drvdata == client->sysmmus[i]) {
			ret = iommu_report_device_fault(dev, &fi->event);
			if (ret == -EAGAIN)
				result = ret;
			break;
		}
	}

	return result;
}

irqreturn_t samsung_sysmmu_irq_thread(int irq, void *dev_id)
{
	int itype, vmid, ret;
	unsigned long addr;
	struct sysmmu_drvdata *drvdata = dev_id;
	bool is_secure = (irq == drvdata->secure_irq);
	struct iommu_group *group = drvdata->group;
	enum iommu_fault_reason reason;
	struct samsung_sysmmu_fault_info fi = {
		.drvdata = drvdata,
		.event.fault.type = IOMMU_FAULT_DMA_UNRECOV,
	};

	sysmmu_get_interrupt_info(drvdata, &itype, &addr, &vmid, is_secure);
	reason = sysmmu_fault_type[itype];

	fi.event.fault.event.addr = addr;
	fi.event.fault.event.reason = reason;
	if (reason == IOMMU_FAULT_REASON_PTE_FETCH ||
	    reason == IOMMU_FAULT_REASON_PERMISSION)
		fi.event.fault.type = IOMMU_FAULT_PAGE_REQ;

	ret = iommu_group_for_each_dev(group, &fi,
				       samsung_sysmmu_fault_notifier);
	if (ret == -EAGAIN && !is_secure) {
		sysmmu_show_fault_info_simple(drvdata, itype, addr, NULL, vmid);
		sysmmu_clear_interrupt(drvdata, &vmid);
		return IRQ_HANDLED;
	}

	if (drvdata->async_fault_mode) {
		if (is_secure)
			sysmmu_show_secure_fault_information(drvdata, itype, addr, vmid);
		else
			sysmmu_show_fault_information(drvdata, itype, addr, vmid);
	}
	panic("Unrecoverable System MMU Fault!!");

	return IRQ_HANDLED;
}
