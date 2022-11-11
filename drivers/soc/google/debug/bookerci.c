// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2022 Google LLC
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <soc/google/exynos-el3_mon.h>

#define NODE_FILENAME_LEN	10
#define ERRCTRL_OFFSET		0x3008
#define ERRCTRL_NS_OFFSET	0x3108
#define ERR_RECORD_OFFSET	0x3000
#define ERR_RECORD_NS_OFFSET	0x3100
#define ERRGSR_OFFSET		0x3000
#define ERRGSR_NS_OFFSET	0x3100
#define ERRGSR_COUNT		6
#define ERRIRQ_COUNT		2
#define ERRGSR_ERROR_OFFSET	0
#define ERRGSR_FAULT_OFFSET	0x80
#define ERRCTRL_ENABLE_ALL	0x10F
#define ERRSTATUS_CLEAR_ALL	0xED800000


enum node_type {
	NODE_TYPE_DN = 1,
	NODE_TYPE_CONFIG,
	NODE_TYPE_DBG_TRACE,
	NODE_TYPE_HNI,
	NODE_TYPE_HNF,
	NODE_TYPE_XP,
	NODE_TYPE_SBSX,
	NODE_TYPE_MPAM_S,
	NODE_TYPE_MPAM_NS,
	NODE_TYPE_RNI = 0xA,
	NODE_TYPE_RND = 0xD,
	NODE_TYPE_RNF = 0xF,
	NODE_TYPE_MTSX = 0x10,
};

struct reg_desc {
	const char *name;
	int offset;
};

struct node_desc {
	const char *name;
	char file_name[NODE_FILENAME_LEN];
	int nid;
	int lid;
	enum node_type type;
	phys_addr_t pa;
	const struct reg_desc *regs;
	int regs_count;
};

struct bci_irq_desc {
	unsigned int irq;
	bool is_secure;
	bool is_error;
};

struct bci_dev {
	struct device *dev;
	struct platform_device *pdev;
	struct dentry *de;
	struct node_desc *nodes;
	struct bci_irq_desc *irqs;
	u32 base;
	u32 nodes_count;
	u32 irqs_count;
};

struct bci_dt_node {
	char *device_type;
	const char *name;
	const struct reg_desc *regs;
	int regs_count;
};

static const struct reg_desc cfg_regs[] = {
	{"por_cfgm_node_info", 0x0},
	{"por_cfgm_periph_id_0_periph_id_1", 0x8},
	{"por_cfgm_periph_id_2_periph_id_3", 0x10},
	{"por_cfgm_periph_id_4_periph_id_5", 0x18},
	{"por_cfgm_periph_id_6_periph_id_7", 0x20},
	{"por_cfgm_component_id_0_component_id_1", 0x28},
	{"por_cfgm_component_id_2_component_id_3", 0x30},
	{"por_cfgm_child_info", 0x80},
	{"por_cfgm_secure_access", 0x980},
	{"por_cfgm_errgsr0", 0x3000},
	{"por_cfgm_errgsr1", 0x3008},
	{"por_cfgm_errgsr2", 0x3010},
	{"por_cfgm_errgsr3", 0x3018},
	{"por_cfgm_errgsr4", 0x3020},
	{"por_cfgm_errgsr5", 0x3028},
	{"por_cfgm_errgsr6", 0x3080},
	{"por_cfgm_errgsr7", 0x3088},
	{"por_cfgm_errgsr8", 0x3090},
	{"por_cfgm_errgsr9", 0x3098},
	{"por_cfgm_errgsr10", 0x30A0},
	{"por_cfgm_errgsr11", 0x30A8},
	{"por_cfgm_errgsr0_NS", 0x3100},
	{"por_cfgm_errgsr1_NS", 0x3108},
	{"por_cfgm_errgsr2_NS", 0x3110},
	{"por_cfgm_errgsr3_NS", 0x3118},
	{"por_cfgm_errgsr4_NS", 0x3120},
	{"por_cfgm_errgsr5_NS", 0x3128},
	{"por_cfgm_errgsr6_NS", 0x3180},
	{"por_cfgm_errgsr7_NS", 0x3188},
	{"por_cfgm_errgsr8_NS", 0x3190},
	{"por_cfgm_errgsr9_NS", 0x3198},
	{"por_cfgm_errgsr10_NS", 0x31A0},
	{"por_cfgm_errgsr11_NS", 0x31A8},
	{"por_cfgm_errdevaff", 0x3FA8},
	{"por_cfgm_errdevarch", 0x3FB8},
	{"por_cfgm_erridr", 0x3FC8},
	{"por_cfgm_errpidr45", 0x3FD0},
	{"por_cfgm_errpidr67", 0x3FD8},
	{"por_cfgm_errpidr01", 0x3FE0},
	{"por_cfgm_errpidr23", 0x3FE8},
	{"por_cfgm_errcidr01", 0x3FF0},
	{"por_cfgm_errcidr23", 0x3FF8},
	{"por_info_global", 0x900},
	{"por_ppu_int_enable", 0x1C00},
	{"por_ppu_int_status", 0x1C08},
	{"por_ppu_qactive_hyst", 0x1C10},
	{"por_mpam_s_err_int_status", 0x1C18},
	{"por_mpam_ns_err_int_status", 0x1C20}
};

static const struct reg_desc hnf_regs[] = {
	{"por_hnf_node_info", 0x0},
	{"por_hnf_sam_control", 0xd00},
	{"por_hnf_sam_sn_properties", 0xd18},
	{"por_hnf_cfg_ctl", 0xa00},
	{"por_hnf_errfr", 0x3000},
	{"por_hnf_errctlr", 0x3008},
	{"por_hnf_errstatus", 0x3010},
	{"por_hnf_erraddr", 0x3018},
	{"por_hnf_errmisc", 0x3020},
	{"por_hnf_errfr_ns", 0x3100},
	{"por_hnf_errctlr_ns", 0x3108},
	{"por_hnf_errstatus_ns", 0x3110},
	{"por_hnf_erraddr_ns", 0x3118},
	{"por_hnf_errmisc_ns", 0x3120},
	{"por_hnf_err_inj", 0x3030},
	{"por_hnf_byte_par_err_inj", 0x3038},
};

static const struct reg_desc hnd_regs[] = {
	{"por_hni_node_info", 0x0},
	{"por_hni_sam_addrregion0_cfg", 0xc00},
	{"por_hni_errfr", 0x3000},
	{"por_hni_errctlr", 0x3008},
	{"por_hni_errstatus", 0x3010},
	{"por_hni_erraddr", 0x3018},
	{"por_hni_errmisc", 0x3020},
	{"por_hni_errfr_ns", 0x3100},
	{"por_hni_errctlr_ns", 0x3108},
	{"por_hni_errstatus_ns", 0x3110},
	{"por_hni_erraddr_ns", 0x3118},
	{"por_hni_errmisc_ns", 0x3120},
};

static const struct reg_desc mtsx_regs[] = {
	{"por_mtu_node_info", 0x0},
	{"mtu_tag_addr_ctl", 0xa40},
	{"mtu_tag_addr_base", 0xa48},
	{"mtu_tag_addr_shutter0", 0xa50},
	{"mtu_tag_addr_shutter1", 0xa58},
	{"por_mtu_errfr", 0x3000},
	{"por_mtu_errctlr", 0x3008},
	{"por_mtu_errstatus", 0x3010},
	{"por_mtu_erraddr", 0x3018},
	{"por_mtu_errmisc", 0x3020},
	{"por_mtu_errfr_ns", 0x3100},
	{"por_mtu_errctlr_ns", 0x3108},
	{"por_mtu_errstatus_ns", 0x3110},
	{"por_mtu_erraddr_ns", 0x3118},
	{"por_mtu_errmisc_ns", 0x3120},
	{"por_mtu_err_inj", 0x3030},
};

static const struct reg_desc rnf_regs[] = {
	{"rn_node_info", 0x0},
	{"sys_cache_grp_region0", 0xe00},
	{"sys_cache_group_hn_count", 0xea0},
	{"sys_cache_grp_hn_nodeid_reg0", 0xf00},
	{"non_hash_mem_region_reg0", 0x0c00},
	{"non_hash_mem_region_reg1", 0x0c08},
	{"non_hash_mem_region_reg2", 0x0c10},
	{"non_hash_mem_region_reg3", 0x0c18},
	{"non_hash_mem_region_reg4", 0x0c20},
	{"non_hash_mem_region_reg5", 0x0c28},
	{"non_hash_tgt_nodeid0", 0x0d80},
	{"non_hash_tgt_nodeid1", 0x0d88},
	{"rnsam_status", 0x1100}
};

static const struct reg_desc xp_regs[] = {
	{"xp_node_info", 0x0},
	{"xp_device_port_connect_info_p0", 0x8},
	{"xp_device_port_connect_info_p1", 0x10},
	{"xp_mesh_port_connect_info_east", 0x18},
	{"xp_mesh_port_connect_info_north", 0x20},
	{"xp_errfr", 0x3000},
	{"xp_errctlr", 0x3008},
	{"xp_errstatus", 0x3010},
	{"xp_errmisc", 0x3028},
	{"xp_errfr_ns", 0x3100},
	{"xp_errctlr_ns", 0x3108},
	{"xp_errstatus_ns", 0x3110},
	{"xp_errmisc_ns", 0x3128}
};

static const struct bci_dt_node dt_nodes[] = {
	{"booker-cfg", "CFG", cfg_regs, ARRAY_SIZE(cfg_regs)},
	{"booker-mtsx", "MTSX", mtsx_regs, ARRAY_SIZE(mtsx_regs)},
	{"booker-hnf", "HNF", hnf_regs, ARRAY_SIZE(hnf_regs)},
	{"booker-rnf", "RNF", rnf_regs, ARRAY_SIZE(rnf_regs)},
	{"booker-rnd", "RND", rnf_regs, ARRAY_SIZE(rnf_regs)},
	{"booker-hnd", "HND", hnd_regs, ARRAY_SIZE(hnd_regs)},
	{"booker-hni", "HNI", hnd_regs, ARRAY_SIZE(hnd_regs)},
	{"booker-xp", "XP", xp_regs, ARRAY_SIZE(xp_regs)}
};

static enum node_type errgsr_to_type_map[] = {
	NODE_TYPE_XP, NODE_TYPE_HNI, NODE_TYPE_HNF, NODE_TYPE_SBSX, NODE_TYPE_MTSX
};

static u64 read_bci_reg(phys_addr_t reg)
{
	u64 res = get_priv_reg(reg + 4);

	res <<= 32;
	res |= get_priv_reg(reg);
	return res;
}

static void write_bci_reg(phys_addr_t reg, u64 val)
{
	set_priv_reg(reg, val);
	set_priv_reg(reg + 4, val >> 32);
}

static int dump_regs_show(struct seq_file *s, void *p)
{
	struct node_desc *node = s->private;
	int i;

	seq_printf(s, "%s at %pap\n", node->file_name, &node->pa);
	seq_puts(s, "Reg                            Offset     Value\n");
	seq_puts(s, "------------------------------------------------------------\n");
	for (i = 0; i < node->regs_count; i++) {
		seq_printf(s, "%-30s %#010x %#018llx\n",
			   node->regs[i].name,
			   node->regs[i].offset,
			   read_bci_reg(node->pa + node->regs[i].offset));
	}
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(dump_regs);

static struct node_desc *find_node(struct bci_dev *bci, int errgsr_idx, int lid) {
	int i;

	for (i = 0; i < bci->nodes_count; i++) {
		if (lid == bci->nodes[i].lid &&
		    errgsr_to_type_map[errgsr_idx] == bci->nodes[i].type)
			return &bci->nodes[i];
	}
	return NULL;
}

static void handle_node_error(struct node_desc *node, struct bci_irq_desc *irq) {
	phys_addr_t pa = node->pa;

	pa += irq->is_secure ? ERR_RECORD_OFFSET : ERR_RECORD_NS_OFFSET;

	pr_err("%s triggered %ssecure %s:\n",
		node->file_name,
		irq->is_secure ? "" : "Non-",
		irq->is_error ? "error" : "fault");

	pr_err("ERR_FR     %#018llx\n", read_bci_reg(pa));
	pr_err("ERR_CTRL   %#018llx\n", read_bci_reg(pa + 8));
	pr_err("ERR_STATUS %#018llx\n", read_bci_reg(pa + 16));
	pr_err("ERR_ADDR   %#018llx\n", read_bci_reg(pa + 24));
	pr_err("ERR_MISC   %#018llx\n", read_bci_reg(pa + 32));

	/* clear interrupt */
	write_bci_reg(pa + 16, ERRSTATUS_CLEAR_ALL);
}

static void process_errgsr(struct bci_dev *bci, u64 errgsr,
			   int errgsr_idx, struct bci_irq_desc *irq_desc) {
	struct node_desc *node;
	int lid;

	/*
	 * Iterate over all set bits in the group.
	 * Every bit means error was triggered by the node
	 * which LogicID(lid) corresponds to bit position.
	 */
	for_each_set_bit(lid, (unsigned long *)&errgsr, BITS_PER_LONG_LONG) {
		node = find_node(bci, errgsr_idx, lid);
		if (node)
			handle_node_error(node, irq_desc);
	}
}

static struct bci_irq_desc *find_irq_desc(struct bci_dev *bci, int irq) {
	int i;

	for (i = 0; i < bci->irqs_count; i++)
		if (bci->irqs[i].irq == irq)
			return &bci->irqs[i];
	return NULL;
}

static irqreturn_t bci_irq_handler(int irq, void *data)
{
	int i;
	struct bci_dev *bci = (struct bci_dev *)data;
	struct bci_irq_desc *irq_desc = find_irq_desc(bci, irq);
	int errgsr_offset;

	if (!irq_desc)
		return IRQ_NONE;

	errgsr_offset = irq_desc->is_secure ? ERRGSR_OFFSET : ERRGSR_NS_OFFSET;
	errgsr_offset += irq_desc->is_error ? ERRGSR_ERROR_OFFSET : ERRGSR_FAULT_OFFSET;

	dev_err(bci->dev, "irq = %d, errgsr_offset = %#010x\n", irq, errgsr_offset);

	/* Check secure error groups */
	for (i = 0; i < ERRGSR_COUNT; i++) {
		u64 errgsr = read_bci_reg(bci->base + errgsr_offset + i * 8);
		process_errgsr(bci, errgsr, i, irq_desc);
	}

	return IRQ_HANDLED;
}

static int bci_init_debugfs(struct bci_dev *bci)
{
	int i;

	bci->de = debugfs_create_dir("booker-ci", NULL);
	if (IS_ERR_OR_NULL(bci->de))
		return -EIO;

	for (i = 0; i < bci->nodes_count; i++) {
		scnprintf(bci->nodes[i].file_name, NODE_FILENAME_LEN, "%s_%d",
			  bci->nodes[i].name, bci->nodes[i].nid);

		debugfs_create_file(bci->nodes[i].file_name, 0400, bci->de,
				    (void *)&bci->nodes[i], &dump_regs_fops);
	}
	return 0;
}

static int bci_init_dt(struct bci_dev *bci)
{
	struct device_node *np = bci->dev->of_node;
	int node_idx = 0;
	int i;

	if (of_property_read_u32(np, "reg_base", &bci->base))
		return -EINVAL;

	bci->nodes_count = of_get_child_count(np);
	bci->nodes = devm_kcalloc(bci->dev, bci->nodes_count,
				  sizeof(struct node_desc), GFP_KERNEL);
	if (!bci->nodes)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(dt_nodes); i++) {
		for_each_node_by_type(np, dt_nodes[i].device_type) {
			u32 id, offset;
			u64 node_info;
			phys_addr_t pa;

			if (of_property_read_u32(np, "id", &id))
				return -EINVAL;
			if (of_property_read_u32(np, "offset", &offset))
				return -EINVAL;

			pa = bci->base + offset;
			node_info = read_bci_reg(pa);
			bci->nodes[node_idx].name = dt_nodes[i].name;
			bci->nodes[node_idx].nid = id;
			bci->nodes[node_idx].lid = (node_info >> 32) & 0xffff;
			bci->nodes[node_idx].type = node_info & 0xffff;
			bci->nodes[node_idx].regs = dt_nodes[i].regs;
			bci->nodes[node_idx].regs_count = dt_nodes[i].regs_count;
			bci->nodes[node_idx].pa = pa;
			node_idx++;
		}
	}
	bci->nodes_count = node_idx;
	return 0;
}

static int bci_init_irq(struct bci_dev *bci)
{
	struct device_node *np = bci->dev->of_node;
	struct property *prop;
	const char *name;
	int i = 0, err = 0, irqs_count;

	irqs_count = of_property_count_strings(np, "interrupt-names");
	if (irqs_count <= 0) {
		dev_err(bci->dev, "invalid Interrupt-names property count(%d)\n", irqs_count);
		return -EINVAL;
	}
	bci->irqs_count = irqs_count;
	bci->irqs = devm_kcalloc(bci->dev, bci->irqs_count,
				  sizeof(struct bci_irq_desc), GFP_KERNEL);
	if (!bci->irqs)
		return -ENOMEM;

	of_property_for_each_string(np, "interrupt-names", prop, name) {
		unsigned int irq;
		u32 val;

		if (!name) {
			dev_err(bci->dev, "no such name\n");
			err = -EINVAL;
			break;
		}

		if (!of_property_read_u32_index(np, "interrupt-is-secure", i, &val)) {
			bci->irqs[i].is_secure = !!val;
		} else {
			dev_err(bci->dev, "cannot find is-secure value for irq\n");
			err = -EINVAL;
			break;
		}

		if (!of_property_read_u32_index(np, "interrupt-is-error", i, &val)) {
			bci->irqs[i].is_error = !!val;
		} else {
			dev_err(bci->dev, "cannot find is-error value for irq\n");
			err = -EINVAL;
			break;
		}

		irq = platform_get_irq(bci->pdev, i);
		err = devm_request_irq(bci->dev, irq, bci_irq_handler,
					IRQF_NOBALANCING | IRQF_NO_THREAD, name, bci);
		if (err) {
			dev_err(bci->dev,
				"unable to request irq%u for BCI handler[%s]\n",
				irq, name);
			break;
		}
		bci->irqs[i].irq = irq;
		dev_info(bci->dev,
			"Success to request irq%u for BCI handler[%s]\n",
			irq, name);
		enable_irq(irq);
		i++;
	}

	/* Enable detection all errors and faults for nodes which can to do that */
	for (i = 0; i < bci->nodes_count; i++) {
		if (bci->nodes[i].type == NODE_TYPE_MTSX ||
		    bci->nodes[i].type == NODE_TYPE_HNF ||
		    bci->nodes[i].type == NODE_TYPE_HNI) {
			write_bci_reg(bci->nodes[i].pa + ERRCTRL_OFFSET,
				      ERRCTRL_ENABLE_ALL);
			write_bci_reg(bci->nodes[i].pa + ERRCTRL_NS_OFFSET,
				      ERRCTRL_ENABLE_ALL);
		}
	}

	return err;
}

static int bci_probe(struct platform_device *pdev)
{
	struct bci_dev *bci;
	int ret;

	bci = devm_kzalloc(&pdev->dev, sizeof(struct bci_dev), GFP_KERNEL);
	if (!bci)
		return -ENOMEM;

	bci->pdev = pdev;
	bci->dev = &pdev->dev;

	ret = bci_init_dt(bci);
	if (ret < 0)
		return ret;

	ret = bci_init_debugfs(bci);
	if (ret < 0)
		return ret;

	ret = bci_init_irq(bci);
	if (ret < 0)
		return ret;

	platform_set_drvdata(pdev, bci);

	return ret;
}

static int bci_remove(struct platform_device *pdev)
{
	struct bci_dev *bci = platform_get_drvdata(pdev);

	debugfs_remove_recursive(bci->de);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static const struct of_device_id bci_dt_match[] = {
	{.compatible = "google,booker-ci",
	 .data = NULL,},
	{},
};
MODULE_DEVICE_TABLE(of, bci_dt_match);

static struct platform_driver bci_driver = {
	.probe = bci_probe,
	.remove = bci_remove,
	.driver = {
		.name = "booker-ci",
		.of_match_table = bci_dt_match,
		},
};
module_platform_driver(bci_driver);

MODULE_DESCRIPTION("Booker-CI driver");
MODULE_AUTHOR("Ivan Zaitsev <zaitsev@google.com>");
MODULE_LICENSE("GPL");
