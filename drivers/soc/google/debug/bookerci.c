// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2022 Google LLC
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/io.h>
#include <soc/google/exynos-el3_mon.h>

#define NODE_FILENAME_LEN	10
struct reg_desc {
	const char *name;
	int offset;
};

struct node_desc {
	const char *name;
	char file_name[NODE_FILENAME_LEN];
	int nid;
	phys_addr_t pa;
	const struct reg_desc *regs;
	int regs_count;
};

struct bci_dev {
	struct device *dev;
	struct dentry *de;
	struct node_desc *nodes;
	u32 base;
	u32 nodes_count;
};

struct bci_dt_node {
	char *device_type;
	const char *name;
	const struct reg_desc *regs;
	int regs_count;
};

static const struct reg_desc hnf_regs[] = {
	{"por_hnf_sam_control", 0xd00},
	{"por_hnf_sam_sn_properties", 0xd18},
	{"por_hnf_cfg_ctl", 0xa00}
};

static const struct reg_desc hnd_regs[] = {
	{"por_hni_sam_addrregion0_cfg", 0xc00}
};

static const struct reg_desc mtsx_regs[] = {
	{"mtu_tag_addr_ctl", 0xa40},
	{"mtu_tag_addr_base", 0xa48},
	{"mtu_tag_addr_shutter0", 0xa50},
	{"mtu_tag_addr_shutter1", 0xa58}
};

static const struct reg_desc rnf_regs[] = {
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

static const struct bci_dt_node dt_nodes[] = {
	{"booker-mtsx", "MTSX", mtsx_regs, ARRAY_SIZE(mtsx_regs)},
	{"booker-hnf", "HNF", hnf_regs, ARRAY_SIZE(hnf_regs)},
	{"booker-rnf", "RNF", rnf_regs, ARRAY_SIZE(rnf_regs)},
	{"booker-rnd", "RND", rnf_regs, ARRAY_SIZE(rnf_regs)},
	{"booker-hnd", "HND", hnd_regs, ARRAY_SIZE(hnd_regs)},
	{"booker-hni", "HNI", hnd_regs, ARRAY_SIZE(hnd_regs)}
};

static const struct reg_desc hni_regs[] = {
	{"por_hni_sam_addrregion0_cfg", 0xc00}
};

static u64 read_bci_reg(phys_addr_t reg)
{
	u64 res = get_priv_reg(reg + 4);

	res <<= 32;
	res |= get_priv_reg(reg);
	return res;
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

			if (of_property_read_u32(np, "id", &id))
				return -EINVAL;
			if (of_property_read_u32(np, "offset", &offset))
				return -EINVAL;

			bci->nodes[node_idx].name = dt_nodes[i].name;
			bci->nodes[node_idx].nid = id;
			bci->nodes[node_idx].regs = dt_nodes[i].regs;
			bci->nodes[node_idx].regs_count = dt_nodes[i].regs_count;
			bci->nodes[node_idx].pa = bci->base + offset;
			node_idx++;
		}
	}
	bci->nodes_count = node_idx;
	return 0;
}

static int bci_probe(struct platform_device *pdev)
{
	struct bci_dev *bci;
	int ret;

	bci = devm_kzalloc(&pdev->dev, sizeof(struct bci_dev), GFP_KERNEL);
	if (!bci)
		return -ENOMEM;

	bci->dev = &pdev->dev;

	ret = bci_init_dt(bci);
	if (ret < 0)
		return ret;

	ret = bci_init_debugfs(bci);
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
