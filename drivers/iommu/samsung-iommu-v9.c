// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 */

#include <linux/dma-iommu.h>
#include <linux/iommu.h>
#include <linux/module.h>
#include <linux/of_iommu.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include "samsung-iommu-v9.h"

static struct iommu_ops samsung_sysmmu_ops;
static struct platform_driver samsung_sysmmu_driver_v9;

struct samsung_sysmmu_domain {
	struct iommu_domain domain;
};

static struct samsung_sysmmu_domain *to_sysmmu_domain(struct iommu_domain *dom)
{
	return container_of(dom, struct samsung_sysmmu_domain, domain);
}

static bool samsung_sysmmu_capable(enum iommu_cap cap)
{
	return cap == IOMMU_CAP_CACHE_COHERENCY;
}

static struct iommu_domain *samsung_sysmmu_domain_alloc(unsigned int type)
{
	struct samsung_sysmmu_domain *domain;

	if (type != IOMMU_DOMAIN_UNMANAGED &&
	    type != IOMMU_DOMAIN_DMA &&
	    type != IOMMU_DOMAIN_IDENTITY) {
		pr_err("invalid domain type %u\n", type);
		return NULL;
	}

	domain = kzalloc(sizeof(*domain), GFP_KERNEL);
	if (!domain)
		return NULL;

	if (type == IOMMU_DOMAIN_DMA) {
		int ret = iommu_get_dma_cookie(&domain->domain);

		if (ret) {
			pr_err("failed to get dma cookie (%d)\n", ret);
			kfree(domain);
			return NULL;
		}
	}

	return &domain->domain;
}

static void samsung_sysmmu_domain_free(struct iommu_domain *dom)
{
	struct samsung_sysmmu_domain *domain = to_sysmmu_domain(dom);

	iommu_put_dma_cookie(dom);
	kfree(domain);
}

static int samsung_sysmmu_attach_dev(struct iommu_domain *dom, struct device *dev)
{
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);

	if (!fwspec || fwspec->ops != &samsung_sysmmu_ops) {
		dev_err(dev, "failed to attach, IOMMU instance data %s.\n",
			!fwspec ? "is not initialized" : "has different ops");
		return -ENXIO;
	}

	if (!dev_iommu_priv_get(dev)) {
		dev_err(dev, "has no IOMMU\n");
		return -ENODEV;
	}

	return 0;
}

static int samsung_sysmmu_map(struct iommu_domain *dom, unsigned long l_iova, phys_addr_t paddr,
			      size_t size, int prot, gfp_t unused)
{
	return 0;
}

static size_t samsung_sysmmu_unmap(struct iommu_domain *dom, unsigned long l_iova, size_t size,
				   struct iommu_iotlb_gather *gather)
{
	return 0;
}

static void samsung_sysmmu_flush_iotlb_all(struct iommu_domain *dom)
{
}

static void samsung_sysmmu_iotlb_sync(struct iommu_domain *dom, struct iommu_iotlb_gather *gather)
{
}

static phys_addr_t samsung_sysmmu_iova_to_phys(struct iommu_domain *dom, dma_addr_t d_iova)
{
	return 0;
}

void samsung_sysmmu_dump_pagetable(struct device *dev, dma_addr_t iova)
{
}

static struct iommu_device *samsung_sysmmu_probe_device(struct device *dev)
{
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);

	if (!fwspec) {
		dev_dbg(dev, "IOMMU instance data is not initialized\n");
		return ERR_PTR(-ENODEV);
	}

	if (fwspec->ops != &samsung_sysmmu_ops) {
		dev_err(dev, "has different IOMMU ops\n");
		return ERR_PTR(-ENODEV);
	}

	return 0;
}

static void samsung_sysmmu_release_device(struct device *dev)
{
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);

	if (!fwspec || fwspec->ops != &samsung_sysmmu_ops)
		return;

	iommu_fwspec_free(dev);
}

static struct iommu_group *samsung_sysmmu_device_group(struct device *dev)
{
	if (device_iommu_mapped(dev))
		return iommu_group_get(dev);

	return generic_device_group(dev);
}

static int samsung_sysmmu_of_xlate(struct device *dev, struct of_phandle_args *args)
{
	struct platform_device *sysmmu = of_find_device_by_node(args->np);
	struct sysmmu_drvdata *data = platform_get_drvdata(sysmmu);
	struct iommu_fwspec *fwspec;
	unsigned int fwid = 0;
	int ret;

	ret = iommu_fwspec_add_ids(dev, &fwid, 1);
	if (ret) {
		dev_err(dev, "failed to add fwspec ids (err:%d)\n", ret);
		iommu_device_unlink(&data->iommu, dev);
		return ret;
	}

	fwspec = dev_iommu_fwspec_get(dev);

	return ret;
}

static struct iommu_ops samsung_sysmmu_ops = {
	.capable		= samsung_sysmmu_capable,
	.domain_alloc		= samsung_sysmmu_domain_alloc,
	.domain_free		= samsung_sysmmu_domain_free,
	.attach_dev		= samsung_sysmmu_attach_dev,
	.map			= samsung_sysmmu_map,
	.unmap			= samsung_sysmmu_unmap,
	.flush_iotlb_all	= samsung_sysmmu_flush_iotlb_all,
	.iotlb_sync		= samsung_sysmmu_iotlb_sync,
	.iova_to_phys		= samsung_sysmmu_iova_to_phys,
	.probe_device		= samsung_sysmmu_probe_device,
	.release_device		= samsung_sysmmu_release_device,
	.device_group		= samsung_sysmmu_device_group,
	.of_xlate		= samsung_sysmmu_of_xlate,
	.pgsize_bitmap		= -1UL << 12,
};

static int samsung_sysmmu_device_probe(struct platform_device *pdev)
{
	struct sysmmu_drvdata *data;
	struct device *dev = &pdev->dev;
	int err = 0;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = dev;
	platform_set_drvdata(pdev, data);

	err = iommu_device_sysfs_add(&data->iommu, data->dev, NULL, dev_name(dev));
	if (err) {
		dev_err(dev, "failed to register iommu in sysfs\n");
		return err;
	}

	iommu_device_set_ops(&data->iommu, &samsung_sysmmu_ops);
	iommu_device_set_fwnode(&data->iommu, dev->fwnode);

	err = iommu_device_register(&data->iommu);
	if (err) {
		dev_err(dev, "failed to register iommu\n");
		goto err_iommu_register;
	}

	if (!iommu_present(&platform_bus_type))
		bus_set_iommu(&platform_bus_type, &samsung_sysmmu_ops);

	dev_info(dev, "initialized IOMMU\n");
	return 0;

err_iommu_register:
	iommu_device_sysfs_remove(&data->iommu);
	return err;
}

static void samsung_sysmmu_device_shutdown(struct platform_device *pdev)
{
}

static const struct of_device_id sysmmu_of_match[] = {
	{ .compatible = "samsung,sysmmu-v9" },
	{ }
};

static struct platform_driver samsung_sysmmu_driver_v9 = {
	.driver	= {
		.name			= "samsung-sysmmu-v9",
		.of_match_table		= of_match_ptr(sysmmu_of_match),
		.suppress_bind_attrs	= true,
	},
	.probe	= samsung_sysmmu_device_probe,
	.shutdown = samsung_sysmmu_device_shutdown,
};
module_platform_driver(samsung_sysmmu_driver_v9);
MODULE_LICENSE("GPL v2");
