// SPDX-License-Identifier: GPL-2.0-only

#include <linux/iova.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <trace/hooks/iommu.h>

static void iommu_alloc_insert_iova(void *unused, struct iova_domain *iovad, unsigned long size,
				    unsigned long limit_pfn, struct iova *new_iova,
				    bool size_aligned, int *ret)
{
	if (!iovad || !ret)
		return;

	if (iovad->android_vendor_data1 == 0) {
		// use default
		*ret = 1;
		return;
	}

	// Call best fit function here. Before linked with best fit, set ret = 0 to use the default
	// one for now
	*ret = 1;
}

static void iommu_iovad_init_alloc_algo(void *unused, struct device *dev, struct iova_domain *iovad)
{
	if (of_property_read_bool(dev->of_node, "iommu-best-fit-algo") ||
	    of_property_read_bool(dev->of_node, "lwis,iommu-best-fit-algo")) {
		iovad->android_vendor_data1 = 1;
		dev_info(dev, "using IOVA best fit algorithm.");
	}
}

static int __init iovad_best_fit_algo_init(void)
{
	register_trace_android_rvh_iommu_alloc_insert_iova(iommu_alloc_insert_iova, NULL);
	register_trace_android_rvh_iommu_iovad_init_alloc_algo(iommu_iovad_init_alloc_algo, NULL);

	return 0;
}

module_init(iovad_best_fit_algo_init);
MODULE_SOFTDEP("post: samsung_iommu_v9");
MODULE_SOFTDEP("post: samsung_iommu");
MODULE_DESCRIPTION("Google Pixel Best Fit IOVA Module");
MODULE_LICENSE("GPL");
