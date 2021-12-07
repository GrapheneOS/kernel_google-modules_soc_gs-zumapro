/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 */

#ifndef __SAMSUNG_IOMMU_V9_H
#define __SAMSUNG_IOMMU_V9_H

struct sysmmu_drvdata {
	struct iommu_device iommu;
	struct device *dev;
};

#endif
