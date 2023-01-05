// SPDX-License-Identifier: GPL-2.0
/*
 * google_bcl_qos.c Google bcl PMQOS driver
 *
 * Copyright (c) 2023, Google LLC. All rights reserved.
 *
 */

#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/mfd/samsung/s2mpg1415.h>
#include <linux/mfd/samsung/s2mpg1415-register.h>
#include <soc/google/bcl.h>

void google_bcl_qos_update(struct bcl_device *bcl_dev, int id, bool throttle)
{
	if (id > TRIGGERED_SOURCE_MAX || id < 0 || !bcl_dev->bcl_qos[id])
		return;
	freq_qos_update_request(&bcl_dev->bcl_qos[id]->cpu0_max_qos_req,
				throttle ? bcl_dev->bcl_qos[id]->cpu0_limit : INT_MAX);
	freq_qos_update_request(&bcl_dev->bcl_qos[id]->cpu1_max_qos_req,
				throttle ? bcl_dev->bcl_qos[id]->cpu1_limit : INT_MAX);
	freq_qos_update_request(&bcl_dev->bcl_qos[id]->cpu2_max_qos_req,
				throttle ? bcl_dev->bcl_qos[id]->cpu2_limit : INT_MAX);
	exynos_pm_qos_update_request(&bcl_dev->bcl_qos[id]->tpu_qos_max,
				     throttle ? bcl_dev->bcl_qos[id]->tpu_limit : INT_MAX);
}

static int init_freq_qos(struct bcl_device *bcl_dev, struct qos_throttle_limit *throttle)
{
	struct cpufreq_policy *policy = NULL;
	int ret;

	policy = cpufreq_cpu_get(bcl_dev->cpu0_cluster);

	ret = freq_qos_add_request(&policy->constraints, &throttle->cpu0_max_qos_req,
				   FREQ_QOS_MAX, INT_MAX);
	cpufreq_cpu_put(policy);
	if (ret < 0)
		return ret;

	policy = cpufreq_cpu_get(bcl_dev->cpu1_cluster);
	ret = freq_qos_add_request(&policy->constraints, &throttle->cpu1_max_qos_req,
				   FREQ_QOS_MAX, INT_MAX);
	cpufreq_cpu_put(policy);
	if (ret < 0)
		return ret;

	policy = cpufreq_cpu_get(bcl_dev->cpu2_cluster);
	ret = freq_qos_add_request(&policy->constraints, &throttle->cpu2_max_qos_req,
				   FREQ_QOS_MAX, INT_MAX);
	cpufreq_cpu_put(policy);
	if (ret < 0)
		return ret;

	return ret;
}

int google_bcl_setup_qos(struct bcl_device *bcl_dev)
{
	int ret, i;

	for (i = 0; i < TRIGGERED_SOURCE_MAX; i++) {
		if (!bcl_dev->bcl_qos[i])
			continue;

		ret = init_freq_qos(bcl_dev, bcl_dev->bcl_qos[i]);
		if (ret < 0) {
			dev_err(bcl_dev->device, "Cannot init pm qos on %d for cpu0\n", i);
			return ret;
		}
		exynos_pm_qos_add_request(&bcl_dev->bcl_qos[i]->tpu_qos_max, PM_QOS_TPU_FREQ_MAX,
					  INT_MAX);
	}
	return 0;
}

void google_bcl_remove_qos(struct bcl_device *bcl_dev)
{
	int i;

	for (i = 0; i < TRIGGERED_SOURCE_MAX; i++) {
		if (!bcl_dev->bcl_qos[i])
			continue;
		freq_qos_remove_request(&bcl_dev->bcl_qos[i]->cpu0_max_qos_req);
		freq_qos_remove_request(&bcl_dev->bcl_qos[i]->cpu1_max_qos_req);
		freq_qos_remove_request(&bcl_dev->bcl_qos[i]->cpu2_max_qos_req);
		exynos_pm_qos_remove_request(&bcl_dev->bcl_qos[i]->tpu_qos_max);
		bcl_dev->bcl_qos[i] = NULL;
	}
}
