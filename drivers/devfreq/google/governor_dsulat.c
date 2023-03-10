// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022, The Linux Foundation. All rights reserved.
 */

#define pr_fmt(fmt) "dsu_lat: " fmt

#include <linux/kernel.h>
#include <linux/sizes.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/time.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/devfreq.h>
#include "governor.h"
#include "governor_memlat.h"
#include "governor_dsulat.h"


#define CREATE_TRACE_POINTS
#include "governor_dsulat_trace.h"

static int dsulat_use_cnt;
static DEFINE_MUTEX(state_lock);

#define show_attr(name) \
static ssize_t show_##name(struct device *dev,                          \
			struct device_attribute *attr, char *buf)       \
{                                                                       \
	struct devfreq *df = to_devfreq(dev);                           \
	struct dsulat_node *dsu_node = df->data;                        \
	return sysfs_emit_at(buf, 0, "%u\n", dsu_node->name);           \
}

#define store_attr(name, _min, _max) \
static ssize_t store_##name(struct device *dev,                         \
			struct device_attribute *attr, const char *buf, \
			size_t count)                                   \
{                                                                       \
	struct devfreq *df = to_devfreq(dev);                           \
	struct dsulat_node *dsu_node = df->data;                        \
	int ret;                                                        \
	unsigned int val;                                               \
	ret = kstrtouint(buf, 10, &val);                                \
	if (ret)                                                        \
		return ret;                                             \
	val = max(val, _min);                                           \
	val = min(val, _max);                                           \
	dsu_node->name = val;                                           \
	return count;                                                   \
}

static ssize_t freq_map_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct devfreq *df = to_devfreq(dev);
	struct dsulat_node *dsu_node = df->data;
	struct core_dev_map *map_cl0 = dsu_node->freq_map_cl0;
	struct core_dev_map *map_cl1 = dsu_node->freq_map_cl1;
	struct core_dev_map *map_cl2 = dsu_node->freq_map_cl2;
	unsigned int cnt = 0;

	cnt += sysfs_emit_at(buf, cnt, "Cluster freq (MHz)\tDevice Freq(MHz)\n");

	while (map_cl0->core_mhz && cnt < PAGE_SIZE) {
		cnt += sysfs_emit_at(buf, cnt, "cl0: %10u\t%9u\n",
			map_cl0->core_mhz, map_cl0->target_freq);
		map_cl0++;
	}
	while (map_cl1->core_mhz && cnt < PAGE_SIZE) {
		cnt += sysfs_emit_at(buf, cnt, "cl1: %10u\t%9u\n",
			map_cl1->core_mhz, map_cl1->target_freq);
		map_cl1++;
	}
	while (map_cl2->core_mhz && cnt < PAGE_SIZE) {
		cnt += sysfs_emit_at(buf, cnt, "cl2: %10u\t%9u\n",
			map_cl2->core_mhz, map_cl2->target_freq);
		map_cl2++;
	}
	if (cnt < PAGE_SIZE)
		cnt += sysfs_emit_at(buf, cnt, "\n");

	return cnt;
}

static DEVICE_ATTR_RO(freq_map);

static unsigned long core_to_dev_freq(int cpu, struct dsulat_node *node,
				      unsigned long coref)
{
	struct core_dev_map *map;
	unsigned long freq = 0;

	if (cpu < CONFIG_VH_MID_CAPACITY_CPU)
		map = node->freq_map_cl0;
	else if (cpu < CONFIG_VH_MAX_CAPACITY_CPU)
		map = node->freq_map_cl1;
	else
		map = node->freq_map_cl2;

	if (!map)
		goto out;

	while (map->core_mhz && map->core_mhz < coref)
		map++;
	if (!map->core_mhz)
		map--;
	freq = map->target_freq;

out:
	pr_debug("freq: %lu -> dev: %lu\n", coref, freq);
	return freq;
}

static int start_monitor(struct devfreq *df)
{
	struct dsulat_node *node = df->data;
	node->mon_started = true;

	return 0;
}

static void stop_monitor(struct devfreq *df)
{
	struct dsulat_node *node = df->data;
	node->mon_started = false;
}


static int gov_start(struct devfreq *df)
{
	int ret = 0;

	struct dsulat_node *node = df->data;
	ret = start_monitor(df);
	if (ret)
		goto err_start;

	ret = sysfs_create_group(&df->dev.kobj, node->attr_grp);
	if (ret)
		goto err_sysfs;

	return 0;

err_sysfs:
	stop_monitor(df);
err_start:
	return ret;
}

static int gov_suspend(struct devfreq *df)
{
	struct dsulat_node *node = df->data;
	unsigned long prev_freq = df->previous_freq;

	node->mon_started = false;

	mutex_lock(&df->lock);
	update_devfreq(df);
	mutex_unlock(&df->lock);

	node->resume_freq = max(prev_freq, 1UL);

	return 0;
}

static int gov_resume(struct devfreq *df)
{
	struct dsulat_node *node = df->data;

	mutex_lock(&df->lock);
	update_devfreq(df);
	mutex_unlock(&df->lock);

	node->resume_freq = 0;
	node->mon_started = true;

	return 0;
}

static void gov_stop(struct devfreq *df)
{
	struct dsulat_node *node = df->data;

	sysfs_remove_group(&df->dev.kobj, node->attr_grp);
	stop_monitor(df);
	df->data = node->orig_data;
	node->orig_data = NULL;
}

extern int get_ev_data(int cpu, unsigned long *inst, unsigned long *cyc,
		unsigned long *stall, unsigned long *l2_cachemiss, unsigned long *l3_cachemiss,
		unsigned long *mem_stall, unsigned long *l2_cache_wb, unsigned long *l3_cache_access,
		unsigned long *mem_count, unsigned long *freq);

static int devfreq_dsulat_get_freq(struct devfreq *df,
				   unsigned long *target_freq)
{
	int cpu;
	unsigned long inst, cyc, stall, l2_cachemiss, l3_cachemiss, freq, mem_stall, mem_count;
	unsigned long l2_cache_wb, l3_cache_access, wb_pct;
	struct dsulat_node *node = df->data;
	unsigned long max_freq = 0, dsu_freq = 0;
	unsigned int ratio, ratio_ceil, wb_pct_thres, wb_filter_ratio;
	int ret;

	/*
	 * node->resume_freq is set to 0 at the end of resume (after the update)
	 * and is set to df->prev_freq at the end of suspend (after the update).
	 * This function will be called as part of the update_devfreq call in
	 * both scenarios. As a result, this block will cause a 0 vote during
	 * suspend and a vote for df->prev_freq during resume.
	 */
	if (!node->mon_started) {
		*target_freq = node->resume_freq;
		return 0;
	}

	for (cpu = 0; cpu < CONFIG_VH_SCHED_CPU_NR; cpu++)
	{
		/* ignore idle cpu for DSU frequency boosting */
		if (get_cpu_idle_state(cpu) == DEEP_MEMLAT_CPUIDLE_STATE_AWARE)
			continue;

		ret = get_ev_data(cpu, &inst, &cyc,
				  &stall, &l2_cachemiss, &l3_cachemiss, &mem_stall,
				  &l2_cache_wb, &l3_cache_access, &mem_count, &freq);

		if (ret) {
			pr_err("ev_data read fail\n");
			return ret;
		}

		if (l2_cachemiss)
			ratio = inst / l2_cachemiss;
		else
			ratio = inst;

		wb_pct = mult_frac(100, l2_cache_wb, l3_cache_access);

		if (!freq)
			continue;
		trace_dsulat_dev_meas(dev_name(df->dev.parent),
					cpu,
					inst,
					l2_cachemiss,
					freq,
					stall,
					l2_cache_wb,
					l3_cache_access,
					wb_pct,
					mem_stall, ratio);

		if (cpu < CONFIG_VH_MID_CAPACITY_CPU) {
			ratio_ceil = node->ratio_ceil_cl0;
			wb_pct_thres = node->wb_pct_thres_cl0;
			wb_filter_ratio = node->wb_filter_ratio_cl0;
		} else if (cpu < CONFIG_VH_MAX_CAPACITY_CPU) {
			ratio_ceil = node->ratio_ceil_cl1;
			wb_pct_thres = node->wb_pct_thres_cl1;
			wb_filter_ratio = node->wb_filter_ratio_cl1;
		} else {
			ratio_ceil = node->ratio_ceil_cl2;
			wb_pct_thres = node->wb_pct_thres_cl2;
			wb_filter_ratio = node->wb_filter_ratio_cl2;
		}

		if ((ratio <= ratio_ceil
			&& stall >= node->stall_floor) ||
			(wb_pct >= wb_pct_thres
			 && ratio <= wb_filter_ratio)) {
			dsu_freq = core_to_dev_freq(cpu, node, freq);
			if (!max_freq || dsu_freq > max_freq) {
				max_freq = dsu_freq;
				trace_dsulat_dev_update(dev_name(df->dev.parent),
							cpu,
							inst,
							l2_cachemiss,
							freq,
							stall,
							l2_cache_wb,
							l3_cache_access,
							wb_pct,
							mem_stall,
							max_freq);
			}
		}

	}

	if (max_freq)
		*target_freq = max_freq;

	return 0;
}

show_attr(ratio_ceil_cl0)
store_attr(ratio_ceil_cl0, 1U, 20000U)
static DEVICE_ATTR(ratio_ceil_cl0, 0644, show_ratio_ceil_cl0, store_ratio_ceil_cl0);

show_attr(ratio_ceil_cl1)
store_attr(ratio_ceil_cl1, 1U, 20000U)
static DEVICE_ATTR(ratio_ceil_cl1, 0644, show_ratio_ceil_cl1, store_ratio_ceil_cl1);

show_attr(ratio_ceil_cl2)
store_attr(ratio_ceil_cl2, 1U, 20000U)
static DEVICE_ATTR(ratio_ceil_cl2, 0644, show_ratio_ceil_cl2, store_ratio_ceil_cl2);

show_attr(wb_pct_thres_cl0)
store_attr(wb_pct_thres_cl0, 1U, 100U)
static DEVICE_ATTR(wb_pct_thres_cl0, 0644, show_wb_pct_thres_cl0, store_wb_pct_thres_cl0);

show_attr(wb_pct_thres_cl1)
store_attr(wb_pct_thres_cl1, 1U, 100U)
static DEVICE_ATTR(wb_pct_thres_cl1, 0644, show_wb_pct_thres_cl1, store_wb_pct_thres_cl1);

show_attr(wb_pct_thres_cl2)
store_attr(wb_pct_thres_cl2, 1U, 100U)
static DEVICE_ATTR(wb_pct_thres_cl2, 0644, show_wb_pct_thres_cl2, store_wb_pct_thres_cl2);

show_attr(wb_filter_ratio_cl0)
store_attr(wb_filter_ratio_cl0, 1U, 50000U)
static DEVICE_ATTR(wb_filter_ratio_cl0, 0644, show_wb_filter_ratio_cl0, store_wb_filter_ratio_cl0);

show_attr(wb_filter_ratio_cl1)
store_attr(wb_filter_ratio_cl1, 1U, 50000U)
static DEVICE_ATTR(wb_filter_ratio_cl1, 0644, show_wb_filter_ratio_cl1, store_wb_filter_ratio_cl1);

show_attr(wb_filter_ratio_cl2)
store_attr(wb_filter_ratio_cl2, 1U, 50000U)
static DEVICE_ATTR(wb_filter_ratio_cl2, 0644, show_wb_filter_ratio_cl2, store_wb_filter_ratio_cl2);

show_attr(stall_floor)
store_attr(stall_floor, 0U, 100U)
static DEVICE_ATTR(stall_floor, 0644, show_stall_floor, store_stall_floor);

static struct attribute *dsulat_dev_attr[] = {
	&dev_attr_ratio_ceil_cl0.attr,
	&dev_attr_ratio_ceil_cl1.attr,
	&dev_attr_ratio_ceil_cl2.attr,
	&dev_attr_wb_pct_thres_cl0.attr,
	&dev_attr_wb_pct_thres_cl1.attr,
	&dev_attr_wb_pct_thres_cl2.attr,
	&dev_attr_wb_filter_ratio_cl0.attr,
	&dev_attr_wb_filter_ratio_cl1.attr,
	&dev_attr_wb_filter_ratio_cl2.attr,
	&dev_attr_stall_floor.attr,
	&dev_attr_freq_map.attr,
	NULL,
};

static struct attribute_group dsulat_dev_attr_group = {
	.name = "dsu_latency",
	.attrs = dsulat_dev_attr,
};

#define MIN_MS  0U
#define MAX_MS  500U
static int devfreq_dsulat_ev_handler(struct devfreq *df,
				     unsigned int event, void *data)
{
	int ret;

	switch (event) {
	case DEVFREQ_GOV_START:
		ret = gov_start(df);
		if (ret)
			return ret;

		dev_dbg(df->dev.parent,
			"Enabled DSU Latency governor\n");
		break;

	case DEVFREQ_GOV_STOP:
		if (df && df->data)
			gov_stop(df);
		dev_dbg(df->dev.parent,
			"Disabled DSU Latency governor\n");
		break;

	case DEVFREQ_GOV_SUSPEND:
		ret = gov_suspend(df);
		if (ret) {
			dev_err(df->dev.parent,
				"Unable to suspend dsulat governor (%d)\n",
				ret);
			return ret;
		}

		dev_dbg(df->dev.parent, "Suspended dsulat governor\n");
		break;

	case DEVFREQ_GOV_RESUME:
		ret = gov_resume(df);
		if (ret) {
			dev_err(df->dev.parent,
				"Unable to resume dsulat governor (%d)\n",
				ret);
			return ret;
		}

		dev_dbg(df->dev.parent, "Resumed dsulat governor\n");
		break;
	}

	return 0;
}

static struct devfreq_governor devfreq_gov_dsulat = {
	.name = "dsu_latency",
	.get_target_freq = devfreq_dsulat_get_freq,
	.event_handler = devfreq_dsulat_ev_handler,
};

#define NUM_COLS        2
static struct core_dev_map *init_core_dev_map(struct device *dev,
					      struct device_node *of_node,
					      char *prop_name)
{
	int len, nf, i, j;
	u32 data;
	struct core_dev_map *tbl;
	int ret;

	if (!of_node)
		of_node = dev->of_node;

	if (!of_find_property(of_node, prop_name, &len))
		return NULL;
	len /= sizeof(data);

	if (len % NUM_COLS || len == 0)
		return NULL;
	nf = len / NUM_COLS;

	tbl = devm_kzalloc(dev, (nf + 1) * sizeof(struct core_dev_map),
			GFP_KERNEL);
	if (!tbl)
		return NULL;

	for (i = 0, j = 0; i < nf; i++, j += 2) {
		ret = of_property_read_u32_index(of_node, prop_name, j,
			&data);
		if (ret)
			return NULL;
		tbl[i].core_mhz = data / 1000;

		ret = of_property_read_u32_index(of_node, prop_name, j + 1,
			&data);
		if (ret)
			return NULL;
		tbl[i].target_freq = data;
		pr_debug("Entry%d CPU:%u, Dev:%u\n", i, tbl[i].core_mhz,
			tbl[i].target_freq);
	}
	tbl[i].core_mhz = 0;

	return tbl;
}

static struct dsulat_node *register_common(struct device *dev)
{
	struct dsulat_node *node;

	node = devm_kzalloc(dev, sizeof(*node), GFP_KERNEL);
	if (!node)
		return ERR_PTR(-ENOMEM);

	node->ratio_ceil_cl0 = 700;
	node->ratio_ceil_cl1 = 1000;
	node->ratio_ceil_cl2 = 3000;

	node->freq_map_cl0 = init_core_dev_map(dev, NULL,
			"core-dev-table-cl0");
	if (!node->freq_map_cl0) {
		dev_err(dev, "Couldn't find the core-dev freq table for cl0!\n");
		return ERR_PTR(-EINVAL);
	}

	node->freq_map_cl1 = init_core_dev_map(dev, NULL,
			"core-dev-table-cl1");
	if (!node->freq_map_cl1) {
		dev_err(dev, "Couldn't find the core-dev freq table for cl1!\n");
		return ERR_PTR(-EINVAL);
	}

	node->freq_map_cl2 = init_core_dev_map(dev, NULL,
                                        "core-dev-table-cl2");
	if (!node->freq_map_cl2) {
		dev_err(dev, "Couldn't find the core-dev freq table for cl2!\n");
		return ERR_PTR(-EINVAL);
	}

	return node;
}

int register_dsulat(struct exynos_devfreq_data *dsu_data)
{
	struct dsulat_node *node;
	int ret = 0;

	node = register_common(dsu_data->dev);
	if (IS_ERR(node)) {
		ret = PTR_ERR(node);
		goto out;
	}

	mutex_lock(&state_lock);
	node->gov = &devfreq_gov_dsulat;
	node->attr_grp = &dsulat_dev_attr_group;
	dsu_data->governor_data  = node;
	if (!dsulat_use_cnt)
		ret = devfreq_add_governor(&devfreq_gov_dsulat);
	if (!ret)
		dsulat_use_cnt++;
	mutex_unlock(&state_lock);

out:
	if (!ret)
		dev_info(dsu_data->dev, "DSU Latency governor registered.\n");
	else
		dev_err(dsu_data->dev, "DSU Latency governor registration failed!\n");

	return ret;
}
EXPORT_SYMBOL_GPL(register_dsulat);

MODULE_AUTHOR("Sophia Wang <yodagump@google.com>");
MODULE_DESCRIPTION("HW monitor based dev DSU governor");
MODULE_LICENSE("GPL v2");

