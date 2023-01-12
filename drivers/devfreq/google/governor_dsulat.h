#ifndef _GOVERNOR_DSULAT_H_
#define _GOVERNOR_DSULAT_H_

#include <linux/kernel.h>
#include <linux/devfreq.h>
#include <soc/google/exynos-devfreq.h>

struct dsulat_node {
	unsigned int ratio_ceil_cl0;
	unsigned int ratio_ceil_cl1;
	unsigned int ratio_ceil_cl2;
	unsigned int stall_floor;
	bool mon_started;
	bool already_zero;
	struct list_head list;
	void *orig_data;
	struct core_dev_map *freq_map_cl0;
	struct core_dev_map *freq_map_cl1;
	struct core_dev_map *freq_map_cl2;
	struct devfreq_governor *gov;
	struct attribute_group *attr_grp;
	unsigned long resume_freq;
};

#if IS_ENABLED(CONFIG_DEVFREQ_GOV_MEMLAT)
int register_dsulat(struct exynos_devfreq_data *dsu_data);
void set_dsu_devfreq(struct devfreq *dsu_devfreq);
#else
static inline int register_dsulat(struct exynos_devfreq_data *dsu_data)
{
	return 0;
}
void set_dsu_devfreq(struct devfreq *dsu_devfreq) {}
#endif
#endif  // _GOVERNOR_DSULAT_H_
