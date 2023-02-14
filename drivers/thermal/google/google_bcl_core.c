// SPDX-License-Identifier: GPL-2.0
/*
 * google_bcl_core.c Google bcl core driver
 *
 * Copyright (c) 2022, Google LLC. All rights reserved.
 *
 */

#define pr_fmt(fmt) "%s:%s " fmt, KBUILD_MODNAME, __func__

#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/thermal.h>
#include <dt-bindings/interrupt-controller/zuma.h>
#include <linux/regulator/pmic_class.h>
#include <soc/google/odpm.h>
#include <soc/google/exynos-cpupm.h>
#include <soc/google/exynos-pm.h>
#include <soc/google/exynos-pmu-if.h>
#include <soc/google/bcl.h>
#if IS_ENABLED(CONFIG_DEBUG_FS)
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#endif

#define SMPL_WARN_CTRL		S2MPG14_PM_SMPL_WARN_CTRL
#define SMPL_WARN_SHIFT		S2MPG14_SMPL_WARN_LVL_SHIFT
#define OCP_WARN_LVL_SHIFT	S2MPG14_OCP_WARN_LVL_SHIFT
#define B3M_OCP_WARN		S2MPG14_PM_B3M_OCP_WARN
#define B3M_SOFT_OCP_WARN	S2MPG14_PM_B3M_SOFT_OCP_WARN
#define B2M_OCP_WARN		S2MPG14_PM_B2M_OCP_WARN
#define B2M_SOFT_OCP_WARN	S2MPG14_PM_B2M_SOFT_OCP_WARN
#define B7M_OCP_WARN		S2MPG14_PM_B7M_OCP_WARN
#define B7M_SOFT_OCP_WARN	S2MPG14_PM_B7M_SOFT_OCP_WARN
#define B2S_OCP_WARN		S2MPG15_PM_B2S_OCP_WARN
#define B2S_SOFT_OCP_WARN	S2MPG15_PM_B2S_SOFT_OCP_WARN
#define MAIN_CHIPID		S2MPG14_COMMON_CHIPID
#define SUB_CHIPID		S2MPG15_COMMON_CHIPID
#define INT3_120C		S2MPG14_IRQ_120C_INT3;
#define INT3_140C		S2MPG14_IRQ_140C_INT3;
#define INT3_TSD		S2MPG14_IRQ_TSD_INT3;
#define GPIO_ALIVE_BASE		(0x154d0000)
#define GPA9_CON		(0x100)
#define DELTA_10MS		(10 * NSEC_PER_MSEC)
#define DELTA_50MS		(50 * NSEC_PER_MSEC)

static const struct platform_device_id google_id_table[] = {
	{.name = "google_mitigation",},
	{},
};

DEFINE_MUTEX(sysreg_lock);


static void update_irq_end_times(struct bcl_device *bcl_dev, int id);

static int triggered_read_level(void *data, int *val, int id)
{
	struct bcl_device *bcl_dev = data;
	bool state = true;
	int polarity = (id == SMPL_WARN) ? 0 : 1;
	int gpio_level;
	u8 irq_val = 0;

	if (id == UVLO1)
		gpio_level = gpio_get_value(bcl_dev->vdroop1_pin);
	else if ((id == UVLO2) || (id == BATOILO))
		gpio_level = gpio_get_value(bcl_dev->vdroop2_pin);
	else
		gpio_level = gpio_get_value(bcl_dev->bcl_pin[id]);

	if ((id >= UVLO2 && id <= BATOILO) && (bcl_dev->bcl_tz_cnt[id] == 0)) {
		if (bcl_cb_vdroop_ok(bcl_dev, &state) < 0) {
			*val = 0;
			if (bcl_dev->bcl_prev_lvl[id] != 0) {
				mod_delayed_work(system_unbound_wq, &bcl_dev->bcl_irq_work[id],
						 msecs_to_jiffies(THRESHOLD_DELAY_MS));
				bcl_dev->bcl_prev_lvl[id] = *val;
			}
			gpio_set_value(bcl_dev->modem_gpio2_pin, 0);
			return -EINVAL;
		} else
			gpio_level = (state) ? 0 : 1;
	}
	/* Check polarity */
	if ((gpio_level == polarity) || (bcl_dev->bcl_tz_cnt[id] == 1)) {
		*val = bcl_dev->bcl_lvl[id] + THERMAL_HYST_LEVEL;
		mod_delayed_work(system_unbound_wq, &bcl_dev->bcl_irq_work[id],
				 msecs_to_jiffies(THRESHOLD_DELAY_MS));
		bcl_dev->bcl_prev_lvl[id] = *val;

		/* Check for any additional IRQs if vdroop2 was held high */
		if (id == UVLO2 || id == BATOILO)
			bcl_cb_get_and_clr_irq(bcl_dev, &irq_val);
		return 0;
	}
	if (id >= UVLO1 && id <= BATOILO) {
		/* Zero is applied in case bcl_lvl[id] has a different value */
		*val = 0;
		if (bcl_dev->bcl_prev_lvl[id] != 0) {
			mod_delayed_work(system_unbound_wq, &bcl_dev->bcl_irq_work[id],
					 msecs_to_jiffies(THRESHOLD_DELAY_MS));
			bcl_dev->bcl_prev_lvl[id] = 0;
			update_irq_end_times(bcl_dev, id);
		}
		return 0;
	}

	*val = 0;
	bcl_dev->bcl_tz_cnt[id] = 0;
	if (bcl_dev->bcl_prev_lvl[id] != *val) {
		mod_delayed_work(system_unbound_wq, &bcl_dev->bcl_irq_work[id],
				 msecs_to_jiffies(THRESHOLD_DELAY_MS));
		bcl_dev->bcl_prev_lvl[id] = *val;
	}
	return 0;
}

static enum BCL_BATT_IRQ id_to_ind(int id)
{
	switch (id) {
	case UVLO1:
		return UVLO1_IRQ_BIN;
	case UVLO2:
		return UVLO2_IRQ_BIN;
	case BATOILO:
		return BATOILO_IRQ_BIN;
	}
	return MAX_BCL_BATT_IRQ;
}

static void bin_incr_ifpmic(struct bcl_device *bcl_dev, enum BCL_BATT_IRQ batt,
				enum CONCURRENT_PWRWARN_IRQ pwrwarn, ktime_t end_time)
{
	ktime_t time_delta;
	if (bcl_dev->ifpmic_irq_bins[batt][pwrwarn].start_time == 0)
		return;

	time_delta = ktime_sub(end_time, bcl_dev->ifpmic_irq_bins[batt][pwrwarn].start_time);
	if (ktime_compare(time_delta, DELTA_10MS) < 0)
		atomic_inc(&bcl_dev->ifpmic_irq_bins[batt][pwrwarn].lt_5ms_count);
	else if (ktime_compare(time_delta, DELTA_50MS) < 0)
		atomic_inc(&bcl_dev->ifpmic_irq_bins[batt][pwrwarn].bt_5ms_10ms_count);
	else
		atomic_inc(&bcl_dev->ifpmic_irq_bins[batt][pwrwarn].gt_10ms_count);

	bcl_dev->ifpmic_irq_bins[batt][pwrwarn].start_time = 0;
}

/*
 * Track UVLO1/UVLO2/BATOILO IRQ starting times, and any PWRWARN events
 * happening at the same time as the UVLO1/UVLO2/BATOILO IRQ.
 */
static void update_irq_start_times(struct bcl_device *bcl_dev, int id)
{
	/* Check if it is a input IRQ */
	ktime_t start_time = ktime_get();
	enum BCL_BATT_IRQ irq_ind = id_to_ind(id);
	if (bcl_dev->ifpmic_irq_bins[irq_ind][NONE_BCL_BIN].start_time != 0)
		update_irq_end_times(bcl_dev, id);
	if (irq_ind == MAX_BCL_BATT_IRQ)
		return;

	bcl_dev->ifpmic_irq_bins[irq_ind][NONE_BCL_BIN].start_time = start_time;
	if (bcl_dev->sub_pwr_warn_triggered[bcl_dev->rffe_channel])
		bcl_dev->ifpmic_irq_bins[irq_ind][MMWAVE_BCL_BIN].start_time = start_time;
	if (bcl_dev->main_pwr_warn_triggered[bcl_dev->rffe_channel])
		bcl_dev->ifpmic_irq_bins[irq_ind][RFFE_BCL_BIN].start_time = start_time;
}

static void update_irq_end_times(struct bcl_device *bcl_dev, int id)
{
	ktime_t end_time;
	int irq_ind = -1;
	int i;
	bool pwrwarn_irq_triggered;

	end_time = ktime_get();
	irq_ind = id_to_ind(id);
	if (irq_ind == MAX_BCL_BATT_IRQ)
		return;

	for (i = 0; i < MAX_CONCURRENT_PWRWARN_IRQ; i++) {
		switch (i) {
		case NONE_BCL_BIN:
			pwrwarn_irq_triggered = true;
			break;
		case MMWAVE_BCL_BIN:
			pwrwarn_irq_triggered =
			    bcl_dev->sub_pwr_warn_triggered[bcl_dev->rffe_channel];
			break;
		case RFFE_BCL_BIN:
			pwrwarn_irq_triggered =
			    bcl_dev->main_pwr_warn_triggered[bcl_dev->rffe_channel];
			break;
		}
		if (pwrwarn_irq_triggered)
			bin_incr_ifpmic(bcl_dev, irq_ind, i, end_time);
	}
}

static void pwrwarn_update_start_time(struct bcl_device *bcl_dev,
					int id, struct irq_duration_stats *bins,
					bool *pwr_warn_triggered,
					enum CONCURRENT_PWRWARN_IRQ bin_ind)
{
	ktime_t start_time;
	bool is_rf = bcl_dev->rffe_channel == id;

	if (bins[id].start_time != 0)
		return;

	start_time = ktime_get();
	if (is_rf && pwr_warn_triggered[id]) {
		if (bcl_dev->bcl_lvl[UVLO1] != 0)
			bcl_dev->ifpmic_irq_bins[UVLO1_IRQ_BIN][bin_ind].start_time =
				start_time;
		if (bcl_dev->bcl_lvl[UVLO2] != 0)
			bcl_dev->ifpmic_irq_bins[UVLO2_IRQ_BIN][bin_ind].start_time =
				start_time;
		if (bcl_dev->bcl_lvl[BATOILO] != 0)
			bcl_dev->ifpmic_irq_bins[BATOILO_IRQ_BIN][bin_ind].start_time =
				start_time;
	}
	bins[id].start_time = start_time;
}

static void pwrwarn_update_end_time(struct bcl_device *bcl_dev, int id,
                                    struct irq_duration_stats *bins,
                                    enum CONCURRENT_PWRWARN_IRQ bin_ind)
{
	ktime_t end_time;
	ktime_t time_delta;
	int i;
	bool is_rf = bcl_dev->rffe_channel == id;

	end_time = ktime_get();
	if (is_rf) {
		for (i = 0; i < MAX_BCL_BATT_IRQ; i++)
			if (bcl_dev->ifpmic_irq_bins[i][bin_ind].start_time != 0)
				bin_incr_ifpmic(bcl_dev, i, bin_ind, end_time);
	}

	if (bins[id].start_time == 0)
		return;

	time_delta = ktime_sub(end_time, bins[id].start_time);
	if (ktime_compare(time_delta, DELTA_10MS) < 0)
		atomic_inc(&(bins[id].lt_5ms_count));
	else if (ktime_compare(time_delta, DELTA_50MS) < 0)
		atomic_inc(&(bins[id].bt_5ms_10ms_count));
	else
		atomic_inc(&(bins[id].gt_10ms_count));
	bins[id].start_time = 0;
}

static struct power_supply *google_get_power_supply(struct bcl_device *bcl_dev)
{
	static struct power_supply *psy[2];
	static struct power_supply *batt_psy;
	int err = 0;

	batt_psy = NULL;
	err = power_supply_get_by_phandle_array(bcl_dev->device->of_node, "google,power-supply",
						psy, ARRAY_SIZE(psy));
	if (err > 0)
		batt_psy = psy[0];
	return batt_psy;
}

static void ocpsmpl_read_stats(struct bcl_device *bcl_dev,
			       struct ocpsmpl_stats *dst, struct power_supply *psy)
{
	union power_supply_propval ret = {0};
	int err = 0;

	if (!psy)
		return;
	dst->_time = ktime_to_ms(ktime_get());
	err = power_supply_get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &ret);
	if (err < 0)
		dst->capacity = -1;
	else {
		dst->capacity = ret.intval;
		bcl_dev->batt_psy_initialized = true;
	}
	err = power_supply_get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &ret);
	if (err < 0)
		dst->voltage = -1;
	else {
		dst->voltage = ret.intval;
		bcl_dev->batt_psy_initialized = true;
	}

}

static u8 irq_to_id(struct bcl_device *bcl_dev, int irq)
{
	int i;

	for (i = 0; i < TRIGGERED_SOURCE_MAX; i++) {
		if (bcl_dev->bcl_irq[i] == irq)
			return i;
	}
	return 0;
}

static void update_tz(struct bcl_device *bcl_dev, int idx)
{
	if ((bcl_dev->bcl_tz[idx]) && (bcl_dev->bcl_tz_cnt[idx] == 0)) {
		bcl_dev->bcl_tz_cnt[idx] = 1;
		bcl_dev->bcl_tz[idx]->temperature = 0;
		bcl_dev->bcl_prev_lvl[idx] = 0;
		thermal_zone_device_update(bcl_dev->bcl_tz[idx], THERMAL_EVENT_UNSPECIFIED);
	}
}

static irqreturn_t irq_handler(int irq, void *data)
{
	struct bcl_device *bcl_dev = data;
	u8 idx;
	u8 irq_val = 0;
	int err = 0;
	int gpio_level;
	int polarity;

	if (!bcl_dev)
		return IRQ_HANDLED;

	disable_irq_nosync(irq);
	idx = irq_to_id(bcl_dev, irq);
	polarity = (idx == SMPL_WARN) ? 0 : 1;

	if (idx == UVLO1)
		gpio_level = gpio_get_value(bcl_dev->vdroop1_pin);
	else if (idx == UVLO2)
		gpio_level = gpio_get_value(bcl_dev->vdroop2_pin);
	else
		gpio_level = gpio_get_value(bcl_dev->bcl_pin[idx]);

	if (gpio_level == polarity) {
		/* IRQ latched */
		if (idx == UVLO2)
			gpio_set_value(bcl_dev->modem_gpio2_pin, 1);

		/* IRQ is either UVLO1, UVLO2, or BATOILO */
		if (idx >= UVLO1 && idx <= BATOILO) {
			bcl_cb_get_and_clr_irq(bcl_dev, &irq_val);
			idx = (idx != UVLO1) ? irq_val : idx;
			update_irq_start_times(bcl_dev, idx);
		}

		if (bcl_dev->batt_psy_initialized) {
			atomic_inc(&bcl_dev->bcl_cnt[idx]);
			ocpsmpl_read_stats(bcl_dev, &bcl_dev->bcl_stats[idx], bcl_dev->batt_psy);
		}
	} else {
		/* IRQ falling edge */
		if (idx == UVLO2) {
			gpio_set_value(bcl_dev->modem_gpio2_pin, 0);
			update_tz(bcl_dev, BATOILO);
		}
		if (idx >= UVLO1 && idx <= BATOILO) {
			err = bcl_cb_get_and_clr_irq(bcl_dev, &irq_val);
			update_irq_end_times(bcl_dev, irq_val);
		}
	}
	update_tz(bcl_dev, idx);
	enable_irq(irq);
	return IRQ_HANDLED;
}

static void google_warn_work(struct work_struct *work, int idx)
{
	struct bcl_device *bcl_dev = container_of(work, struct bcl_device,
						  bcl_irq_work[idx].work);

	bcl_dev->bcl_tz_cnt[idx] = 0;
	if (bcl_dev->bcl_tz[idx])
		thermal_zone_device_update(bcl_dev->bcl_tz[idx], THERMAL_EVENT_UNSPECIFIED);
}

static void google_smpl_warn_work(struct work_struct *work)
{
	google_warn_work(work, SMPL_WARN);
}

static int smpl_warn_read_voltage(void *data, int *val)
{
	return triggered_read_level(data, val, SMPL_WARN);
}

static int ocp_cpu1_read_current(void *data, int *val)
{
	return triggered_read_level(data, val, OCP_WARN_CPUCL1);
}

static void google_cpu1_warn_work(struct work_struct *work)
{
	google_warn_work(work, OCP_WARN_CPUCL1);
}

static int ocp_cpu2_read_current(void *data, int *val)
{
	return triggered_read_level(data, val, OCP_WARN_CPUCL2);
}

static void google_cpu2_warn_work(struct work_struct *work)
{
	google_warn_work(work, OCP_WARN_CPUCL2);
}

static int soft_ocp_cpu1_read_current(void *data, int *val)
{
	return triggered_read_level(data, val, SOFT_OCP_WARN_CPUCL1);
}

static void google_soft_cpu1_warn_work(struct work_struct *work)
{
	google_warn_work(work, SOFT_OCP_WARN_CPUCL1);
}

static int soft_ocp_cpu2_read_current(void *data, int *val)
{
	return triggered_read_level(data, val, SOFT_OCP_WARN_CPUCL2);
}

static void google_soft_cpu2_warn_work(struct work_struct *work)
{
	google_warn_work(work, SOFT_OCP_WARN_CPUCL2);
}

static int ocp_tpu_read_current(void *data, int *val)
{
	return triggered_read_level(data, val, OCP_WARN_TPU);
}

static void google_tpu_warn_work(struct work_struct *work)
{
	google_warn_work(work, OCP_WARN_TPU);
}

static int soft_ocp_tpu_read_current(void *data, int *val)
{
	return triggered_read_level(data, val, SOFT_OCP_WARN_TPU);
}

static void google_soft_tpu_warn_work(struct work_struct *work)
{
	google_warn_work(work, SOFT_OCP_WARN_TPU);
}

static int ocp_gpu_read_current(void *data, int *val)
{
	return triggered_read_level(data, val, OCP_WARN_GPU);
}

static void google_gpu_warn_work(struct work_struct *work)
{
	google_warn_work(work, OCP_WARN_GPU);
}

static int soft_ocp_gpu_read_current(void *data, int *val)
{
	return triggered_read_level(data, val, SOFT_OCP_WARN_GPU);
}

static void google_soft_gpu_warn_work(struct work_struct *work)
{
	google_warn_work(work, SOFT_OCP_WARN_GPU);
}

static void google_pmic_120c_work(struct work_struct *work)
{
	google_warn_work(work, PMIC_120C);
}

static int pmic_120c_read_temp(void *data, int *val)
{
	return triggered_read_level(data, val, PMIC_120C);
}

static void google_pmic_140c_work(struct work_struct *work)
{
	google_warn_work(work, PMIC_140C);
}

static int pmic_140c_read_temp(void *data, int *val)
{
	return triggered_read_level(data, val, PMIC_140C);
}

static void google_pmic_overheat_work(struct work_struct *work)
{
	google_warn_work(work, PMIC_OVERHEAT);
}

static int tsd_overheat_read_temp(void *data, int *val)
{
	return triggered_read_level(data, val, PMIC_OVERHEAT);
}

static int google_bcl_uvlo1_read_temp(void *data, int *val)
{
	return triggered_read_level(data, val, UVLO1);
}

static int google_bcl_uvlo2_read_temp(void *data, int *val)
{
	return triggered_read_level(data, val, UVLO2);
}

static int google_bcl_batoilo_read_temp(void *data, int *val)
{
	return triggered_read_level(data, val, BATOILO);
}

static int google_bcl_set_soc(void *data, int low, int high)
{
	struct bcl_device *bcl_dev = data;

	if (high == bcl_dev->trip_high_temp)
		return 0;

	mutex_lock(&bcl_dev->state_trans_lock);
	bcl_dev->trip_low_temp = low;
	bcl_dev->trip_high_temp = high;
	schedule_delayed_work(&bcl_dev->bcl_irq_work[PMIC_SOC], 0);

	mutex_unlock(&bcl_dev->state_trans_lock);
	return 0;
}

static int google_bcl_read_soc(void *data, int *val)
{
	struct bcl_device *bcl_dev = data;
	union power_supply_propval ret = {
		0,
	};
	int err = 0;

	*val = 100;
	if (!bcl_dev->batt_psy)
		bcl_dev->batt_psy = google_get_power_supply(bcl_dev);
	if (bcl_dev->batt_psy) {
		err = power_supply_get_property(bcl_dev->batt_psy,
						POWER_SUPPLY_PROP_CAPACITY, &ret);
		if (err < 0) {
			dev_err(bcl_dev->device, "battery percentage read error:%d\n", err);
			return err;
		}
		bcl_dev->batt_psy_initialized = true;
		*val = 100 - ret.intval;
	}
	pr_debug("soc:%d\n", *val);

	return err;
}

static void google_bcl_evaluate_soc(struct work_struct *work)
{
	int battery_percentage_reverse;
	struct bcl_device *bcl_dev = container_of(work, struct bcl_device,
						  bcl_irq_work[PMIC_SOC].work);

	if (google_bcl_read_soc(bcl_dev, &battery_percentage_reverse))
		return;

	mutex_lock(&bcl_dev->state_trans_lock);
	if ((battery_percentage_reverse < bcl_dev->trip_high_temp) &&
		(battery_percentage_reverse > bcl_dev->trip_low_temp))
		goto eval_exit;

	bcl_dev->trip_val = battery_percentage_reverse;
	mutex_unlock(&bcl_dev->state_trans_lock);
	if (!bcl_dev->bcl_tz[PMIC_SOC]) {
		bcl_dev->bcl_tz[PMIC_SOC] =
				thermal_zone_of_sensor_register(bcl_dev->device,
								PMIC_SOC, bcl_dev,
								&bcl_dev->bcl_ops[PMIC_SOC]);
		if (IS_ERR(bcl_dev->bcl_tz[PMIC_SOC])) {
			dev_err(bcl_dev->device, "soc TZ register failed. err:%ld\n",
				PTR_ERR(bcl_dev->bcl_tz[PMIC_SOC]));
			return;
		}
	}
	if (!IS_ERR(bcl_dev->bcl_tz[PMIC_SOC]))
		thermal_zone_device_update(bcl_dev->bcl_tz[PMIC_SOC], THERMAL_EVENT_UNSPECIFIED);
	return;
eval_exit:
	mutex_unlock(&bcl_dev->state_trans_lock);
}

static int battery_supply_callback(struct notifier_block *nb,
				   unsigned long event, void *data)
{
	struct power_supply *psy = data;
	struct bcl_device *bcl_dev = container_of(nb, struct bcl_device, psy_nb);
	struct power_supply *bcl_psy;

	if (!bcl_dev)
		return NOTIFY_OK;

	bcl_psy = bcl_dev->batt_psy;

	if (!bcl_psy || event != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_OK;

	if (!strcmp(psy->desc->name, bcl_psy->desc->name))
		schedule_delayed_work(&bcl_dev->bcl_irq_work[PMIC_SOC], 0);

	return NOTIFY_OK;
}

static int google_bcl_remove_thermal(struct bcl_device *bcl_dev)
{
	int i = 0;
	struct device *dev;

	power_supply_unreg_notifier(&bcl_dev->psy_nb);
	dev = bcl_dev->main_dev;
	for (i = 0; i < TRIGGERED_SOURCE_MAX; i++) {
		if (i > SOFT_OCP_WARN_TPU)
			dev = bcl_dev->sub_dev;
		if (bcl_dev->bcl_tz[i])
			thermal_zone_of_sensor_unregister(dev, bcl_dev->bcl_tz[i]);
	}

	return 0;
}

static int google_bcl_init_clk_div(struct bcl_device *bcl_dev, int idx, unsigned int value)
{
	void __iomem *addr;

	if (!bcl_dev)
		return -EIO;
	addr = get_addr_by_subsystem(bcl_dev, clk_stats_source[idx]);
	if (addr == NULL)
		return -EINVAL;

	mutex_lock(&bcl_dev->ratio_lock);
	__raw_writel(value, addr);
	mutex_unlock(&bcl_dev->ratio_lock);

	return 0;
}

int google_bcl_register_ifpmic(struct bcl_device *bcl_dev,
			       const struct bcl_ifpmic_ops *pmic_ops)
{
	if (!bcl_dev)
		return -EIO;

	if (!pmic_ops || !pmic_ops->cb_get_vdroop_ok ||
	    !pmic_ops->cb_uvlo_read || !pmic_ops->cb_uvlo_write ||
	    !pmic_ops->cb_batoilo_read || !pmic_ops->cb_batoilo_write ||
	    !pmic_ops->cb_get_and_clr_irq)
		return -EINVAL;

	bcl_dev->pmic_ops = pmic_ops;

	return 0;
}
EXPORT_SYMBOL_GPL(google_bcl_register_ifpmic);

struct bcl_device *google_retrieve_bcl_handle(void)
{
	struct device_node *np;
	struct platform_device *pdev;
	struct bcl_device *bcl_dev;

	np = of_find_node_by_name(NULL, "google,mitigation");
	if (!np)
		return NULL;
	pdev = of_find_device_by_node(np);
	if (!pdev)
		return NULL;
	bcl_dev = platform_get_drvdata(pdev);
	if (!bcl_dev)
		return NULL;

	return bcl_dev;
}
EXPORT_SYMBOL_GPL(google_retrieve_bcl_handle);

int google_init_tpu_ratio(struct bcl_device *data)
{
	void __iomem *addr;

	if (!data)
		return -ENOMEM;

	if (!data->sysreg_cpucl0)
		return -ENOMEM;

	if (!bcl_is_subsystem_on(subsystem_pmu[TPU]))
		return -EIO;

	mutex_lock(&data->ratio_lock);
	bcl_disable_power();
	addr = data->base_mem[TPU] + CPUCL12_CLKDIVSTEP_CON_HEAVY;
	__raw_writel(data->tpu_con_heavy, addr);
	addr = data->base_mem[TPU] + CPUCL12_CLKDIVSTEP_CON_LIGHT;
	__raw_writel(data->tpu_con_light, addr);
	addr = data->base_mem[TPU] + CLKDIVSTEP;
	__raw_writel(data->tpu_clkdivstep, addr);
	addr = data->base_mem[TPU] + VDROOP_FLT;
	__raw_writel(data->tpu_vdroop_flt, addr);
	addr = data->base_mem[TPU] + CLKOUT;
	__raw_writel(data->tpu_clk_out, addr);
	data->tpu_clk_stats = __raw_readl(data->base_mem[TPU] + clk_stats_offset[TPU]);
	bcl_enable_power();
	mutex_unlock(&data->ratio_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(google_init_tpu_ratio);

int google_init_gpu_ratio(struct bcl_device *data)
{
	void __iomem *addr;

	if (!data)
		return -ENOMEM;

	if (!data->sysreg_cpucl0)
		return -ENOMEM;

	if (!bcl_is_subsystem_on(subsystem_pmu[GPU]))
		return -EIO;

	mutex_lock(&data->ratio_lock);
	bcl_disable_power();
	addr = data->base_mem[GPU] + CPUCL12_CLKDIVSTEP_CON_HEAVY;
	__raw_writel(data->gpu_con_heavy, addr);
	addr = data->base_mem[GPU] + CPUCL12_CLKDIVSTEP_CON_LIGHT;
	__raw_writel(data->gpu_con_light, addr);
	addr = data->base_mem[GPU] + CLKDIVSTEP;
	__raw_writel(data->gpu_clkdivstep, addr);
	addr = data->base_mem[GPU] + VDROOP_FLT;
	__raw_writel(data->gpu_vdroop_flt, addr);
	addr = data->base_mem[GPU] + CLKOUT;
	__raw_writel(data->gpu_clk_out, addr);
	data->gpu_clk_stats = __raw_readl(data->base_mem[GPU] + clk_stats_offset[GPU]);
	bcl_enable_power();
	mutex_unlock(&data->ratio_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(google_init_gpu_ratio);

unsigned int google_get_ppm(struct bcl_device *data)
{
	void __iomem *addr;
	unsigned int reg;

	if (!data)
		return -ENOMEM;
	if (!data->sysreg_cpucl0) {
		pr_err("Error in sysreg_cpucl0\n");
		return -ENOMEM;
	}

	mutex_lock(&sysreg_lock);
	addr = data->sysreg_cpucl0 + CLUSTER0_PPM;
	reg = __raw_readl(addr);
	mutex_unlock(&sysreg_lock);

	return reg;
}
EXPORT_SYMBOL_GPL(google_get_ppm);

unsigned int google_get_mpmm(struct bcl_device *data)
{
	void __iomem *addr;
	unsigned int reg;

	if (!data)
		return -ENOMEM;
	if (!data->sysreg_cpucl0) {
		pr_err("Error in sysreg_cpucl0\n");
		return -ENOMEM;
	}

	mutex_lock(&sysreg_lock);
	addr = data->sysreg_cpucl0 + CLUSTER0_MPMM;
	reg = __raw_readl(addr);
	mutex_unlock(&sysreg_lock);

	return reg;
}
EXPORT_SYMBOL_GPL(google_get_mpmm);

int google_set_ppm(struct bcl_device *data, unsigned int value)
{
	void __iomem *addr;

	if (!data)
		return -ENOMEM;
	if (!data->sysreg_cpucl0) {
		pr_err("Error in sysreg_cpucl0\n");
		return -ENOMEM;
	}

	mutex_lock(&sysreg_lock);
	addr = data->sysreg_cpucl0 + CLUSTER0_PPM;
	__raw_writel(value, addr);
	mutex_unlock(&sysreg_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(google_set_ppm);

int google_set_mpmm(struct bcl_device *data, unsigned int value)
{
	void __iomem *addr;

	if (!data)
		return -ENOMEM;
	if (!data->sysreg_cpucl0) {
		pr_err("Error in sysreg_cpucl0\n");
		return -ENOMEM;
	}

	mutex_lock(&sysreg_lock);
	addr = data->sysreg_cpucl0 + CLUSTER0_MPMM;
	__raw_writel(value, addr);
	mutex_unlock(&sysreg_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(google_set_mpmm);

static int google_bcl_register_irq(struct bcl_device *bcl_dev, int id, const char *devname,
				   u32 intr_flag)
{
	int ret = 0;

	ret = devm_request_threaded_irq(bcl_dev->device, bcl_dev->bcl_irq[id], NULL, irq_handler,
					intr_flag | IRQF_ONESHOT, devname, bcl_dev);
	if (ret < 0) {
		dev_err(bcl_dev->device, "Failed to request IRQ: %d: %d\n", bcl_dev->bcl_irq[id],
			ret);
		return ret;
	}

	bcl_dev->bcl_tz[id] = thermal_zone_of_sensor_register(bcl_dev->device, id, bcl_dev,
							      &bcl_dev->bcl_ops[id]);
	if (IS_ERR(bcl_dev->bcl_tz[id])) {
		dev_err(bcl_dev->device, "TZ register failed. %d, err:%ld\n", id,
			PTR_ERR(bcl_dev->bcl_tz[id]));
	} else {
		thermal_zone_device_enable(bcl_dev->bcl_tz[id]);
		thermal_zone_device_update(bcl_dev->bcl_tz[id], THERMAL_DEVICE_UP);
	}
	return ret;
}

static int get_cpu0clk(void *data, u64 *val)
{
	struct bcl_device *bcl_dev = data;
	void __iomem *addr;
	unsigned int value;

	bcl_disable_power();
	addr = bcl_dev->base_mem[CPU0] + CLKOUT;
	*val = __raw_readl(addr);
	bcl_enable_power();
	exynos_pmu_read(PMU_CLK_OUT, &value);
	return 0;
}

static int set_cpu0clk(void *data, u64 val)
{
	struct bcl_device *bcl_dev = data;
	void __iomem *addr;

	bcl_disable_power();
	addr = bcl_dev->base_mem[CPU0] + CLKOUT;
	__raw_writel(val, addr);
	bcl_enable_power();
	exynos_pmu_write(PMU_CLK_OUT, val ? 0x3001 : 0);
	return 0;
}

static int get_cpu1clk(void *data, u64 *val)
{
	struct bcl_device *bcl_dev = data;
	void __iomem *addr;
	unsigned int value;

	bcl_disable_power();
	addr = bcl_dev->base_mem[CPU1] + CLKOUT;
	*val = __raw_readl(addr);
	bcl_enable_power();
	exynos_pmu_read(PMU_CLK_OUT, &value);
	return 0;
}

static int set_cpu1clk(void *data, u64 val)
{
	struct bcl_device *bcl_dev = data;
	void __iomem *addr;

	bcl_disable_power();
	addr = bcl_dev->base_mem[CPU1] + CLKOUT;
	__raw_writel(val, addr);
	bcl_enable_power();
	exynos_pmu_write(PMU_CLK_OUT, val ? 0x1101 : 0);
	return 0;
}

static int get_cpu2clk(void *data, u64 *val)
{
	struct bcl_device *bcl_dev = data;
	void __iomem *addr;
	unsigned int value;

	bcl_disable_power();
	addr = bcl_dev->base_mem[CPU2] + CLKOUT;
	*val = __raw_readl(addr);
	bcl_enable_power();
	exynos_pmu_read(PMU_CLK_OUT, &value);
	return 0;
}

static int set_cpu2clk(void *data, u64 val)
{
	struct bcl_device *bcl_dev = data;
	void __iomem *addr;

	bcl_disable_power();
	addr = bcl_dev->base_mem[CPU2] + CLKOUT;
	__raw_writel(val, addr);
	bcl_enable_power();
	exynos_pmu_write(PMU_CLK_OUT, val ? 0x1201 : 0);
	return 0;
}

static int get_gpuclk(void *data, u64 *val)
{
	struct bcl_device *bcl_dev = data;
	unsigned int value;

	*val = bcl_dev->gpu_clk_out;
	exynos_pmu_read(PMU_CLK_OUT, &value);
	return 0;
}

static int set_gpuclk(void *data, u64 val)
{
	struct bcl_device *bcl_dev = data;

	bcl_dev->gpu_clk_out = val;
	exynos_pmu_write(PMU_CLK_OUT, val ? 0x1a01 : 0);
	return 0;
}

static int get_tpuclk(void *data, u64 *val)
{
	struct bcl_device *bcl_dev = data;
	unsigned int value;

	*val = bcl_dev->tpu_clk_out;
	exynos_pmu_read(PMU_CLK_OUT, &value);
	return 0;
}

static int set_tpuclk(void *data, u64 val)
{
	struct bcl_device *bcl_dev = data;

	bcl_dev->tpu_clk_out = val;
	exynos_pmu_write(PMU_CLK_OUT, val ? 0x2E01 : 0);
	return 0;
}

static int get_modem_gpio1(void *data, u64 *val)
{
	struct bcl_device *bcl_dev = data;

	*val = gpio_get_value(bcl_dev->modem_gpio1_pin);
	return 0;
}

static int set_modem_gpio1(void *data, u64 val)
{
	struct bcl_device *bcl_dev = data;

	gpio_set_value(bcl_dev->modem_gpio1_pin, val);
	return 0;
}

static int get_modem_gpio2(void *data, u64 *val)
{
	struct bcl_device *bcl_dev = data;

	*val = gpio_get_value(bcl_dev->modem_gpio2_pin);
	return 0;
}

static int set_modem_gpio2(void *data, u64 val)
{
	struct bcl_device *bcl_dev = data;

	gpio_set_value(bcl_dev->modem_gpio2_pin, val);
	return 0;
}

static int get_add_perph(void *data, u64 *val)
{
	struct bcl_device *bcl_dev = data;

	*val = (u64)bcl_dev->add_perph;
	return 0;
}

static int set_add_perph(void *data, u64 val)
{
	struct bcl_device *bcl_dev = data;

	if (val < 0 || val > SUBSYSTEM_SOURCE_MAX)
		return -EINVAL;

	bcl_dev->add_perph = (u8)val;
	return 0;
}

static int get_add_addr(void *data, u64 *val)
{
	struct bcl_device *bcl_dev = data;

	*val = bcl_dev->add_addr;
	return 0;
}

static int set_add_addr(void *data, u64 val)
{
	struct bcl_device *bcl_dev = data;

	if (val < 0 || val > SZ_128)
		return -EINVAL;

	bcl_dev->add_addr = val;
	return 0;
}

static int get_add_data(void *data, u64 *val)
{
	struct bcl_device *bcl_dev = data;
	void __iomem *read_addr;

	if (bcl_dev->add_addr < 0 || bcl_dev->add_addr > SZ_128)
		return -EINVAL;

	if (bcl_dev->add_perph < 0 || bcl_dev->add_perph > SUBSYSTEM_SOURCE_MAX)
		return -EINVAL;

	mutex_lock(&sysreg_lock);
	read_addr = bcl_dev->base_add_mem[bcl_dev->add_perph] + bcl_dev->add_addr;
	*val = __raw_readl(read_addr);
	mutex_unlock(&sysreg_lock);

	return 0;
}

static int set_add_data(void *data, u64 val)
{
	struct bcl_device *bcl_dev = data;
	void __iomem *write_addr;

	if (bcl_dev->add_addr < 0 || bcl_dev->add_addr > SZ_128)
		return -EINVAL;

	if (bcl_dev->add_perph < 0 || bcl_dev->add_perph > SUBSYSTEM_SOURCE_MAX)
		return -EINVAL;

	if (!bcl_dev)
		return -ENOMEM;

	if (!bcl_dev->base_add_mem[bcl_dev->add_perph]) {
		pr_err("Error in ADD perph\n");
		return -ENOMEM;
	}

	mutex_lock(&sysreg_lock);
	write_addr = bcl_dev->base_add_mem[bcl_dev->add_perph] + bcl_dev->add_addr;
	__raw_writel(val, write_addr);
	mutex_unlock(&sysreg_lock);
	return 0;
}


DEFINE_SIMPLE_ATTRIBUTE(cpu0_clkout_fops, get_cpu0clk, set_cpu0clk, "0x%llx\n");
DEFINE_SIMPLE_ATTRIBUTE(cpu1_clkout_fops, get_cpu1clk, set_cpu1clk, "0x%llx\n");
DEFINE_SIMPLE_ATTRIBUTE(cpu2_clkout_fops, get_cpu2clk, set_cpu2clk, "0x%llx\n");
DEFINE_SIMPLE_ATTRIBUTE(gpu_clkout_fops, get_gpuclk, set_gpuclk, "0x%llx\n");
DEFINE_SIMPLE_ATTRIBUTE(tpu_clkout_fops, get_tpuclk, set_tpuclk, "0x%llx\n");
DEFINE_SIMPLE_ATTRIBUTE(modem_gpio1_fops, get_modem_gpio1, set_modem_gpio1, "0x%llx\n");
DEFINE_SIMPLE_ATTRIBUTE(modem_gpio2_fops, get_modem_gpio2, set_modem_gpio2, "0x%llx\n");
DEFINE_SIMPLE_ATTRIBUTE(add_perph_fops, get_add_perph, set_add_perph, "0x%llx\n");
DEFINE_SIMPLE_ATTRIBUTE(add_addr_fops, get_add_addr, set_add_addr, "0x%llx\n");
DEFINE_SIMPLE_ATTRIBUTE(add_data_fops, get_add_data, set_add_data, "0x%llx\n");

static void google_init_debugfs(struct bcl_device *bcl_dev)
{
	struct dentry *dentry_add;
	bcl_dev->debug_entry = debugfs_create_dir("google_bcl", 0);
	debugfs_create_file("cpu0_clk_out", 0644, bcl_dev->debug_entry, bcl_dev, &cpu0_clkout_fops);
	debugfs_create_file("cpu1_clk_out", 0644, bcl_dev->debug_entry, bcl_dev, &cpu1_clkout_fops);
	debugfs_create_file("cpu2_clk_out", 0644, bcl_dev->debug_entry, bcl_dev, &cpu2_clkout_fops);
	debugfs_create_file("gpu_clk_out", 0644, bcl_dev->debug_entry, bcl_dev, &gpu_clkout_fops);
	debugfs_create_file("tpu_clk_out", 0644, bcl_dev->debug_entry, bcl_dev, &tpu_clkout_fops);
	debugfs_create_file("modem_gpio1", 0644, bcl_dev->debug_entry, bcl_dev, &modem_gpio1_fops);
	debugfs_create_file("modem_gpio2", 0644, bcl_dev->debug_entry, bcl_dev, &modem_gpio2_fops);
	dentry_add = debugfs_create_dir("add", bcl_dev->debug_entry);
	debugfs_create_file("perph", 0600, dentry_add, bcl_dev, &add_perph_fops);
	debugfs_create_file("addr", 0600, dentry_add, bcl_dev, &add_addr_fops);
	debugfs_create_file("data", 0600, dentry_add, bcl_dev, &add_data_fops);
}

static void google_set_throttling(struct bcl_device *bcl_dev)
{
	struct device_node *np = bcl_dev->device->of_node;
	int ret;
	u32 val, ppm_settings, mpmm_settings;
	void __iomem *addr;

	if (!bcl_dev->sysreg_cpucl0) {
		dev_err(bcl_dev->device, "sysreg_cpucl0 ioremap not mapped\n");
		return;
	}
	ret = of_property_read_u32(np, "ppm_settings", &val);
	ppm_settings = ret ? 0 : val;

	ret = of_property_read_u32(np, "mpmm_settings", &val);
	mpmm_settings = ret ? 0 : val;

	mutex_lock(&sysreg_lock);
	addr = bcl_dev->sysreg_cpucl0 + CLUSTER0_PPM;
	__raw_writel(ppm_settings, addr);
	addr = bcl_dev->sysreg_cpucl0 + CLUSTER0_MPMM;
	__raw_writel(mpmm_settings, addr);
	mutex_unlock(&sysreg_lock);

}

static void main_pwrwarn_irq_work(struct work_struct *work)
{
	struct bcl_device *bcl_dev = container_of(work, struct bcl_device,
						  main_pwr_irq_work.work);
	bool revisit_needed = false;
	int i;
	u32 micro_unit[ODPM_CHANNEL_MAX];
	u32 measurement;

	mutex_lock(&bcl_dev->main_odpm->lock);

	odpm_get_raw_lpf_values(bcl_dev->main_odpm, S2MPG1415_METER_CURRENT, micro_unit);
	for (i = 0; i < METER_CHANNEL_MAX; i++) {
		const int rail_i = bcl_dev->main_odpm->channels[i].rail_i;
		if (bcl_dev->main_odpm->chip.rails[rail_i].type == ODPM_RAIL_TYPE_SHUNT)
			measurement = micro_unit[i];
		else
			measurement = micro_unit[i] >> LPF_CURRENT_SHIFT;
		bcl_dev->main_pwr_warn_triggered[i] = (measurement > bcl_dev->main_limit[i]);
		if (!revisit_needed)
			revisit_needed = bcl_dev->main_pwr_warn_triggered[i];
		if ((!revisit_needed) && (i == bcl_dev->rffe_channel))
			gpio_set_value(bcl_dev->modem_gpio1_pin, 0);
		if (!bcl_dev->main_pwr_warn_triggered[i])
			pwrwarn_update_end_time(bcl_dev, i, bcl_dev->pwrwarn_main_irq_bins,
						RFFE_BCL_BIN);
		else
			pwrwarn_update_start_time(bcl_dev, i, bcl_dev->pwrwarn_main_irq_bins,
							bcl_dev->main_pwr_warn_triggered,
							RFFE_BCL_BIN);
	}

	mutex_unlock(&bcl_dev->main_odpm->lock);

	if (revisit_needed)
		mod_delayed_work(system_unbound_wq, &bcl_dev->main_pwr_irq_work,
				 msecs_to_jiffies(PWRWARN_DELAY_MS));
}

static void sub_pwrwarn_irq_work(struct work_struct *work)
{
	struct bcl_device *bcl_dev = container_of(work, struct bcl_device,
						  sub_pwr_irq_work.work);
	bool revisit_needed = false;
	int i;
	u32 micro_unit[ODPM_CHANNEL_MAX];
	u32 measurement;

	mutex_lock(&bcl_dev->sub_odpm->lock);

	odpm_get_raw_lpf_values(bcl_dev->sub_odpm, S2MPG1415_METER_CURRENT, micro_unit);
	for (i = 0; i < METER_CHANNEL_MAX; i++) {
		const int rail_i = bcl_dev->sub_odpm->channels[i].rail_i;
		if (bcl_dev->sub_odpm->chip.rails[rail_i].type == ODPM_RAIL_TYPE_SHUNT)
			measurement = micro_unit[i];
		else
			measurement = micro_unit[i] >> LPF_CURRENT_SHIFT;
		bcl_dev->sub_pwr_warn_triggered[i] = (measurement > bcl_dev->sub_limit[i]);
		if (!revisit_needed)
			revisit_needed = bcl_dev->sub_pwr_warn_triggered[i];
		if ((!revisit_needed) && (i == bcl_dev->rffe_channel))
			gpio_set_value(bcl_dev->modem_gpio1_pin, 0);
		if (!bcl_dev->sub_pwr_warn_triggered[i])
			pwrwarn_update_end_time(bcl_dev, i, bcl_dev->pwrwarn_sub_irq_bins,
						MMWAVE_BCL_BIN);
		else
			pwrwarn_update_start_time(bcl_dev, i, bcl_dev->pwrwarn_sub_irq_bins,
							bcl_dev->sub_pwr_warn_triggered,
							MMWAVE_BCL_BIN);
	}

	mutex_unlock(&bcl_dev->sub_odpm->lock);

	if (revisit_needed)
		mod_delayed_work(system_unbound_wq, &bcl_dev->sub_pwr_irq_work,
				 msecs_to_jiffies(PWRWARN_DELAY_MS));
}

static irqreturn_t sub_pwr_warn_irq_handler(int irq, void *data)
{
	struct bcl_device *bcl_dev = data;
	int i;

	mutex_lock(&bcl_dev->sub_odpm->lock);

	for (i = 0; i < METER_CHANNEL_MAX; i++) {
		if (bcl_dev->sub_pwr_warn_irq[i] == irq) {
			bcl_dev->sub_pwr_warn_triggered[i] = 1;
			/* Check for Modem MMWAVE */
			if (i == bcl_dev->rffe_channel)
				gpio_set_value(bcl_dev->modem_gpio1_pin, 1);

			/* Setup Timer to clear the triggered */
			mod_delayed_work(system_unbound_wq, &bcl_dev->sub_pwr_irq_work,
					 msecs_to_jiffies(PWRWARN_DELAY_MS));
			pwrwarn_update_start_time(bcl_dev, i, bcl_dev->pwrwarn_sub_irq_bins,
							bcl_dev->sub_pwr_warn_triggered,
							MMWAVE_BCL_BIN);
			break;
		}
	}

	mutex_unlock(&bcl_dev->sub_odpm->lock);

	return IRQ_HANDLED;
}

static irqreturn_t main_pwr_warn_irq_handler(int irq, void *data)
{
	struct bcl_device *bcl_dev = data;
	int i;

	mutex_lock(&bcl_dev->main_odpm->lock);

	for (i = 0; i < METER_CHANNEL_MAX; i++) {
		if (bcl_dev->main_pwr_warn_irq[i] == irq) {
			bcl_dev->main_pwr_warn_triggered[i] = 1;
			/* Check for Modem RFFE */
			if (i == bcl_dev->rffe_channel)
				gpio_set_value(bcl_dev->modem_gpio1_pin, 1);

			/* Setup Timer to clear the triggered */
			mod_delayed_work(system_unbound_wq, &bcl_dev->main_pwr_irq_work,
					 msecs_to_jiffies(PWRWARN_DELAY_MS));
			pwrwarn_update_start_time(bcl_dev, i, bcl_dev->pwrwarn_main_irq_bins,
							bcl_dev->main_pwr_warn_triggered,
							RFFE_BCL_BIN);
			break;
		}
	}

	mutex_unlock(&bcl_dev->main_odpm->lock);

	return IRQ_HANDLED;
}

static int google_set_sub_pmic(struct bcl_device *bcl_dev)
{
	struct s2mpg15_platform_data *pdata_sub;
	struct s2mpg15_dev *sub_dev = NULL;
	struct device_node *p_np;
	struct device_node *np = bcl_dev->device->of_node;
	struct i2c_client *i2c;
	u8 val = 0;
	int ret, i;

	INIT_DELAYED_WORK(&bcl_dev->bcl_irq_work[OCP_WARN_GPU], google_gpu_warn_work);
	INIT_DELAYED_WORK(&bcl_dev->bcl_irq_work[SOFT_OCP_WARN_GPU], google_soft_gpu_warn_work);
	INIT_DELAYED_WORK(&bcl_dev->sub_pwr_irq_work, sub_pwrwarn_irq_work);

	p_np = of_parse_phandle(np, "google,sub-power", 0);
	if (p_np) {
		i2c = of_find_i2c_device_by_node(p_np);
		if (!i2c) {
			dev_err(bcl_dev->device, "Cannot find sub-power I2C\n");
			return -ENODEV;
		}
		sub_dev = i2c_get_clientdata(i2c);
	}
	of_node_put(p_np);
	if (!sub_dev) {
		dev_err(bcl_dev->device, "SUB PMIC device not found\n");
		return -ENODEV;
	}
	pdata_sub = dev_get_platdata(sub_dev->dev);
	bcl_dev->sub_odpm = pdata_sub->meter;
	for (i = 0; i < METER_CHANNEL_MAX; i++)
		bcl_dev->sub_rail_names[i] = bcl_dev->sub_odpm->chip.rails[i].schematic_name;
	bcl_dev->sub_irq_base = pdata_sub->irq_base;
	bcl_dev->sub_pmic_i2c = sub_dev->pmic;
	bcl_dev->sub_meter_i2c = sub_dev->meter;
	bcl_dev->sub_dev = sub_dev->dev;
	bcl_dev->bcl_lvl[OCP_WARN_GPU] = B2S_UPPER_LIMIT - THERMAL_HYST_LEVEL -
			(pdata_sub->b2_ocp_warn_lvl * B2S_STEP);
	bcl_dev->bcl_lvl[SOFT_OCP_WARN_GPU] = B2S_UPPER_LIMIT - THERMAL_HYST_LEVEL -
			(pdata_sub->b2_soft_ocp_warn_lvl * B2S_STEP);
	bcl_dev->bcl_pin[OCP_WARN_GPU] = pdata_sub->b2_ocp_warn_pin;
	bcl_dev->bcl_pin[SOFT_OCP_WARN_GPU] = pdata_sub->b2_soft_ocp_warn_pin;
	bcl_dev->bcl_irq[OCP_WARN_GPU] = gpio_to_irq(pdata_sub->b2_ocp_warn_pin);
	bcl_dev->bcl_irq[SOFT_OCP_WARN_GPU] = gpio_to_irq(pdata_sub->b2_soft_ocp_warn_pin);
	bcl_dev->bcl_ops[OCP_WARN_GPU].get_temp = ocp_gpu_read_current;
	bcl_dev->bcl_ops[SOFT_OCP_WARN_GPU].get_temp = soft_ocp_gpu_read_current;
	if (pmic_read(SUB, bcl_dev, SUB_CHIPID, &val)) {
		dev_err(bcl_dev->device, "Failed to read PMIC chipid.\n");
		return -ENODEV;
	}
	pmic_read(SUB, bcl_dev, S2MPG15_PM_OFFSRC1, &val);
	dev_info(bcl_dev->device, "SUB OFFSRC1 : %#x\n", val);
	bcl_dev->sub_offsrc1 = val;
	pmic_read(SUB, bcl_dev, S2MPG15_PM_OFFSRC2, &val);
	dev_info(bcl_dev->device, "SUB OFFSRC2 : %#x\n", val);
	bcl_dev->sub_offsrc2 = val;
	pmic_write(SUB, bcl_dev, S2MPG15_PM_OFFSRC1, 0);
	pmic_write(SUB, bcl_dev, S2MPG15_PM_OFFSRC2, 0);

	ret = google_bcl_register_irq(bcl_dev, OCP_WARN_GPU, "GPU_OCP_IRQ",
				      IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: GPU\n");
		return -ENODEV;
	}
	ret = google_bcl_register_irq(bcl_dev, SOFT_OCP_WARN_GPU, "SOFT_GPU_OCP_IRQ",
				      IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: SOFT_GPU\n");
		return -ENODEV;
	}
	for (i = 0; i < S2MPG1415_METER_CHANNEL_MAX; i++) {
		bcl_dev->sub_pwr_warn_irq[i] =
				bcl_dev->sub_irq_base + S2MPG15_IRQ_PWR_WARN_CH0_INT5 + i;
		ret = devm_request_threaded_irq(bcl_dev->device, bcl_dev->sub_pwr_warn_irq[i],
						NULL, sub_pwr_warn_irq_handler, 0, "PWR_WARN",
						bcl_dev);
		if (ret < 0) {
			dev_err(bcl_dev->device, "Failed to request PWR_WARN_CH%d IRQ: %d: %d\n",
				i, bcl_dev->sub_pwr_warn_irq[i], ret);
		}
	}

	return 0;
}

static void google_bcl_uvlo1_irq_work(struct work_struct *work)
{
	google_warn_work(work, UVLO1);
}

static void google_bcl_uvlo2_irq_work(struct work_struct *work)
{
	google_warn_work(work, UVLO2);
}

static void google_bcl_batoilo_irq_work(struct work_struct *work)
{
	google_warn_work(work, BATOILO);
}

static void google_set_intf_pmic_work(struct work_struct *work)
{
	struct bcl_device *bcl_dev = container_of(work, struct bcl_device, init_work.work);
	int ret = 0;
	unsigned int uvlo1_lvl, uvlo2_lvl, batoilo_lvl;

	if (!bcl_dev->intf_pmic_i2c)
		goto retry_init_work;
	if (IS_ERR_OR_NULL(bcl_dev->pmic_ops) || IS_ERR_OR_NULL(bcl_dev->pmic_ops->cb_uvlo_read))
		goto retry_init_work;
	if (bcl_cb_uvlo1_read(bcl_dev, &uvlo1_lvl) < 0)
		goto retry_init_work;
	if (bcl_cb_uvlo2_read(bcl_dev, &uvlo2_lvl) < 0)
		goto retry_init_work;
	if (bcl_cb_batoilo_read(bcl_dev, &batoilo_lvl) < 0)
		goto retry_init_work;

	bcl_dev->batt_psy = google_get_power_supply(bcl_dev);
	bcl_dev->bcl_tz[PMIC_SOC] = thermal_zone_of_sensor_register(bcl_dev->device,
								    PMIC_SOC, bcl_dev,
								    &bcl_dev->bcl_ops[PMIC_SOC]);
	bcl_dev->bcl_ops[PMIC_SOC].get_temp = google_bcl_read_soc;
	bcl_dev->bcl_ops[PMIC_SOC].set_trips = google_bcl_set_soc;
	if (IS_ERR(bcl_dev->bcl_tz[PMIC_SOC])) {
		dev_err(bcl_dev->device, "soc TZ register failed. err:%ld\n",
			PTR_ERR(bcl_dev->bcl_tz[PMIC_SOC]));
		ret = PTR_ERR(bcl_dev->bcl_tz[PMIC_SOC]);
		bcl_dev->bcl_tz[PMIC_SOC]= NULL;
	} else {
		bcl_dev->psy_nb.notifier_call = battery_supply_callback;
		ret = power_supply_reg_notifier(&bcl_dev->psy_nb);
		if (ret < 0)
			dev_err(bcl_dev->device,
				"soc notifier registration error. defer. err:%d\n", ret);
		thermal_zone_device_update(bcl_dev->bcl_tz[PMIC_SOC], THERMAL_DEVICE_UP);
	}
	bcl_dev->batt_psy_initialized = false;

	bcl_dev->bcl_lvl[UVLO1] = VD_BATTERY_VOLTAGE - uvlo1_lvl - THERMAL_HYST_LEVEL;
	bcl_dev->bcl_lvl[UVLO2] = VD_BATTERY_VOLTAGE - uvlo2_lvl - THERMAL_HYST_LEVEL;
	bcl_dev->bcl_lvl[BATOILO] = batoilo_lvl - THERMAL_HYST_LEVEL;
	bcl_dev->bcl_ops[UVLO1].get_temp = google_bcl_uvlo1_read_temp;
	bcl_dev->bcl_ops[UVLO2].get_temp = google_bcl_uvlo2_read_temp;
	bcl_dev->bcl_ops[BATOILO].get_temp = google_bcl_batoilo_read_temp;
	bcl_dev->bcl_irq[UVLO1] = gpio_to_irq(bcl_dev->vdroop1_pin);
	bcl_dev->bcl_pin[UVLO1] = bcl_dev->vdroop1_pin;
	bcl_dev->bcl_irq[UVLO2] = gpio_to_irq(bcl_dev->vdroop2_pin);
	bcl_dev->bcl_pin[UVLO2] = bcl_dev->vdroop2_pin;

	ret = google_bcl_register_irq(bcl_dev, UVLO1, "UVLO1",
				      IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: UVLO1\n");
		return;
	}
	ret = google_bcl_register_irq(bcl_dev, UVLO2, "UVLO2",
				      IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: UVLO2\n");
		return;
	}
	bcl_dev->bcl_tz[BATOILO] = thermal_zone_of_sensor_register(bcl_dev->device, BATOILO,
								   bcl_dev,
								   &bcl_dev->bcl_ops[BATOILO]);
	if (IS_ERR(bcl_dev->bcl_tz[BATOILO])) {
		dev_err(bcl_dev->device, "TZ register vdroop%d failed, err:%ld\n", BATOILO,
			PTR_ERR(bcl_dev->bcl_tz[BATOILO]));
	} else {
		thermal_zone_device_enable(bcl_dev->bcl_tz[BATOILO]);
		thermal_zone_device_update(bcl_dev->bcl_tz[BATOILO], THERMAL_DEVICE_UP);
	}

	bcl_dev->ready = true;
	return;

retry_init_work:
	schedule_delayed_work(&bcl_dev->init_work, msecs_to_jiffies(THERMAL_DELAY_INIT_MS));
}

static int google_set_intf_pmic(struct bcl_device *bcl_dev)
{
	int ret = 0;
	u8 val;
	struct device_node *p_np;
	struct device_node *np = bcl_dev->device->of_node;
	struct i2c_client *i2c;
	struct s2mpg14_platform_data *pdata_main;
	p_np = of_parse_phandle(np, "google,charger", 0);
	if (p_np) {
		i2c = of_find_i2c_device_by_node(p_np);
		if (!i2c) {
			dev_err(bcl_dev->device, "Cannot find Charger I2C\n");
			return -ENODEV;
		}
		bcl_dev->intf_pmic_i2c = i2c;
	}
	of_node_put(p_np);
	if (!bcl_dev->intf_pmic_i2c) {
		dev_err(bcl_dev->device, "Interface PMIC device not found\n");
		return -ENODEV;
	}

	pdata_main = dev_get_platdata(bcl_dev->main_dev);
	INIT_DELAYED_WORK(&bcl_dev->bcl_irq_work[PMIC_SOC], google_bcl_evaluate_soc);
	INIT_DELAYED_WORK(&bcl_dev->bcl_irq_work[PMIC_120C], google_pmic_120c_work);
	INIT_DELAYED_WORK(&bcl_dev->bcl_irq_work[PMIC_140C], google_pmic_140c_work);
	INIT_DELAYED_WORK(&bcl_dev->bcl_irq_work[PMIC_OVERHEAT], google_pmic_overheat_work);
	INIT_DELAYED_WORK(&bcl_dev->bcl_irq_work[UVLO1], google_bcl_uvlo1_irq_work);
	INIT_DELAYED_WORK(&bcl_dev->bcl_irq_work[UVLO2], google_bcl_uvlo2_irq_work);
	INIT_DELAYED_WORK(&bcl_dev->bcl_irq_work[BATOILO], google_bcl_batoilo_irq_work);
	bcl_dev->bcl_irq[PMIC_120C] = pdata_main->irq_base + INT3_120C;
	bcl_dev->bcl_irq[PMIC_140C] = pdata_main->irq_base + INT3_140C;
	bcl_dev->bcl_irq[PMIC_OVERHEAT] = pdata_main->irq_base + INT3_TSD;
	bcl_dev->bcl_ops[PMIC_120C].get_temp = pmic_120c_read_temp;
	bcl_dev->bcl_ops[PMIC_140C].get_temp = pmic_140c_read_temp;
	bcl_dev->bcl_ops[PMIC_OVERHEAT].get_temp = tsd_overheat_read_temp;
	if (pmic_read(MAIN, bcl_dev, MAIN_CHIPID, &val)) {
		dev_err(bcl_dev->device, "Failed to read MAIN chipid.\n");
		return -ENODEV;
	}
	bcl_dev->bcl_lvl[PMIC_120C] = PMIC_120C_UPPER_LIMIT - THERMAL_HYST_LEVEL;
	bcl_dev->bcl_lvl[PMIC_140C] = PMIC_140C_UPPER_LIMIT - THERMAL_HYST_LEVEL;
	bcl_dev->bcl_lvl[PMIC_OVERHEAT] = PMIC_OVERHEAT_UPPER_LIMIT - THERMAL_HYST_LEVEL;

	ret = google_bcl_register_irq(bcl_dev, PMIC_120C, "PMIC_120C",
				      IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: PMIC_120C\n");
		return -ENODEV;
	}
	ret = google_bcl_register_irq(bcl_dev, PMIC_140C, "PMIC_140C",
				      IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: PMIC_140C\n");
		return -ENODEV;
	}
	ret = google_bcl_register_irq(bcl_dev, PMIC_OVERHEAT, "PMIC_OVERHEAT",
				      IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: PMIC_OVERHEAT\n");
		return -ENODEV;
	}

	return 0;
}

static int google_set_main_pmic(struct bcl_device *bcl_dev)
{
	struct s2mpg14_platform_data *pdata_main;
	struct s2mpg14_dev *main_dev = NULL;
	u8 val;
	struct device_node *p_np;
	struct device_node *np = bcl_dev->device->of_node;
	struct i2c_client *i2c;
	bool bypass_smpl_warn = false;
	int ret, i;

	INIT_DELAYED_WORK(&bcl_dev->bcl_irq_work[SMPL_WARN], google_smpl_warn_work);
	INIT_DELAYED_WORK(&bcl_dev->bcl_irq_work[OCP_WARN_TPU], google_tpu_warn_work);
	INIT_DELAYED_WORK(&bcl_dev->bcl_irq_work[SOFT_OCP_WARN_TPU], google_soft_tpu_warn_work);
	INIT_DELAYED_WORK(&bcl_dev->bcl_irq_work[OCP_WARN_CPUCL2], google_cpu2_warn_work);
	INIT_DELAYED_WORK(&bcl_dev->bcl_irq_work[OCP_WARN_CPUCL1], google_cpu1_warn_work);
	INIT_DELAYED_WORK(&bcl_dev->bcl_irq_work[SOFT_OCP_WARN_CPUCL2], google_soft_cpu2_warn_work);
	INIT_DELAYED_WORK(&bcl_dev->bcl_irq_work[SOFT_OCP_WARN_CPUCL1], google_soft_cpu1_warn_work);
	INIT_DELAYED_WORK(&bcl_dev->main_pwr_irq_work, main_pwrwarn_irq_work);

	for (i = 0; i < TRIGGERED_SOURCE_MAX; i++) {
		bcl_dev->bcl_tz_cnt[i] = 0;
		bcl_dev->bcl_prev_lvl[i] = 0;
		atomic_set(&bcl_dev->bcl_cnt[i], 0);
		mutex_init(&bcl_dev->bcl_irq_lock[i]);
	}
	p_np = of_parse_phandle(np, "google,main-power", 0);
	if (p_np) {
		i2c = of_find_i2c_device_by_node(p_np);
		if (!i2c) {
			dev_err(bcl_dev->device, "Cannot find main-power I2C\n");
			return -ENODEV;
		}
		main_dev = i2c_get_clientdata(i2c);
	}
	of_node_put(p_np);
	if (!main_dev) {
		dev_err(bcl_dev->device, "Main PMIC device not found\n");
		return -ENODEV;
	}
	pdata_main = dev_get_platdata(main_dev->dev);
	bcl_dev->main_odpm = pdata_main->meter;
	for (i = 0; i < METER_CHANNEL_MAX; i++)
		bcl_dev->main_rail_names[i] = bcl_dev->main_odpm->chip.rails[i].schematic_name;
	bcl_dev->main_irq_base = pdata_main->irq_base;
	/* request smpl_warn interrupt */
	if (!gpio_is_valid(pdata_main->smpl_warn_pin)) {
		dev_err(bcl_dev->device, "smpl_warn GPIO NOT VALID\n");
		devm_free_irq(bcl_dev->device, bcl_dev->bcl_irq[SMPL_WARN], bcl_dev);
		bypass_smpl_warn = true;
	}
	bcl_dev->main_pmic_i2c = main_dev->pmic;
	bcl_dev->main_meter_i2c = main_dev->meter;
	bcl_dev->main_dev = main_dev->dev;
	/* clear MAIN information every boot */
	/* see b/215371539 */
	pmic_read(MAIN, bcl_dev, S2MPG14_PM_OFFSRC1, &val);
	dev_info(bcl_dev->device, "MAIN OFFSRC1 : %#x\n", val);
	bcl_dev->main_offsrc1 = val;
	pmic_read(MAIN, bcl_dev, S2MPG14_PM_OFFSRC2, &val);
	dev_info(bcl_dev->device, "MAIN OFFSRC2 : %#x\n", val);
	bcl_dev->main_offsrc2 = val;
	pmic_read(MAIN, bcl_dev, S2MPG14_PM_PWRONSRC, &val);
	dev_info(bcl_dev->device, "MAIN PWRONSRC: %#x\n", val);
	bcl_dev->pwronsrc = val;
	pmic_write(MAIN, bcl_dev, S2MPG14_PM_OFFSRC1, 0);
	pmic_write(MAIN, bcl_dev, S2MPG14_PM_OFFSRC2, 0);
	pmic_write(MAIN, bcl_dev, S2MPG14_PM_PWRONSRC, 0);
	bcl_dev->bcl_irq[SMPL_WARN] = gpio_to_irq(pdata_main->smpl_warn_pin);
	irq_set_status_flags(bcl_dev->bcl_irq[SMPL_WARN], IRQ_DISABLE_UNLAZY);
	bcl_dev->bcl_pin[SMPL_WARN] = pdata_main->smpl_warn_pin;
	bcl_dev->bcl_lvl[SMPL_WARN] = SMPL_BATTERY_VOLTAGE -
			(pdata_main->smpl_warn_lvl * SMPL_STEP + SMPL_LOWER_LIMIT);
	bcl_dev->bcl_lvl[OCP_WARN_CPUCL1] = B3M_UPPER_LIMIT -
			THERMAL_HYST_LEVEL - (pdata_main->b3_ocp_warn_lvl * B3M_STEP);
	bcl_dev->bcl_lvl[SOFT_OCP_WARN_CPUCL1] = B3M_UPPER_LIMIT -
			THERMAL_HYST_LEVEL - (pdata_main->b3_soft_ocp_warn_lvl * B3M_STEP);
	bcl_dev->bcl_lvl[OCP_WARN_CPUCL2] = B2M_UPPER_LIMIT -
			THERMAL_HYST_LEVEL - (pdata_main->b2_ocp_warn_lvl * B2M_STEP);
	bcl_dev->bcl_lvl[SOFT_OCP_WARN_CPUCL2] = B2M_UPPER_LIMIT -
			THERMAL_HYST_LEVEL - (pdata_main->b2_soft_ocp_warn_lvl * B2M_STEP);
	bcl_dev->bcl_lvl[OCP_WARN_TPU] = B7M_UPPER_LIMIT -
			THERMAL_HYST_LEVEL - (pdata_main->b7_ocp_warn_lvl * B7M_STEP);
	bcl_dev->bcl_lvl[SOFT_OCP_WARN_TPU] = B7M_UPPER_LIMIT -
			THERMAL_HYST_LEVEL - (pdata_main->b7_soft_ocp_warn_lvl * B7M_STEP);
	bcl_dev->bcl_pin[OCP_WARN_CPUCL1] = pdata_main->b3_ocp_warn_pin;
	bcl_dev->bcl_pin[OCP_WARN_CPUCL2] = pdata_main->b2_ocp_warn_pin;
	bcl_dev->bcl_pin[SOFT_OCP_WARN_CPUCL1] = pdata_main->b3_soft_ocp_warn_pin;
	bcl_dev->bcl_pin[SOFT_OCP_WARN_CPUCL2] = pdata_main->b2_soft_ocp_warn_pin;
	bcl_dev->bcl_pin[OCP_WARN_TPU] = pdata_main->b7_ocp_warn_pin;
	bcl_dev->bcl_pin[SOFT_OCP_WARN_TPU] = pdata_main->b7_soft_ocp_warn_pin;
	bcl_dev->bcl_irq[OCP_WARN_CPUCL1] = gpio_to_irq(pdata_main->b3_ocp_warn_pin);
	bcl_dev->bcl_irq[OCP_WARN_CPUCL2] = gpio_to_irq(pdata_main->b2_ocp_warn_pin);
	bcl_dev->bcl_irq[SOFT_OCP_WARN_CPUCL1] = gpio_to_irq(pdata_main->b3_soft_ocp_warn_pin);
	bcl_dev->bcl_irq[SOFT_OCP_WARN_CPUCL2] = gpio_to_irq(pdata_main->b2_soft_ocp_warn_pin);
	bcl_dev->bcl_irq[OCP_WARN_TPU] = gpio_to_irq(pdata_main->b7_ocp_warn_pin);
	bcl_dev->bcl_irq[SOFT_OCP_WARN_TPU] = gpio_to_irq(pdata_main->b7_soft_ocp_warn_pin);
	bcl_dev->bcl_ops[SMPL_WARN].get_temp = smpl_warn_read_voltage;
	bcl_dev->bcl_ops[OCP_WARN_CPUCL1].get_temp = ocp_cpu1_read_current;
	bcl_dev->bcl_ops[OCP_WARN_CPUCL2].get_temp = ocp_cpu2_read_current;
	bcl_dev->bcl_ops[SOFT_OCP_WARN_CPUCL1].get_temp = soft_ocp_cpu1_read_current;
	bcl_dev->bcl_ops[SOFT_OCP_WARN_CPUCL2].get_temp = soft_ocp_cpu2_read_current;
	bcl_dev->bcl_ops[OCP_WARN_TPU].get_temp = ocp_tpu_read_current;
	bcl_dev->bcl_ops[SOFT_OCP_WARN_TPU].get_temp = soft_ocp_tpu_read_current;
	if (!bypass_smpl_warn) {
		ret = google_bcl_register_irq(bcl_dev, SMPL_WARN, "SMPL_WARN_IRQ",
					      IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING);
		if (ret < 0) {
			dev_err(bcl_dev->device, "bcl_register fail: SMPL_WARN\n");
			return -ENODEV;
		}
	}
	ret = google_bcl_register_irq(bcl_dev, OCP_WARN_CPUCL1, "CPU1_OCP_IRQ",
				      IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: CPUCL1\n");
		return -ENODEV;
	}
	ret = google_bcl_register_irq(bcl_dev, OCP_WARN_CPUCL2, "CPU2_OCP_IRQ",
				      IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: CPUCL2\n");
		return -ENODEV;
	}
	ret = google_bcl_register_irq(bcl_dev, SOFT_OCP_WARN_CPUCL1, "SOFT_CPU1_OCP_IRQ",
				      IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: SOFT_CPUCL1\n");
		return -ENODEV;
	}
	ret = google_bcl_register_irq(bcl_dev, SOFT_OCP_WARN_CPUCL2, "SOFT_CPU2_OCP_IRQ",
				      IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: SOFT_CPUCL2\n");
		return -ENODEV;
	}
	ret = google_bcl_register_irq(bcl_dev, OCP_WARN_TPU, "TPU_OCP_IRQ",
				      IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: TPU\n");
		return -ENODEV;
	}
	ret = google_bcl_register_irq(bcl_dev, SOFT_OCP_WARN_TPU, "SOFT_TPU_OCP_IRQ",
				      IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: SOFT_TPU\n");
		return -ENODEV;
	}
	for (i = 0; i < S2MPG1415_METER_CHANNEL_MAX; i++) {
		bcl_dev->main_pwr_warn_irq[i] = bcl_dev->main_irq_base
				+ S2MPG14_IRQ_PWR_WARN_CH0_INT6 + i;
		ret = devm_request_threaded_irq(bcl_dev->device, bcl_dev->main_pwr_warn_irq[i],
						NULL, main_pwr_warn_irq_handler, 0, "PWR_WARN",
						bcl_dev);
		if (ret < 0) {
			dev_err(bcl_dev->device, "Failed to request PWR_WARN_CH%d IRQ: %d: %d\n",
				i, bcl_dev->main_pwr_warn_irq[i], ret);
		}
	}


	return 0;

}

extern const struct attribute_group *mitigation_groups[];

static int google_init_fs(struct bcl_device *bcl_dev)
{
	bcl_dev->mitigation_dev = pmic_subdevice_create(NULL, mitigation_groups,
							bcl_dev, "mitigation");
	if (IS_ERR(bcl_dev->mitigation_dev))
		return -ENODEV;

	return 0;
}

static void google_bcl_enable_vdroop_irq(struct bcl_device *bcl_dev)
{
	void __iomem *gpio_alive;
	unsigned int reg;

	gpio_alive = ioremap(GPIO_ALIVE_BASE, SZ_4K);
	reg = __raw_readl(gpio_alive + GPA9_CON);
	reg |= 0xFF0000;
	__raw_writel(0xFFFFF22, gpio_alive + GPA9_CON);
}

static int google_bcl_init_instruction(struct bcl_device *bcl_dev)
{
	unsigned int reg;

	if (!bcl_dev)
		return -EIO;

	bcl_dev->base_mem[CPU0] = devm_ioremap(bcl_dev->device, CPUCL0_BASE, SZ_8K);
	if (!bcl_dev->base_mem[CPU0]) {
		dev_err(bcl_dev->device, "cpu0_mem ioremap failed\n");
		return -EIO;
	}
	bcl_dev->base_mem[CPU1] = devm_ioremap(bcl_dev->device, CPUCL1_BASE, SZ_8K);
	if (!bcl_dev->base_mem[CPU1]) {
		dev_err(bcl_dev->device, "cpu1_mem ioremap failed\n");
		return -EIO;
	}
	bcl_dev->base_mem[CPU2] = devm_ioremap(bcl_dev->device, CPUCL2_BASE, SZ_8K);
	if (!bcl_dev->base_mem[CPU2]) {
		dev_err(bcl_dev->device, "cpu2_mem ioremap failed\n");
		return -EIO;
	}
	bcl_dev->base_mem[TPU] = devm_ioremap(bcl_dev->device, TPU_BASE, SZ_8K);
	if (!bcl_dev->base_mem[TPU]) {
		dev_err(bcl_dev->device, "tpu_mem ioremap failed\n");
		return -EIO;
	}
	bcl_dev->base_mem[GPU] = devm_ioremap(bcl_dev->device, G3D_BASE, SZ_8K);
	if (!bcl_dev->base_mem[GPU]) {
		dev_err(bcl_dev->device, "gpu_mem ioremap failed\n");
		return -EIO;
	}
	bcl_dev->sysreg_cpucl0 = devm_ioremap(bcl_dev->device, SYSREG_CPUCL0_BASE, SZ_8K);
	if (!bcl_dev->sysreg_cpucl0) {
		dev_err(bcl_dev->device, "sysreg_cpucl0 ioremap failed\n");
		return -EIO;
	}

	mutex_lock(&sysreg_lock);
	reg = __raw_readl(bcl_dev->sysreg_cpucl0 + CLUSTER0_GENERAL_CTRL_64);
	reg |= MPMMEN_MASK;
	__raw_writel(reg, bcl_dev->sysreg_cpucl0 + CLUSTER0_GENERAL_CTRL_64);
	reg = __raw_readl(bcl_dev->sysreg_cpucl0 + CLUSTER0_PPM);
	reg |= PPMEN_MASK;
	__raw_writel(reg, bcl_dev->sysreg_cpucl0 + CLUSTER0_PPM);

	mutex_unlock(&sysreg_lock);
	mutex_init(&bcl_dev->state_trans_lock);
	mutex_init(&bcl_dev->ratio_lock);
	google_bcl_enable_vdroop_irq(bcl_dev);

	bcl_dev->base_add_mem[CPU0] = devm_ioremap(bcl_dev->device, ADD_CPUCL0, SZ_128);
	if (!bcl_dev->base_add_mem[CPU0]) {
		dev_err(bcl_dev->device, "cpu0_add_mem ioremap failed\n");
		return -EIO;
	}

	bcl_dev->base_add_mem[CPU1] = devm_ioremap(bcl_dev->device, ADD_CPUCL1, SZ_128);
	if (!bcl_dev->base_add_mem[CPU1]) {
		dev_err(bcl_dev->device, "cpu1_add_mem ioremap failed\n");
		return -EIO;
	}

	bcl_dev->base_add_mem[CPU2] = devm_ioremap(bcl_dev->device, ADD_CPUCL2, SZ_128);
	if (!bcl_dev->base_add_mem[CPU2]) {
		dev_err(bcl_dev->device, "cpu2_add_mem ioremap failed\n");
		return -EIO;
	}

	bcl_dev->base_add_mem[TPU] = devm_ioremap(bcl_dev->device, ADD_TPU, SZ_128);
	if (!bcl_dev->base_add_mem[TPU]) {
		dev_err(bcl_dev->device, "tpu_add_mem ioremap failed\n");
		return -EIO;
	}

	bcl_dev->base_add_mem[GPU] = devm_ioremap(bcl_dev->device, ADD_G3D, SZ_128);
	if (!bcl_dev->base_add_mem[GPU]) {
		dev_err(bcl_dev->device, "gpu_add_mem ioremap failed\n");
		return -EIO;
	}

	bcl_dev->base_add_mem[AUR] = devm_ioremap(bcl_dev->device, ADD_AUR, SZ_128);
	if (!bcl_dev->base_add_mem[AUR]) {
		dev_err(bcl_dev->device, "aur_add_mem ioremap failed\n");
		return -EIO;
	}
	return 0;
}

u64 settings_to_current(struct bcl_device *bcl_dev, int pmic, int idx, u32 setting)
{
	int rail_i;
	s2mpg1415_meter_muxsel muxsel;
	struct odpm_info *info;
	const u32 one_billion = 1000000000;
	u64 resolution_max, raw_unit;
	u32 resolution;

	if (!bcl_dev)
		return 0;
	if (pmic == MAIN)
		info = bcl_dev->main_odpm;
	else
		info = bcl_dev->sub_odpm;

	if (!info)
		return 0;

	rail_i = info->channels[idx].rail_i;
	muxsel = info->chip.rails[rail_i].mux_select;
	if (idx < 9) {
		if (pmic == MAIN)
			resolution = s2mpg14_muxsel_to_current_resolution(muxsel);
		else
			resolution = s2mpg15_muxsel_to_current_resolution(muxsel);
		resolution_max = _IQ30_to_int((u64)resolution * one_billion);

		return (setting << LPF_CURRENT_SHIFT) * resolution_max / one_billion;
	} else {
		raw_unit = EXTERNAL_RESOLUTION_VSHUNT / info->chip.rails[rail_i].shunt_uohms;
		return (setting << LPF_CURRENT_SHIFT) * raw_unit;
	}
}

static void google_bcl_parse_dtree(struct bcl_device *bcl_dev)
{
	int ret, i = 0;
	struct device_node *np = bcl_dev->device->of_node;
	struct device_node *child;
	struct device_node *p_np;
	u32 val;
	int read;

	if (!bcl_dev) {
		dev_err(bcl_dev->device, "Cannot parse device tree\n");
		return;
	}
	ret = of_property_read_u32(np, "tpu_con_heavy", &val);
	bcl_dev->tpu_con_heavy = ret ? 0 : val;
	ret = of_property_read_u32(np, "tpu_con_light", &val);
	bcl_dev->tpu_con_light = ret ? 0 : val;
	ret = of_property_read_u32(np, "gpu_con_heavy", &val);
	bcl_dev->gpu_con_heavy = ret ? 0 : val;
	ret = of_property_read_u32(np, "gpu_con_light", &val);
	bcl_dev->gpu_con_light = ret ? 0 : val;
	ret = of_property_read_u32(np, "gpu_clkdivstep", &val);
	bcl_dev->gpu_clkdivstep = ret ? 0 : val;
	ret = of_property_read_u32(np, "tpu_clkdivstep", &val);
	bcl_dev->tpu_clkdivstep = ret ? 0 : val;
	ret = of_property_read_u32(np, "cpu2_clkdivstep", &val);
	bcl_dev->cpu2_clkdivstep = ret ? 0 : val;
	ret = of_property_read_u32(np, "cpu1_clkdivstep", &val);
	bcl_dev->cpu1_clkdivstep = ret ? 0 : val;
	ret = of_property_read_u32(np, "cpu0_clkdivstep", &val);
	bcl_dev->cpu0_clkdivstep = ret ? 0 : val;
	bcl_dev->vdroop1_pin = of_get_gpio(np, 0);
	bcl_dev->vdroop2_pin = of_get_gpio(np, 1);
	bcl_dev->modem_gpio1_pin = of_get_gpio(np, 2);
	bcl_dev->modem_gpio2_pin = of_get_gpio(np, 3);
	ret = of_property_read_u32(np, "rffe_channel", &val);
	bcl_dev->rffe_channel = ret ? 11 : val;

	/* parse ODPM main limit */
	p_np = of_get_child_by_name(np, "main_limit");
	if (p_np) {
		for_each_child_of_node(p_np, child) {
			of_property_read_u32(child, "setting", &read);
			if (i < METER_CHANNEL_MAX) {
				bcl_dev->main_setting[i] = read;
				meter_write(MAIN, bcl_dev, S2MPG14_METER_PWR_WARN0 + i, read);
				bcl_dev->main_limit[i] = settings_to_current(bcl_dev, MAIN, i,
									     read);
				i++;
			}
		}
	}

	/* parse ODPM sub limit */
	p_np = of_get_child_by_name(np, "sub_limit");
	i = 0;
	if (p_np) {
		for_each_child_of_node(p_np, child) {
			of_property_read_u32(child, "setting", &read);
			if (i < METER_CHANNEL_MAX) {
				bcl_dev->sub_setting[i] = read;
				meter_write(SUB, bcl_dev, S2MPG15_METER_PWR_WARN0 + i, read);
				bcl_dev->sub_limit[i] = settings_to_current(bcl_dev, SUB, i, read);
				i++;
			}
		}
	}

	bcl_disable_power();
	if (google_bcl_init_clk_div(bcl_dev, CPU2, bcl_dev->cpu2_clkdivstep) != 0)
		dev_err(bcl_dev->device, "CPU2 Address is NULL\n");
	if (google_bcl_init_clk_div(bcl_dev, CPU1, bcl_dev->cpu1_clkdivstep) != 0)
		dev_err(bcl_dev->device, "CPU1 Address is NULL\n");
	if (google_bcl_init_clk_div(bcl_dev, CPU0, bcl_dev->cpu0_clkdivstep) != 0)
		dev_err(bcl_dev->device, "CPU0 Address is NULL\n");
	bcl_enable_power();
}

static int google_bcl_configure_modem(struct bcl_device *bcl_dev)
{
	struct pinctrl *modem_pinctrl;
	struct pinctrl_state *batoilo_pinctrl_state, *rffe_pinctrl_state;
	int ret;

	modem_pinctrl = devm_pinctrl_get(bcl_dev->device);
	if (IS_ERR_OR_NULL(modem_pinctrl)) {
		dev_err(bcl_dev->device, "Cannot find modem_pinctrl!\n");
		return -EINVAL;
	}
	batoilo_pinctrl_state = pinctrl_lookup_state(modem_pinctrl, "bcl-batoilo-modem");
	if (IS_ERR_OR_NULL(batoilo_pinctrl_state)) {
		dev_err(bcl_dev->device, "batoilo: pinctrl lookup state failed!\n");
		return -EINVAL;
	}
	rffe_pinctrl_state = pinctrl_lookup_state(modem_pinctrl, "bcl-rffe-modem");
	if (IS_ERR_OR_NULL(rffe_pinctrl_state)) {
		dev_err(bcl_dev->device, "rffe: pinctrl lookup state failed!\n");
		return -EINVAL;
	}
	ret = pinctrl_select_state(modem_pinctrl, batoilo_pinctrl_state);
	if (ret < 0) {
		dev_err(bcl_dev->device, "batoilo: pinctrl select state failed!!\n");
		return -EINVAL;
	}
	ret = pinctrl_select_state(modem_pinctrl, rffe_pinctrl_state);
	if (ret < 0) {
		dev_err(bcl_dev->device, "rffe: pinctrl select state failed!!\n");
		return -EINVAL;
	}
	return 0;
}

static int google_bcl_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct bcl_device *bcl_dev;

	bcl_dev = devm_kzalloc(&pdev->dev, sizeof(*bcl_dev), GFP_KERNEL);
	if (!bcl_dev)
		return -ENOMEM;
	bcl_dev->device = &pdev->dev;

	INIT_DELAYED_WORK(&bcl_dev->init_work, google_set_intf_pmic_work);
	platform_set_drvdata(pdev, bcl_dev);

	bcl_dev->ready = false;
	ret = google_bcl_init_instruction(bcl_dev);
	if (ret < 0)
		goto bcl_soc_probe_exit;

	google_set_throttling(bcl_dev);
	google_set_main_pmic(bcl_dev);
	google_set_sub_pmic(bcl_dev);
	google_set_intf_pmic(bcl_dev);
	google_bcl_parse_dtree(bcl_dev);
	google_bcl_configure_modem(bcl_dev);

	ret = google_init_fs(bcl_dev);
	if (ret < 0)
		goto bcl_soc_probe_exit;
	schedule_delayed_work(&bcl_dev->init_work, msecs_to_jiffies(THERMAL_DELAY_INIT_MS));
	bcl_dev->enabled = true;
	google_init_debugfs(bcl_dev);

	return 0;

bcl_soc_probe_exit:
	google_bcl_remove_thermal(bcl_dev);
	return ret;
}

static int google_bcl_remove(struct platform_device *pdev)
{
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	pmic_device_destroy(bcl_dev->mitigation_dev->devt);
	debugfs_remove_recursive(bcl_dev->debug_entry);
	google_bcl_remove_thermal(bcl_dev);

	return 0;
}

static const struct of_device_id match_table[] = {
	{ .compatible = "google,google-bcl"},
	{},
};

static struct platform_driver google_bcl_driver = {
	.probe  = google_bcl_probe,
	.remove = google_bcl_remove,
	.id_table = google_id_table,
	.driver = {
		.name           = "google_mitigation",
		.owner          = THIS_MODULE,
		.of_match_table = match_table,
	},
};

module_platform_driver(google_bcl_driver);

MODULE_SOFTDEP("pre: i2c-acpm");
MODULE_DESCRIPTION("Google Battery Current Limiter");
MODULE_AUTHOR("George Lee <geolee@google.com>");
MODULE_LICENSE("GPL");
