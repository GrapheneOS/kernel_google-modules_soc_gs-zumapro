// SPDX-License-Identifier: GPL-2.0-only
/*
 * gs_tmu_v3.c - Samsung GS TMU (Thermal Management Unit)
 *
 *  Copyright (C) 2019-2022 Samsung Electronics
 *  Hyeonseong Gil <hs.gil@samsung.com>
 *
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/cpufreq.h>
#include <linux/suspend.h>
#include <soc/google/exynos_pm_qos.h>
#include <linux/threads.h>
#include <linux/thermal.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <uapi/linux/sched/types.h>
#include <soc/google/bcl.h>
#include <soc/google/gs_tmu_v3.h>
#include <soc/google/tmu.h>
#include <soc/google/ect_parser.h>
#include <soc/google/isp_cooling.h>
#include <soc/google/acpm_ipc_ctrl.h>
#if IS_ENABLED(CONFIG_EXYNOS_MCINFO)
#include <soc/google/exynos-mcinfo.h>
#endif

#include <thermal_core.h>
#if IS_ENABLED(CONFIG_EXYNOS_ACPM_THERMAL)
#include "exynos_acpm_tmu.h"
#endif
#include <soc/google/exynos-cpuhp.h>

#include <trace/events/power.h>

#define CREATE_TRACE_POINTS
#include <trace/events/thermal_exynos.h>

#define EXYNOS_GPU_TMU_GRP_ID		(3)

#define FRAC_BITS 10
#define int_to_frac(x) ((x) << FRAC_BITS)
#define frac_to_int(x) ((x) >> FRAC_BITS)

#define INVALID_TRIP -1
/* current version of trace struct is only supported in verion 3 */
#define EXPECT_BUF_VER 4
#define ACPM_BUF_VER (acpm_gov_common.buffer_version & 0xff)

enum tmu_type_t {
	TMU_TYPE_CPU,
	TMU_TYPE_GPU,
	TMU_TYPE_ISP,
	TMU_TYPE_TPU,
#if IS_ENABLED(CONFIG_SOC_ZUMA)
	TMU_TYPE_AUR,
#endif
	TMU_TYPE_END,
};

#if IS_ENABLED(CONFIG_SOC_GS101)
#define TZ_BIG_SENSOR_MASK (TMU_P0_SENSOR_MASK | \
			    TMU_P6_SENSOR_MASK | \
			    TMU_P7_SENSOR_MASK | \
			    TMU_P8_SENSOR_MASK | \
			    TMU_P9_SENSOR_MASK)
#define TZ_MID_SENSOR_MASK (TMU_P4_SENSOR_MASK | \
			    TMU_P5_SENSOR_MASK)
#define TZ_LIT_SENSOR_MASK (TMU_P1_SENSOR_MASK | \
			    TMU_P2_SENSOR_MASK)
#define TZ_GPU_SENSOR_MASK (TMU_P1_SENSOR_MASK | \
			    TMU_P2_SENSOR_MASK | \
			    TMU_P3_SENSOR_MASK | \
			    TMU_P4_SENSOR_MASK | \
			    TMU_P5_SENSOR_MASK | \
			    TMU_P6_SENSOR_MASK | \
			    TMU_P7_SENSOR_MASK)
#define TZ_ISP_SENSOR_MASK (TMU_P14_SENSOR_MASK)
#define TZ_TPU_SENSOR_MASK (TMU_P8_SENSOR_MASK | \
			    TMU_P9_SENSOR_MASK | \
			    TMU_P10_SENSOR_MASK | \
			    TMU_P11_SENSOR_MASK)

#elif IS_ENABLED(CONFIG_SOC_ZUMA)
#define TZ_BIG_SENSOR_MASK (TMU_P1_SENSOR_MASK | \
			    TMU_P2_SENSOR_MASK)
#define TZ_MID_SENSOR_MASK (TMU_P4_SENSOR_MASK | \
			    TMU_P5_SENSOR_MASK | \
			    TMU_P6_SENSOR_MASK | \
			    TMU_P7_SENSOR_MASK)
#define TZ_LIT_SENSOR_MASK (TMU_P8_SENSOR_MASK | \
			    TMU_P9_SENSOR_MASK)
#define TZ_GPU_SENSOR_MASK (TMU_P10_SENSOR_MASK | \
			    TMU_P11_SENSOR_MASK | \
			    TMU_P12_SENSOR_MASK)
#define TZ_ISP_SENSOR_MASK (TMU_P14_SENSOR_MASK)
#define TZ_TPU_SENSOR_MASK (TMU_P2_SENSOR_MASK | \
			    TMU_P3_SENSOR_MASK | \
			    TMU_P4_SENSOR_MASK)
#define TZ_AUR_SENSOR_MASK (TMU_P6_SENSOR_MASK | \
			    TMU_P7_SENSOR_MASK | \
			    TMU_P8_SENSOR_MASK)
#endif

static struct thermal_zone_data tz_config[] = {
	[TZ_BIG] = {
		.tmu_zone_id = TMU_TOP,
		.sensors_mask = TZ_BIG_SENSOR_MASK,
	},
	[TZ_MID] = {
		.tmu_zone_id = TMU_TOP,
		.sensors_mask = TZ_MID_SENSOR_MASK,
	},
	[TZ_LIT] = {
		.tmu_zone_id = TMU_TOP,
		.sensors_mask = TZ_LIT_SENSOR_MASK,
	},
	[TZ_GPU] = {
		.tmu_zone_id = TMU_SUB,
		.sensors_mask = TZ_GPU_SENSOR_MASK,
	},
	[TZ_ISP] = {
		.tmu_zone_id = TMU_SUB,
		.sensors_mask = TZ_ISP_SENSOR_MASK,
	},
	[TZ_TPU] = {
		.tmu_zone_id = TMU_SUB,
		.sensors_mask = TZ_TPU_SENSOR_MASK,
	},
#if IS_ENABLED(CONFIG_SOC_ZUMA)
	[TZ_AUR] = {
		.tmu_zone_id = TMU_SUB,
		.sensors_mask = TZ_AUR_SENSOR_MASK,
	},
#endif
};

/**
 * mul_frac() - multiply two fixed-point numbers
 * @x:	first multiplicand
 * @y:	second multiplicand
 *
 * Return: the result of multiplying two fixed-point numbers.  The
 * result is also a fixed-point number.
 */
static inline s64 mul_frac(s64 x, s64 y)
{
	return (x * y) >> FRAC_BITS;
}

/**
 * div_frac() - divide two fixed-point numbers
 * @x:	the dividend
 * @y:	the divisor
 *
 * Return: the result of dividing two fixed-point numbers.  The
 * result is also a fixed-point number.
 */
static inline s64 div_frac(s64 x, s64 y)
{
	return div_s64(x << FRAC_BITS, y);
}

static atomic_t gs_tmu_in_suspend;
static struct acpm_tmu_cap cap;
static unsigned int num_of_devices, suspended_count;
struct cpumask tmu_enabled_mask;
EXPORT_SYMBOL(tmu_enabled_mask);

/* list of multiple instance for each thermal sensor */
static LIST_HEAD(dtm_dev_list);
static DEFINE_SPINLOCK(dev_list_spinlock);

static struct acpm_gov_common acpm_gov_common = {
#if IS_ENABLED(CONFIG_SOC_ZUMA)
	.turn_on = true,
#else
	.turn_on = false,
#endif
	.sm_base = 0,
	.sm_size = 0,
	.last_ts = 0,
	.tracing_buffer_flush_pending = false,
	.tracing_mode = ACPM_GOV_DEBUG_MODE_DISABLED,
	.buffer_version = -1,
	.bulk_trace_buffer = NULL,
};

static void sync_kernel_acpm_timestamp(void)
{
	u64 ts_before, ts_after;

	ts_before = (u64)ktime_get_boottime_ns();
	acpm_gov_common.acpm_ts = get_frc_time();
	ts_after = (u64)ktime_get_boottime_ns();

	acpm_gov_common.kernel_ts = (ts_before + ts_after)>>1;
}

static u64 acpm_to_kernel_ts(u64 acpm_ts)
{
	u64 acpm_count_since_sync, kernel_time_ns_since_sync;

	acpm_count_since_sync = (u64)acpm_ts - acpm_gov_common.acpm_ts;
	/* each systick is 20.345ns, approximating to be 20 + 1/3 */
	kernel_time_ns_since_sync = (acpm_count_since_sync * ACPM_SYSTICK_NUMERATOR) +
				    (acpm_count_since_sync / ACPM_SYSTICK_FRACTIONAL_DENOMINATOR);

	return acpm_gov_common.kernel_ts + kernel_time_ns_since_sync;
}

static bool get_bulk_mode_curr_state_buffer(void __iomem *base,
					    struct gov_trace_data_struct *bulk_trace_buffer)
{
	int offset = 0;

	if(ACPM_BUF_VER > 0) {
		/* offset for u64 buffer_version field */
		offset = ACPM_SM_BUFFER_VERSION_SIZE;
	}

	if (base) {
		memcpy_fromio(&bulk_trace_buffer->buffered_curr_state, base + offset,
			      sizeof(*bulk_trace_buffer->buffered_curr_state) *
				      BULK_TRACE_DATA_LEN);
		return true;
	} else {
		return false;
	}
}

static bool get_curr_state_from_acpm(void __iomem *base, int id, struct curr_state *curr_state)
{
	if (base) {
		int cdev_state_offset = 0;
		if (ACPM_BUF_VER > 0) {
			/* offset for u64 buffer_version field */
			cdev_state_offset = ACPM_SM_BUFFER_VERSION_SIZE;
		}
		cdev_state_offset += sizeof(struct buffered_curr_state) * BULK_TRACE_DATA_LEN +
				    sizeof(struct curr_state) * id;
		memcpy_fromio(curr_state, base + cdev_state_offset,
			      sizeof(struct curr_state));
		return true;
	} else {
		return false;
	}
}

static bool get_all_curr_state_from_acpm(void __iomem *base, struct curr_state *curr_state[])
{
	if (base) {
		int cdev_state_offset = 0;
		if (ACPM_BUF_VER > 0) {
			/* offset for u64 buffer_version field */
			cdev_state_offset = ACPM_SM_BUFFER_VERSION_SIZE;
		}
		cdev_state_offset += sizeof(struct buffered_curr_state) * BULK_TRACE_DATA_LEN;
		memcpy_fromio(*curr_state, base + cdev_state_offset,
			      sizeof(struct curr_state) * NR_TZ);
		return true;
	} else {
		return false;
	}
}

#define INVALID_TZID(tzid)          (((tzid) < 0) || ((tzid) >= TZ_END))

static int get_tr_handle_id(tr_handle instance)
{
	struct gs_tmu_data *data;

	list_for_each_entry (data, &dtm_dev_list, node) {
		if (data->tr_handle == instance)
			return data->id;
	}
	return -EINVAL;
}

static int thermal_metrics_get_tr_stats(tr_handle instance, atomic64_t *stats)
{
	int i, num_thresholds, acpm_temp, ipc_stat;
	int tzid = get_tr_handle_id(instance);
	u64 single_stats;

	if (!stats || INVALID_TZID(tzid))
		return -EINVAL;

	exynos_acpm_tmu_ipc_get_tr_num_thresholds(tzid, &num_thresholds);
	/* force the acpm to update tr stats */
	exynos_acpm_tmu_set_read_temp(tzid, &acpm_temp, &ipc_stat);
	for (i = 0; i < num_thresholds + 1; ++i) {
		exynos_acpm_tmu_ipc_get_tr_stats(tzid, i, &single_stats);
		atomic64_set(&stats[i], acpm_systick_to_ms(single_stats));
	}
	return 0;
}

#define NUM_THRESHOLD_PER_IPC  8
static int thermal_metrics_get_tr_thresholds(tr_handle instance, int *thresholds,
                                                int *num_thresholds)
{
	int i, j, ipc_cnt;
	int tzid = get_tr_handle_id(instance);
	union {
		u8 byte[8];
		u64 qword;
	} val;

	if (!thresholds || !num_thresholds || INVALID_TZID(tzid))
		return -EINVAL;

	exynos_acpm_tmu_ipc_get_tr_num_thresholds(tzid, num_thresholds);
	/* each ipc call sends 8 thresholds. determine num of ipc call*/
	ipc_cnt = (*num_thresholds + NUM_THRESHOLD_PER_IPC - 1) / NUM_THRESHOLD_PER_IPC;
	for (i = 0; i < ipc_cnt; ++i) {
		exynos_acpm_tmu_ipc_get_tr_thresholds(tzid, (u8)i, &val.qword);
		for (j = 0; j < NUM_THRESHOLD_PER_IPC; ++j) {
			int thresholds_index = i * NUM_THRESHOLD_PER_IPC + j;
			if (thresholds_index >= *num_thresholds)
				break;
			thresholds[thresholds_index] = val.byte[j] * MCELSIUS;
		}
	}
	return 0;
}

static int thermal_metrics_set_tr_thresholds(tr_handle instance, const int *thresholds,
                                             int num_thresholds)
{
	int i, j, ipc_cnt, acpm_temp, ipc_stat;
	int tzid = get_tr_handle_id(instance);
	union {
		u8 byte[8];
		u64 qword;
	} val;

	if (exynos_acpm_tmu_ipc_set_tr_num_thresholds(tzid, num_thresholds))
		return -EINVAL;

	/* each ipc call sends 8 thresholds. determine num of ipc call*/
	ipc_cnt = (num_thresholds + NUM_THRESHOLD_PER_IPC - 1) / NUM_THRESHOLD_PER_IPC;
	for (i = 0; i < ipc_cnt; ++i) {
		val.qword = 0;
		for (j = 0; j < NUM_THRESHOLD_PER_IPC; ++j) {
			int thresholds_index = i * NUM_THRESHOLD_PER_IPC + j;
			if (thresholds_index >= num_thresholds)
				break;
			val.byte[j] = (u8)(thresholds[thresholds_index] / MCELSIUS);
		}
		exynos_acpm_tmu_ipc_set_tr_thresholds(tzid, (u8)i, val.qword);
	}
	/* force the acpm to update tr stats */
	exynos_acpm_tmu_set_read_temp(tzid, &acpm_temp, &ipc_stat);
	return 0;
}

static int thermal_metrics_reset_stats(tr_handle instance)
{
	int tzid = get_tr_handle_id(instance);
	int acpm_temp, ipc_stat, ret;

	if (INVALID_TZID(tzid))
		return -EINVAL;

	ret = exynos_acpm_tmu_ipc_reset_tr_stats(tzid);
	/* force the acpm to update tr stats */
	exynos_acpm_tmu_set_read_temp(tzid, &acpm_temp, &ipc_stat);

	return ret;
}

static bool get_thermal_state_from_acpm(void __iomem *base, struct thermal_state *thermal_state)
{
	if (base) {
		int thermal_state_offset = ACPM_SM_BUFFER_VERSION_SIZE;
		thermal_state_offset += sizeof(struct buffered_curr_state) * BULK_TRACE_DATA_LEN +
					sizeof(struct curr_state) * NR_TZ;
		memcpy_fromio(thermal_state, base + thermal_state_offset,
			      sizeof(struct thermal_state));
		return true;
	}

	return false;
}

static u64 get_bulk_trace_buffer_timestamp(struct gov_trace_data_struct *bulk_trace_buffer, int idx)
{
	return bulk_trace_buffer->buffered_curr_state[idx].timestamp;
}

int find_rotated_idx(struct gov_trace_data_struct *bulk_trace_buffer, int length)
{
	int left = 0;
	int right = length - 1;

	if (get_bulk_trace_buffer_timestamp(bulk_trace_buffer, left) <
	    get_bulk_trace_buffer_timestamp(bulk_trace_buffer, right)) {
		return left;
	}

	while (left < right) {
		int mid = (left + right) >> 1;
		if (get_bulk_trace_buffer_timestamp(bulk_trace_buffer, left) >
		    get_bulk_trace_buffer_timestamp(bulk_trace_buffer, mid)) {
			right = mid;
		} else if (get_bulk_trace_buffer_timestamp(bulk_trace_buffer, mid) >
			   get_bulk_trace_buffer_timestamp(bulk_trace_buffer, right)) {
			left = mid + 1;
		} else if (get_bulk_trace_buffer_timestamp(bulk_trace_buffer, left) ==
			   get_bulk_trace_buffer_timestamp(bulk_trace_buffer, mid)) {
			if (get_bulk_trace_buffer_timestamp(bulk_trace_buffer, mid) ==
			    get_bulk_trace_buffer_timestamp(bulk_trace_buffer, right)) {
				right--;
			} else {
				return left;
			}
		} else {
			return left;
		}
	}

	return get_bulk_trace_buffer_timestamp(bulk_trace_buffer, left) <
			       get_bulk_trace_buffer_timestamp(bulk_trace_buffer, right) ?
		       left :
		       right;
}

int next_greater_rotated_idx(struct gov_trace_data_struct *bulk_trace_buffer, int array_len,
			     int rotated_idx, u64 value)
{
	int left = 0, right = array_len - 1;

	if (value >=
	    get_bulk_trace_buffer_timestamp(bulk_trace_buffer, (right + rotated_idx) % array_len)) {
		return -1;
	}

	while (left < right) {
		int mid = (left + right) >> 1;
		if (value >= get_bulk_trace_buffer_timestamp(bulk_trace_buffer,
							     (mid + rotated_idx) % array_len)) {
			left = mid + 1;
		} else {
			right = mid;
		}
	}
	return (left + rotated_idx) % array_len;
}

static void capture_bulk_trace(void)
{
	struct gov_trace_data_struct *bulk_trace_buffer;
	int start_idx, end_idx;
	int rotated_idx;
	int i;
	int len;

	struct gs_tmu_data *data;
	bool pi_enable[TZ_END + 1];
	int k_po[TZ_END + 1], k_pu[TZ_END + 1], k_i[TZ_END + 1];
	int k_p = 0;

	list_for_each_entry (data, &dtm_dev_list, node) {
		pi_enable[data->id] = data->acpm_pi_enable;
		if (pi_enable[data->id]) {
			k_po[data->id] = data->pi_param->k_po;
			k_pu[data->id] = data->pi_param->k_pu;
			k_i[data->id] = data->pi_param->k_i;
		} else {
			k_po[data->id] = 0;
			k_pu[data->id] = 0;
			k_i[data->id] = 0;
		}
	}

	spin_lock(&acpm_gov_common.lock);
	bulk_trace_buffer = acpm_gov_common.bulk_trace_buffer;
	if (!bulk_trace_buffer)
		goto unlock;

	if (!get_bulk_mode_curr_state_buffer(acpm_gov_common.sm_base, bulk_trace_buffer) == true)
		goto unlock;
	rotated_idx = find_rotated_idx(bulk_trace_buffer, BULK_TRACE_DATA_LEN);
	if (rotated_idx == 0)
		end_idx = BULK_TRACE_DATA_LEN - 1;
	else
		end_idx = rotated_idx - 1;

	start_idx = next_greater_rotated_idx(bulk_trace_buffer, BULK_TRACE_DATA_LEN, rotated_idx,
					     acpm_gov_common.last_ts);
	if (start_idx == -1)
		goto unlock;

	acpm_gov_common.last_ts = get_bulk_trace_buffer_timestamp(bulk_trace_buffer, end_idx);

	if (start_idx <= end_idx)
		len = end_idx - start_idx + 1;
	else
		len = BULK_TRACE_DATA_LEN - (start_idx - end_idx) + 1;

	for (i = 0; i < len; i++) {
		k_p = (bulk_trace_buffer->buffered_curr_state[start_idx].ctrl_temp -
		       bulk_trace_buffer->buffered_curr_state[start_idx].temperature) < 0 ?
			      k_po[bulk_trace_buffer->buffered_curr_state[start_idx].tzid] :
			      k_pu[bulk_trace_buffer->buffered_curr_state[start_idx].tzid];
		trace_thermal_exynos_acpm_bulk(
			(u8)(bulk_trace_buffer->buffered_curr_state[start_idx].tzid),
			(u8)(bulk_trace_buffer->buffered_curr_state[start_idx].temperature),
			(u8)(bulk_trace_buffer->buffered_curr_state[start_idx].ctrl_temp),
			(u8)(bulk_trace_buffer->buffered_curr_state[start_idx].cdev_state),
			(s32)(bulk_trace_buffer->buffered_curr_state[start_idx].pid_err_integral),
			(s16)(bulk_trace_buffer->buffered_curr_state[start_idx].pid_power_range),
			(s16)(bulk_trace_buffer->buffered_curr_state[start_idx].pid_p),
			(s32)(bulk_trace_buffer->buffered_curr_state[start_idx].pid_i),
			(s32)frac_to_int(k_p),
			(s32)frac_to_int(
				k_i[bulk_trace_buffer->buffered_curr_state[start_idx].tzid]),
			acpm_to_kernel_ts(
				get_bulk_trace_buffer_timestamp(bulk_trace_buffer, start_idx)));
		if (++start_idx >= BULK_TRACE_DATA_LEN)
			start_idx = 0;
	}
unlock:
	spin_unlock(&acpm_gov_common.lock);
}

static bool cpu_switch_on_status_changed(struct thermal_state thermal_state)
{
	bool prev_switch_on_state =
		((acpm_gov_common.thermal_pressure.state.switched_on & CPU_TZ_MASK) != 0);
	bool curr_switch_on_state = ((thermal_state.switched_on & CPU_TZ_MASK) != 0);

	return prev_switch_on_state != curr_switch_on_state;
}

static u8 get_dfs_status_changed(struct thermal_state thermal_state)
{
	u8 prev_dfs_on_state = acpm_gov_common.thermal_pressure.state.dfs_on;
	u8 curr_dfs_on_state = thermal_state.dfs_on;

	return prev_dfs_on_state ^ curr_dfs_on_state;
}

static void acpm_irq_cb(unsigned int *cmd, unsigned int size)
{
	struct thermal_state thermal_state;
	struct curr_state curr_state_all[NR_TZ];
	struct curr_state *curr_state_ptr = curr_state_all;
	bool curr_state_read = false;

	if (ACPM_BUF_VER != EXPECT_BUF_VER)
		return;

	switch (acpm_gov_common.tracing_mode) {
	case ACPM_GOV_DEBUG_MODE_BULK:
		capture_bulk_trace();
		break;
	case ACPM_GOV_DEBUG_MODE_HIGH_OVERHEAD: {
		struct gs_tmu_data *data;

		if (get_all_curr_state_from_acpm(acpm_gov_common.sm_base, &curr_state_ptr)) {
			curr_state_read = true;
			list_for_each_entry (data, &dtm_dev_list, node) {
				struct curr_state curr_state = curr_state_all[data->id];

				int k_p = 0, k_i = 0;
				if (data->acpm_pi_enable) {
					k_p = (curr_state.ctrl_temp - curr_state.temperature) < 0 ?
						      data->pi_param->k_po :
						      data->pi_param->k_pu;
					k_i = data->pi_param->k_i;
				}
				trace_thermal_exynos_acpm_high_overhead(
					(int)data->id, (u8)curr_state.temperature,
					(u8)curr_state.ctrl_temp,
					(u8)curr_state.cdev_state,
					(s32)curr_state.pid_err_integral,
					(s32)frac_to_int(k_p),
					(s32)frac_to_int(k_i));
			}
		}
	} break;
	default:
		break;
	}

	if (get_thermal_state_from_acpm(acpm_gov_common.sm_base, &thermal_state)) {
		u8 dfs_status_changed;
		bool switch_on_work_needed = false;
		struct gs_tmu_data *data;

		spin_lock(&acpm_gov_common.thermal_pressure.lock);
		switch_on_work_needed = cpu_switch_on_status_changed(thermal_state);
		dfs_status_changed = get_dfs_status_changed(thermal_state);
		acpm_gov_common.thermal_pressure.state = thermal_state;
		spin_unlock(&acpm_gov_common.thermal_pressure.lock);

		if (switch_on_work_needed)
			kthread_queue_work(&acpm_gov_common.thermal_pressure.worker,
					   &acpm_gov_common.thermal_pressure.switch_on_work);

		if (!dfs_status_changed)
			return;

		if (!curr_state_read)
			get_all_curr_state_from_acpm(acpm_gov_common.sm_base, &curr_state_ptr);

		list_for_each_entry (data, &dtm_dev_list, node) {
			struct curr_state curr_state = curr_state_all[data->id];
			if ((!(dfs_status_changed & (1 << data->id))) ||
			    (!(thermal_state.dfs_on & (1 << data->id))))
				continue;
			pr_info_ratelimited("%s DFS on: temperature = %dC, cdev_state = %d\n",
					    data->tmu_name, curr_state.temperature,
					    curr_state.cdev_state);
		}
	}
}

static struct acpm_irq_callback cb = { .fn = acpm_irq_cb, .ipc_ch = -1, .ipc_ch_size = -1 };

void register_thermal_pressure_cb(thermal_pressure_cb cb)
{
	if (WARN_ON(!cb)) {
		pr_err("Failed to register in %s\n", __func__);
		return;
	}

	if (WARN_ON(acpm_gov_common.thermal_pressure.cb)) {
		pr_err("thermal_pressure_cb function is already set");
		return;
	}

	acpm_gov_common.thermal_pressure.cb = cb;
}
EXPORT_SYMBOL(register_thermal_pressure_cb);

static void start_thermal_pressure_polling(int delay)
{
	kthread_mod_delayed_work(&acpm_gov_common.thermal_pressure.worker,
				 &acpm_gov_common.thermal_pressure.polling_work, msecs_to_jiffies(delay));
}

static void thermal_pressure_work(bool apply_thermal_pressure, struct thermal_state state)
{
	int delay;
	bool switched_on = state.switched_on & CPU_TZ_MASK;

	if (apply_thermal_pressure && acpm_gov_common.thermal_pressure.cb) {
		struct gs_tmu_data *data;

		list_for_each_entry (data, &dtm_dev_list, node) {
			if ((data->pressure_index == -1) || cpumask_empty(&data->mapped_cpus))
				continue;
			acpm_gov_common.thermal_pressure.cb(&data->mapped_cpus,
							    (int)state.therm_press[data->pressure_index]);
		}
	}

	if (switched_on) {
		delay = acpm_gov_common.thermal_pressure.polling_delay_on;
	} else
		delay = acpm_gov_common.thermal_pressure.polling_delay_off;

	if (delay)
		start_thermal_pressure_polling(delay);
}

static void acpm_switch_on_work(struct kthread_work *work)
{
	thermal_pressure_work(true, acpm_gov_common.thermal_pressure.state);
}

static void thermal_pressure_polling(struct kthread_work *work)
{
	struct thermal_state state;
	bool switched_on = false;
	bool apply_thermal_pressure = false;

	if (get_thermal_state_from_acpm(acpm_gov_common.sm_base, &state)) {
		spin_lock(&acpm_gov_common.thermal_pressure.lock);
		switched_on = state.switched_on & CPU_TZ_MASK;
		if (cpu_switch_on_status_changed(state) || switched_on) {
			apply_thermal_pressure = true;
		}
		acpm_gov_common.thermal_pressure.state = state;
		spin_unlock(&acpm_gov_common.thermal_pressure.lock);
	}
	thermal_pressure_work(apply_thermal_pressure, state);
}

static get_cpu_power_table_ect_offset_cb get_cpu_power_table_ect_offset;
void register_get_cpu_power_table_ect_offset(get_cpu_power_table_ect_offset_cb cb)
{
	if (WARN_ON(!cb)) {
		pr_err("Failed to register in %s\n", __func__);
		return;
	}

	if (WARN_ON(get_cpu_power_table_ect_offset)) {
		pr_err("get_cpu_power_table_ect_offset function is already set");
		return;
	}

	get_cpu_power_table_ect_offset = cb;
}
EXPORT_SYMBOL(register_get_cpu_power_table_ect_offset);

void update_tj_power_table_ect_offset(struct gs_tmu_data *data)
{
	int ect_offset;

	switch (data->id) {
	case TZ_BIG:
	case TZ_MID:
	case TZ_LIT:
		if (get_cpu_power_table_ect_offset &&
		    !get_cpu_power_table_ect_offset(&data->mapped_cpus, &ect_offset))
			exynos_acpm_tmu_ipc_set_pi_param(data->id, POWER_TABLE_ECT_OFFSET,
							 ect_offset);
		break;
	default:
		break;
	}
}

static void update_time_in_state(struct throttling_stats *stats)
{
	ktime_t now = ktime_get(), delta;

	delta = ktime_sub(now, stats->last_time);
	if (stats->stats_type == DISABLE_STATS) {
		stats->disable_time_in_state[stats->disable_state] =
				ktime_add(stats->disable_time_in_state[stats->disable_state],
					  delta);
	}

	if (stats->stats_type == HARDLIMIT_STATS) {
		stats->hardlimit_time_in_state[stats->hardlimit_state] =
				ktime_add(stats->hardlimit_time_in_state[stats->hardlimit_state],
					  delta);
	}
	stats->last_time = now;
}

static void hard_limit_stats_update(struct throttling_stats *stats,
					int new_state)
{
	if (!stats)
		return;

	spin_lock(&stats->lock);

	if (stats->hardlimit_state == new_state)
		goto unlock;

	update_time_in_state(stats);
	stats->hardlimit_state = new_state;
	if (new_state)
		stats->hardlimit_total_count++;

unlock:
	spin_unlock(&stats->lock);
}

static void disable_stats_update(struct throttling_stats *stats,
				 int new_state)
{
	if (!stats)
		return;

	spin_lock(&stats->lock);

	if (stats->disable_state == new_state)
		goto unlock;

	update_time_in_state(stats);
	stats->disable_state = new_state;
	if (new_state)
		stats->disable_total_count++;

unlock:
	spin_unlock(&stats->lock);
}

static int gs_tmu_tz_config_init(struct platform_device *pdev)
{
	struct thermal_zone_data *tz_config_p;
	u16 cnt;
	enum tmu_grp_idx_t tmu_grp_idx;
	enum tmu_sensor_t probe_id;

	for (tmu_grp_idx = 0; tmu_grp_idx < TZ_END; tmu_grp_idx++) {
		tz_config_p = &tz_config[tmu_grp_idx];
		if (tz_config_p->tmu_zone_id >= TMU_END) {
			dev_err(&pdev->dev,
				"Invalid tmu zone id %d for tz id %d\n",
				tz_config_p->tmu_zone_id, tmu_grp_idx);
			return -EINVAL;
		}

		cnt = 0;
		for (probe_id = 0; probe_id < TMU_SENSOR_PROBE_NUM; probe_id++) {
			if ((1 << probe_id) & tz_config_p->sensors_mask) {
				tz_config_p->sensors[cnt].probe_id = probe_id;
				cnt++;
			}
		}
		tz_config_p->sensor_cnt = cnt;
	}

	return 0;
}

static const char * const trace_suffix[] = {
	[CPU_THROTTLE] = "cpu_throttle",
	[HARD_LIMIT] = "hard_limit",
	[HOTPLUG] = "hotplug",
	[PAUSE] = "pause",
};
#define MAX_TRACE_SUFFIX_STR_LEN (13)

#define update_thermal_trace(pdata, feature, value)                                                \
	do {                                                                                       \
		if (unlikely(trace_clock_set_rate_enabled()))                                      \
			update_thermal_trace_internal(pdata, feature, value);                      \
	} while (0);

static void update_thermal_trace_internal(struct gs_tmu_data *pdata,
					  enum thermal_feature feature, int value)
{
	char clock_name[THERMAL_NAME_LENGTH + MAX_TRACE_SUFFIX_STR_LEN + 1];
	scnprintf(clock_name, (THERMAL_NAME_LENGTH + 1 + strlen(trace_suffix[feature])),
		 "%s_%s", pdata->tmu_name, trace_suffix[feature]);
	trace_clock_set_rate(clock_name, value, raw_smp_processor_id());
}

static bool has_tz_pending_irq(struct gs_tmu_data *pdata)
{
	struct thermal_zone_data *tz_config_p = &tz_config[pdata->id];
	u32 val, counter = 0;
	u16 cnt;
	enum tmu_sensor_t probe_id;
	int i;
	bool ret = false;

	for (cnt = 0; cnt < tz_config_p->sensor_cnt; cnt++) {
		probe_id = tz_config_p->sensors[cnt].probe_id;
		val = readl(pdata->base + TMU_REG_INTPEND(probe_id));
		counter |= val;

		if (val)
			ret = true;
	}

	if (acpm_gov_common.turn_on)
		return ret;

	for (i = 0; i < TRIP_LEVEL_NUM; i++) {
		if (counter & TMU_REG_INTPEND_RISE_MASK(i))
			atomic64_inc(&(pdata->trip_counter[i]));
	}

	return ret;
}

static void gs_report_trigger(struct gs_tmu_data *p)
{
	struct thermal_zone_device *tz = p->tzd;

	if (!tz) {
		pr_err("No thermal zone device defined\n");
		return;
	}

	thermal_zone_device_update(tz, THERMAL_EVENT_UNSPECIFIED);
}

static int gs_tmu_initialize(struct platform_device *pdev)
{
	struct gs_tmu_data *data = platform_get_drvdata(pdev);
	struct thermal_zone_device *tz = data->tzd;
	enum thermal_trip_type type;
	int i, temp, ret = 0;
	unsigned char threshold[8] = {0, };
	unsigned char hysteresis[8] = {0, };
	unsigned char inten = 0;

	mutex_lock(&data->lock);

	for (i = (of_thermal_get_ntrips(tz) - 1); i >= 0; i--) {
		ret = tz->ops->get_trip_type(tz, i, &type);
		if (ret) {
			dev_err(&pdev->dev, "Failed to get trip type(%d)\n", i);
			goto out;
		}

		ret = tz->ops->get_trip_temp(tz, i, &temp);
		if (ret) {
			dev_err(&pdev->dev, "Failed to get trip temp(%d)\n", i);
			goto out;
		}

		threshold[i] = (unsigned char)(temp / MCELSIUS);
		if (type == THERMAL_TRIP_PASSIVE)
			continue;
		inten |= (1 << i);

		ret = tz->ops->get_trip_hyst(tz, i, &temp);
		if (ret) {
			dev_err(&pdev->dev, "Failed to get trip hyst(%d)\n", i);
			goto out;
		}

		hysteresis[i] = (unsigned char)(temp / MCELSIUS);
	}

	ret = gs_tmu_tz_config_init(pdev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to initialize tmu tz config\n");
		goto out;
	}

	exynos_acpm_tmu_set_threshold(data->id, threshold);
	exynos_acpm_tmu_set_hysteresis(data->id, hysteresis);
	exynos_acpm_tmu_set_interrupt_enable(data->id, inten);

out:
	mutex_unlock(&data->lock);

	return ret;
}

static void gs_tmu_control(struct platform_device *pdev, bool on)
{
	struct gs_tmu_data *data = platform_get_drvdata(pdev);

	mutex_lock(&data->lock);
	exynos_acpm_tmu_tz_control(data->id, on);
	data->enabled = on;
	mutex_unlock(&data->lock);
}

#define MCINFO_LOG_THRESHOLD	(4)

static int gs_get_temp(void *p, int *temp)
{
	struct gs_tmu_data *data = p;
#if IS_ENABLED(CONFIG_EXYNOS_MCINFO)
	unsigned int mcinfo_count;
	unsigned int mcinfo_result[4] = {0, 0, 0, 0};
	unsigned int mcinfo_logging = 0;
	unsigned int mcinfo_temp = 0;
	unsigned int i;
#endif
	int acpm_temp = 0, stat = 0;

	if (!data || !data->enabled)
		return -EINVAL;

	mutex_lock(&data->lock);

	exynos_acpm_tmu_set_read_temp(data->id, &acpm_temp, &stat);

	*temp = acpm_temp * MCELSIUS;

	if (data->limited_frequency) {
		if (!data->limited) {
			if (*temp >= data->limited_threshold) {
				exynos_pm_qos_update_request(&data->thermal_limit_request,
							     data->limited_frequency);
				data->limited = true;
			}
		} else {
			if (*temp < data->limited_threshold_release) {
				exynos_pm_qos_update_request(&data->thermal_limit_request,
							     INT_MAX);
				data->limited = false;
			}
		}
	}

	data->temperature = *temp / 1000;

	if (data->hotplug_enable &&
		((data->is_cpu_hotplugged_out &&
		  data->temperature < data->hotplug_in_threshold) ||
		 (!data->is_cpu_hotplugged_out &&
		  data->temperature >= data->hotplug_out_threshold)))
		kthread_queue_work(&data->pause_worker, &data->hotplug_work);

	if (data->cpu_hw_throttling_enable &&
	    ((data->is_cpu_hw_throttled &&
	      data->temperature < data->cpu_hw_throttling_clr_threshold) ||
	     (!data->is_cpu_hw_throttled &&
	      data->temperature >= data->cpu_hw_throttling_trigger_threshold)))
		kthread_queue_work(&data->cpu_hw_throttle_worker, &data->cpu_hw_throttle_work);

	if (data->pause_enable &&
	    ((data->is_paused &&
	      data->temperature < data->resume_threshold) ||
	     (!data->is_paused &&
	      data->temperature >= data->pause_threshold)))
		kthread_queue_work(&data->pause_worker, &data->pause_work);

	if (data->hardlimit_enable &&
	    ((data->is_hardlimited &&
	      data->temperature < data->hardlimit_clr_threshold) ||
	     (!data->is_hardlimited &&
	      data->temperature >= data->hardlimit_threshold)))
		kthread_queue_work(&data->hardlimit_worker, &data->hardlimit_work);

	mutex_unlock(&data->lock);

#if IS_ENABLED(CONFIG_EXYNOS_MCINFO)
	if (data->id == 0) {
		mcinfo_count = get_mcinfo_base_count();
		get_refresh_rate(mcinfo_result);

		for (i = 0; i < mcinfo_count; i++) {
			mcinfo_temp |= (mcinfo_result[i] & 0xf) << (8 * i);

			if (mcinfo_result[i] >= MCINFO_LOG_THRESHOLD)
				mcinfo_logging = 1;
		}

		if (mcinfo_logging == 1)
			dbg_snapshot_thermal(NULL, mcinfo_temp, "MCINFO", 0);
	}
#endif
	return 0;
}

static int gs_get_trend(void *p, int trip, enum thermal_trend *trend)
{
	struct gs_tmu_data *data = p;
	struct thermal_zone_device *tz = data->tzd;
	int trip_temp, ret = 0;

	if (!tz)
		return ret;

	ret = tz->ops->get_trip_temp(tz, trip, &trip_temp);
	if (ret < 0)
		return ret;

	if (data->use_pi_thermal) {
		*trend = THERMAL_TREND_STABLE;
	} else {
		if (tz->temperature >= trip_temp)
			*trend = THERMAL_TREND_RAISE_FULL;
		else
			*trend = THERMAL_TREND_DROP_FULL;
	}

	return 0;
}

static int gs_tmu_set_trip_temp(void *drv_data, int trip, int temp)
{
	struct gs_tmu_data *data = drv_data;
	struct thermal_zone_device *tz = data->tzd;
	enum thermal_trip_type type;
	int i, trip_temp, ret = 0;
	unsigned char threshold[8] = {0, };

	ret = tz->ops->get_trip_type(tz, trip, &type);
	if (ret) {
		pr_err("Failed to get trip type(%d)\n", trip);
		return ret;
	}

	for (i = (of_thermal_get_ntrips(tz) - 1); i >= 0; i--) {
		ret = tz->ops->get_trip_type(tz, i, &type);
		if (ret) {
			pr_err("Failed to get trip type(%d)\n", i);
			return ret;
		}


		ret = tz->ops->get_trip_temp(tz, i, &trip_temp);
		if (ret) {
			pr_err("Failed to get trip temp(%d)\n", i);
			return ret;
		}
		if (i == trip)
			threshold[trip] = (unsigned char)(temp / MCELSIUS);
		else
			threshold[i] = (unsigned char)(trip_temp / MCELSIUS);
	}
	mutex_lock(&data->lock);
	if (data->enabled) {
		exynos_acpm_tmu_tz_control(data->id, false);
		exynos_acpm_tmu_set_threshold(data->id, threshold);
		exynos_acpm_tmu_tz_control(data->id, true);
	} else {
		exynos_acpm_tmu_set_threshold(data->id, threshold);
	}
	mutex_unlock(&data->lock);

	return ret;
}

#if IS_ENABLED(CONFIG_THERMAL_EMULATION)
static int gs_tmu_set_emulation(void *drv_data, int temp)
{
	struct gs_tmu_data *data = drv_data;
	int ret = -EINVAL;
	unsigned char emul_temp;

	if (temp && temp < MCELSIUS)
		goto out;

	mutex_lock(&data->lock);
	emul_temp = (unsigned char)(temp / MCELSIUS);
	exynos_acpm_tmu_set_emul_temp(data->id, emul_temp);
	mutex_unlock(&data->lock);
	return 0;
out:
	return ret;
}
#else
static int gs_tmu_set_emulation(void *drv_data, int temp)
{
	return -EINVAL;
}
#endif /* CONFIG_THERMAL_EMULATION */

static void start_pi_polling(struct gs_tmu_data *data, int delay)
{
	kthread_mod_delayed_work(&data->thermal_worker, &data->pi_work,
				 msecs_to_jiffies(delay));
}

static void get_control_trips(struct gs_tmu_data *data)
{
	struct thermal_zone_device *tz = data->tzd;
	int i, last_active, last_passive;
	bool found_first_passive;

	found_first_passive = false;
	last_active = INVALID_TRIP;
	last_passive = INVALID_TRIP;

	for (i = 0; i < tz->num_trips; i++) {
		enum thermal_trip_type type;
		int ret;

		ret = tz->ops->get_trip_type(tz, i, &type);
		if (ret) {
			dev_warn(&tz->device,
				 "Failed to get trip point %d type: %d\n", i,
				 ret);
			continue;
		}

		if (type == THERMAL_TRIP_PASSIVE) {
			if (!found_first_passive) {
				data->trip_switch_on = i;
				found_first_passive = true;
				break;
			}

			last_passive = i;
		} else if (type == THERMAL_TRIP_ACTIVE) {
			last_active = i;
		} else {
			break;
		}
	}

	if (last_passive != INVALID_TRIP) {
		data->trip_control_temp = last_passive;
	} else if (found_first_passive) {
		data->trip_control_temp = data->trip_switch_on;
		data->trip_switch_on = last_active;
	} else {
		data->trip_switch_on = INVALID_TRIP;
		data->trip_control_temp = last_active;
	}
}

static void reset_pi_params(struct gs_tmu_data *data)
{
	s64 i = int_to_frac(data->pi_param->i_max);

	data->pi_param->err_integral = div_frac(i, data->pi_param->k_i);
}

static void allow_maximum_power(struct gs_tmu_data *data)
{
	struct thermal_instance *instance;
	struct thermal_zone_device *tz = data->tzd;
	int control_temp = data->trip_control_temp;

	mutex_unlock(&data->lock);
	mutex_lock(&tz->lock);
	list_for_each_entry(instance, &tz->thermal_instances, tz_node) {
		if (instance->trip != control_temp ||
		    (!cdev_is_power_actor(instance->cdev)))
			continue;
		if (data->hardlimit_enable && data->is_hardlimited)
			instance->target = data->max_cdev;
		else
			instance->target = 0;

		data->max_cdev = instance->target;
		mutex_lock(&instance->cdev->lock);
		instance->cdev->updated = false;
		mutex_unlock(&instance->cdev->lock);
		trace_thermal_exynos_allow_max_power(data->tmu_name, data->is_hardlimited,
						     instance->cdev->type, instance->target);
		thermal_cdev_update(instance->cdev);
	}

	mutex_unlock(&tz->lock);
	mutex_lock(&data->lock);
}

static u32 pi_calculate(struct gs_tmu_data *data, int control_temp,
			u32 max_allocatable_power)
{
	struct thermal_zone_device *tz = data->tzd;
	struct gs_pi_param *params = data->pi_param;
	s64 p, i, power_range;
	s32 err, max_power_frac;

	max_power_frac = int_to_frac(max_allocatable_power);

	err = (control_temp - tz->temperature) / 1000;
	err = int_to_frac(err);

	/* Calculate the proportional term */
	p = mul_frac(err < 0 ? params->k_po : params->k_pu, err);

	/*
	 * Calculate the integral term
	 *
	 * if the error is less than cut off allow integration (but
	 * the integral is limited to max power)
	 */
	i = mul_frac(params->k_i, params->err_integral);

	if (err < int_to_frac(params->integral_cutoff)) {
		s64 i_next = i + mul_frac(params->k_i, err);
		s64 i_windup = int_to_frac(-1 * (s64)params->sustainable_power);

		if (i_next > int_to_frac((s64)params->i_max)) {
			i = int_to_frac((s64)params->i_max);
			params->err_integral = div_frac(i, params->k_i);
		} else if (i_next <= i_windup) {
			i = i_windup;
			params->err_integral = div_frac(i, params->k_i);
		} else {
			i = i_next;
			params->err_integral += err;
		}
	}

	power_range = p + i;

	power_range = params->sustainable_power + frac_to_int(power_range);

	power_range = clamp(power_range, (s64)0, (s64)max_allocatable_power);

	trace_thermal_exynos_power_allocator_pid(tz, frac_to_int(err),
						 frac_to_int(params->err_integral),
						 frac_to_int(p), frac_to_int(i),
						 power_range);

	return power_range;
}

static int gs_pi_controller(struct gs_tmu_data *data, int control_temp)
{
	struct thermal_zone_device *tz = data->tzd;
	struct thermal_instance *instance;
	struct thermal_cooling_device *cdev;
	int ret = 0;
	bool found_actor = false;
	u32 max_power, power_range;
	unsigned long state;

	// TODO: refactor locking
	mutex_unlock(&data->lock);
	mutex_lock(&tz->lock);
	list_for_each_entry(instance, &tz->thermal_instances, tz_node) {
		if (instance->trip == data->trip_control_temp &&
		    cdev_is_power_actor(instance->cdev)) {
			found_actor = true;
			cdev = instance->cdev;
			break;
		}
	}
	mutex_unlock(&tz->lock);
	mutex_lock(&data->lock);

	if (!found_actor)
		return -ENODEV;

	cdev->ops->state2power(cdev, 0, &max_power);

	power_range = pi_calculate(data, control_temp, max_power);

	ret = cdev->ops->power2state(cdev, power_range, &state);
	if (ret)
		return ret;

	if (data->hardlimit_enable && data->is_hardlimited)
		state = max(state, data->max_cdev);

	// TODO: refactor locking
	mutex_unlock(&data->lock);
	mutex_lock(&tz->lock);
	instance->target = state;
	mutex_lock(&cdev->lock);
	cdev->updated = false;
	mutex_unlock(&cdev->lock);
	thermal_cdev_update(cdev);
	mutex_unlock(&tz->lock);
	mutex_lock(&data->lock);
	data->max_cdev = state;

	trace_thermal_exynos_power_allocator(tz, power_range, max_power, tz->temperature,
					     control_temp - tz->temperature, state,
					     data->is_hardlimited);
	return ret;
}

static void gs_pi_thermal(struct gs_tmu_data *data)
{
	struct thermal_zone_device *tz = data->tzd;
	struct gs_pi_param *params = data->pi_param;
	int ret = 0;
	int switch_on_temp, control_temp, delay;

	if (atomic_read(&gs_tmu_in_suspend))
		return;

	if (tz) {
		if (READ_ONCE(tz->mode) == THERMAL_DEVICE_DISABLED) {
			mutex_lock(&data->lock);
			reset_pi_params(data);
			allow_maximum_power(data);
			params->switched_on = false;
			goto polling;
		}
	}

	thermal_zone_device_update(tz, THERMAL_EVENT_UNSPECIFIED);

	mutex_lock(&data->lock);

	ret = tz->ops->get_trip_temp(tz, data->trip_switch_on,
				     &switch_on_temp);
	if (!ret && tz->temperature < switch_on_temp) {
		reset_pi_params(data);
		allow_maximum_power(data);
		params->switched_on = false;
		goto polling;
	}

	params->switched_on = true;

	ret = tz->ops->get_trip_temp(tz, data->trip_control_temp,
				     &control_temp);
	if (ret) {
		pr_warn("Failed to get the maximum desired temperature: %d\n",
			ret);
		goto polling;
	}

	ret = gs_pi_controller(data, control_temp);

	if (ret) {
		pr_debug("Failed to calculate pi controller: %d\n",
			 ret);
		goto polling;
	}

polling:
	if (params->switched_on)
		delay = data->polling_delay_on;
	else
		delay = data->polling_delay_off;

	if (delay)
		start_pi_polling(data, delay);

	mutex_unlock(&data->lock);
}

static void gs_pi_polling(struct kthread_work *work)
{
	struct gs_tmu_data *data =
			container_of(work, struct gs_tmu_data, pi_work.work);

	gs_pi_thermal(data);
}

static void gs_tmu_work(struct kthread_work *work)
{
	struct gs_tmu_data *data = container_of(work,
			struct gs_tmu_data, irq_work);
	struct thermal_zone_device *tz = data->tzd;

	gs_report_trigger(data);
	mutex_lock(&data->lock);

	exynos_acpm_tmu_clear_tz_irq(data->id);

	dev_dbg_ratelimited(&tz->device, "IRQ handled: tz:%s, temp:%d\n",
			    tz->type, tz->temperature);

	mutex_unlock(&data->lock);

	if (data->use_pi_thermal)
			gs_pi_thermal(data);

	enable_irq(data->irq);
}

static irqreturn_t gs_tmu_irq(int irq, void *id)
{
	struct gs_tmu_data *data = id;

	disable_irq_nosync(irq);
	if (has_tz_pending_irq(data)) {
		kthread_queue_work(&data->thermal_worker, &data->irq_work);
		goto irq_handler_exit;
	}
	enable_irq(irq);

irq_handler_exit:
	return IRQ_HANDLED;
}

static void init_bcl_dev(struct kthread_work *work)
{
	struct gs_tmu_data *data = container_of(work,
						   struct gs_tmu_data,
						   cpu_hw_throttle_init_work.work);
	int ret = 0;

	mutex_lock(&data->lock);
	data->bcl_dev = google_retrieve_bcl_handle();

	if (!data->bcl_dev) {
		pr_warn_ratelimited("%s: failed to retrieve bcl_dev. Retry.\n", data->tmu_name);
		kthread_mod_delayed_work(&data->cpu_hw_throttle_worker,
					 &data->cpu_hw_throttle_init_work,
					 msecs_to_jiffies(500));
		goto init_exit;
	}

	if (!data->ppm_clr_throttle_level)
		data->ppm_clr_throttle_level = google_get_ppm(data->bcl_dev);

	if (!data->mpmm_clr_throttle_level)
		data->mpmm_clr_throttle_level = google_get_mpmm(data->bcl_dev, LITTLE);
	if (!data->mpmm_clr_throttle_level)
		data->mpmm_clr_throttle_level = google_get_mpmm(data->bcl_dev, MID);
	if (!data->mpmm_clr_throttle_level)
		data->mpmm_clr_throttle_level = google_get_mpmm(data->bcl_dev, BIG);

	if (data->ppm_clr_throttle_level < 0)
		ret = data->ppm_clr_throttle_level;

	if (data->mpmm_clr_throttle_level < 0)
		ret = data->mpmm_clr_throttle_level;

	if (ret < 0) {
		pr_err_ratelimited("%s: failed to get ppm(%#x)/mpmm(%#x) setting, ret = %d\n",
				   data->tmu_name,
				   data->ppm_clr_throttle_level,
				   data->mpmm_clr_throttle_level, ret);
		goto init_exit;
	}

	pr_info("%s: parsing default setting ppm: %#x, mpmm: %#x\n", data->tmu_name,
		data->ppm_clr_throttle_level, data->mpmm_clr_throttle_level);

init_exit:
	mutex_unlock(&data->lock);
}

static void gs_throttle_arm(struct kthread_work *work)
{
	struct gs_tmu_data *data = container_of(work,
						   struct gs_tmu_data, cpu_hw_throttle_work);

	int ret = 0;

	if (!data->bcl_dev) {
		pr_err_ratelimited("Failed to retrieve bcl_dev, ppm/mpmm throttling failed\n");
		return;
	}

	mutex_lock(&data->lock);

	if (data->is_cpu_hw_throttled) {
		if (data->temperature < data->cpu_hw_throttling_clr_threshold) {
			pr_info_ratelimited("ppm/mpmm thermal throttling disable!\n");

			ret = google_set_ppm(data->bcl_dev, data->ppm_clr_throttle_level);
			if (ret) {
				pr_err_ratelimited("Failed to clr ppm throttle to %#x, ret = %d",
						   data->ppm_clr_throttle_level, ret);
				goto unlock;
			}
			pr_info_ratelimited("Set ppm throttle to %#x\n",
					    data->ppm_clr_throttle_level);

			ret = google_set_mpmm(data->bcl_dev, data->mpmm_clr_throttle_level,
					      LITTLE);
			if (ret) {
				pr_err_ratelimited("Failed to clr LITTLE mpmm throttle to %#x, ret = %d",
						   data->mpmm_clr_throttle_level, ret);
				goto unlock;
			}
			pr_info_ratelimited("Set LITTLE mpmm throttle to %#x\n",
					    data->mpmm_clr_throttle_level);

			ret = google_set_mpmm(data->bcl_dev, data->mpmm_clr_throttle_level,
					      MID);
			if (ret) {
				pr_err_ratelimited("Failed to clr MID mpmm throttle to %#x, ret = %d",
						   data->mpmm_clr_throttle_level, ret);
				goto unlock;
			}
			pr_info_ratelimited("Set MID mpmm throttle to %#x\n",
					    data->mpmm_clr_throttle_level);

			ret = google_set_mpmm(data->bcl_dev, data->mpmm_clr_throttle_level,
					      BIG);
			if (ret) {
				pr_err_ratelimited("Failed to clr BIG mpmm throttle to %#x, ret = %d",
						   data->mpmm_clr_throttle_level, ret);
				goto unlock;
			}
			pr_info_ratelimited("Set BIG mpmm throttle to %#x\n",
					    data->mpmm_clr_throttle_level);

			data->is_cpu_hw_throttled = false;
		}
	} else {
		if (data->temperature >= data->cpu_hw_throttling_trigger_threshold) {
			pr_info_ratelimited("ppm/mpmm thermal throttling enable!\n");

			ret = google_set_ppm(data->bcl_dev, data->ppm_throttle_level);
			if (ret) {
				pr_err_ratelimited("Failed to set ppm throttle to %#x, ret = %d",
						   data->ppm_throttle_level, ret);
				goto unlock;
			}
			pr_info_ratelimited("Set ppm throttle to %#x\n",
					    data->ppm_throttle_level);

			ret = google_set_mpmm(data->bcl_dev, data->mpmm_throttle_level,
			                      LITTLE);
			if (ret) {
				pr_err_ratelimited("Failed to set LITTLE mpmm to %#x, ret = %d",
						   data->mpmm_throttle_level, ret);
				goto unlock;
			}
			pr_info_ratelimited("Set LITTLE mpmm throttle to %#x\n",
					    data->mpmm_throttle_level);

			ret = google_set_mpmm(data->bcl_dev, data->mpmm_throttle_level,
			                      MID);
			if (ret) {
				pr_err_ratelimited("Failed to set MID mpmm to %#x, ret = %d",
						   data->mpmm_throttle_level, ret);
				goto unlock;
			}
			pr_info_ratelimited("Set MID mpmm throttle to %#x\n",
					    data->mpmm_throttle_level);

			ret = google_set_mpmm(data->bcl_dev, data->mpmm_throttle_level,
			                      BIG);
			if (ret) {
				pr_err_ratelimited("Failed to set BIG mpmm to %#x, ret = %d",
						   data->mpmm_throttle_level, ret);
				goto unlock;
			}
			pr_info_ratelimited("Set BIG mpmm throttle to %#x\n",
					    data->mpmm_throttle_level);

			data->is_cpu_hw_throttled = true;
		}
	}
	update_thermal_trace(data, CPU_THROTTLE, data->is_cpu_hw_throttled);

unlock:
	mutex_unlock(&data->lock);
}

static void gs_throttle_cpu_hotplug(struct kthread_work *work)
{
	struct gs_tmu_data *data = container_of(work,
						   struct gs_tmu_data, hotplug_work);
	struct cpumask mask;

	mutex_lock(&data->lock);

	if (data->is_cpu_hotplugged_out) {
		if (data->temperature < data->hotplug_in_threshold) {
			/*
			 * If current temperature is lower than low threshold,
			 * call exynos_cpuhp_request for hotplugged out cpus.
			 */
			if (exynos_cpuhp_request(data->cpuhp_name, *cpu_possible_mask)) {
				// queue the work again in case failure
				// also do not queue again when prepare to suspend
				if (!atomic_read(&gs_tmu_in_suspend))
					kthread_queue_work(&data->pause_worker,
							   &data->hotplug_work);
			} else {
				data->is_cpu_hotplugged_out = false;
			}
		}
	} else {
		if (data->temperature >= data->hotplug_out_threshold) {
			/*
			 * If current temperature is higher than high threshold,
			 * call exynos_cpuhp_request to hold temperature down.
			 */
			cpumask_andnot(&mask, cpu_possible_mask, &data->hotplug_cpus);
			if (exynos_cpuhp_request(data->cpuhp_name, mask)) {
				// queue the work again in case of failure
				// also do not queue again when prepare to suspend
				if (!atomic_read(&gs_tmu_in_suspend))
					kthread_queue_work(&data->pause_worker,
							   &data->hotplug_work);
			} else {
				data->is_cpu_hotplugged_out = true;
			}
		}
	}
	update_thermal_trace(data, HOTPLUG, data->is_cpu_hotplugged_out);
	disable_stats_update(data->disable_stats, data->is_cpu_hotplugged_out);

	mutex_unlock(&data->lock);
}

static tpu_pause_cb tpu_thermal_pause_cb;
static void *tpu_data;
void register_tpu_thermal_pause_cb(tpu_pause_cb tpu_cb, void *data)
{
	if (!tpu_cb || !data) {
		pr_err("Failed to register tpu_thermal_pause_cb\n");
		return;
	}

	tpu_data = data;
	/* Ensure tpu_data is assigned and has been committed. */
	smp_wmb();
	tpu_thermal_pause_cb = tpu_cb;
}
EXPORT_SYMBOL(register_tpu_thermal_pause_cb);

/*
static void gs_throttle_pause(struct kthread_work *work)
{
	struct gs_tmu_data *data = container_of(work,
						   struct gs_tmu_data, pause_work);
	struct cpumask mask;
	int cpu_pause_mask, ret;

	mutex_lock(&data->lock);

	switch (data->tmu_type) {
	case TMU_TYPE_CPU:
		cpu_pause_mask = cpumask_and(&mask, cpu_possible_mask, &data->pause_cpus);
		break;
	case TMU_TYPE_TPU:
		// tpu_data has been assigned and ensure it's not NULL.
		// So, just need to ensure tpu_thermal_pause_cb is not NULL.
		// This is the pair of read side register_tpu_thermal_pause_cb().
		if (!tpu_thermal_pause_cb) {
			pr_err_ratelimited("%s: TPU pause callback not registered\n",
					   data->tmu_name);
			goto unlock;
		}
		smp_rmb();
		break;
	case TMU_TYPE_GPU:
	case TMU_TYPE_ISP:
	default:
		pr_warn_ratelimited("%s: %u unsupported type for pause function\n",
				    data->tmu_name, data->tmu_type);
		goto unlock;
	}

	if (data->is_paused) {
		if (data->temperature < data->resume_threshold) {
			if (data->tmu_type == TMU_TYPE_CPU) {
				if (resume_cpus(&mask)) {
					// queue the work again in case failure
					// also do not queue again when prepare to suspend
					if (!atomic_read(&gs_tmu_in_suspend))
						kthread_queue_work(&data->pause_worker,
								   &data->pause_work);
				} else {
					data->is_paused = false;
					pr_info_ratelimited("%s thermal resume cpus: %*pbl\n",
						data->tmu_name, cpumask_pr_args(&mask));
				}
			} else if (data->tmu_type == TMU_TYPE_TPU) {
				ret = tpu_thermal_pause_cb(THERMAL_RESUME, tpu_data);
				if (!ret) {
					data->is_paused = false;
					pr_info_ratelimited("%s thermal resumed\n", data->tmu_name);
				} else {
					if (!atomic_read(&gs_tmu_in_suspend))
						kthread_queue_work(&data->pause_worker,
								   &data->pause_work);
				}
			}
		}
	} else {
		if (data->temperature >= data->pause_threshold) {
			if (data->tmu_type == TMU_TYPE_CPU) {
				if (pause_cpus(&mask)) {
					// queue the work again in case of failure
					// also do not queue again when prepare to suspend
					if (!atomic_read(&gs_tmu_in_suspend))
						kthread_queue_work(&data->pause_worker,
								   &data->pause_work);
				} else {
					data->is_paused = true;
					pr_info_ratelimited("%s thermal pause cpus: %*pbl\n",
						data->tmu_name, cpumask_pr_args(&mask));
				}
			} else if (data->tmu_type == TMU_TYPE_TPU) {
				ret = tpu_thermal_pause_cb(THERMAL_SUSPEND, tpu_data);
				if (!ret) {
					data->is_paused = true;
					pr_info_ratelimited("%s thermal paused\n", data->tmu_name);
				} else {
					if (!atomic_read(&gs_tmu_in_suspend))
						kthread_queue_work(&data->pause_worker,
								   &data->pause_work);
				}
			}
		}
	}

	update_thermal_trace(data, PAUSE, data->is_paused);
	disable_stats_update(data->disable_stats, data->is_paused);

unlock:
	mutex_unlock(&data->lock);
}
*/

static void gs_throttle_hard_limit(struct kthread_work *work)
{
	struct gs_tmu_data *data = container_of(work,
						   struct gs_tmu_data, hardlimit_work);
	struct thermal_zone_device *tz = data->tzd;
	struct thermal_instance *instance;
	struct thermal_cooling_device *cdev = NULL;
	unsigned long state, max_state, prev_max_state;

	mutex_lock(&tz->lock);
	list_for_each_entry(instance, &tz->thermal_instances, tz_node) {
		if (!strncmp(data->tmu_name, instance->tz->type, THERMAL_NAME_LENGTH)) {
			cdev = instance->cdev;
			break;
		}
	}
	mutex_unlock(&tz->lock);

	if (!cdev) {
		pr_err_ratelimited("%s: cannot find cdev, hard limit throttling failed\n",
				   data->tmu_name);
		return;
	}

	mutex_lock(&data->lock);

	if (data->is_hardlimited) {
		if (data->temperature < data->hardlimit_clr_threshold) {
			if (data->use_pi_thermal && data->pi_param->switched_on) {
				state = data->max_cdev;
			} else {
				state = 0;
			}
			prev_max_state = data->max_cdev;
			data->max_cdev = state;
			mutex_unlock(&data->lock);

			// TODO: refactor locking
			mutex_lock(&tz->lock);
			mutex_lock(&cdev->lock);
			cdev->updated = false;
			mutex_unlock(&cdev->lock);
			instance->target = state;
			thermal_cdev_update(cdev);
			mutex_unlock(&tz->lock);

			mutex_lock(&data->lock);
			data->is_hardlimited = false;
			pr_info_ratelimited("%s: clear hard limit, is_hardlimited = %d, pid swithed_on = %d\n",
					    data->tmu_name, data->is_hardlimited,
					    data->pi_param->switched_on);
		}
	} else {
		if (data->temperature >= data->hardlimit_threshold) {
			pr_info_ratelimited("%s: enable hard limit\n", data->tmu_name);
			if (data->hardlimit_cooling_state == THERMAL_NO_LIMIT) {
				if (cdev->ops->get_max_state(cdev, &max_state)) {
					pr_err_ratelimited("%s: %s failed to get_max_cdev, hard limit throttling failed\n",
							   cdev->type, data->tmu_name);
					goto err_exit;
				}
				data->hardlimit_cooling_state = (unsigned int)max_state;
			}

			state = max((unsigned long)data->hardlimit_cooling_state,
					       data->max_cdev);
			prev_max_state = data->max_cdev;
			data->max_cdev = state;
			mutex_unlock(&data->lock);

			// TODO: refactor locking
			mutex_lock(&tz->lock);
			mutex_lock(&cdev->lock);
			cdev->updated = false;
			mutex_unlock(&cdev->lock);
			instance->target = state;
			thermal_cdev_update(cdev);
			mutex_unlock(&tz->lock);

			mutex_lock(&data->lock);
			data->is_hardlimited = true;
			pr_info_ratelimited("%s: %s set cur_state to hardlimit cooling state %d, is_hardlimited = %d, pid swithed_on = %d\n",
					    data->tmu_name, cdev->type,
					    data->hardlimit_cooling_state, data->is_hardlimited,
					    data->pi_param->switched_on);
		}
	}
	update_thermal_trace(data, HARD_LIMIT, data->is_hardlimited);
	hard_limit_stats_update(data->hardlimit_stats, data->is_hardlimited);

err_exit:
	mutex_unlock(&data->lock);
}

static int gs_tmu_pm_notify(struct notifier_block *nb,
			       unsigned long mode, void *_unused)
{
	struct gs_tmu_data *data;

	switch (mode) {
	case PM_HIBERNATION_PREPARE:
	case PM_RESTORE_PREPARE:
	case PM_SUSPEND_PREPARE:
		atomic_set(&gs_tmu_in_suspend, 1);
		if (!acpm_gov_common.turn_on)
			list_for_each_entry (data, &dtm_dev_list, node) {
				if (data->use_pi_thermal)
					kthread_cancel_delayed_work_sync(&data->pi_work);
			}
		else if (acpm_gov_common.thermal_pressure.enabled)
			kthread_cancel_delayed_work_sync(
				&acpm_gov_common.thermal_pressure.polling_work);
		break;
	case PM_POST_HIBERNATION:
	case PM_POST_RESTORE:
	case PM_POST_SUSPEND:
		atomic_set(&gs_tmu_in_suspend, 0);
		if (!acpm_gov_common.turn_on)
			list_for_each_entry (data, &dtm_dev_list, node) {
				if (data->use_pi_thermal)
					start_pi_polling(data, 0);
			}
		else if (acpm_gov_common.thermal_pressure.enabled)
			start_thermal_pressure_polling(0);
		break;
	default:
		break;
	}
	return 0;
}

static struct notifier_block gs_tmu_pm_nb = {
	.notifier_call = gs_tmu_pm_notify,
};

static const struct of_device_id gs_tmu_match[] = {
	{ .compatible = "samsung,gs101-tmu-v2", },
	{ .compatible = "samsung,gs201-tmu-v2", },
	{ .compatible = "samsung,gs-tmu-v3", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, gs_tmu_match);

static int gs_tmu_irq_work_init(struct platform_device *pdev)
{
	struct gs_tmu_data *data = platform_get_drvdata(pdev);
	struct cpumask mask;
	struct sched_param param = { .sched_priority = MAX_RT_PRIO / 4 - 1 };
	struct task_struct *thread;
	int ret = 0;
	char kworker_name[CPUHP_USER_NAME_LEN + 1];

	kthread_init_worker(&data->thermal_worker);
	thread = kthread_create(kthread_worker_fn, &data->thermal_worker,
				"thermal_%s", data->tmu_name);
	if (IS_ERR(thread)) {
		dev_err(&pdev->dev, "failed to create thermal thread: %ld\n",
			PTR_ERR(thread));
		return PTR_ERR(thread);
	}

	cpumask_and(&mask, cpu_possible_mask, &data->tmu_work_affinity);
	set_cpus_allowed_ptr(thread, &mask);

	ret = sched_setscheduler_nocheck(thread, SCHED_FIFO, &param);
	if (ret) {
		kthread_stop(thread);
		dev_warn(&pdev->dev, "thermal failed to set SCHED_FIFO\n");
		return ret;
	}

	kthread_init_work(&data->irq_work, gs_tmu_work);

	wake_up_process(thread);

	if (data->hotplug_enable || data->pause_enable) {
		if (data->hotplug_enable) {
			scnprintf(data->cpuhp_name, CPUHP_USER_NAME_LEN, "DTM_%s", data->tmu_name);
			exynos_cpuhp_register(data->cpuhp_name, *cpu_online_mask);
			kthread_init_work(&data->hotplug_work, gs_throttle_cpu_hotplug);
		}

		/* TODO: b/199473897 fix this in android-mainline (commit c7f998a0cd01 "Revert
				"ANDROID: cpu/hotplug: add pause/resume_cpus interface")
		if (data->pause_enable) {
			kthread_init_work(&data->pause_work, gs_throttle_pause);
		}
		*/

		kthread_init_worker(&data->pause_worker);
		scnprintf(kworker_name, CPUHP_USER_NAME_LEN, "%s_pause", data->tmu_name);
		thread = kthread_create(kthread_worker_fn,
					&data->pause_worker, kworker_name);

		cpumask_and(&mask, cpu_possible_mask, &data->hotplug_work_affinity);
		set_cpus_allowed_ptr(thread, &mask);

		ret = sched_setscheduler_nocheck(thread, SCHED_FIFO, &param);
		if (ret) {
			kthread_stop(thread);
			dev_warn(&pdev->dev, "thermal cpu disable failed to set SCHED_FIFO\n");
			return ret;
		}
		wake_up_process(thread);
	}

	if (data->cpu_hw_throttling_enable) {
		kthread_init_work(&data->cpu_hw_throttle_work, gs_throttle_arm);
		kthread_init_worker(&data->cpu_hw_throttle_worker);

		scnprintf(kworker_name, CPUHP_USER_NAME_LEN, "%s_hw_throttle", data->tmu_name);
		thread = kthread_create(kthread_worker_fn, &data->cpu_hw_throttle_worker,
					kworker_name);

		cpumask_and(&mask, cpu_possible_mask, &data->tmu_work_affinity);
		set_cpus_allowed_ptr(thread, &mask);

		ret = sched_setscheduler_nocheck(thread, SCHED_FIFO, &param);
		if (ret) {
			kthread_stop(thread);
			dev_warn(&pdev->dev, "cpu_hw_throttling failed to set SCHED_FIFO\n");
			return ret;
		}
		wake_up_process(thread);

		kthread_init_delayed_work(&data->cpu_hw_throttle_init_work, init_bcl_dev);
		kthread_mod_delayed_work(&data->cpu_hw_throttle_worker,
					 &data->cpu_hw_throttle_init_work,
					 msecs_to_jiffies(0));
	}

	if (data->hardlimit_enable) {
		kthread_init_work(&data->hardlimit_work, gs_throttle_hard_limit);
		kthread_init_worker(&data->hardlimit_worker);

		scnprintf(kworker_name, CPUHP_USER_NAME_LEN, "%s_hardlimit", data->tmu_name);
		thread = kthread_create(kthread_worker_fn, &data->hardlimit_worker, kworker_name);

		cpumask_and(&mask, cpu_possible_mask, &data->tmu_work_affinity);
		set_cpus_allowed_ptr(thread, &mask);

		ret = sched_setscheduler_nocheck(thread, SCHED_FIFO, &param);
		if (ret) {
			kthread_stop(thread);
			dev_warn(&pdev->dev, "hardlimit failed to set SCHED_FIFO\n");
			return ret;
		}
		wake_up_process(thread);
	}

	return ret;
}

static int gs_map_dt_data(struct platform_device *pdev)
{
	struct gs_tmu_data *data = platform_get_drvdata(pdev);
	struct resource res;
	const char *tmu_name, *buf;
	int ret;

	if (!data || !pdev->dev.of_node)
		return -ENODEV;

	data->np = pdev->dev.of_node;

	if (of_property_read_u32(pdev->dev.of_node, "id", &data->id)) {
		dev_err(&pdev->dev, "failed to get TMU ID\n");
		return -ENODEV;
	}

	data->irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (data->irq <= 0) {
		dev_err(&pdev->dev, "failed to get IRQ\n");
		return -ENODEV;
	}

	if (of_address_to_resource(pdev->dev.of_node, 0, &res)) {
		dev_err(&pdev->dev, "failed to get Resource 0\n");
		return -ENODEV;
	}

	data->base = devm_ioremap(&pdev->dev, res.start, resource_size(&res));
	if (!data->base) {
		dev_err(&pdev->dev, "Failed to ioremap memory\n");
		return -EADDRNOTAVAIL;
	}

	if (of_property_read_string(pdev->dev.of_node, "tmu_name", &tmu_name))
		dev_err(&pdev->dev, "failed to get tmu_name\n");
	else
		strncpy(data->tmu_name, tmu_name, THERMAL_NAME_LENGTH);

	data->tmu_type = TMU_TYPE_CPU;
	if (of_property_read_u32(pdev->dev.of_node, "tmu_type", &data->tmu_type))
		dev_warn(&pdev->dev, "Failed to get TMU type\n");

	if (data->tmu_type >= TMU_TYPE_END)
		dev_warn(&pdev->dev, "Invalid tmu type %d\n", data->tmu_type);

	dev_info(&pdev->dev, "tmu type: %d \n", data->tmu_type);

	data->pause_enable = of_property_read_bool(pdev->dev.of_node,
						   "pause_enable");
	if (data->pause_enable) {
		dev_info(&pdev->dev, "thermal zone use pause function\n");

		of_property_read_u32(pdev->dev.of_node, "pause_threshold",
				     &data->pause_threshold);

		if (!data->pause_threshold)
			dev_err(&pdev->dev, "No input pause_threshold\n");

		of_property_read_u32(pdev->dev.of_node, "resume_threshold",
				     &data->resume_threshold);

		if (!data->resume_threshold)
			dev_err(&pdev->dev, "No input resume_threshold\n");

		ret = of_property_read_string(pdev->dev.of_node, "pause_cpus", &buf);
		if (!ret)
			cpulist_parse(buf, &data->pause_cpus);
	}

	data->hardlimit_enable = of_property_read_bool(pdev->dev.of_node,
							    "hardlimit_enable");
	if (data->hardlimit_enable) {
		dev_info(&pdev->dev, "thermal zone use hardlimit function\n");
		of_property_read_u32(pdev->dev.of_node, "hardlimit_threshold",
				     &data->hardlimit_threshold);

		if (!data->hardlimit_threshold)
			dev_err(&pdev->dev, "No input hardlimit_threshold\n");

		of_property_read_u32(pdev->dev.of_node, "hardlimit_clr_threshold",
				     &data->hardlimit_clr_threshold);

		if (!data->hardlimit_clr_threshold)
			dev_err(&pdev->dev, "No input hardlimit_clr_threshold\n");

		of_property_read_u32(pdev->dev.of_node, "hardlimit_cooling_state",
				     &data->hardlimit_cooling_state);

		if (!data->hardlimit_cooling_state)
			dev_err(&pdev->dev, "No input hardlimit_cooling_state\n");

	}

#if IS_ENABLED(CONFIG_EXYNOS_CPUHP)
	data->hotplug_enable = of_property_read_bool(pdev->dev.of_node, "hotplug_enable");
	if (data->hotplug_enable) {
		dev_info(&pdev->dev, "thermal zone use hotplug function\n");
		of_property_read_u32(pdev->dev.of_node, "hotplug_in_threshold",
				     &data->hotplug_in_threshold);
		if (!data->hotplug_in_threshold)
			dev_err(&pdev->dev, "No input hotplug_in_threshold\n");

		of_property_read_u32(pdev->dev.of_node, "hotplug_out_threshold",
				     &data->hotplug_out_threshold);
		if (!data->hotplug_out_threshold)
			dev_err(&pdev->dev, "No input hotplug_out_threshold\n");

		ret = of_property_read_string(pdev->dev.of_node, "hotplug_cpus", &buf);
		if (!ret)
			cpulist_parse(buf, &data->hotplug_cpus);

		ret = of_property_read_string(pdev->dev.of_node, "hotplug_work_affinity", &buf);
		if (!ret)
			cpulist_parse(buf, &data->hotplug_work_affinity);
	}
#endif

#if IS_ENABLED(CONFIG_GOOGLE_BCL)
	data->cpu_hw_throttling_enable = of_property_read_bool(pdev->dev.of_node,
							       "cpu_hw_throttling_enable");
	if (data->cpu_hw_throttling_enable) {
		dev_info(&pdev->dev, "thermal zone use cpu hw throttling function\n");
		of_property_read_u32(pdev->dev.of_node, "cpu_hw_throttling_trigger_threshold",
				     &data->cpu_hw_throttling_trigger_threshold);

		if (!data->cpu_hw_throttling_trigger_threshold)
			dev_err(&pdev->dev, "No input cpu_hw_throttling_trigger_threshold\n");

		of_property_read_u32(pdev->dev.of_node, "cpu_hw_throttling_clr_threshold",
				     &data->cpu_hw_throttling_clr_threshold);

		if (!data->cpu_hw_throttling_clr_threshold)
			dev_err(&pdev->dev, "No input cpu_hw_throttling_clr_threshold\n");

		of_property_read_u32(pdev->dev.of_node, "ppm_level",
				     &data->ppm_throttle_level);

		if (!data->ppm_throttle_level)
			dev_err(&pdev->dev, "No input ppm_level\n");

		of_property_read_u32(pdev->dev.of_node, "mpmm_level",
				     &data->mpmm_throttle_level);

		if (!data->mpmm_throttle_level)
			dev_err(&pdev->dev, "No input mpmm_level\n");
	}
#endif

	ret = of_property_read_string(pdev->dev.of_node, "tmu_work_affinity", &buf);
	if (!ret)
		cpulist_parse(buf, &data->tmu_work_affinity);

	if (of_property_read_bool(pdev->dev.of_node, "use-pi-thermal")) {
		struct gs_pi_param *params;
		u32 value;

		data->use_pi_thermal = true;

		params = kzalloc(sizeof(*params), GFP_KERNEL);
		if (!params)
			return -ENOMEM;

		ret = of_property_read_u32(pdev->dev.of_node, "k_po",
					   &value);
		if (ret < 0)
			dev_err(&pdev->dev, "No input k_po\n");
		else
			params->k_po = int_to_frac(value);

		ret = of_property_read_u32(pdev->dev.of_node, "k_pu",
					   &value);
		if (ret < 0)
			dev_err(&pdev->dev, "No input k_pu\n");
		else
			params->k_pu = int_to_frac(value);

		ret = of_property_read_u32(pdev->dev.of_node, "k_i",
					   &value);
		if (ret < 0)
			dev_err(&pdev->dev, "No input k_i\n");
		else
			params->k_i = int_to_frac(value);

		ret = of_property_read_u32(pdev->dev.of_node, "i_max",
					   &value);
		if (ret < 0)
			dev_err(&pdev->dev, "No input i_max\n");
		else
			params->i_max = int_to_frac(value);

		ret = of_property_read_u32(pdev->dev.of_node, "integral_cutoff",
					   &value);
		if (ret < 0)
			dev_err(&pdev->dev, "No input integral_cutoff\n");
		else
			params->integral_cutoff = value;

		ret = of_property_read_u32(pdev->dev.of_node, "sustainable_power",
					   &value);
		if (ret < 0)
			dev_err(&pdev->dev, "No input sustainable_power\n");
		else
			params->sustainable_power = value;

		data->pi_param = params;
	} else {
		data->use_pi_thermal = false;
	}

	of_property_read_u32(pdev->dev.of_node, "polling_delay_on", &data->polling_delay_on);
	if (acpm_gov_common.turn_on == true) {
		if (data->polling_delay_on < ACPM_GOV_TIMER_INTERVAL_MS_MIN) {
			dev_info(&pdev->dev, "polling_delay_on is out of range, using min value %d\n",
				 ACPM_GOV_TIMER_INTERVAL_MS_MIN);
			data->polling_delay_on = ACPM_GOV_TIMER_INTERVAL_MS_MIN;
		} else if (data->polling_delay_on > ACPM_GOV_TIMER_INTERVAL_MS_MAX) {
			dev_info(&pdev->dev, "polling_delay_on is out of range, using max value %d\n",
				 ACPM_GOV_TIMER_INTERVAL_MS_MAX);
			data->polling_delay_on = ACPM_GOV_TIMER_INTERVAL_MS_MAX;
		} else if (data->polling_delay_on == 0) {
			dev_info(&pdev->dev, "No input polling_delay_on, using default value %d\n",
				 ACPM_GOV_TIMER_INTERVAL_MS_DEFAULT);
			data->polling_delay_on = ACPM_GOV_TIMER_INTERVAL_MS_DEFAULT;
		}
	} else if (data->polling_delay_on == 0)
		dev_err(&pdev->dev, "No input polling_delay_on\n");

	of_property_read_u32(pdev->dev.of_node, "polling_delay_off", &data->polling_delay_off);

	ret = of_property_read_u32(pdev->dev.of_node, "thermal_pressure_time_window",
				   &data->thermal_pressure_time_window);
	if (ret < 0) {
		data->thermal_pressure_time_window = 0;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "control_temp_step",
		&data->control_temp_step);
	if (ret < 0) {
		data->control_temp_step = 0;
		dev_err(&pdev->dev, "No input control_temp_step\n");
	}

	data->acpm_gov_params.qword = 0;
	if (of_property_read_bool(pdev->dev.of_node, "use-acpm-gov")) {
		u32 temp;

		ret = of_property_read_u32(pdev->dev.of_node, "irq_stepwise_gain", &temp);
		if (ret < 0)
			data->acpm_gov_params.fields.irq_stepwise_gain = STEPWISE_GAIN_MIN;
		else {
			if ((temp >= STEPWISE_GAIN_MIN) && (temp <= STEPWISE_GAIN_MAX)) {
				data->acpm_gov_params.fields.irq_stepwise_gain = temp;
			} else {
				data->acpm_gov_params.fields.irq_stepwise_gain = STEPWISE_GAIN_MIN;
			}
		}

		ret = of_property_read_u32(pdev->dev.of_node, "timer_stepwise_gain", &temp);
		if (ret < 0)
			data->acpm_gov_params.fields.timer_stepwise_gain = STEPWISE_GAIN_MIN;
		else {
			if ((temp >= STEPWISE_GAIN_MIN) && (temp <= STEPWISE_GAIN_MAX)) {
				data->acpm_gov_params.fields.timer_stepwise_gain = temp;
			} else {
				data->acpm_gov_params.fields.timer_stepwise_gain =
					STEPWISE_GAIN_MIN;
			}
		}

		ret = of_property_read_u32(pdev->dev.of_node, "integral_thresh", &temp);
		if (ret < 0)
			data->acpm_gov_params.fields.integral_thresh = INTEGRAL_THRESH_MIN;
		else {
			if ((temp >= INTEGRAL_THRESH_MIN) && (temp <= INTEGRAL_THRESH_MAX)) {
				data->acpm_gov_params.fields.integral_thresh = temp;
			} else {
				data->acpm_gov_params.fields.integral_thresh = INTEGRAL_THRESH_MIN;
			}
		}

		data->acpm_gov_params.fields.enable = 1;
	}

	ret = of_property_read_string(pdev->dev.of_node, "mapped_cpus", &buf);
	if (!ret) {
		cpulist_parse(buf, &data->mapped_cpus);
		cpumask_and(&data->mapped_cpus, &data->mapped_cpus, cpu_possible_mask);
	} else
		cpumask_clear(&data->mapped_cpus);

	ret = of_property_read_u32(pdev->dev.of_node, "pressure_index", &data->pressure_index);
	if (ret || (data->pressure_index >= NR_PRESSURE_TZ))
		data->pressure_index = -1;

	return 0;
}

static const struct thermal_zone_of_device_ops gs_sensor_ops = {
	.get_temp = gs_get_temp,
	.set_emul_temp = gs_tmu_set_emulation,
	.get_trend = gs_get_trend,
	.set_trip_temp = gs_tmu_set_trip_temp,
};

static ssize_t
cpu_hw_throttling_trigger_temp_show(struct device *dev, struct device_attribute *devattr,
				 char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", data->cpu_hw_throttling_trigger_threshold);
}

static ssize_t
cpu_hw_throttling_trigger_temp_store(struct device *dev, struct device_attribute *devattr,
				  const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);
	int cpu_hw_throttling_trigger = 0;

	mutex_lock(&data->lock);

	if (kstrtos32(buf, 10, &cpu_hw_throttling_trigger)) {
		mutex_unlock(&data->lock);
		return -EINVAL;
	}

	data->cpu_hw_throttling_trigger_threshold = cpu_hw_throttling_trigger;

	mutex_unlock(&data->lock);

	return count;
}

static ssize_t control_temp_step_show(struct device *dev, struct device_attribute *devattr,
					       char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", data->control_temp_step);
}

static ssize_t control_temp_step_store(struct device *dev,
				struct device_attribute *devattr, const char *buf,
				size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);
	u32 control_temp_step = 0;

	if (kstrtou32(buf, 10, &control_temp_step))
		return -EINVAL;

	mutex_lock(&data->lock);
	data->control_temp_step = control_temp_step;
	exynos_acpm_tmu_ipc_set_control_temp_step(data->id, data->control_temp_step);
	mutex_unlock(&data->lock);

	return count;
}

/* ACPM GOV tunables */
static ssize_t acpm_gov_irq_stepwise_gain_show(struct device *dev, struct device_attribute *devattr,
					       char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", data->acpm_gov_params.fields.irq_stepwise_gain);
}

static ssize_t acpm_gov_irq_stepwise_gain_store(struct device *dev,
						struct device_attribute *devattr, const char *buf,
						size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);
	int irq_stepwise_gain = 0;

	mutex_lock(&data->lock);

	if (kstrtos32(buf, 10, &irq_stepwise_gain)) {
		mutex_unlock(&data->lock);
		return -EINVAL;
	}

	if (irq_stepwise_gain < STEPWISE_GAIN_MIN) {
		irq_stepwise_gain = STEPWISE_GAIN_MIN;
	} else if (irq_stepwise_gain > STEPWISE_GAIN_MAX) {
		irq_stepwise_gain = STEPWISE_GAIN_MAX;
	}

	data->acpm_gov_params.fields.irq_stepwise_gain = irq_stepwise_gain;

	mutex_unlock(&data->lock);

	exynos_acpm_tmu_ipc_set_gov_config(data->id, data->acpm_gov_params.qword);

	return count;
}

static ssize_t acpm_gov_timer_stepwise_gain_show(struct device *dev,
						 struct device_attribute *devattr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", data->acpm_gov_params.fields.timer_stepwise_gain);
}

static ssize_t acpm_gov_timer_stepwise_gain_store(struct device *dev,
						  struct device_attribute *devattr, const char *buf,
						  size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);
	int timer_stepwise_gain = 0;

	mutex_lock(&data->lock);

	if (kstrtos32(buf, 10, &timer_stepwise_gain)) {
		mutex_unlock(&data->lock);
		return -EINVAL;
	}

	if (timer_stepwise_gain < 0) {
		timer_stepwise_gain = 0;
	} else if (timer_stepwise_gain > 5) { //TODO: better value to saturate?
		timer_stepwise_gain = 5;
	}

	data->acpm_gov_params.fields.timer_stepwise_gain = timer_stepwise_gain;

	mutex_unlock(&data->lock);

	exynos_acpm_tmu_ipc_set_gov_config(data->id, data->acpm_gov_params.qword);

	return count;
}

static ssize_t tj_cur_cdev_state_show(struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);

	struct curr_state curr_state;
	u8 tj_cur_cdev_state_val;

	if (ACPM_BUF_VER == EXPECT_BUF_VER &&
	    get_curr_state_from_acpm(acpm_gov_common.sm_base, data->id, &curr_state))
		tj_cur_cdev_state_val = curr_state.cdev_state;
	else
		return -EIO;

	return sysfs_emit(buf, "%u\n", tj_cur_cdev_state_val);
}

static ssize_t thermal_pressure_time_window_store(struct device *dev,
						  struct device_attribute *devattr, const char *buf,
						  size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);

	u8 thermal_pressure_time_window;

	if (kstrtou8(buf, 10, &thermal_pressure_time_window)) {
		pr_err("%s: thermal_pressure_time_window parse error", __func__);
		return -EINVAL;
	}

	if ((thermal_pressure_time_window < ACPM_GOV_THERMAL_PRESS_WINDOW_MS_MIN) ||
	    (thermal_pressure_time_window > ACPM_GOV_THERMAL_PRESS_WINDOW_MS_MAX)) {
		return -ERANGE;
	}

	if (exynos_acpm_tmu_ipc_set_gov_tz_time_windows(data->id, data->polling_delay_on,
							thermal_pressure_time_window)) {
		pr_err("%s: unable to set thermal_pressure_time_window", __func__);
		return -EINVAL;
	}

	data->thermal_pressure_time_window = thermal_pressure_time_window;

	return count;
}

static ssize_t thermal_pressure_time_window_show(struct device *dev,
						 struct device_attribute *devattr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);

	int timer_interval;
	int time_window;

	if (exynos_acpm_tmu_ipc_get_gov_tz_time_windows(data->id, &timer_interval, &time_window))
		return -EIO;

	return sysfs_emit(buf, "%d\n", time_window);
}

static int param_acpm_gov_kernel_ts_get(char *buf, const struct kernel_param *kp)
{
	return sysfs_emit(buf, "%llu\n", acpm_gov_common.kernel_ts);
}

static int param_acpm_gov_kernel_ts_set(const char *val, const struct kernel_param *kp)
{
	if (kstrtou64(val, 10, &acpm_gov_common.kernel_ts)) {
		pr_err("%s: kernel_ts parse error", __func__);
		return -1;
	}
	return 0;
}

static const struct kernel_param_ops param_ops_acpm_gov_kernel_ts = {
	.get = param_acpm_gov_kernel_ts_get,
	.set = param_acpm_gov_kernel_ts_set,
};

module_param_cb(acpm_gov_kernel_ts, &param_ops_acpm_gov_kernel_ts, NULL, 0644);

static int param_acpm_gov_acpm_ts_get(char *buf, const struct kernel_param *kp)
{
	return sysfs_emit(buf, "%llu\n", acpm_gov_common.acpm_ts);
}

static int param_acpm_gov_acpm_ts_set(const char *val, const struct kernel_param *kp)
{
	if (kstrtou64(val, 10, &acpm_gov_common.acpm_ts)) {
		pr_err("%s: acpm_ts parse error", __func__);
		return -1;
	}
	return 0;
}

static const struct kernel_param_ops param_ops_acpm_gov_acpm_ts = {
	.get = param_acpm_gov_acpm_ts_get,
	.set = param_acpm_gov_acpm_ts_set,
};

module_param_cb(acpm_gov_acpm_ts, &param_ops_acpm_gov_acpm_ts, NULL, 0644);

static int param_acpm_gov_last_ts_get(char *buf, const struct kernel_param *kp)
{
	return sysfs_emit(buf, "%llu\n", acpm_gov_common.last_ts);
}

static int param_acpm_gov_last_ts_set(const char *val, const struct kernel_param *kp)
{
	if (kstrtou64(val, 10, &acpm_gov_common.last_ts)) {
		pr_err("%s: last_ts parse error", __func__);
		return -1;
	}
	return 0;
}

static const struct kernel_param_ops param_ops_acpm_gov_last_ts = {
	.get = param_acpm_gov_last_ts_get,
	.set = param_acpm_gov_last_ts_set,
};

module_param_cb(acpm_gov_last_ts, &param_ops_acpm_gov_last_ts, NULL, 0644);

static int param_acpm_gov_tracing_mode_get(char *buf, const struct kernel_param *kp)
{
	return sysfs_emit(buf, "%d\n", acpm_gov_common.tracing_mode);
}

static int param_acpm_gov_tracing_mode_set(const char *val, const struct kernel_param *kp)
{
	u8 tracing_mode_val;

	if (kstrtou8(val, 10, &tracing_mode_val)) {
		pr_err("%s: tracing_mode parse error", __func__);
		return -EINVAL;
	}

	if ((tracing_mode_val < ACPM_GOV_DEBUG_MODE_DISABLED) ||
	    (tracing_mode_val >= ACPM_GOV_DEBUG_MODE_INVALID)) {
		return -ERANGE;
	}

	if (acpm_gov_common.turn_on == false)
		return -EINVAL;

	switch (tracing_mode_val) {
	case ACPM_GOV_DEBUG_MODE_DISABLED:
	case ACPM_GOV_DEBUG_MODE_HIGH_OVERHEAD:
		acpm_gov_common.tracing_mode = tracing_mode_val;
		exynos_acpm_tmu_ipc_set_gov_debug_tracing_mode(acpm_gov_common.tracing_mode);
		if (acpm_gov_common.bulk_trace_buffer) {
			if ((acpm_gov_common.tracing_mode == ACPM_GOV_DEBUG_MODE_BULK) &&
			    (ACPM_BUF_VER == EXPECT_BUF_VER)) {
				capture_bulk_trace();
			}
			acpm_gov_common.tracing_buffer_flush_pending = false;
			spin_lock(&acpm_gov_common.lock);
			kfree(acpm_gov_common.bulk_trace_buffer);
			acpm_gov_common.bulk_trace_buffer = NULL;
			spin_unlock(&acpm_gov_common.lock);
		}
		break;
	case ACPM_GOV_DEBUG_MODE_BULK:
		if (!acpm_gov_common.bulk_trace_buffer) {
			acpm_gov_common.bulk_trace_buffer =
				kmalloc(sizeof(struct gov_trace_data_struct), GFP_KERNEL);
			if (!acpm_gov_common.bulk_trace_buffer) {
				return -ENOMEM;
			}
		}
		acpm_gov_common.tracing_buffer_flush_pending = true;
		acpm_gov_common.tracing_mode = tracing_mode_val;
		exynos_acpm_tmu_ipc_set_gov_debug_tracing_mode(acpm_gov_common.tracing_mode);
		break;
	default:
		pr_err("%s: tracing_mode_val invalid error", __func__);
		return -EINVAL;
	}

	return 0;
}

static const struct kernel_param_ops param_ops_acpm_gov_tracing_mode = {
	.get = param_acpm_gov_tracing_mode_get,
	.set = param_acpm_gov_tracing_mode_set,
};

module_param_cb(acpm_gov_tracing_mode, &param_ops_acpm_gov_tracing_mode, NULL, 0644);

static int param_acpm_gov_thermal_state_get(char *buf, const struct kernel_param *kp)
{
	struct thermal_state therm_state;

	if (ACPM_BUF_VER == EXPECT_BUF_VER &&
	    get_thermal_state_from_acpm(acpm_gov_common.sm_base, &therm_state)) {
		return sysfs_emit(buf,
				  "switch_on=%x, dfs_on=%x, pressure: [0]=%d, [1]=%d, [2]=%d\n",
				  therm_state.switched_on, therm_state.dfs_on,
				  therm_state.therm_press[0], therm_state.therm_press[1],
				  therm_state.therm_press[2]);
	}

	return -EIO;
}

static const struct kernel_param_ops param_ops_acpm_gov_thermal_state = {
	.get = param_acpm_gov_thermal_state_get,
};

module_param_cb(acpm_gov_thermal_state, &param_ops_acpm_gov_thermal_state, NULL, 0444);

static int param_acpm_gov_turn_on_get(char *buf, const struct kernel_param *kp)
{
	return sysfs_emit(buf, "%d\n", acpm_gov_common.turn_on);
}

static int param_acpm_gov_turn_on_set(const char *val, const struct kernel_param *kp)
{
	struct gs_tmu_data *gsdata;
	u8 turn_on_val;

	if (kstrtou8(val, 10, &turn_on_val)) {
		pr_err("%s: turn_on parse error", __func__);
		return -EINVAL;
	}

	/* only allowing 0 -> 1 */
	if ((acpm_gov_common.turn_on == true) || (turn_on_val == 0))
		return -EINVAL;

	if (acpm_ipc_get_buffer("GOV_DBG", (char **)&acpm_gov_common.sm_base, &acpm_gov_common.sm_size)) {
		pr_err("GOV: unavailable\n");
		return -EINVAL;
	}

	if (acpm_gov_common.sm_size == 0xf10) {
		acpm_gov_common.buffer_version = 0x0;
	} else {
		memcpy_fromio(&acpm_gov_common.buffer_version, acpm_gov_common.sm_base,
			      sizeof(acpm_gov_common.buffer_version));
		if ((acpm_gov_common.buffer_version >> 32) != ACPM_SM_BUFFER_VERSION_UPPER_32b) {
			pr_err("GOV: shared memory table version mismatch\n");
			return -EINVAL;
		}
	}

	if (exynos_acpm_tmu_cb_init(&cb))
		return -EINVAL;

	exynos_acpm_tmu_ipc_set_gov_debug_tracing_mode(acpm_gov_common.tracing_mode);

	//run loop for all TZ
	list_for_each_entry (gsdata, &dtm_dev_list, node) {
		disable_irq_nosync(gsdata->irq);
		if (gsdata->acpm_gov_params.fields.enable) {
			int tzid = gsdata->id;
			u32 control_temp_step = gsdata->control_temp_step;

			gsdata->acpm_gov_params.fields.ctrl_temp_idx =
				gsdata->trip_control_temp;
			gsdata->acpm_gov_params.fields.switch_on_temp_idx =
				gsdata->trip_switch_on;

			//sending an IPC to setup GOV param and control temperature step
			exynos_acpm_tmu_ipc_set_gov_config(tzid, gsdata->acpm_gov_params.qword);
			exynos_acpm_tmu_ipc_set_control_temp_step(tzid, control_temp_step);
			exynos_acpm_tmu_ipc_set_gov_tz_time_windows(
				gsdata->id, gsdata->polling_delay_on,
				gsdata->thermal_pressure_time_window);
		}
		thermal_zone_device_disable(gsdata->tzd);
	}

	acpm_gov_common.turn_on = turn_on_val ? true : false;
	return 0;
}

static const struct kernel_param_ops param_ops_acpm_gov_turn_on = {
	.get = param_acpm_gov_turn_on_get,
	.set = param_acpm_gov_turn_on_set,
};

module_param_cb(acpm_gov_turn_on, &param_ops_acpm_gov_turn_on, NULL, 0644);

static int param_update_acpm_pi_table_set(const char *val, const struct kernel_param *kp)
{
	struct gs_tmu_data *data;

	list_for_each_entry (data, &dtm_dev_list, node) {
		if (data->use_pi_thermal) {
			struct thermal_zone_device *tz = data->tzd;
			struct thermal_instance *instance;
			struct thermal_cooling_device *cdev;
			bool found_actor = false;

			list_for_each_entry (instance, &tz->thermal_instances, tz_node) {
				if (instance->trip == data->trip_control_temp &&
				    cdev_is_power_actor(instance->cdev)) {
					found_actor = true;
					cdev = instance->cdev;
					break;
				}
			}

			if (found_actor) {
				int i;
				unsigned long max_state;
				cdev->ops->get_max_state(cdev, &max_state);
				for (i = 0; i <= (int)max_state; i++) {
					int power;
					cdev->ops->state2power(cdev, i, &power);
					exynos_acpm_tmu_ipc_set_table(data->id, i, power);
				}
			}

			update_tj_power_table_ect_offset(data);
		}
	}

	return 0;
}

static const struct kernel_param_ops param_ops_update_acpm_pi_table = {
	.set = param_update_acpm_pi_table_set,
};

module_param_cb(update_acpm_pi_table, &param_ops_update_acpm_pi_table, NULL, 0200);

static ssize_t fvp_get_target_freq_show(struct device *dev, struct device_attribute *devattr,
					char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);

	exynos_acpm_tmu_ipc_get_target_freq(data->id, &data->fvp_get_target_freq);
	return sysfs_emit(buf, "%d\n", data->fvp_get_target_freq);
}

static ssize_t fvp_get_target_freq_store(struct device *dev, struct device_attribute *devattr,
					 const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);
	int fvp_get_target_freq_data = 0;

	mutex_lock(&data->lock);

	if (kstrtos32(buf, 10, &fvp_get_target_freq_data)) {
		mutex_unlock(&data->lock);
		return -EINVAL;
	}

	data->fvp_get_target_freq = fvp_get_target_freq_data;

	mutex_unlock(&data->lock);

	return count;
}

static ssize_t
cpu_hw_throttling_clr_temp_show(struct device *dev, struct device_attribute *devattr,
						 char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", data->cpu_hw_throttling_clr_threshold);
}

static ssize_t
cpu_hw_throttling_clr_temp_store(struct device *dev, struct device_attribute *devattr,
			      const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);
	int cpu_hw_throttling_clear = 0;

	mutex_lock(&data->lock);

	if (kstrtos32(buf, 10, &cpu_hw_throttling_clear)) {
		mutex_unlock(&data->lock);
		return -EINVAL;
	}

	data->cpu_hw_throttling_clr_threshold = cpu_hw_throttling_clear;

	mutex_unlock(&data->lock);

	return count;
}

static ssize_t
hotplug_out_temp_show(struct device *dev, struct device_attribute *devattr,
		      char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", data->hotplug_out_threshold);
}

static ssize_t
hotplug_out_temp_store(struct device *dev, struct device_attribute *devattr,
		       const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);
	int hotplug_out = 0;

	mutex_lock(&data->lock);

	if (kstrtos32(buf, 10, &hotplug_out)) {
		mutex_unlock(&data->lock);
		return -EINVAL;
	}

	data->hotplug_out_threshold = hotplug_out;

	mutex_unlock(&data->lock);

	return count;
}

static ssize_t
hotplug_in_temp_show(struct device *dev, struct device_attribute *devattr,
		     char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", data->hotplug_in_threshold);
}

static ssize_t
hotplug_in_temp_store(struct device *dev, struct device_attribute *devattr,
		      const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);
	int hotplug_in = 0;

	mutex_lock(&data->lock);

	if (kstrtos32(buf, 10, &hotplug_in)) {
		mutex_unlock(&data->lock);
		return -EINVAL;
	}

	data->hotplug_in_threshold = hotplug_in;

	mutex_unlock(&data->lock);

	return count;
}

static ssize_t
hardlimit_temp_show(struct device *dev, struct device_attribute *devattr,
			    char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", data->hardlimit_threshold);
}

static ssize_t
hardlimit_temp_store(struct device *dev, struct device_attribute *devattr,
			     const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);
	int hardlimit_throttling_trigger = 0;

	mutex_lock(&data->lock);

	if (kstrtos32(buf, 10, &hardlimit_throttling_trigger)) {
		mutex_unlock(&data->lock);
		return -EINVAL;
	}

	data->hardlimit_threshold = hardlimit_throttling_trigger;

	mutex_unlock(&data->lock);

	return count;
}

static ssize_t
hardlimit_clr_temp_show(struct device *dev, struct device_attribute *devattr,
			     char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", data->hardlimit_clr_threshold);
}

static ssize_t
hardlimit_clr_temp_store(struct device *dev, struct device_attribute *devattr,
			      const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);
	int hardlimit_clr_threshold = 0;

	mutex_lock(&data->lock);

	if (kstrtos32(buf, 10, &hardlimit_clr_threshold)) {
		mutex_unlock(&data->lock);
		return -EINVAL;
	}

	data->hardlimit_clr_threshold = hardlimit_clr_threshold;

	mutex_unlock(&data->lock);

	return count;
}

static ssize_t
pause_cpus_temp_show(struct device *dev, struct device_attribute *devattr,
			    char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", data->pause_threshold);
}

static ssize_t
pause_cpus_temp_store(struct device *dev, struct device_attribute *devattr,
			     const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);
	int pause_throttling_trigger = 0;

	mutex_lock(&data->lock);

	if (kstrtos32(buf, 10, &pause_throttling_trigger)) {
		mutex_unlock(&data->lock);
		return -EINVAL;
	}

	data->pause_threshold = pause_throttling_trigger;

	mutex_unlock(&data->lock);

	return count;
}

static ssize_t
resume_cpus_temp_show(struct device *dev, struct device_attribute *devattr,
			     char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", data->resume_threshold);
}

static ssize_t
resume_cpus_temp_store(struct device *dev, struct device_attribute *devattr,
			      const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);
	int resume_threshold = 0;

	mutex_lock(&data->lock);

	if (kstrtos32(buf, 10, &resume_threshold)) {
		mutex_unlock(&data->lock);
		return -EINVAL;
	}

	data->resume_threshold = resume_threshold;

	mutex_unlock(&data->lock);

	return count;
}

static ssize_t
sustainable_power_show(struct device *dev, struct device_attribute *devattr,
		       char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);

	if (data->pi_param)
		return sysfs_emit(buf, "%u\n", data->pi_param->sustainable_power);

	return -EIO;
}

static ssize_t
sustainable_power_store(struct device *dev, struct device_attribute *devattr,
			const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);
	u32 sustainable_power;

	if (!data->pi_param)
		return -EIO;

	if (kstrtou32(buf, 10, &sustainable_power))
		return -EINVAL;

	data->pi_param->sustainable_power = sustainable_power;

	return count;
}

static ssize_t
integral_cutoff_show(struct device *dev, struct device_attribute *devattr,
		       char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);

	if (data->pi_param)
		return sysfs_emit(buf, "%u\n", data->pi_param->integral_cutoff);

	return -EIO;
}

static ssize_t
integral_cutoff_store(struct device *dev, struct device_attribute *devattr,
			const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);
	int integral_cutoff;

	if (!data->pi_param)
		return -EIO;

	if (kstrtos32(buf, 10, &integral_cutoff))
		return -EINVAL;

	data->pi_param->integral_cutoff = integral_cutoff;

	return count;
}

static ssize_t
power_table_ect_offset_show(struct device *dev, struct device_attribute *devattr,
					   char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);

	if (data->use_pi_thermal) {
		u32 power_table_ect_offset;
		exynos_acpm_tmu_ipc_get_pi_param(data->id, POWER_TABLE_ECT_OFFSET,
						 &power_table_ect_offset);
		return sysfs_emit(buf, "%u\n", power_table_ect_offset);
	}
	return -EIO;
}

static ssize_t
power_table_ect_offset_store(struct device *dev, struct device_attribute *devattr,
			const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);

	if (data->use_pi_thermal) {
		u8 power_table_ect_offset;
		if (kstrtou8(buf, 10, &power_table_ect_offset))
			return -EINVAL;

		exynos_acpm_tmu_ipc_set_pi_param(data->id, POWER_TABLE_ECT_OFFSET, (u32)power_table_ect_offset);
		return count;
	}

	return -EIO;
}

static ssize_t
acpm_pi_enable_show(struct device *dev, struct device_attribute *devattr,
		       char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%u\n", data->acpm_pi_enable);
}

static ssize_t
acpm_pi_enable_store(struct device *dev, struct device_attribute *devattr,
			const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);

	if (data->use_pi_thermal) {
		if (kstrtobool(buf, &data->acpm_pi_enable))
			return -EINVAL;

		exynos_acpm_tmu_ipc_set_pi_param(data->id, PI_ENABLE, data->acpm_pi_enable);
	} else {
		return -EINVAL;
	}
	return count;
}

static ssize_t
acpm_pi_table_show(struct device *dev, struct device_attribute *devattr,
		       char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);
	int count = 0;
	int i;
	struct thermal_zone_device *tz;
	struct gs_pi_param *params;
	struct thermal_instance *instance;
	struct thermal_cooling_device *cdev;
	bool found_actor = false;
	unsigned long max_state;

	if (!data->use_pi_thermal)
		return 0;

	tz = data->tzd;
	params = data->pi_param;

	list_for_each_entry (instance, &tz->thermal_instances, tz_node) {
		if (instance->trip == data->trip_control_temp &&
		    cdev_is_power_actor(instance->cdev)) {
			found_actor = true;
			cdev = instance->cdev;
			break;
		}
	}

	if (!found_actor) {
		count += sysfs_emit_at(buf, count, "No cdev found\n");
		return count;
	}

	cdev->ops->get_max_state(cdev, &max_state);
	for (i = 0; i <= (int)max_state; i++) {
		int value;
		exynos_acpm_tmu_ipc_get_table(data->id, i, &value);
		count += sysfs_emit_at(buf, count, "%d ", value);
	}
	count += sysfs_emit_at(buf, count, "\n");

	return count;
}

static ssize_t
acpm_pi_table_store(struct device *dev, struct device_attribute *devattr,
			const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);
	int ret = 0;
	char **argv;
	int argc;
	struct thermal_zone_device *tz;
	struct gs_pi_param *params;
	struct thermal_instance *instance;
	struct thermal_cooling_device *cdev;
	bool found_actor = false;
	unsigned long max_state;

	if (!data->use_pi_thermal)
		return 0;

	tz = data->tzd;
	params = data->pi_param;

	list_for_each_entry (instance, &tz->thermal_instances, tz_node) {
		if (instance->trip == data->trip_control_temp &&
		    cdev_is_power_actor(instance->cdev)) {
			found_actor = true;
			cdev = instance->cdev;
			break;
		}
	}

	if (!found_actor)
		return ret;

	argv = argv_split(GFP_KERNEL, buf, &argc);
	if (!argv) {
		ret = -ENOMEM;
		pr_err("%s: memory allocation error, ret=%d", __func__, ret);
		goto out;
	}
	cdev->ops->get_max_state(cdev, &max_state);
	if (argc != (int)max_state + 1) {
		ret = -EINVAL;
		pr_err("%s: invalid args count, ret=%d, argc=%d, max_state=%d", __func__, ret, argc,
		       (int)max_state);
	} else {
		int i;
		for (i = 0; i <= (int)max_state; i++) {
			int val;
			ret = kstrtou32(argv[i], 10, &val);
			if (ret) {
				pr_err("%s: parse acpm_pi_table error, ret=%d", __func__, ret);
				goto out;
			}
			exynos_acpm_tmu_ipc_set_table(data->id, i, val);
		}
	}
out:
	argv_free(argv);
	return count;
}

static ssize_t
polling_delay_on_show(struct device *dev, struct device_attribute *devattr,
		       char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);

	if (acpm_gov_common.turn_on) {
		int timer_interval;
		int time_window;

		if (exynos_acpm_tmu_ipc_get_gov_tz_time_windows(data->id, &timer_interval,
								&time_window))
			return -EIO;

		return sysfs_emit(buf, "%d\n", timer_interval);
	}

	return sysfs_emit(buf, "%u\n", data->polling_delay_on);
}

static ssize_t
polling_delay_on_store(struct device *dev, struct device_attribute *devattr,
			const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);
	u32 polling_delay_on;

	if (kstrtou32(buf, 10, &polling_delay_on))
		return -EINVAL;

	if (acpm_gov_common.turn_on) {
		if ((polling_delay_on < ACPM_GOV_TIMER_INTERVAL_MS_MIN) ||
		    (polling_delay_on > ACPM_GOV_TIMER_INTERVAL_MS_MAX)) {
			return -ERANGE;
		}

		if (exynos_acpm_tmu_ipc_set_gov_tz_time_windows(
			    data->id, polling_delay_on, data->thermal_pressure_time_window)) {
			pr_err("%s: unable to set acpm gov polling_delay_on", __func__);
			return -EINVAL;
		}

		data->polling_delay_on = polling_delay_on;
	} else {
		data->polling_delay_on = polling_delay_on;

		/*
	 	 * This sysfs node is mainly used for debugging and could race with
	 	 * suspend/resume path as we don't use a lock to avoid it. The race
	 	 * could cause pi-polling work re-queued after suspend so the pid
	 	 * sample time might not run as our expectation. Please do NOT use
	 	 * this for the production line.
	 	 */
		if (data->use_pi_thermal) {
			WARN(1, "%s could potentially race with suspend/resume path!", __func__);
			start_pi_polling(data, 0);
		}
	}

	return count;
}

static ssize_t
polling_delay_off_show(struct device *dev, struct device_attribute *devattr,
		       char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%u\n", data->polling_delay_off);
}

static ssize_t
polling_delay_off_store(struct device *dev, struct device_attribute *devattr,
			const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);
	u32 polling_delay_off;

	if (kstrtou32(buf, 10, &polling_delay_off))
		return -EINVAL;

	data->polling_delay_off = polling_delay_off;

	/*
	 * This sysfs node is mainly used for debugging and could race with
	 * suspend/resume path as we don't use a lock to avoid it. The race
	 * could cause pi-polling work re-queued after suspend so the pid
	 * sample time might not run as our expectation. Please do NOT use
	 * this for the production line.
	 */
	if (data->use_pi_thermal) {
		WARN(1, "%s could potentially race with suspend/resume path!", __func__);
		start_pi_polling(data, 0);
	}

	return count;
}

static ssize_t
hardlimit_total_count_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);
	struct throttling_stats *stats = data->hardlimit_stats;
	int ret = 0;

	if (!data->hardlimit_enable)
		return ret;

	spin_lock(&stats->lock);
	ret = sysfs_emit(buf, "%u\n", stats->hardlimit_total_count);
	spin_unlock(&stats->lock);

	return ret;
}

static const char *switch_stats[] = {"disabled", " enabled"};

static ssize_t
hardlimit_time_in_state_ms_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);
	struct throttling_stats *stats = data->hardlimit_stats;
	ssize_t len = 0;
	int i;

	if (!data->hardlimit_enable)
		return len;

	spin_lock(&stats->lock);
	update_time_in_state(stats);

	for (i = 0; i < ARRAY_SIZE(switch_stats); i++) {
		len += scnprintf(buf + len, PAGE_SIZE - len, "%s\t%llu\n", switch_stats[i],
				 ktime_to_ms(stats->hardlimit_time_in_state[i]));
	}
	spin_unlock(&stats->lock);

	return len;
}

static ssize_t
hardlimit_reset_store(struct device *dev, struct device_attribute *attr, const char *buf,
		      size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);
	struct throttling_stats *stats = data->hardlimit_stats;
	int i;

	if (!data->hardlimit_enable)
		return count;

	spin_lock(&stats->lock);
	stats->hardlimit_total_count = 0;
	stats->last_time = ktime_get();

	for (i = 0; i < ARRAY_SIZE(switch_stats); i++)
		stats->hardlimit_time_in_state[i] = ktime_set(0, 0);

	spin_unlock(&stats->lock);

	return count;
}

static const char *pause_switch_stats[] = {"resumed ", "suspended"};

static ssize_t
pause_total_count_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);
	struct throttling_stats *stats = data->disable_stats;
	int ret = 0;

	if (!data->hotplug_enable && !data->pause_enable)
		return ret;

	spin_lock(&stats->lock);
	ret = sysfs_emit(buf, "%u\n", stats->disable_total_count);
	spin_unlock(&stats->lock);

	return ret;
}

static ssize_t
pause_time_in_state_ms_show(struct device *dev, struct device_attribute *attr,
				  char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);
	struct throttling_stats *stats = data->disable_stats;
	ssize_t len = 0;
	int i;

	if (!data->hotplug_enable && !data->pause_enable)
		return len;

	spin_lock(&stats->lock);
	update_time_in_state(stats);

	for (i = 0; i < ARRAY_SIZE(pause_switch_stats); i++) {
		len += scnprintf(buf + len, PAGE_SIZE - len, "%s\t%llu\n", pause_switch_stats[i],
				 ktime_to_ms(stats->disable_time_in_state[i]));
	}
	spin_unlock(&stats->lock);

	return len;
}

static ssize_t
pause_reset_store(struct device *dev, struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);
	struct throttling_stats *stats = data->disable_stats;
	int i;

	if (!data->hotplug_enable && !data->pause_enable)
		return count;

	spin_lock(&stats->lock);
	stats->disable_total_count = 0;
	stats->last_time = ktime_get();

	for (i = 0; i < ARRAY_SIZE(pause_switch_stats); i++)
		stats->disable_time_in_state[i] = ktime_set(0, 0);

	spin_unlock(&stats->lock);

	return count;
}

static ssize_t trip_counter_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);
	int i;
	int len = 0;

	if (acpm_gov_common.turn_on) {
		u64 trip_counter = 0;

		for (i = 0; i < TRIP_LEVEL_NUM; i++) {
			exynos_acpm_tmu_ipc_get_trip_counter(data->id, i, &trip_counter);
			atomic64_set(&(data->trip_counter[i]), trip_counter);
		}
	}

	for (i = 0; i < TRIP_LEVEL_NUM; i++)
		len += sysfs_emit_at(buf, len, "%lld ", atomic64_read(&(data->trip_counter[i])));

	len += sysfs_emit_at(buf, len, "\n");

	return len;
}

static ssize_t trip_counter_reset_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < TRIP_LEVEL_NUM; i++)
		atomic64_set(&(data->trip_counter[i]), 0);

	if (acpm_gov_common.turn_on)
		exynos_acpm_tmu_ipc_reset_trip_counter(data->id);

	return count;
}

static ssize_t ipc_dump1_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	union {
		unsigned int dump[2];
		unsigned char val[8];
	} data;

	exynos_acpm_tmu_ipc_dump(0, data.dump);

	return sysfs_emit(buf, "%3u %3u %3u %3u %3u %3u %3u\n",
			data.val[1], data.val[2], data.val[3],
			data.val[4], data.val[5], data.val[6], data.val[7]);
}

static ssize_t ipc_dump2_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	union {
		unsigned int dump[2];
		unsigned char val[8];
	} data;

	exynos_acpm_tmu_ipc_dump(EXYNOS_GPU_TMU_GRP_ID, data.dump);

	return sysfs_emit(buf, "%3u %3u %3u %3u %3u %3u %3u\n",
			data.val[1], data.val[2], data.val[3],
			data.val[4], data.val[5], data.val[6], data.val[7]);
}

#define create_s32_param_attr(name, Name)                                                          \
	static ssize_t name##_show(struct device *dev, struct device_attribute *devattr,           \
				   char *buf)                                                      \
	{                                                                                          \
		struct platform_device *pdev = to_platform_device(dev);                            \
		struct gs_tmu_data *data = platform_get_drvdata(pdev);                             \
                                                                                                   \
		if (data->pi_param) {                                                              \
			u32 val;                                                                   \
			exynos_acpm_tmu_ipc_get_pi_param(data->id, Name, &val);                    \
			if (int_to_frac(val) != data->pi_param->name)                              \
				return sysfs_emit(buf, "%u\n", -1);                                \
			else                                                                       \
				return sysfs_emit(buf, "%d\n", data->pi_param->name);              \
		} else                                                                             \
			return -EIO;                                                               \
	}                                                                                          \
                                                                                                   \
	static ssize_t name##_store(struct device *dev, struct device_attribute *devattr,          \
				    const char *buf, size_t count)                                 \
	{                                                                                          \
		struct platform_device *pdev = to_platform_device(dev);                            \
		struct gs_tmu_data *data = platform_get_drvdata(pdev);                             \
		s32 value;                                                                         \
                                                                                                   \
		if (!data->pi_param)                                                               \
			return -EIO;                                                               \
                                                                                                   \
		if (kstrtos32(buf, 10, &value))                                                    \
			return -EINVAL;                                                            \
                                                                                                   \
		data->pi_param->name = value;                                                      \
                                                                                                   \
		exynos_acpm_tmu_ipc_set_pi_param(data->id, Name, frac_to_int(value));              \
		return count;                                                                      \
	}                                                                                          \
	static DEVICE_ATTR_RW(name)

static DEVICE_ATTR_RW(pause_cpus_temp);
static DEVICE_ATTR_RW(resume_cpus_temp);
static DEVICE_ATTR_RW(hardlimit_temp);
static DEVICE_ATTR_RW(hardlimit_clr_temp);
static DEVICE_ATTR_RW(hotplug_out_temp);
static DEVICE_ATTR_RW(hotplug_in_temp);
static DEVICE_ATTR_RW(cpu_hw_throttling_clr_temp);
static DEVICE_ATTR_RW(cpu_hw_throttling_trigger_temp);
static DEVICE_ATTR_RW(sustainable_power);
static DEVICE_ATTR_RW(acpm_pi_table);
static DEVICE_ATTR_RW(polling_delay_off);
static DEVICE_ATTR_RW(polling_delay_on);
static DEVICE_ATTR_RO(pause_time_in_state_ms);
static DEVICE_ATTR_RO(pause_total_count);
static DEVICE_ATTR_WO(pause_reset);
static DEVICE_ATTR_RO(hardlimit_time_in_state_ms);
static DEVICE_ATTR_RO(hardlimit_total_count);
static DEVICE_ATTR_WO(hardlimit_reset);
static DEVICE_ATTR_RO(trip_counter);
static DEVICE_ATTR_WO(trip_counter_reset);
static DEVICE_ATTR_RO(ipc_dump1);
static DEVICE_ATTR_RO(ipc_dump2);
create_s32_param_attr(k_po, K_PO);
create_s32_param_attr(k_pu, K_PU);
create_s32_param_attr(k_i, K_I);
create_s32_param_attr(i_max, I_MAX);
static DEVICE_ATTR_RW(integral_cutoff);
static DEVICE_ATTR_RW(acpm_pi_enable);
static DEVICE_ATTR_RW(power_table_ect_offset);
static DEVICE_ATTR_RW(fvp_get_target_freq);
static DEVICE_ATTR_RW(acpm_gov_irq_stepwise_gain);
static DEVICE_ATTR_RW(acpm_gov_timer_stepwise_gain);
static DEVICE_ATTR_RO(tj_cur_cdev_state);
static DEVICE_ATTR_RW(control_temp_step);
static DEVICE_ATTR_RW(thermal_pressure_time_window);

static struct attribute *gs_tmu_attrs[] = {
	&dev_attr_pause_cpus_temp.attr,
	&dev_attr_resume_cpus_temp.attr,
	&dev_attr_hardlimit_temp.attr,
	&dev_attr_hardlimit_clr_temp.attr,
	&dev_attr_polling_delay_off.attr,
	&dev_attr_polling_delay_on.attr,
	&dev_attr_hotplug_out_temp.attr,
	&dev_attr_hotplug_in_temp.attr,
	&dev_attr_cpu_hw_throttling_clr_temp.attr,
	&dev_attr_cpu_hw_throttling_trigger_temp.attr,
	&dev_attr_sustainable_power.attr,
	&dev_attr_acpm_pi_table.attr,
	&dev_attr_k_po.attr,
	&dev_attr_k_pu.attr,
	&dev_attr_k_i.attr,
	&dev_attr_i_max.attr,
	&dev_attr_integral_cutoff.attr,
	&dev_attr_pause_time_in_state_ms.attr,
	&dev_attr_pause_total_count.attr,
	&dev_attr_pause_reset.attr,
	&dev_attr_hardlimit_time_in_state_ms.attr,
	&dev_attr_hardlimit_total_count.attr,
	&dev_attr_hardlimit_reset.attr,
	&dev_attr_trip_counter.attr,
	&dev_attr_trip_counter_reset.attr,
	&dev_attr_ipc_dump1.attr,
	&dev_attr_ipc_dump2.attr,
	&dev_attr_acpm_pi_enable.attr,
	&dev_attr_power_table_ect_offset.attr,
	&dev_attr_fvp_get_target_freq.attr,
	&dev_attr_acpm_gov_irq_stepwise_gain.attr,
	&dev_attr_acpm_gov_timer_stepwise_gain.attr,
	&dev_attr_tj_cur_cdev_state.attr,
	&dev_attr_control_temp_step.attr,
	&dev_attr_thermal_pressure_time_window.attr,
	NULL,
};

ATTRIBUTE_GROUPS(gs_tmu);

static void hard_limit_stats_setup(struct gs_tmu_data *data)
{
	struct throttling_stats *stats;
	int var;

	var = sizeof(*stats);
	var += sizeof(*stats->hardlimit_time_in_state) * ARRAY_SIZE(switch_stats);

	stats = kzalloc(var, GFP_KERNEL);
	if (!stats)
		return;

	stats->stats_type = HARDLIMIT_STATS;
	stats->hardlimit_time_in_state = (ktime_t *)(stats + 1);
	mutex_lock(&data->lock);
	data->hardlimit_stats = stats;
	mutex_unlock(&data->lock);
	stats->last_time = ktime_get();
	spin_lock_init(&stats->lock);
}

static void pause_stats_setup(struct gs_tmu_data *data)
{
	struct throttling_stats *stats;
	int var;

	var = sizeof(*stats);
	var += sizeof(*stats->disable_time_in_state) * ARRAY_SIZE(switch_stats);
	stats = kzalloc(var, GFP_KERNEL);
	if (!stats)
		return;

	stats->stats_type = DISABLE_STATS;
	stats->disable_time_in_state = (ktime_t *)(stats + 1);
	mutex_lock(&data->lock);
	data->disable_stats = stats;
	mutex_unlock(&data->lock);
	stats->last_time = ktime_get();
	spin_lock_init(&stats->lock);
}

#if IS_ENABLED(CONFIG_EXYNOS_ACPM_THERMAL)
static u8 tmu_id;
static u16 tmu_reg_offset;
static u32 tmu_reg_val;

static int param_tmu_reg_read_get(char *buf, const struct kernel_param *kp)
{
	exynos_acpm_tmu_reg_read(tmu_id, tmu_reg_offset, &tmu_reg_val);

	return sysfs_emit(buf, "0x%08x\n", tmu_reg_val);
}

static int param_tmu_reg_read_set(const char *val, const struct kernel_param *kp)
{
	int ret = 0;
	char **argv;
	int argc;

	argv = argv_split(GFP_KERNEL, val, &argc);
	if (!argv) {
		ret = -ENOMEM;
		pr_err("%s: memory allocation error", __func__);
		goto out;
	}
	if (argc != 2) {
		ret = -EINVAL;
		pr_err("%s: invalid args count", __func__);
	} else {
		ret = kstrtou8(argv[0], 16, &tmu_id);
		if (ret) {
			pr_err("%s: parse tmu_id error", __func__);
			goto out;
		} else if (tmu_id > TMU_END) {
			pr_err("%s: tmu_id input error", __func__);
			goto out;
		}

		ret = kstrtou16(argv[1], 16, &tmu_reg_offset);
		if (ret) {
			pr_err("%s: parse tmu_reg_offset error", __func__);
			goto out;
		} else if (tmu_reg_offset >
			   TMU_REG_PAST_TEMP1_0(TMU_P15_SENSOR)) {
			pr_err("%s: tmu_reg_offset input error", __func__);
			goto out;
		}
	}
out:
	argv_free(argv);
	return ret;
}

static const struct kernel_param_ops param_ops_tmu_reg_read = {
	.get = param_tmu_reg_read_get,
	.set = param_tmu_reg_read_set,
};

module_param_cb(tmu_reg_read, &param_ops_tmu_reg_read, NULL, 0644);
MODULE_PARM_DESC(tmu_reg_read,
		 "read tmu register: <tmu_id> <tmu_reg_offset>");

static int param_tmu_reg_write_set(const char *val, const struct kernel_param *kp)
{
	int ret = 0;
	char **argv;
	int argc;

	argv = argv_split(GFP_KERNEL, val, &argc);
	if (!argv) {
		ret = -ENOMEM;
		pr_err("%s: memory allocation error", __func__);
		goto out;
	}
	if (argc != 3) {
		ret = -EINVAL;
		pr_err("%s: invalid args count", __func__);
	} else {
		ret = kstrtou8(argv[0], 16, &tmu_id);
		if (ret) {
			pr_err("%s: parse tmu_id error", __func__);
			goto out;
		} else if (tmu_id > TMU_END) {
			pr_err("%s: tmu_id input error", __func__);
			goto out;
		}

		ret = kstrtou16(argv[1], 16, &tmu_reg_offset);
		if (ret) {
			pr_err("%s: parse tmu_reg_offset error", __func__);
			goto out;
		} else if (tmu_reg_offset >
			   TMU_REG_PAST_TEMP1_0(TMU_P15_SENSOR)) {
			pr_err("%s: tmu_reg_offset input error", __func__);
			goto out;
		}

		ret = kstrtou32(argv[2], 16, &tmu_reg_val);
		if (ret) {
			pr_err("%s: parse tmu_reg_val error", __func__);
			goto out;
		} else if (tmu_reg_val > U32_MAX) {
			pr_err("%s: tmu_reg_val input error", __func__);
			goto out;
		}

		exynos_acpm_tmu_reg_write(tmu_id, tmu_reg_offset,
					  tmu_reg_val);
	}
out:
	argv_free(argv);
	return ret;
}

static const struct kernel_param_ops param_ops_tmu_reg_write = {
	.set = param_tmu_reg_write_set,
};

module_param_cb(tmu_reg_write, &param_ops_tmu_reg_write, NULL, 0200);
MODULE_PARM_DESC(tmu_reg_write,
		 "write tmu register: <tmu_id> <tmu_reg_offset> <value>");

static int param_tmu_reg_dump_state(char *buf, const struct kernel_param *kp)
{
	int i,j;
	u16 offset;
	u32 val;
	int len = 0;

	if (suspended_count || atomic_read(&gs_tmu_in_suspend))
		return sysfs_emit(buf, "in tmu suspending..try again\n");

	for (i = 0; i < TMU_END; i++) {
		len += sysfs_emit_at(buf, len, "=======================\n");
		len += sysfs_emit_at(buf, len,
				     "tmu_id:0x%02x register dump start\n", i);

		offset = TMU_REG_CONTROL;
		exynos_acpm_tmu_reg_read(i, offset, &val);
		len += sysfs_emit_at(buf, len, "=======================\n");
		len += sysfs_emit_at(buf, len, "TMU_REG_CONTROL\n");
		len += sysfs_emit_at(buf, len, "=======================\n");
		len += sysfs_emit_at(buf, len,
				  "tmu_reg_offset:0x%04x --> ", offset);
		len += sysfs_emit_at(buf, len,
				  "tmu_reg_val:0x%08x\n", val);

		offset = TMU_REG_TMU_STATUS;
		exynos_acpm_tmu_reg_read(i, offset, &val);
		len += sysfs_emit_at(buf, len, "=======================\n");
		len += sysfs_emit_at(buf, len, "TMU_REG_TMU_STATUS\n");
		len += sysfs_emit_at(buf, len, "=======================\n");
		len += sysfs_emit_at(buf, len,
				  "tmu_reg_offset:0x%04x --> ", offset);
		len += sysfs_emit_at(buf, len,
				  "tmu_reg_val:0x%08x\n", val);

		len += sysfs_emit_at(buf, len, "=======================\n");
		len += sysfs_emit_at(buf, len, "TMU_REG_INTEN\n");
		len += sysfs_emit_at(buf, len, "=======================\n");
		for (j = 0; j <= TMU_P15_SENSOR; j++ ) {
			offset = TMU_REG_INTEN(j);
			exynos_acpm_tmu_reg_read(i, offset, &val);
			len += sysfs_emit_at(buf, len,
					  "tmu_reg_offset:0x%04x --> ", offset);
			len += sysfs_emit_at(buf, len,
					  "tmu_reg_val:0x%08x\n", val);
		}

		len += sysfs_emit_at(buf, len, "=======================\n");
		len += sysfs_emit_at(buf, len, "TMU_REG_INTPEND\n");
		len += sysfs_emit_at(buf, len, "=======================\n");
		for (j = 0; j <= TMU_P15_SENSOR; j++ ) {
			offset = TMU_REG_INTPEND(j);
			exynos_acpm_tmu_reg_read(i, offset, &val);
			len += sysfs_emit_at(buf, len,
					  "tmu_reg_offset:0x%04x --> ", offset);
			len += sysfs_emit_at(buf, len,
					  "tmu_reg_val:0x%08x\n", val);
		}
	}

	return len;
}

static const struct kernel_param_ops param_ops_tmu_reg_dump_state = {
	.get = param_tmu_reg_dump_state,
};

module_param_cb(tmu_reg_dump_state, &param_ops_tmu_reg_dump_state, NULL, 0444);
MODULE_PARM_DESC(tmu_reg_dump_state,
		 "tmu register dump about tmu state");

static int param_tmu_reg_dump_current_temp(char *buf,
					   const struct kernel_param *kp)
{
	int i,j;
	u16 offset;
	u32 val;
	int len = 0;

	for (i = 0; i < TMU_END; i++) {
		len += sysfs_emit_at(buf, len, "=======================\n");
		len += sysfs_emit_at(buf, len,
				     "tmu_id:0x%02x register dump start\n", i);

		len += sysfs_emit_at(buf, len, "=======================\n");
		len += sysfs_emit_at(buf, len, "TMU_REG_CURRENT_TEMP\n");
		len += sysfs_emit_at(buf, len, "=======================\n");
		for (j = 0; j <= TMU_P15_SENSOR; j++ ) {
			if ((j % 2) > 0)
				continue;

			offset = TMU_REG_CURRENT_TEMP(j);
			exynos_acpm_tmu_reg_read(i, offset, &val);
			len += sysfs_emit_at(buf, len,
					  "tmu_reg_offset:0x%04x --> ", offset);
			len += sysfs_emit_at(buf, len,
					  "tmu_reg_val:0x%08x\n", val);
		}
	}

	return len;
}

static const struct kernel_param_ops param_ops_tmu_reg_dump_current_temp = {
	.get = param_tmu_reg_dump_current_temp,
};

module_param_cb(tmu_reg_dump_current_temp, &param_ops_tmu_reg_dump_current_temp,
		NULL, 0444);
MODULE_PARM_DESC(tmu_reg_dump_current_temp,
		 "tmu register dump about sensor current temperature");

static int param_tmu_top_reg_dump_rise_thres(char *buf, const struct kernel_param *kp)
{
	int i;
	u16 offset;
	u32 val;
	int len = 0;

	len += sysfs_emit_at(buf, len, "=======================\n");
	len += sysfs_emit_at(buf, len,
			     "TMU_REG_THRESHOLD_TEMP_RISE7_6\n");
	len += sysfs_emit_at(buf, len, "=======================\n");
	for (i = 0; i <= TMU_P15_SENSOR; i++ ) {
		offset = TMU_REG_THRESHOLD_TEMP_RISE7_6(i);
		exynos_acpm_tmu_reg_read(TMU_TOP, offset, &val);
		len += sysfs_emit_at(buf, len,
				  "tmu_reg_offset:0x%04x --> ", offset);
		len += sysfs_emit_at(buf, len,
				  "tmu_reg_val:0x%08x\n", val);
	}

	len += sysfs_emit_at(buf, len, "=======================\n");
	len += sysfs_emit_at(buf, len,
			     "TMU_REG_THRESHOLD_TEMP_RISE5_4\n");
	len += sysfs_emit_at(buf, len, "=======================\n");
	for (i = 0; i <= TMU_P15_SENSOR; i++ ) {
		offset = TMU_REG_THRESHOLD_TEMP_RISE5_4(i);
		exynos_acpm_tmu_reg_read(TMU_TOP, offset, &val);
		len += sysfs_emit_at(buf, len,
				  "tmu_reg_offset:0x%04x --> ", offset);
		len += sysfs_emit_at(buf, len,
				  "tmu_reg_val:0x%08x\n", val);
	}

	len += sysfs_emit_at(buf, len, "=======================\n");
	len += sysfs_emit_at(buf, len,
			     "TMU_REG_THRESHOLD_TEMP_RISE3_2\n");
	len += sysfs_emit_at(buf, len, "=======================\n");
	for (i = 0; i <= TMU_P15_SENSOR; i++ ) {
		offset = TMU_REG_THRESHOLD_TEMP_RISE3_2(i);
		exynos_acpm_tmu_reg_read(TMU_TOP, offset, &val);
		len += sysfs_emit_at(buf, len,
				  "tmu_reg_offset:0x%04x --> ", offset);
		len += sysfs_emit_at(buf, len,
				  "tmu_reg_val:0x%08x\n", val);
	}

	len += sysfs_emit_at(buf, len, "=======================\n");
	len += sysfs_emit_at(buf, len,
			     "TMU_REG_THRESHOLD_TEMP_RISE1_0\n");
	len += sysfs_emit_at(buf, len, "=======================\n");
	for (i = 0; i <= TMU_P15_SENSOR; i++ ) {
		offset = TMU_REG_THRESHOLD_TEMP_RISE1_0(i);
		exynos_acpm_tmu_reg_read(TMU_TOP, offset, &val);
		len += sysfs_emit_at(buf, len,
				  "tmu_reg_offset:0x%04x --> ", offset);
		len += sysfs_emit_at(buf, len,
				  "tmu_reg_val:0x%08x\n", val);
	}

	return len;
}

static const struct kernel_param_ops param_ops_tmu_top_reg_dump_rise_thres = {
	.get = param_tmu_top_reg_dump_rise_thres,
};

module_param_cb(tmu_top_reg_dump_rise_thres,
		&param_ops_tmu_top_reg_dump_rise_thres, NULL, 0444);
MODULE_PARM_DESC(tmu_top_reg_dump_rise_thres,
		 "tmu top register dump about sensor rise threshold");

static int param_tmu_sub_reg_dump_rise_thres(char *buf, const struct kernel_param *kp)
{
	int i;
	u16 offset;
	u32 val;
	int len = 0;

	len += sysfs_emit_at(buf, len, "=======================\n");
	len += sysfs_emit_at(buf, len,
			     "TMU_REG_THRESHOLD_TEMP_RISE7_6\n");
	len += sysfs_emit_at(buf, len, "=======================\n");
	for (i = 0; i <= TMU_P15_SENSOR; i++ ) {
		offset = TMU_REG_THRESHOLD_TEMP_RISE7_6(i);
		exynos_acpm_tmu_reg_read(TMU_SUB, offset, &val);
		len += sysfs_emit_at(buf, len,
				  "tmu_reg_offset:0x%04x --> ", offset);
		len += sysfs_emit_at(buf, len,
				  "tmu_reg_val:0x%08x\n", val);
	}

	len += sysfs_emit_at(buf, len, "=======================\n");
	len += sysfs_emit_at(buf, len,
			     "TMU_REG_THRESHOLD_TEMP_RISE5_4\n");
	len += sysfs_emit_at(buf, len, "=======================\n");
	for (i = 0; i <= TMU_P15_SENSOR; i++ ) {
		offset = TMU_REG_THRESHOLD_TEMP_RISE5_4(i);
		exynos_acpm_tmu_reg_read(TMU_SUB, offset, &val);
		len += sysfs_emit_at(buf, len,
				  "tmu_reg_offset:0x%04x --> ", offset);
		len += sysfs_emit_at(buf, len,
				  "tmu_reg_val:0x%08x\n", val);
	}

	len += sysfs_emit_at(buf, len, "=======================\n");
	len += sysfs_emit_at(buf, len,
			     "TMU_REG_THRESHOLD_TEMP_RISE3_2\n");
	len += sysfs_emit_at(buf, len, "=======================\n");
	for (i = 0; i <= TMU_P15_SENSOR; i++ ) {
		offset = TMU_REG_THRESHOLD_TEMP_RISE3_2(i);
		exynos_acpm_tmu_reg_read(TMU_SUB, offset, &val);
		len += sysfs_emit_at(buf, len,
				  "tmu_reg_offset:0x%04x --> ", offset);
		len += sysfs_emit_at(buf, len,
				  "tmu_reg_val:0x%08x\n", val);
	}

	len += sysfs_emit_at(buf, len, "=======================\n");
	len += sysfs_emit_at(buf, len,
			     "TMU_REG_THRESHOLD_TEMP_RISE1_0\n");
	len += sysfs_emit_at(buf, len, "=======================\n");
	for (i = 0; i <= TMU_P15_SENSOR; i++ ) {
		offset = TMU_REG_THRESHOLD_TEMP_RISE1_0(i);
		exynos_acpm_tmu_reg_read(TMU_SUB, offset, &val);
		len += sysfs_emit_at(buf, len,
				  "tmu_reg_offset:0x%04x --> ", offset);
		len += sysfs_emit_at(buf, len,
				  "tmu_reg_val:0x%08x\n", val);
	}

	return len;
}

static const struct kernel_param_ops param_ops_tmu_sub_reg_dump_rise_thres = {
	.get = param_tmu_sub_reg_dump_rise_thres,
};

module_param_cb(tmu_sub_reg_dump_rise_thres,
		&param_ops_tmu_sub_reg_dump_rise_thres, NULL, 0444);
MODULE_PARM_DESC(tmu_sub_reg_dump_rise_thres,
		 "tmu sub register dump about sensor rise threshold");

static int param_tmu_top_reg_dump_fall_thres(char *buf, const struct kernel_param *kp)
{
	int i;
	u16 offset;
	u32 val;
	int len = 0;

	len += sysfs_emit_at(buf, len, "=======================\n");
	len += sysfs_emit_at(buf, len,
			     "TMU_REG_THRESHOLD_TEMP_FALL7_6\n");
	len += sysfs_emit_at(buf, len, "=======================\n");
	for (i = 0; i <= TMU_P15_SENSOR; i++ ) {
		offset = TMU_REG_THRESHOLD_TEMP_FALL7_6(i);
		exynos_acpm_tmu_reg_read(TMU_TOP, offset, &val);
		len += sysfs_emit_at(buf, len,
				  "tmu_reg_offset:0x%04x --> ", offset);
		len += sysfs_emit_at(buf, len,
				  "tmu_reg_val:0x%08x\n", val);
	}

	len += sysfs_emit_at(buf, len, "=======================\n");
	len += sysfs_emit_at(buf, len,
			     "TMU_REG_THRESHOLD_TEMP_FALL5_4\n");
	len += sysfs_emit_at(buf, len, "=======================\n");
	for (i = 0; i <= TMU_P15_SENSOR; i++ ) {
		offset = TMU_REG_THRESHOLD_TEMP_FALL5_4(i);
		exynos_acpm_tmu_reg_read(TMU_TOP, offset, &val);
		len += sysfs_emit_at(buf, len,
				  "tmu_reg_offset:0x%04x --> ", offset);
		len += sysfs_emit_at(buf, len,
				  "tmu_reg_val:0x%08x\n", val);
	}

	len += sysfs_emit_at(buf, len, "=======================\n");
	len += sysfs_emit_at(buf, len,
			     "TMU_REG_THRESHOLD_TEMP_FALL3_2\n");
	len += sysfs_emit_at(buf, len, "=======================\n");
	for (i = 0; i <= TMU_P15_SENSOR; i++ ) {
		offset = TMU_REG_THRESHOLD_TEMP_FALL3_2(i);
		exynos_acpm_tmu_reg_read(TMU_TOP, offset, &val);
		len += sysfs_emit_at(buf, len,
				  "tmu_reg_offset:0x%04x --> ", offset);
		len += sysfs_emit_at(buf, len,
				  "tmu_reg_val:0x%08x\n", val);
	}

	len += sysfs_emit_at(buf, len, "=======================\n");
	len += sysfs_emit_at(buf, len,
			     "TMU_REG_THRESHOLD_TEMP_FALL1_0\n");
	len += sysfs_emit_at(buf, len, "=======================\n");
	for (i = 0; i <= TMU_P15_SENSOR; i++ ) {
		offset = TMU_REG_THRESHOLD_TEMP_FALL1_0(i);
		exynos_acpm_tmu_reg_read(TMU_TOP, offset, &val);
		len += sysfs_emit_at(buf, len,
				  "tmu_reg_offset:0x%04x --> ", offset);
		len += sysfs_emit_at(buf, len,
				  "tmu_reg_val:0x%08x\n", val);
	}

	return len;
}

static const struct kernel_param_ops param_ops_tmu_top_reg_dump_fall_thres = {
	.get = param_tmu_top_reg_dump_fall_thres,
};

module_param_cb(tmu_top_reg_dump_fall_thres,
		&param_ops_tmu_top_reg_dump_fall_thres, NULL, 0444);
MODULE_PARM_DESC(tmu_top_reg_dump_fall_thres,
		 "tmu top register dump about sensor fall threshold");

static int param_tmu_sub_reg_dump_fall_thres(char *buf, const struct kernel_param *kp)
{
	int i;
	u16 offset;
	u32 val;
	int len = 0;

	len += sysfs_emit_at(buf, len, "=======================\n");
	len += sysfs_emit_at(buf, len,
			     "TMU_REG_THRESHOLD_TEMP_FALL7_6\n");
	len += sysfs_emit_at(buf, len, "=======================\n");
	for (i = 0; i <= TMU_P15_SENSOR; i++ ) {
		offset = TMU_REG_THRESHOLD_TEMP_FALL7_6(i);
		exynos_acpm_tmu_reg_read(TMU_SUB, offset, &val);
		len += sysfs_emit_at(buf, len,
				  "tmu_reg_offset:0x%04x --> ", offset);
		len += sysfs_emit_at(buf, len,
				  "tmu_reg_val:0x%08x\n", val);
	}

	len += sysfs_emit_at(buf, len, "=======================\n");
	len += sysfs_emit_at(buf, len,
			     "TMU_REG_THRESHOLD_TEMP_FALL5_4\n");
	len += sysfs_emit_at(buf, len, "=======================\n");
	for (i = 0; i <= TMU_P15_SENSOR; i++ ) {
		offset = TMU_REG_THRESHOLD_TEMP_FALL5_4(i);
		exynos_acpm_tmu_reg_read(TMU_SUB, offset, &val);
		len += sysfs_emit_at(buf, len,
				  "tmu_reg_offset:0x%04x --> ", offset);
		len += sysfs_emit_at(buf, len,
				  "tmu_reg_val:0x%08x\n", val);
	}

	len += sysfs_emit_at(buf, len, "=======================\n");
	len += sysfs_emit_at(buf, len,
			     "TMU_REG_THRESHOLD_TEMP_FALL3_2\n");
	len += sysfs_emit_at(buf, len, "=======================\n");
	for (i = 0; i <= TMU_P15_SENSOR; i++ ) {
		offset = TMU_REG_THRESHOLD_TEMP_FALL3_2(i);
		exynos_acpm_tmu_reg_read(TMU_SUB, offset, &val);
		len += sysfs_emit_at(buf, len,
				  "tmu_reg_offset:0x%04x --> ", offset);
		len += sysfs_emit_at(buf, len,
				  "tmu_reg_val:0x%08x\n", val);
	}

	len += sysfs_emit_at(buf, len, "=======================\n");
	len += sysfs_emit_at(buf, len,
			     "TMU_REG_THRESHOLD_TEMP_FALL1_0\n");
	len += sysfs_emit_at(buf, len, "=======================\n");
	for (i = 0; i <= TMU_P15_SENSOR; i++ ) {
		offset = TMU_REG_THRESHOLD_TEMP_FALL1_0(i);
		exynos_acpm_tmu_reg_read(TMU_SUB, offset, &val);
		len += sysfs_emit_at(buf, len,
				  "tmu_reg_offset:0x%04x --> ", offset);
		len += sysfs_emit_at(buf, len,
				  "tmu_reg_val:0x%08x\n", val);
	}

	return len;
}

static const struct kernel_param_ops param_ops_tmu_sub_reg_dump_fall_thres = {
	.get = param_tmu_sub_reg_dump_fall_thres,
};

module_param_cb(tmu_sub_reg_dump_fall_thres,
		&param_ops_tmu_sub_reg_dump_fall_thres, NULL, 0444);
MODULE_PARM_DESC(tmu_sub_reg_dump_fall_thres,
		 "tmu sub register dump about sensor fall threshold");

static void exynos_acpm_tmu_test_cp_call(bool mode)
{
	struct gs_tmu_data *devnode;

	if (mode) {
		list_for_each_entry(devnode, &dtm_dev_list, node) {
			disable_irq(devnode->irq);
		}
		exynos_acpm_tmu_set_cp_call();
	} else {
		exynos_acpm_tmu_set_resume();
		sync_kernel_acpm_timestamp();
		list_for_each_entry(devnode, &dtm_dev_list, node) {
			enable_irq(devnode->irq);
		}
	}
}

static int
emul_call_show(char *buf, const struct kernel_param *kp)
{
	return sysfs_emit(buf, "%d\n", exynos_acpm_tmu_is_test_mode());
}

static int emul_call_store(const char *buf, const struct kernel_param *kp)
{
	bool status = exynos_acpm_tmu_is_test_mode();
	bool enable;

	if (kstrtobool(buf, &enable))
		return -EINVAL;

	if (enable != status) {
		exynos_acpm_tmu_set_test_mode(enable);
		exynos_acpm_tmu_test_cp_call(enable);
	}

	return 0;
}

module_param_call(emul_call, emul_call_store, emul_call_show, NULL, 0600);

static int
log_print_show(char *buf, const struct kernel_param *kp)
{
	return sysfs_emit(buf, "%d\n", exynos_acpm_tmu_is_log_enabled());
}

static int
log_print_store(const char *buf, const struct kernel_param *kp)
{
	bool enable;

	if (kstrtobool(buf, &enable))
		return -EINVAL;

	exynos_acpm_tmu_enable_log(enable);

	return 0;
}

module_param_call(log_print, log_print_store, log_print_show, NULL, 0600);
#endif

#define PARAM_NAME_LENGTH	25

#if IS_ENABLED(CONFIG_ECT)
static int gs_tmu_ect_get_param(struct ect_pidtm_block *pidtm_block, char *name)
{
	int i;
	int param_value = -1;

	for (i = 0; i < pidtm_block->num_of_parameter; i++) {
		if (!strncasecmp(pidtm_block->param_name_list[i], name, PARAM_NAME_LENGTH)) {
			param_value = pidtm_block->param_value_list[i];
			break;
		}
	}

	return param_value;
}

static int gs_tmu_parse_ect(struct gs_tmu_data *data)
{
	struct thermal_zone_device *tz = data->tzd;
	int ntrips = 0;

	if (!tz)
		return -EINVAL;

	if (!data->use_pi_thermal) {
		/* if pi thermal not used */

		void *thermal_block;
		struct ect_ap_thermal_function *function;
		int i, temperature;
		int hotplug_threshold_temp = 0, hotplug_flag = 0;
		unsigned int freq;

		thermal_block = ect_get_block(BLOCK_AP_THERMAL);
		if (!thermal_block) {
			pr_err("Failed to get thermal block");
			return -EINVAL;
		}

		pr_info("%s thermal zone_name = %s\n", __func__, tz->type);

		function = ect_ap_thermal_get_function(thermal_block, tz->type);
		if (!function) {
			pr_err("Failed to get thermal block %s", tz->type);
			return -EINVAL;
		}

		ntrips = of_thermal_get_ntrips(tz);
		pr_info("Trip count parsed from ECT : %d, ntrips: %d, zone : %s",
			function->num_of_range, ntrips, tz->type);

		for (i = 0; i < function->num_of_range; ++i) {
			temperature = function->range_list[i].lower_bound_temperature;
			freq = function->range_list[i].max_frequency;

			tz->ops->set_trip_temp(tz, i, temperature  * MCELSIUS);

			pr_info("Parsed From ECT : [%d] Temperature : %d, frequency : %u\n",
				i, temperature, freq);

			if (function->range_list[i].flag != hotplug_flag) {
				if (function->range_list[i].flag != hotplug_flag) {
					hotplug_threshold_temp = temperature;
					hotplug_flag = function->range_list[i].flag;
					data->hotplug_out_threshold = temperature;

					if (i) {
						struct ect_ap_thermal_range range;
						unsigned int temperature;

						range = function->range_list[i - 1];
						temperature = range.lower_bound_temperature;
						data->hotplug_in_threshold = temperature;
					}

					pr_info("[ECT]hotplug_threshold : %d\n",
						hotplug_threshold_temp);
					pr_info("[ECT]hotplug_in_threshold : %d\n",
						data->hotplug_in_threshold);
					pr_info("[ECT]hotplug_out_threshold : %d\n",
						data->hotplug_out_threshold);
				}
			}

			if (hotplug_threshold_temp != 0)
				data->hotplug_enable = true;
			else
				data->hotplug_enable = false;
		}
	} else {
		void *block;
		struct ect_pidtm_block *pidtm_block;
		struct gs_pi_param *params;
		int i, temperature, value;
		int hotplug_out_threshold = 0, hotplug_in_threshold = 0, limited_frequency = 0;
		int limited_threshold = 0, limited_threshold_release = 0;

		block = ect_get_block(BLOCK_PIDTM);
		if (!block) {
			pr_err("Failed to get PIDTM block");
			return -EINVAL;
		}

		pr_info("%s %d thermal zone_name = %s\n", __func__, __LINE__, tz->type);

		pidtm_block = ect_pidtm_get_block(block, tz->type);
		if (!pidtm_block) {
			pr_err("Failed to get PIDTM block %s", tz->type);
			return -EINVAL;
		}

		ntrips = of_thermal_get_ntrips(tz);
		pr_info("Trip count parsed from ECT : %d, ntrips: %d, zone : %s",
			pidtm_block->num_of_temperature, ntrips, tz->type);

		for (i = 0; i < pidtm_block->num_of_temperature; ++i) {
			temperature = pidtm_block->temperature_list[i];

			tz->ops->set_trip_temp(tz, i, temperature  * MCELSIUS);

			pr_info("Parsed From ECT : [%d] Temperature : %d\n", i, temperature);
		}

		params = data->pi_param;

		value = gs_tmu_ect_get_param(pidtm_block, "k_po");
		if (value != -1) {
			pr_info("Parse from ECT k_po: %d\n", value);
			params->k_po = int_to_frac(value);
		} else {
			pr_err("Fail to parse k_po parameter\n");
		}

		value = gs_tmu_ect_get_param(pidtm_block, "k_pu");
		if (value != -1) {
			pr_info("Parse from ECT k_pu: %d\n", value);
			params->k_pu = int_to_frac(value);
		} else {
			pr_err("Fail to parse k_pu parameter\n");
		}

		value = gs_tmu_ect_get_param(pidtm_block, "k_i");
		if (value != -1) {
			pr_info("Parse from ECT k_i: %d\n", value);
			params->k_i = int_to_frac(value);
		} else {
			pr_err("Fail to parse k_i parameter\n");
		}

		value = gs_tmu_ect_get_param(pidtm_block, "i_max");
		if (value != -1) {
			pr_info("Parse from ECT i_max: %d\n", value);
			params->i_max = value;
		} else {
			pr_err("Fail to parse i_max parameter\n");
		}

		value = gs_tmu_ect_get_param(pidtm_block, "integral_cutoff");
		if (value != -1) {
			pr_info("Parse from ECT integral_cutoff: %d\n", value);
			params->integral_cutoff = value;
		} else {
			pr_err("Fail to parse integral_cutoff parameter\n");
		}

		value = gs_tmu_ect_get_param(pidtm_block, "p_control_t");
		if (value != -1) {
			pr_info("Parse from ECT p_control_t: %d\n", value);
			params->sustainable_power = value;
		} else {
			pr_err("Fail to parse p_control_t parameter\n");
		}

		value = gs_tmu_ect_get_param(pidtm_block, "hotplug_out_threshold");
		if (value != -1) {
			pr_info("Parse from ECT hotplug_out_threshold: %d\n", value);
			hotplug_out_threshold = value;
		}

		value = gs_tmu_ect_get_param(pidtm_block, "hotplug_in_threshold");
		if (value != -1) {
			pr_info("Parse from ECT hotplug_in_threshold: %d\n", value);
			hotplug_in_threshold = value;
		}

		value = gs_tmu_ect_get_param(pidtm_block, "limited_frequency");
		if (value != -1) {
			pr_info("Parse from ECT limited_frequency: %d\n", value);
			limited_frequency = value;
		}

		value = gs_tmu_ect_get_param(pidtm_block, "limited_threshold");
		if (value != -1) {
			pr_info("Parse from ECT limited_threshold: %d\n", value);
			limited_threshold = value * MCELSIUS;
			tz->ops->set_trip_temp(tz, 3, temperature  * MCELSIUS);
			data->limited_threshold = value;
		}

		value = gs_tmu_ect_get_param(pidtm_block, "limited_threshold_release");
		if (value != -1) {
			pr_info("Parse from ECT limited_threshold_release: %d\n", value);
			limited_threshold_release = value * MCELSIUS;
			data->limited_threshold_release = value;
		}

		if (hotplug_out_threshold != 0 && hotplug_in_threshold != 0) {
			data->hotplug_out_threshold = hotplug_out_threshold;
			data->hotplug_in_threshold = hotplug_in_threshold;
			data->hotplug_enable = true;
		} else {
			data->hotplug_enable = false;
		}

		if (limited_frequency) {
			data->limited_frequency = limited_frequency;
			data->limited = false;
		}
	}
	return 0;
};
#endif

#if IS_ENABLED(CONFIG_MALI_DEBUG_KERNEL_SYSFS)
struct gs_tmu_data *gpu_thermal_data;
#endif

int set_acpm_tj_power_status(enum tmu_grp_idx_t tzid, bool on)
{
	if ((tzid < 0) || (tzid >= TZ_END))
		return -EINVAL;
	exynos_acpm_tmu_ipc_set_power_status(tzid, on);
	return 0;
}
EXPORT_SYMBOL(set_acpm_tj_power_status);

extern void register_tz_id_ignore_genl(int tz_id);

static int parse_acpm_gov_common_dt(void)
{
	int ret;
	struct device_node *np;
	const char *buf;

	np = of_find_node_by_name(NULL, "acpm_gov");
	if (!np) {
		pr_err("GOV: No acpm_gov available\n");
		return -ENODEV;
	}

	ret = of_property_read_string(np, "work_affinity", &buf);
	if (!ret) {
		cpulist_parse(buf, &acpm_gov_common.thermal_pressure.work_affinity);
		cpumask_and(&acpm_gov_common.thermal_pressure.work_affinity, cpu_possible_mask,
			    &acpm_gov_common.thermal_pressure.work_affinity);
	} else
		cpumask_copy(&acpm_gov_common.thermal_pressure.work_affinity, cpu_possible_mask);

	ret = of_property_read_u32(np, "thermal_pressure_polling_delay_on",
				   &acpm_gov_common.thermal_pressure.polling_delay_on);
	if (ret < 0)
		acpm_gov_common.thermal_pressure.polling_delay_on =
			ACPM_GOV_THERMAL_PRESS_POLLING_DELAY_ON;

	ret = of_property_read_u32(np, "thermal_pressure_polling_delay_off",
				   &acpm_gov_common.thermal_pressure.polling_delay_off);
	if (ret < 0)
		acpm_gov_common.thermal_pressure.polling_delay_off =
			ACPM_GOV_THERMAL_PRESS_POLLING_DELAY_OFF;

	return 0;
}

static int gs_tmu_probe(struct platform_device *pdev)
{
	struct gs_tmu_data *data;
	int ret, val;
	bool is_first = false;
	struct temp_residency_stats_callbacks tr_cb_struct = {
		.set_thresholds = thermal_metrics_set_tr_thresholds,
		.get_thresholds = thermal_metrics_get_tr_thresholds,
		.get_stats = thermal_metrics_get_tr_stats,
		.reset_stats = thermal_metrics_reset_stats,
	};

	data = devm_kzalloc(&pdev->dev, sizeof(struct gs_tmu_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	platform_set_drvdata(pdev, data);
	mutex_init(&data->lock);

	ret = gs_map_dt_data(pdev);
	if (ret)
		goto err_sensor;

	spin_lock(&dev_list_spinlock);
	is_first = list_empty(&dtm_dev_list);
	list_add_tail(&data->node, &dtm_dev_list);
	num_of_devices++;
	spin_unlock(&dev_list_spinlock);

	if (is_first) {
		spin_lock_init(&acpm_gov_common.lock);
		if (acpm_gov_common.turn_on) {
			int i;
			spin_lock_init(&acpm_gov_common.thermal_pressure.lock);

			if (parse_acpm_gov_common_dt())
				goto err_dtm_dev_list;

			acpm_gov_common.thermal_pressure.state.switched_on = 0;
			for(i = 0; i < NR_PRESSURE_TZ; i++)
				acpm_gov_common.thermal_pressure.state.therm_press[i] = 0;

			if (acpm_ipc_get_buffer("GOV_DBG", (char **)&acpm_gov_common.sm_base,
						&acpm_gov_common.sm_size)) {
				pr_info("GOV: unavailable\n");
				goto err_dtm_dev_list;
			}
			if (acpm_gov_common.sm_size == 0xf10) {
				acpm_gov_common.buffer_version = 0x0;
			} else {
				memcpy_fromio(&acpm_gov_common.buffer_version,
					      acpm_gov_common.sm_base,
					      sizeof(acpm_gov_common.buffer_version));
				if ((acpm_gov_common.buffer_version >> 32) != ACPM_SM_BUFFER_VERSION_UPPER_32b) {
					pr_err("GOV: shared memory table version mismatch\n");
					goto err_dtm_dev_list;
				}
			}
			if (exynos_acpm_tmu_cb_init(&cb))
				goto err_dtm_dev_list;

			/* If the buffer version doesn't match, thermal pressure functionality is unavailable */
			acpm_gov_common.thermal_pressure.enabled = (ACPM_BUF_VER == EXPECT_BUF_VER);
		}
#if IS_ENABLED(CONFIG_EXYNOS_ACPM_THERMAL)
		exynos_acpm_tmu_init();
		exynos_acpm_tmu_set_init(&cap);
#endif
		if (acpm_gov_common.turn_on) {
			struct task_struct *thread;
			exynos_acpm_tmu_ipc_set_gov_debug_tracing_mode(
				acpm_gov_common.tracing_mode);

			if (acpm_gov_common.thermal_pressure.enabled) {
				kthread_init_worker(&acpm_gov_common.thermal_pressure.worker);
				thread = kthread_create(kthread_worker_fn,
							&acpm_gov_common.thermal_pressure.worker,
							"thermal_pressure");
				if (IS_ERR(thread)) {
					dev_err(&pdev->dev,
						"failed to create thermal pressure thread: %ld\n",
						PTR_ERR(thread));
					goto err_dtm_dev_list;
				}

				set_cpus_allowed_ptr(
					thread, &acpm_gov_common.thermal_pressure.work_affinity);

				sched_set_fifo(thread);

				kthread_init_work(&acpm_gov_common.thermal_pressure.switch_on_work,
						  acpm_switch_on_work);
				kthread_init_delayed_work(
					&acpm_gov_common.thermal_pressure.polling_work,
					thermal_pressure_polling);

				start_thermal_pressure_polling(0);

				wake_up_process(thread);
			}
		}
	}

	data->tzd = thermal_zone_of_sensor_register(&pdev->dev, 0, data, &gs_sensor_ops);
	if (IS_ERR(data->tzd)) {
		ret = PTR_ERR(data->tzd);
		dev_err(&pdev->dev, "Failed to register sensor: %d\n", ret);
		goto err_dtm_dev_list;
	}

	thermal_zone_device_disable(data->tzd);

#if IS_ENABLED(CONFIG_ECT)
	if (!of_property_read_bool(pdev->dev.of_node, "ect_nouse"))
		gs_tmu_parse_ect(data);

	if (data->limited_frequency) {
		exynos_pm_qos_add_request(&data->thermal_limit_request,
					  PM_QOS_CLUSTER2_FREQ_MAX,
					  PM_QOS_CLUSTER2_FREQ_MAX_DEFAULT_VALUE);
	}
#endif

	ret = gs_tmu_initialize(pdev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to initialize TMU\n");
		goto err_thermal;
	}

	if (!acpm_gov_common.turn_on) {
		ret = devm_request_irq(&pdev->dev, data->irq, gs_tmu_irq, IRQF_SHARED,
				       dev_name(&pdev->dev), data);
		if (ret) {
			dev_err(&pdev->dev, "Failed to request irq: %d\n", data->irq);
			goto err_thermal;
		}

		ret = gs_tmu_irq_work_init(pdev);
		if (ret) {
			dev_err(&pdev->dev, "cannot gs tmu interrupt work initialize\n");
			goto err_thermal;
		}

		if (data->use_pi_thermal)
			kthread_init_delayed_work(&data->pi_work, gs_pi_polling);
	} else {
		disable_irq_nosync(data->irq);
	}

	gs_tmu_control(pdev, true);

	if (data->hotplug_enable || data->pause_enable)
		pause_stats_setup(data);

	if (data->hardlimit_enable)
		hard_limit_stats_setup(data);

	get_control_trips(data);

	if (data->use_pi_thermal) {
		reset_pi_params(data);
		if (!acpm_gov_common.turn_on)
			start_pi_polling(data, 0);
	}

	if (acpm_gov_common.turn_on) {
		if (data->acpm_gov_params.fields.enable) {
			int tzid = data->id;
			u32 control_temp_step = data->control_temp_step;

			data->acpm_gov_params.fields.ctrl_temp_idx =
				data->trip_control_temp;
			data->acpm_gov_params.fields.switch_on_temp_idx =
				data->trip_switch_on;

			//sending an IPC to setup GOV param and control temperature step
			exynos_acpm_tmu_ipc_set_gov_config(tzid, data->acpm_gov_params.qword);
			exynos_acpm_tmu_ipc_set_control_temp_step(tzid, control_temp_step);

			if (exynos_acpm_tmu_ipc_set_gov_tz_time_windows(
				    data->id, data->polling_delay_on,
				    data->thermal_pressure_time_window)) {
				/* falling back to original interface as
				 * the new individual timer feature is unavailable
				 */
				exynos_acpm_tmu_ipc_set_gov_time_windows(
					ACPM_GOV_TIMER_INTERVAL_MS_DEFAULT,
					ACPM_GOV_THERMAL_PRESS_WINDOW_MS_DEFAULT);
			}
		}
	} else {
		thermal_zone_device_enable(data->tzd);
	}
	/* compatibility check */
	ret = exynos_acpm_tmu_ipc_get_tr_num_thresholds(data->id, &val);
	if (!ret) {
		data->tr_handle = register_temp_residency_stats(data->tmu_name, "tmu");
		register_temp_residency_stats_callbacks(data->tr_handle, &tr_cb_struct);
	}

	if (is_first) {
		sync_kernel_acpm_timestamp();
		register_pm_notifier(&gs_tmu_pm_nb);
	}

	if (data->use_pi_thermal) {
		exynos_acpm_tmu_ipc_set_pi_param(data->id, K_PO, frac_to_int(data->pi_param->k_po));
		exynos_acpm_tmu_ipc_set_pi_param(data->id, K_PU, frac_to_int(data->pi_param->k_pu));
		exynos_acpm_tmu_ipc_set_pi_param(data->id, K_I, frac_to_int(data->pi_param->k_i));
		exynos_acpm_tmu_ipc_set_pi_param(data->id, I_MAX, frac_to_int(data->pi_param->i_max));
		data->acpm_pi_enable = true;
		/* Enabling PID governor should be performed at last
		 * after all the PID param configuration is complete
		 */
		exynos_acpm_tmu_ipc_set_pi_param(data->id, PI_ENABLE, data->acpm_pi_enable);
	} else {
		data->acpm_pi_enable = false;
	}

#if IS_ENABLED(CONFIG_MALI_DEBUG_KERNEL_SYSFS)
	if (data->id == EXYNOS_GPU_TMU_GRP_ID)
		gpu_thermal_data = data;
#endif

#if IS_ENABLED(CONFIG_ISP_THERMAL)
	if (!strncmp(data->tmu_name, "ISP", 3))
		exynos_isp_cooling_init();
#endif

	spin_lock(&dev_list_spinlock);
	cpumask_or(&tmu_enabled_mask, &tmu_enabled_mask, &data->mapped_cpus);
	spin_unlock(&dev_list_spinlock);

	register_tz_id_ignore_genl(data->tzd->id);

	return 0;

err_thermal:
	thermal_zone_of_sensor_unregister(&pdev->dev, data->tzd);
err_dtm_dev_list:
	spin_lock(&dev_list_spinlock);
	list_del(&data->node);
	spin_unlock(&dev_list_spinlock);
err_sensor:
	return ret;
}

static int gs_tmu_remove(struct platform_device *pdev)
{
	return -EBUSY;
}

#if IS_ENABLED(CONFIG_PM_SLEEP)
static int gs_tmu_suspend(struct device *dev)
{
#if IS_ENABLED(CONFIG_EXYNOS_ACPM_THERMAL)
	struct platform_device *pdev = to_platform_device(dev);
	struct gs_tmu_data *data = platform_get_drvdata(pdev);

	suspended_count++;
	if (!acpm_gov_common.turn_on)
		disable_irq(data->irq);

	if (data->hotplug_enable)
		kthread_flush_work(&data->hotplug_work);
	if (data->pause_enable)
		kthread_flush_work(&data->pause_work);
	if (!acpm_gov_common.turn_on)
		kthread_flush_work(&data->irq_work);

	gs_tmu_control(pdev, false);
	if (suspended_count == num_of_devices) {
		if (acpm_gov_common.turn_on) {
			if (acpm_gov_common.tracing_buffer_flush_pending == true) {
				acpm_gov_common.last_ts = 0;
				acpm_gov_common.tracing_buffer_flush_pending = false;
			}
			if (acpm_gov_common.thermal_pressure.enabled)
				kthread_cancel_work_sync(
					&acpm_gov_common.thermal_pressure.switch_on_work);
		}
		exynos_acpm_tmu_set_suspend(false);
		pr_info("%s: TMU suspend\n", __func__);
	}
#endif
	return 0;
}

static int gs_tmu_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
#if IS_ENABLED(CONFIG_EXYNOS_ACPM_THERMAL)
	struct gs_tmu_data *data = platform_get_drvdata(pdev);
	int temp, stat;

	if (suspended_count == num_of_devices) {
		if (acpm_gov_common.turn_on)
			if (acpm_gov_common.tracing_mode == ACPM_GOV_DEBUG_MODE_BULK)
				acpm_gov_common.tracing_buffer_flush_pending = true;
		exynos_acpm_tmu_set_resume();
		sync_kernel_acpm_timestamp();
	}

	gs_tmu_control(pdev, true);

	exynos_acpm_tmu_set_read_temp(data->id, &temp, &stat);

	pr_info("%s: thermal zone %d temp %d stat %d\n", __func__, data->tzd->id, temp, stat);

	if (!acpm_gov_common.turn_on)
		enable_irq(data->irq);
	suspended_count--;

	if (!acpm_gov_common.turn_on) {
		struct cpumask mask;
		cpumask_and(&mask, cpu_possible_mask, &data->tmu_work_affinity);
		set_cpus_allowed_ptr(data->thermal_worker.task, &mask);
	}

	if (!suspended_count)
		pr_info("%s: TMU resume complete\n", __func__);
#endif

	return 0;
}

static SIMPLE_DEV_PM_OPS(gs_tmu_pm,
			 gs_tmu_suspend, gs_tmu_resume);
#define EXYNOS_TMU_PM	(&gs_tmu_pm)
#else
#define EXYNOS_TMU_PM	NULL
#endif

static struct platform_driver gs_tmu_driver = {
	.driver = {
		.name   = "gs-tmu",
		.dev_groups = gs_tmu_groups,
		.pm     = EXYNOS_TMU_PM,
		.of_match_table = gs_tmu_match,
		.suppress_bind_attrs = true,
		.probe_type = PROBE_FORCE_SYNCHRONOUS,
	},
	.probe = gs_tmu_probe,
	.remove	= gs_tmu_remove,
};

module_platform_driver(gs_tmu_driver);

MODULE_DESCRIPTION("GS TMU Driver");
MODULE_AUTHOR("Hyeonseong Gil <hs.gil@samsung.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:gs-tmu");
