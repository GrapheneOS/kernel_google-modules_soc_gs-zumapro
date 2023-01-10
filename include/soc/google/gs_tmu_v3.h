/* SPDX-License-Identifier: GPL-2.0-only
 *
 * gs_tmu_v3.h - Samsung GS TMU (Thermal Management Unit)
 *
 *  Copyright (C) 2019 Samsung Electronics
 *  Hyeonseong Gil <hs.gill@samsung.com>
 */

#ifndef _GS_TMU_V3_H
#define _GS_TMU_V3_H
#include <linux/kthread.h>
#include <soc/google/exynos_pm_qos.h>
#include <soc/google/exynos-cpuhp.h>

#define MCELSIUS        1000

struct gs_pi_param {
	s64 err_integral;
	int trip_switch_on;
	int trip_control_temp;

	u32 sustainable_power;
	s32 k_po;
	s32 k_pu;
	s32 k_i;
	s32 i_max;
	s32 integral_cutoff;

	int polling_delay_on;
	int polling_delay_off;

	bool switched_on;
};

enum pi_param {
	SUSTAINABLE_POWER = 0,
	K_PO = 1,
	K_PU = 2,
	K_I = 3,
	I_MAX = 4,
	INTEGRAL_CUTOFF = 5,
	PI_ENABLE = 6
};

#define STEPWISE_GAIN_MIN 0
#define STEPWISE_GAIN_MAX 5
#define ACPM_GOV_TIMER_INTERVAL_MS 50
#define INTEGRAL_THRESH_MIN 0
#define INTEGRAL_THRESH_MAX 255
struct acpm_gov_params_st {
	u8 ctrl_temp_idx;
	u8 switch_on_temp_idx;
	u8 irq_stepwise_gain;
	u8 timer_stepwise_gain;
	u8 integral_thresh;
	u8 enable;
	u8 reserved[2];
};

union acpm_gov_params_un {
	struct acpm_gov_params_st fields;
	u64 qword;
};

enum acpm_gov_debug_mode_enum {
	ACPM_GOV_DEBUG_MODE_DISABLED,
	ACPM_GOV_DEBUG_MODE_BULK,
	ACPM_GOV_DEBUG_MODE_HIGH_OVERHEAD,
	ACPM_GOV_DEBUG_MODE_INVALID,
};

struct acpm_gov_common {
	u64 kernel_ts;
	u64 acpm_ts;
	u64 last_ts;
	void __iomem *sm_base;
	u32 sm_size;
	enum acpm_gov_debug_mode_enum tracing_mode;
	u8 timer_interval;
	bool tracing_buffer_flush_pending;
	bool turn_on;
	u64 buffer_version;
};

#define TRIP_LEVEL_NUM 8

/**
 * struct gs_tmu_data : A structure to hold the private data of the TMU
	driver
 * @id: identifier of the one instance of the TMU controller.
 * @base: base address of the single instance of the TMU controller.
 * @irq: irq number of the TMU controller.
 * @soc: id of the SOC type.
 * @irq_work: pointer to the irq work structure.
 * @lock: lock to implement synchronization.
 * @regulator: pointer to the TMU regulator structure.
 * @reg_conf: pointer to structure to register with core thermal.
 * @ntrip: number of supported trip points.
 * @tmu_initialize: SoC specific TMU initialization method
 * @tmu_control: SoC specific TMU control method
 * @tmu_read: SoC specific TMU temperature read method
 * @tmu_set_emulation: SoC specific TMU emulation setting method
 * @tmu_clear_irqs: SoC specific TMU interrupts clearing method
 */
struct gs_tmu_data {
	int id;
	/* Throttle hotplug related variables */
	bool pause_enable;
	unsigned int tmu_type;
	int pause_threshold;
	int resume_threshold;
	bool hardlimit_enable;
	int hardlimit_threshold;
	int hardlimit_clr_threshold;
	unsigned int hardlimit_cooling_state;
	unsigned long max_cdev;
	bool hotplug_enable;
	int hotplug_in_threshold;
	int hotplug_out_threshold;
	bool cpu_hw_throttling_enable;
	int cpu_hw_throttling_trigger_threshold;
	int cpu_hw_throttling_clr_threshold;
	int ppm_clr_throttle_level;
	int ppm_throttle_level;
	int mpmm_clr_throttle_level;
	int mpmm_throttle_level;
	int limited_frequency;
	int limited_threshold;
	int limited_threshold_release;
	struct exynos_pm_qos_request thermal_limit_request;
	bool limited;
	void __iomem *base;
	int irq;
	struct kthread_worker hardlimit_worker;
	struct kthread_worker thermal_worker;
	struct kthread_worker pause_worker;
	struct kthread_worker cpu_hw_throttle_worker;
	struct kthread_work irq_work;
	struct kthread_work pause_work;
	struct kthread_work hardlimit_work;
	struct kthread_work hotplug_work;
	struct kthread_work cpu_hw_throttle_work;
	struct kthread_delayed_work cpu_hw_throttle_init_work;
	struct mutex lock;			/* lock to protect gs tmu */
	struct thermal_zone_device *tzd;
	struct bcl_device *bcl_dev;
	unsigned int ntrip;
	bool enabled;
	struct thermal_cooling_device *cool_dev;
	struct list_head node;
	char tmu_name[THERMAL_NAME_LENGTH + 1];
	struct device_node *np;
	bool is_paused;
	bool is_hardlimited;
	bool is_cpu_hotplugged_out;
	bool is_cpu_hw_throttled;
	int temperature;
	bool use_pi_thermal;
	struct kthread_delayed_work pi_work;
	struct gs_pi_param *pi_param;
	struct cpumask pause_cpus;
	struct cpumask hotplug_cpus;
	struct cpumask tmu_work_affinity;
	struct cpumask hotplug_work_affinity;
	char cpuhp_name[CPUHP_USER_NAME_LEN + 1];
	void *disable_stats;
	void *hardlimit_stats;
	atomic64_t trip_counter[TRIP_LEVEL_NUM];
	union acpm_gov_params_un acpm_gov_params;
	u32 fvp_get_target_freq;
};

enum throttling_stats_type {
	DISABLE_STATS = 0,
	HARDLIMIT_STATS,
};

struct throttling_stats {
	spinlock_t lock;
	int stats_type;
	unsigned int disable_total_count;
	unsigned int disable_state;
	unsigned int hardlimit_total_count;
	unsigned int hardlimit_state;
	ktime_t last_time;
	ktime_t *disable_time_in_state;
	ktime_t *hardlimit_time_in_state;
};

#define TMU_SENSOR_PROBE_NUM 16

enum tmu_zone_t {
	TMU_TOP = 0,
	TMU_SUB = 1,
	TMU_END = 2,
};

enum tmu_sensor_t {
	TMU_P0_SENSOR = 0,
	TMU_P1_SENSOR = 1,
	TMU_P2_SENSOR = 2,
	TMU_P3_SENSOR = 3,
	TMU_P4_SENSOR = 4,
	TMU_P5_SENSOR = 5,
	TMU_P6_SENSOR = 6,
	TMU_P7_SENSOR = 7,
	TMU_P8_SENSOR = 8,
	TMU_P9_SENSOR = 9,
	TMU_P10_SENSOR = 10,
	TMU_P11_SENSOR = 11,
	TMU_P12_SENSOR = 12,
	TMU_P13_SENSOR = 13,
	TMU_P14_SENSOR = 14,
	TMU_P15_SENSOR = 15,
};

#define TMU_P0_SENSOR_MASK (1 << TMU_P0_SENSOR)
#define TMU_P1_SENSOR_MASK (1 << TMU_P1_SENSOR)
#define TMU_P2_SENSOR_MASK (1 << TMU_P2_SENSOR)
#define TMU_P3_SENSOR_MASK (1 << TMU_P3_SENSOR)
#define TMU_P4_SENSOR_MASK (1 << TMU_P4_SENSOR)
#define TMU_P5_SENSOR_MASK (1 << TMU_P5_SENSOR)
#define TMU_P6_SENSOR_MASK (1 << TMU_P6_SENSOR)
#define TMU_P7_SENSOR_MASK (1 << TMU_P7_SENSOR)
#define TMU_P8_SENSOR_MASK (1 << TMU_P8_SENSOR)
#define TMU_P9_SENSOR_MASK (1 << TMU_P9_SENSOR)
#define TMU_P10_SENSOR_MASK (1 << TMU_P10_SENSOR)
#define TMU_P11_SENSOR_MASK (1 << TMU_P11_SENSOR)
#define TMU_P12_SENSOR_MASK (1 << TMU_P12_SENSOR)
#define TMU_P13_SENSOR_MASK (1 << TMU_P13_SENSOR)
#define TMU_P14_SENSOR_MASK (1 << TMU_P14_SENSOR)
#define TMU_P15_SENSOR_MASK (1 << TMU_P15_SENSOR)

#if defined(CONFIG_SOC_GS101)
#define TMU_REG_TRIMINFO_CONFIG			(0)
#define TMU_REG_TRIMINFO_0			(0x0010)
#define TMU_REG_TRIMINFO(p)			((p) * 4 + TMU_REG_TRIMINFO_0)
#define TMU_REG_CONTROL				(0x0050)
#define TMU_REG_CONTROL1			(0x0054)
#define TMU_REG_AVG_CONTROL			(0x0058)
#define TMU_REG_TMU_TRIM0			(0x005C)
#define TMU_REG_PROBE_EN_CON			(0x0060)
#define TMU_REG_CPU_PROBE_REMAP			(0x0068)
#define TMU_REG_SAMPLING_INTERVAL		(0x0070)
#define TMU_REG_COUNTER_VALUE0			(0x0074)
#define TMU_REG_COUNTER_VALUE1			(0x0078)
#define TMU_REG_COUNTER_VALUE2			(0x007C)
#define TMU_REG_TMU_STATUS			(0x0080)
#define TMU_REG_CURRENT_TEMP1_0			(0x0084)
#define TMU_REG_CURRENT_TEMP(p)			((p / 2) * 0x4 + TMU_REG_CURRENT_TEMP1_0)
#define TMU_REG_EMUL_CON			(0x00B0)
#define TMU_REG_P0_THRESHOLD_TEMP_RISE7_6	(0x00D0)
#define TMU_REG_THRESHOLD_TEMP_RISE7_6(p)	((p) * 0x50 + TMU_REG_P0_THRESHOLD_TEMP_RISE7_6)
#define TMU_REG_P0_THRESHOLD_TEMP_RISE5_4	(0x00D4)
#define TMU_REG_THRESHOLD_TEMP_RISE5_4(p)	((p) * 0x50 + TMU_REG_P0_THRESHOLD_TEMP_RISE5_4)
#define TMU_REG_P0_THRESHOLD_TEMP_RISE3_2	(0x00D8)
#define TMU_REG_THRESHOLD_TEMP_RISE3_2(p)	((p) * 0x50 + TMU_REG_P0_THRESHOLD_TEMP_RISE3_2)
#define TMU_REG_P0_THRESHOLD_TEMP_RISE1_0	(0x00DC)
#define TMU_REG_THRESHOLD_TEMP_RISE1_0(p)	((p) * 0x50 + TMU_REG_P0_THRESHOLD_TEMP_RISE1_0)
#define TMU_REG_P0_THRESHOLD_TEMP_FALL7_6	(0x00E0)
#define TMU_REG_THRESHOLD_TEMP_FALL7_6(p)	((p) * 0x50 + TMU_REG_P0_THRESHOLD_TEMP_FALL7_6)
#define TMU_REG_P0_THRESHOLD_TEMP_FALL5_4	(0x00E4)
#define TMU_REG_THRESHOLD_TEMP_FALL5_4(p)	((p) * 0x50 + TMU_REG_P0_THRESHOLD_TEMP_FALL5_4)
#define TMU_REG_P0_THRESHOLD_TEMP_FALL3_2	(0x00E8)
#define TMU_REG_THRESHOLD_TEMP_FALL3_2(p)	((p) * 0x50 + TMU_REG_P0_THRESHOLD_TEMP_FALL3_2)
#define TMU_REG_P0_THRESHOLD_TEMP_FALL1_0	(0x00EC)
#define TMU_REG_THRESHOLD_TEMP_FALL1_0(p)	((p) * 0x50 + TMU_REG_P0_THRESHOLD_TEMP_FALL1_0)
#define TMU_REG_P0_INTEN			(0x00F0)
#define TMU_REG_INTEN(p)			((p) * 0x50 + TMU_REG_P0_INTEN)
#define TMU_REG_P0_INTPEND			(0x00F8)
#define TMU_REG_INTPEND(p)			((p) * 0x50 + TMU_REG_P0_INTPEND)
#define TMU_REG_INTPEND_RISE_MASK(l)		(1 << (l))
#define TMU_REG_P0_PAST_TEMP1_0			(0x0100)
#define TMU_REG_PAST_TEMP1_0(p)			((p) * 0x50 + TMU_REG_P0_PAST_TEMP1_0)
#define TMU_REG_P0_PAST_TEMP3_2			(0x0104)
#define TMU_REG_PAST_TEMP3_2(p)			((p) * 0x50 + TMU_REG_P0_PAST_TEMP3_2)
#define TMU_REG_P0_PAST_TEMP5_4			(0x0108)
#define TMU_REG_PAST_TEMP5_4(p)			((p) * 0x50 + TMU_REG_P0_PAST_TEMP5_4)
#define TMU_REG_P0_PAST_TEMP7_6			(0x010C)
#define TMU_REG_PAST_TEMP7_6(p)			((p) * 0x50 + TMU_REG_P0_PAST_TEMP7_6)
#define TMU_REG_P0_PAST_TEMP9_8			(0x0110)
#define TMU_REG_PAST_TEMP9_8(p)			((p) * 0x50 + TMU_REG_P0_PAST_TEMP9_8)
#define TMU_REG_P0_PAST_TEMP11_10		(0x0114)
#define TMU_REG_PAST_TEMP11_10(p)		((p) * 0x50 + TMU_REG_P0_PAST_TEMP11_10)
#define TMU_REG_P0_PAST_TEMP13_12		(0x0118)
#define TMU_REG_PAST_TEMP13_12(p)		((p) * 0x50 + TMU_REG_P0_PAST_TEMP13_12)
#define TMU_REG_P0_PAST_TEMP15_14		(0x011C)
#define TMU_REG_PAST_TEMP15_14(p)		((p) * 0x50 + TMU_REG_P0_PAST_TEMP15_14)

#elif defined(CONFIG_SOC_ZUMA)
#define TMU_REG_TRIMINFO_CONFIG			(0)
#define TMU_REG_TRIMINFO_0			(0x0010)
#define TMU_REG_TRIMINFO(p)			((p) * 4 + TMU_REG_TRIMINFO_0)
#define TMU_REG_CONTROL				(0x0090)
#define TMU_REG_CONTROL1			(0x0094)
#define TMU_REG_AVG_CONTROL			(0x0098)
#define TMU_REG_TMU_TRIM0			(0x009C)
#define TMU_REG_PROBE_EN_CON			(0x00A0)
#define TMU_REG_SAMPLING_INTERVAL		(0x00A4)
#define TMU_REG_COUNTER_VALUE			(0x00A8)
#define TMU_REG_TMU_STATUS			(0x00AC)
#define TMU_REG_CURRENT_TEMP1_0			(0x00C0)
#define TMU_REG_CURRENT_TEMP(p)			((p / 2) * 0x4 + TMU_REG_CURRENT_TEMP1_0)
#define TMU_REG_EMUL_CON			(0x00B8)
#define TMU_REG_P0_THRESHOLD_TEMP_RISE7_6	(0x0100)
#define TMU_REG_THRESHOLD_TEMP_RISE7_6(p)	((p) * 0x40 + TMU_REG_P0_THRESHOLD_TEMP_RISE7_6)
#define TMU_REG_P0_THRESHOLD_TEMP_RISE5_4	(0x0104)
#define TMU_REG_THRESHOLD_TEMP_RISE5_4(p)	((p) * 0x40 + TMU_REG_P0_THRESHOLD_TEMP_RISE5_4)
#define TMU_REG_P0_THRESHOLD_TEMP_RISE3_2	(0x0108)
#define TMU_REG_THRESHOLD_TEMP_RISE3_2(p)	((p) * 0x40 + TMU_REG_P0_THRESHOLD_TEMP_RISE3_2)
#define TMU_REG_P0_THRESHOLD_TEMP_RISE1_0	(0x010C)
#define TMU_REG_THRESHOLD_TEMP_RISE1_0(p)	((p) * 0x40 + TMU_REG_P0_THRESHOLD_TEMP_RISE1_0)
#define TMU_REG_P0_THRESHOLD_TEMP_FALL7_6	(0x0110)
#define TMU_REG_THRESHOLD_TEMP_FALL7_6(p)	((p) * 0x40 + TMU_REG_P0_THRESHOLD_TEMP_FALL7_6)
#define TMU_REG_P0_THRESHOLD_TEMP_FALL5_4	(0x0114)
#define TMU_REG_THRESHOLD_TEMP_FALL5_4(p)	((p) * 0x40 + TMU_REG_P0_THRESHOLD_TEMP_FALL5_4)
#define TMU_REG_P0_THRESHOLD_TEMP_FALL3_2	(0x0118)
#define TMU_REG_THRESHOLD_TEMP_FALL3_2(p)	((p) * 0x40 + TMU_REG_P0_THRESHOLD_TEMP_FALL3_2)
#define TMU_REG_P0_THRESHOLD_TEMP_FALL1_0	(0x011C)
#define TMU_REG_THRESHOLD_TEMP_FALL1_0(p)	((p) * 0x40 + TMU_REG_P0_THRESHOLD_TEMP_FALL1_0)
#define TMU_REG_P0_INTEN			(0x0120)
#define TMU_REG_INTEN(p)			((p) * 0x40 + TMU_REG_P0_INTEN)
#define TMU_REG_P0_INTPEND			(0x0128)
#define TMU_REG_INTPEND(p)			((p) * 0x40 + TMU_REG_P0_INTPEND)
#define TMU_REG_INTPEND_RISE_MASK(l)		(1 << (l))
#define TMU_REG_P0_PAST_TEMP1_0			(0x0130)
#define TMU_REG_PAST_TEMP1_0(p)			((p) * 0x40 + TMU_REG_P0_PAST_TEMP1_0)
#define TMU_REG_P0_PAST_TEMP3_2			(0x0134)
#define TMU_REG_PAST_TEMP3_2(p)			((p) * 0x40 + TMU_REG_P0_PAST_TEMP3_2)
#define TMU_REG_P0_PAST_TEMP5_4			(0x0138)
#define TMU_REG_PAST_TEMP5_4(p)			((p) * 0x40 + TMU_REG_P0_PAST_TEMP5_4)
#define TMU_REG_P0_PAST_TEMP7_6			(0x013C)
#define TMU_REG_PAST_TEMP7_6(p)			((p) * 0x40 + TMU_REG_P0_PAST_TEMP7_6)
#endif

enum thermal_feature {
	CPU_THROTTLE = 0,
	HARD_LIMIT = 1,
	HOTPLUG = 2,
	PAUSE = 3,
};

struct sensor_data {
	enum tmu_sensor_t probe_id;
};

struct thermal_zone_data {
	enum tmu_zone_t tmu_zone_id;
	u16 sensors_mask;
	struct sensor_data sensors[TMU_SENSOR_PROBE_NUM];
	u16 sensor_cnt;
};

enum thermal_pause_state {
	THERMAL_RESUME = 0,
	THERMAL_SUSPEND,
};

typedef int (*tpu_pause_cb)(enum thermal_pause_state action, void *data);

void register_tpu_thermal_pause_cb(tpu_pause_cb tpu_cb, void *data);

#define ACPM_SM_BUFFER_VERSION_UPPER_32b 0x5A554D41ULL
#define GOV_TRACE_DATA_LEN 240
#define ACPM_SYSTICK_NUMERATOR 20
#define ACPM_SYSTICK_FRACTIONAL_DENOMINATOR 3
struct gov_data {
	u64 timestamp;
	u32 freq_req;
	u32 reserved;
};

struct curr_state {
	u8 cdev_state;
	u8 temperature;
	u8 ctrl_temp;
	u8 reserved;
};

struct buffered_curr_state {
	u64 timestamp;
	u8 tzid;
	u8 cdev_state;
	u8 temperature;
	u8 ctrl_temp;
	u8 reserved[4];
};

struct gov_trace_data_struct {
	struct buffered_curr_state buffered_curr_state[GOV_TRACE_DATA_LEN];
	struct curr_state curr_state[7];
};

#endif /* _GS_TMU_V3_H */
