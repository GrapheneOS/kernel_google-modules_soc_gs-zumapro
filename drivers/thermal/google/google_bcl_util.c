// SPDX-License-Identifier: GPL-2.0
/*
 * google_bcl_core.c Google bcl driver
 *
 * Copyright (c) 2022, Google LLC. All rights reserved.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/mfd/samsung/s2mpg1415.h>
#include <linux/mfd/samsung/s2mpg1415-register.h>
#include <soc/google/exynos-cpupm.h>
#include <soc/google/exynos-pm.h>
#include <soc/google/exynos-pmu-if.h>
#include <soc/google/bcl.h>

#define CPU0_CLUSTER_MIN 0
#define CPU1_CLUSTER_MIN 4
#define CPU2_CLUSTER_MIN 8

const char * const clk_stats_source[] = {
	"cpu0", "cpu1", "cpu2", "tpu", "gpu"
};

const unsigned int subsystem_pmu[] = {
	PMU_ALIVE_CPU0_OUT,
	PMU_ALIVE_CPU1_OUT,
	PMU_ALIVE_CPU2_OUT,
	PMU_ALIVE_TPU_OUT,
	PMU_ALIVE_GPU_OUT
};

int meter_write(int pmic, struct bcl_device *bcl_dev, u8 reg, u8 value)
{
	switch (pmic) {
		case SUB:
			return s2mpg15_write_reg((bcl_dev)->sub_meter_i2c, reg, value);
		case MAIN:
			return s2mpg14_write_reg((bcl_dev)->main_meter_i2c, reg, value);
	}
	return 0;
}

int meter_read(int pmic, struct bcl_device *bcl_dev, u8 reg, u8 *value)
{
	switch (pmic) {
		case SUB:
			return s2mpg15_read_reg((bcl_dev)->sub_meter_i2c, reg, value);
		case MAIN:
			return s2mpg14_read_reg((bcl_dev)->main_meter_i2c, reg, value);
	}
	return 0;
}

int pmic_write(int pmic, struct bcl_device *bcl_dev, u8 reg, u8 value)
{
	switch (pmic) {
		case SUB:
			return s2mpg15_write_reg((bcl_dev)->sub_pmic_i2c, reg, value);
		case MAIN:
			return s2mpg14_write_reg((bcl_dev)->main_pmic_i2c, reg, value);
	}
	return 0;
}

int pmic_read(int pmic, struct bcl_device *bcl_dev, u8 reg, u8 *value)
{
	switch (pmic) {
		case SUB:
			return s2mpg15_read_reg((bcl_dev)->sub_pmic_i2c, reg, value);
		case MAIN:
			return s2mpg14_read_reg((bcl_dev)->main_pmic_i2c, reg, value);
	}
	return 0;
}

bool bcl_is_subsystem_on(unsigned int addr)
{
	unsigned int value;

	if ((addr == PMU_ALIVE_TPU_OUT) || (addr == PMU_ALIVE_GPU_OUT)) {
		exynos_pmu_read(addr, &value);
		return value & BIT(6);
	}
	return true;
}

void bcl_disable_power(void)
{
	int i;
	for (i = CPU1_CLUSTER_MIN; i <= CPU2_CLUSTER_MIN; i++)
		disable_power_mode(i, POWERMODE_TYPE_CLUSTER);
}

void bcl_enable_power(void)
{
	int i;
	for (i = CPU1_CLUSTER_MIN; i <= CPU2_CLUSTER_MIN; i++)
		enable_power_mode(i, POWERMODE_TYPE_CLUSTER);
}

void __iomem *get_addr_by_subsystem(void *dev, const char *subsystem)
{
	int i = 0;
	struct bcl_device *bcl_dev = dev;

	for (i = 0; i < ARRAY_SIZE(clk_stats_source); i++) {
		if (strcmp(subsystem, clk_stats_source[i]) == 0) {
			if (!bcl_is_subsystem_on(subsystem_pmu[i]))
				return NULL;
			return bcl_dev->base_mem[i] + CLKDIVSTEP;
		}
	}
	return NULL;
}
