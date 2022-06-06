/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * include/linux/mfd/samsung/s2mpg1415-register.h
 *
 * Copyright (C) 2015 Samsung Electronics
 *
 * header file including common register information of s2mpg14, s2mpg15
 */

#ifndef __LINUX_MFD_S2MPG1415_REGISTER_H
#define __LINUX_MFD_S2MPG1415_REGISTER_H

#include <linux/bits.h>

#define S2MPG1415_METER_CHANNEL_MAX 12
#define MASK(width, shift) GENMASK(shift + width - 1, shift)

typedef enum {
	MUXSEL_NONE,

	BUCK1 = 0x1,
	BUCK2,
	BUCK3,
	BUCK4,
	BUCK5,
	BUCK6,
	BUCK7,
	BUCK8,
	BUCK9,
	BUCK10,
	BUCK11,
	BUCK12,

	BUCKD,
	BUCKA,
	BUCKC,

	BUCKBOOST,

	VSEN_V1,
	VSEN_V2,
	VSEN_V3,

	VSEN_P4,
	VSEN_P5,
	VSEN_P6,

	VBAT,

	LDO1,
	LDO2,
	LDO3,
	LDO4,
	LDO5,
	LDO6,
	LDO7,
	LDO8,
	LDO9,
	LDO10,
	LDO11,
	LDO12,
	LDO13,
	LDO14,
	LDO15,
	LDO16,
	LDO17,
	LDO18,
	LDO19,
	LDO20,
	LDO21,
	LDO22,
	LDO23,
	LDO24,
	LDO25,
	LDO26,
	LDO27,
	LDO28,
	LDO29,
} s2mpg1415_meter_muxsel;

/**
 * sec_opmode_data - regulator operation mode data
 * @id: regulator id
 * @mode: regulator operation mode
 */
struct sec_opmode_data {
	int id;
	unsigned int mode;
};

/**
 * struct sec_wtsr_smpl - settings for WTSR/SMPL
 * @wtsr_en:		WTSR Function Enable Control
 * @smpl_en:		SMPL Function Enable Control
 * @wtsr_timer_val:	Set the WTSR timer Threshold
 * @smpl_timer_val:	Set the SMPL timer Threshold
 * @check_jigon:	if this value is true, do not enable SMPL function when
 *			JIGONB is low(JIG cable is attached)
 */
struct sec_wtsr_smpl {
	bool wtsr_en;
	bool coldrst_en;
	bool smpl_en;
	bool sub_smpl_en;
	int wtsr_timer_val;
	int coldrst_timer_val;
	int smpl_timer_val;
	bool check_jigon;
};

#define REG_VAL(val, offset, mask) (((val) << (offset)) & (mask))

#define ENUM_STR(x, s, r)		\
	{				\
		case x:			\
			r = #x s;	\
			break;		\
	}

typedef enum {
	INT_7P_8125HZ = 0,
	INT_15P_625HZ,
	INT_31P_25HZ,
	INT_62P_5HZ,
	INT_125HZ,
	INT_250HZ,
	INT_1000HZ,
	S2MPG1415_INT_FREQ_COUNT,
} s2mpg1415_int_samp_rate;

typedef enum {
	EXT_7P_8125HZ = 0,
	EXT_15P_625HZ,
	EXT_31P_25HZ,
	EXT_62P_5HZ,
	EXT_125HZ,
	S2MPG1415_EXT_FREQ_COUNT,
} s2mpg1415_ext_samp_rate;

typedef enum {
	S2MPG1415_METER_POWER = 0,
	S2MPG1415_METER_CURRENT = 1,
} s2mpg1415_meter_mode;

/* MUXSEL0~7 */
#define MUXSEL_MASK 0x7F

#define S2MPG1415_METER_LPF 0
#define S2MPG1415_METER_ACC 1

#define S2MPG1415_METER_LPF_BUF 3 /* 21-bit */
#define S2MPG1415_METER_ACC_BUF 6 /* 41-bit */
#define S2MPG1415_METER_COUNT_BUF 3 /* 20-bit */
#define S2MPG1415_METER_BUCKEN_BUF 2

/* S2MPG1415_METER_CTRL1 */
#define NTC_SAMP_RATE_SHIFT 5
#define NTC_SAMP_RATE_MASK (0x7 << NTC_SAMP_RATE_SHIFT)
#define INT_SAMP_RATE_SHIFT 2
#define INT_SAMP_RATE_MASK (0x7 << INT_SAMP_RATE_SHIFT)
#define EXT_METER_EN_MASK BIT(1)
#define METER_EN_MASK BIT(0)

typedef enum {
	NTC_0P15625HZ = 1,
	NTC_0P3125HZ,
	NTC_0P625HZ,
	NTC_1P25HZ,
	NTC_2P5HZ,
	NTC_5HZ,
	NTC_10HZ,
} s2mpg1415_ntc_samp_rate;

/* S2MPG1415_METER_CTRL2 */
#define ASYNC_RD_MASK BIT(7)
#define ONESHOT_RD_MASK BIT(6)
#define EXT_METER_CHANNEL_EN_ALL 0x7
#define EXT_METER_CHANNEL_EN_OFFSET 3
#define EXT_METER_CHANNEL_EN_MASK (0x7 << EXT_METER_CHANNEL_EN_OFFSET)
#define EXT_SAMP_RATE_MASK (0x7 << 0)

#define _IQX(_pow, _type, _val) ((_type)((_val) * (1 << (_pow))))
#define _IQX_to_int(_pow, _iqval) ((_iqval) >> (_pow))
// Note: powx > powy
#define _IQX_to_IQY(_powx, _powy, _iqxval) \
	((_iqxval) >> ((_powx) - (_powy)))

#define _IQ30(_type, _val) _IQX(30, _type, _val)
#define _IQ30_to_IQ22(_iqval) _IQX_to_IQY(30, 22, _iqval)
#define _IQ22_to_int(_iqval) _IQX_to_int(22, _iqval)
#define _IQ30_to_int(_iqval) _IQX_to_int(30, _iqval)

/* 1-bit resolution define for meter */
/* Unit : 1mA, 1mW, 1mV, converted to unsigned _iq30 */
/* Note: A value above or equal 4.0 would cause an overflow on a u32 IQ30 */
#define INVALID_RESOLUTION ((u32)(0))
#define CMS_BUCK_CURRENT _IQ30(u32, 1.0989010989011)
#define CMS_BUCK_POWER _IQ30(u32, 0.006868131868131)
#define CMD_BUCK_CURRENT _IQ30(u32, 2.1978021978022)
#define CMD_BUCK_POWER _IQ30(u32, 0.013736263736263)
#define CMT_BUCK_CURRENT _IQ30(u32, 3.2967032967033)
#define CMT_BUCK_POWER _IQ30(u32, 0.020604395604395)
#define VM_CURRENT _IQ30(u32, 1.0989010989011)
#define VM_POWER _IQ30(u32, 0.013736263736263)
#define BB_CURRENT _IQ30(u32, 0.879120879)
#define BB_POWER _IQ30(u32, 0.010989011)
#define NLDO_CURRENT_150mA _IQ30(u32, 0.036630036630036)
#define NLDO_POWER_150mA _IQ30(u32, 0.000457875457875)
#define PLDO_CURRENT_150mA _IQ30(u32, 0.036630036630036)
#define PLDO_POWER_150mA _IQ30(u32, 0.000915750915750)
#define NLDO_CURRENT_300mA _IQ30(u32, 0.073260073260073)
#define NLDO_POWER_300mA _IQ30(u32, 0.000915750915750)
#define PLDO_CURRENT_300mA _IQ30(u32, 0.073260073260073)
#define PLDO_POWER_300mA _IQ30(u32, 0.001831501831501)
#define DVS_NLDO_CURRENT_450mA _IQ30(u32, 0.10989010989011)
#define DVS_NLDO_POWER_450mA _IQ30(u32, 0.001373626373626)
#define NLDO_CURRENT_450mA _IQ30(u32, 0.10989010989011)
#define NLDO_POWER_450mA _IQ30(u32, 0.001373626373626)
#define PLDO_CURRENT_600mA _IQ30(u32, 0.146520146520147)
#define PLDO_POWER_600mA _IQ30(u32, 0.003663003663003)
#define DVS_NLDO_CURRENT_800mA _IQ30(u32, 0.195360195360195)
#define DVS_NLDO_POWER_800mA_OFF _IQ30(u32, 0.002442002442002)
#define DVS_NLDO_POWER_800mA_ON _IQ30(u32, 0.001221001221001)
#define NLDO_CURRENT_800mA _IQ30(u32, 0.195360195360195)
#define NLDO_POWER_800mA _IQ30(u32, 0.002442002442002)
#define PLDO_CURRENT_800mA _IQ30(u32,0.195360195360195)
#define PLDO_POWER_800mA _IQ30(u32, 0.004884004884004)
#define DVS_NLDO_CURRENT_1200mA _IQ30(u32, 0.293040293040293)
#define DVS_NLDO_POWER_1200mA_OFF _IQ30(u32, 0.003663003663003)
#define DVS_NLDO_POWER_1200mA_ON _IQ30(u32, 0.001831501831501)
#define NLDO_CURRENT_1200mA _IQ30(u32, 0.293040293040293)
#define NLDO_POWER_1200mA _IQ30(u32, 0.003663003663003)
#define EXT_UNDER_62_5HZ_CURRENT _IQ30(u32, 0.007935698)
#define EXT_UNDER_62_5HZ_POWER _IQ30(u32, 0.01397676)
#define EXT_125HZ_CURRENT _IQ30(u32, 0.008899436)
#define EXT_125HZ_POWER _IQ30(u32, 0.0156741534246575)
#define HRMODE_CM_BUCK_CURRENT _IQ30(u32, 0.0017)
#define HRMODE_VM_BUCK_CURRENT _IQ30(u32, 0.0009)
#define HRMODE_LDOS_CURRENT _IQ30(u32, 0.001470588)

#define EXTERNAL_RESOLUTION_VRAIL _IQ30(u32, 1.3186813)
#define EXTERNAL_RESOLUTION_VSHUNT _IQ30(u32, 0.0079356982)
#define EXTERNAL_RESOLUTION_TRIM BIT(3) // 3 bits

#endif /* __LINUX_MFD_S2MPG1415_REGISTER_H */
