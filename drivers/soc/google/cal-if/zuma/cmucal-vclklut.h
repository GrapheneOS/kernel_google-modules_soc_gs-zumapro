/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Samsung Electronics Co., Ltd.
 */

#ifndef __CMUCAL_VCLKLUT_H__
#define __CMUCAL_VCLKLUT_H__

#include "../cmucal.h"

extern unsigned int vdd_int_nm_lut_params[];
extern unsigned int vdd_int_ud_lut_params[];
extern unsigned int vdd_int_sud_lut_params[];
extern unsigned int vdd_int_uud_lut_params[];
extern unsigned int vdd_mif_od_lut_params[];
extern unsigned int vdd_mif_nm_lut_params[];
extern unsigned int vdd_mif_ud_lut_params[];
extern unsigned int vdd_mif_sud_lut_params[];
extern unsigned int vdd_mif_uud_lut_params[];
extern unsigned int vdd_g3d_nm_lut_params[];
extern unsigned int vdd_g3d_ud_lut_params[];
extern unsigned int vdd_g3d_sud_lut_params[];
extern unsigned int vdd_g3d_uud_lut_params[];
extern unsigned int vdd_cam_nm_lut_params[];
extern unsigned int vdd_cam_ud_lut_params[];
extern unsigned int vdd_cam_sud_lut_params[];
extern unsigned int vdd_cam_uud_lut_params[];
extern unsigned int vdd_cpucl0_sod_lut_params[];
extern unsigned int vdd_cpucl0_od_lut_params[];
extern unsigned int vdd_cpucl0_nm_lut_params[];
extern unsigned int vdd_cpucl0_ud_lut_params[];
extern unsigned int vdd_cpucl0_sud_lut_params[];
extern unsigned int vdd_cpucl0_uud_lut_params[];
extern unsigned int vdd_cpucl1_sod_lut_params[];
extern unsigned int vdd_cpucl1_od_lut_params[];
extern unsigned int vdd_cpucl1_nm_lut_params[];
extern unsigned int vdd_cpucl1_ud_lut_params[];
extern unsigned int vdd_cpucl1_sud_lut_params[];
extern unsigned int vdd_cpucl1_uud_lut_params[];
extern unsigned int vdd_tpu_nm_lut_params[];
extern unsigned int vdd_tpu_ud_lut_params[];
extern unsigned int vdd_tpu_sud_lut_params[];
extern unsigned int vdd_tpu_uud_lut_params[];
extern unsigned int vdd_cpucl2_sod_lut_params[];
extern unsigned int vdd_cpucl2_od_lut_params[];
extern unsigned int vdd_cpucl2_nm_lut_params[];
extern unsigned int vdd_cpucl2_ud_lut_params[];
extern unsigned int vdd_cpucl2_sud_lut_params[];
extern unsigned int vdd_cpucl2_uud_lut_params[];
extern unsigned int mux_cmu_cmuref_ud_lut_params[];
extern unsigned int mux_cmu_cmuref_sud_lut_params[];
extern unsigned int mux_cmu_cmuref_uud_lut_params[];
extern unsigned int mux_cpucl0_cmuref_uud_lut_params[];
extern unsigned int mux_cpucl1_cmuref_uud_lut_params[];
extern unsigned int mux_cpucl2_cmuref_uud_lut_params[];
extern unsigned int mux_clk_hsi0_usb20_ref_nm_lut_params[];
extern unsigned int clkcmu_hsi0_usb32drd_uud_lut_params[];
extern unsigned int mux_mif_cmuref_uud_lut_params[];
extern unsigned int mux_nocl0_cmuref_uud_lut_params[];
extern unsigned int mux_nocl1b_cmuref_uud_lut_params[];
extern unsigned int mux_nocl2aa_cmuref_uud_lut_params[];
extern unsigned int mux_nocl2ab_cmuref_uud_lut_params[];
extern unsigned int clkcmu_dpub_dsim_uud_lut_params[];
extern unsigned int clkcmu_hsi0_dpgtc_uud_lut_params[];
extern unsigned int div_clk_apm_usi0_usi_nm_lut_params[];
extern unsigned int div_clk_apm_usi0_uart_nm_lut_params[];
extern unsigned int div_clk_apm_usi1_uart_nm_lut_params[];
extern unsigned int div_clk_apm_i3c_pmic_nm_lut_params[];
extern unsigned int clk_aur_add_ch_clk_uud_lut_params[];
extern unsigned int mux_clkcmu_cis_clk0_uud_lut_params[];
extern unsigned int mux_clkcmu_cis_clk1_uud_lut_params[];
extern unsigned int mux_clkcmu_cis_clk2_uud_lut_params[];
extern unsigned int mux_clkcmu_cis_clk3_uud_lut_params[];
extern unsigned int mux_clkcmu_cis_clk4_uud_lut_params[];
extern unsigned int mux_clkcmu_cis_clk5_uud_lut_params[];
extern unsigned int mux_clkcmu_cis_clk6_uud_lut_params[];
extern unsigned int mux_clkcmu_cis_clk7_uud_lut_params[];
extern unsigned int div_clk_cpucl0_cmuref_sod_lut_params[];
extern unsigned int div_clk_cpucl0_cmuref_od_lut_params[];
extern unsigned int div_clk_cpucl0_cmuref_nm_lut_params[];
extern unsigned int div_clk_cpucl0_cmuref_ud_lut_params[];
extern unsigned int div_clk_cpucl0_cmuref_sud_lut_params[];
extern unsigned int div_clk_cpucl0_cmuref_uud_lut_params[];
extern unsigned int div_clk_cpucl0_add_ch_clk_uud_lut_params[];
extern unsigned int div_clk_cpucl1_cmuref_sod_lut_params[];
extern unsigned int div_clk_cpucl1_cmuref_od_lut_params[];
extern unsigned int div_clk_cpucl1_cmuref_nm_lut_params[];
extern unsigned int div_clk_cpucl1_cmuref_ud_lut_params[];
extern unsigned int div_clk_cpucl1_cmuref_sud_lut_params[];
extern unsigned int div_clk_cpucl1_cmuref_uud_lut_params[];
extern unsigned int div_clk_cpucl2_cmuref_sod_lut_params[];
extern unsigned int div_clk_cpucl2_cmuref_od_lut_params[];
extern unsigned int div_clk_cpucl2_cmuref_nm_lut_params[];
extern unsigned int div_clk_cpucl2_cmuref_ud_lut_params[];
extern unsigned int div_clk_cpucl2_cmuref_sud_lut_params[];
extern unsigned int div_clk_cpucl2_cmuref_uud_lut_params[];
extern unsigned int clk_g3d_add_ch_clk_uud_lut_params[];
extern unsigned int div_clk_gsacore_spi_fps_nm_lut_params[];
extern unsigned int div_clk_gsacore_spi_gsc_nm_lut_params[];
extern unsigned int div_clk_gsacore_uart_nm_lut_params[];
extern unsigned int div_clk_hsi0_usi2_nm_lut_params[];
extern unsigned int clkcmu_hsi0_peri_uud_lut_params[];
extern unsigned int clkcmu_hsi0_peri_nm_lut_params[];
extern unsigned int div_clk_hsi0_usi0_nm_lut_params[];
extern unsigned int div_clk_hsi0_usi1_nm_lut_params[];
extern unsigned int div_clk_hsi0_usi3_nm_lut_params[];
extern unsigned int div_clk_hsi0_usi4_nm_lut_params[];
extern unsigned int div_clk_slc_dclk_od_lut_params[];
extern unsigned int div_clk_slc_dclk_uud_lut_params[];
extern unsigned int div_clk_slc1_dclk_od_lut_params[];
extern unsigned int div_clk_slc1_dclk_uud_lut_params[];
extern unsigned int div_clk_slc2_dclk_od_lut_params[];
extern unsigned int div_clk_slc2_dclk_uud_lut_params[];
extern unsigned int div_clk_slc3_dclk_od_lut_params[];
extern unsigned int div_clk_slc3_dclk_uud_lut_params[];
extern unsigned int div_clk_peric0_usi6_usi_uud_lut_params[];
extern unsigned int mux_clkcmu_peric0_ip_uud_lut_params[];
extern unsigned int div_clk_peric0_usi3_usi_uud_lut_params[];
extern unsigned int div_clk_peric0_usi4_usi_uud_lut_params[];
extern unsigned int div_clk_peric0_usi5_usi_uud_lut_params[];
extern unsigned int div_clk_peric0_usi14_usi_uud_lut_params[];
extern unsigned int div_clk_peric0_usi1_usi_uud_lut_params[];
extern unsigned int div_clk_peric0_usi0_uart_uud_lut_params[];
extern unsigned int div_clk_peric0_usi2_usi_uud_lut_params[];
extern unsigned int div_clk_peric1_usi11_usi_uud_lut_params[];
extern unsigned int mux_clkcmu_peric1_ip_uud_lut_params[];
extern unsigned int div_clk_peric1_i3c_uud_lut_params[];
extern unsigned int div_clk_peric1_usi12_usi_uud_lut_params[];
extern unsigned int div_clk_peric1_usi0_usi_uud_lut_params[];
extern unsigned int div_clk_peric1_usi9_usi_uud_lut_params[];
extern unsigned int div_clk_peric1_usi10_usi_uud_lut_params[];
extern unsigned int div_clk_peric1_usi13_usi_uud_lut_params[];
extern unsigned int div_clk_peric1_usi15_usi_uud_lut_params[];
extern unsigned int clk_tpu_add_ch_clk_uud_lut_params[];
extern unsigned int blk_cmu_nm_lut_params[];
extern unsigned int blk_cmu_ud_lut_params[];
extern unsigned int blk_cmu_uud_lut_params[];
extern unsigned int blk_gsactrl_nm_lut_params[];
extern unsigned int blk_s2d_nm_lut_params[];
extern unsigned int blk_apm_nm_lut_params[];
extern unsigned int blk_cpucl0_sod_lut_params[];
extern unsigned int blk_cpucl0_od_lut_params[];
extern unsigned int blk_cpucl0_nm_lut_params[];
extern unsigned int blk_cpucl0_ud_lut_params[];
extern unsigned int blk_cpucl0_uud_lut_params[];
extern unsigned int blk_cpucl1_sod_lut_params[];
extern unsigned int blk_cpucl1_od_lut_params[];
extern unsigned int blk_cpucl1_nm_lut_params[];
extern unsigned int blk_cpucl1_ud_lut_params[];
extern unsigned int blk_cpucl1_sud_lut_params[];
extern unsigned int blk_cpucl1_uud_lut_params[];
extern unsigned int blk_cpucl2_sod_lut_params[];
extern unsigned int blk_cpucl2_od_lut_params[];
extern unsigned int blk_cpucl2_nm_lut_params[];
extern unsigned int blk_cpucl2_ud_lut_params[];
extern unsigned int blk_cpucl2_sud_lut_params[];
extern unsigned int blk_cpucl2_uud_lut_params[];
extern unsigned int blk_gsacore_nm_lut_params[];
extern unsigned int blk_hsi0_nm_lut_params[];
extern unsigned int blk_hsi1_ud_lut_params[];
extern unsigned int blk_hsi1_uud_lut_params[];
extern unsigned int blk_nocl0_od_lut_params[];
extern unsigned int blk_nocl0_uud_lut_params[];
extern unsigned int blk_nocl1b_uud_lut_params[];
extern unsigned int blk_aoc_nm_lut_params[];
extern unsigned int blk_aoc_ud_lut_params[];
extern unsigned int blk_aoc_sud_lut_params[];
extern unsigned int blk_aoc_uud_lut_params[];
extern unsigned int blk_aur_uud_lut_params[];
extern unsigned int blk_bw_uud_lut_params[];
extern unsigned int blk_dpub_uud_lut_params[];
extern unsigned int blk_dpuf0_uud_lut_params[];
extern unsigned int blk_dpuf1_uud_lut_params[];
extern unsigned int blk_eh_uud_lut_params[];
extern unsigned int blk_eh_ud_lut_params[];
extern unsigned int blk_g2d_uud_lut_params[];
extern unsigned int blk_g3d_nm_lut_params[];
extern unsigned int blk_g3d_uud_lut_params[];
extern unsigned int blk_gdc_uud_lut_params[];
extern unsigned int blk_gse_uud_lut_params[];
extern unsigned int blk_hsi2_uud_lut_params[];
extern unsigned int blk_ispfe_uud_lut_params[];
extern unsigned int blk_mcsc_uud_lut_params[];
extern unsigned int blk_mif_uud_lut_params[];
extern unsigned int blk_misc_uud_lut_params[];
extern unsigned int blk_nocl1a_uud_lut_params[];
extern unsigned int blk_nocl2aa_uud_lut_params[];
extern unsigned int blk_nocl2ab_uud_lut_params[];
extern unsigned int blk_peric0_uud_lut_params[];
extern unsigned int blk_peric1_uud_lut_params[];
extern unsigned int blk_rgbp_uud_lut_params[];
extern unsigned int blk_tnr_uud_lut_params[];
extern unsigned int blk_tpu_uud_lut_params[];
extern unsigned int blk_yuvp_uud_lut_params[];
#endif
