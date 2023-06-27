/* SPDX-License-Identifier: GPL-2.0 */
/*
 * exynos-otg.h - Samsung EXYNOS OTG Header
 *
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 */

#ifndef __LINUX_USB_DWC3_OTG_H
#define __LINUX_USB_DWC3_OTG_H
#include <linux/pm_qos.h>
#include <linux/pm_wakeup.h>
#include <linux/power_supply.h>
#include <linux/usb/otg-fsm.h>
#include <linux/usb/otg.h>

#include <soc/google/exynos_pm_qos.h>

/**
 * struct dwc3_otg: OTG driver data. Shared by HCD and DCD.
 * @otg: USB OTG Transceiver structure.
 * @fsm: OTG Final State Machine.
 * @dwc: pointer to our controller context structure.
 * @wakelock: prevents the system from entering suspend while
 *		host or peripheral mode is active.
 * @vbus_reg: Vbus regulator.
 * @ready: is one when OTG is ready for operation.
 */
struct dwc3_otg {
	struct usb_otg          otg;
	struct otg_fsm		fsm;
	struct dwc3             *dwc;
	struct dwc3_exynos      *exynos;
	struct wakeup_source	*wakelock;

	unsigned		ready:1;
	int			otg_connection;

	struct regulator	*vbus_reg;

	struct exynos_pm_qos_request	pm_qos_int_req;
	int				pm_qos_int_usb2_val;
	int				pm_qos_int_usb3_val;

	struct work_struct	work;

	struct notifier_block	pm_nb;
	struct notifier_block	psy_notifier;
	struct completion	resume_cmpl;
	int			dwc3_suspended;
	int			in_shutdown;
	bool			usb_charged;

	struct mutex lock;
};

void dwc3_otg_run_sm(struct otg_fsm *fsm);
int dwc3_exynos_otg_init(struct dwc3 *dwc, struct dwc3_exynos *exynos);
void dwc3_exynos_otg_exit(struct dwc3 *dwc, struct dwc3_exynos *exynos);
int dwc3_otg_start(struct dwc3 *dwc, struct dwc3_exynos *exynos);
bool dwc3_otg_check_usb_suspend(struct dwc3_exynos *exynos);
bool dwc3_otg_check_usb_activity(struct dwc3_exynos *exynos);
int dwc3_otg_start_host(struct otg_fsm *fsm, int on);
int dwc3_otg_start_gadget(struct otg_fsm *fsm, int on);
void dwc3_otg_drv_vbus(struct otg_fsm *fsm, int on);

extern void __iomem *phycon_base_addr;
extern int exynos_usbdrd_pipe3_enable(struct phy *phy);
extern int exynos_usbdrd_pipe3_disable(struct phy *phy);

#endif /* __LINUX_USB_DWC3_OTG_H */
