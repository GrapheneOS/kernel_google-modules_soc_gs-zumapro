// SPDX-License-Identifier: GPL-2.0
/**
 * dwc3-exynos-otg.c - DesignWare Exynos USB3 DRD Controller OTG
 *
 * Copyright (c) 2012, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 */

#include <dwc3/core.h> /* $(srctree)/drivers/usb/dwc3/core.h */
#include <dwc3/io.h> /* $(srctree)/drivers/usb/dwc3/io.h */

#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/usb/composite.h>
#include <linux/usb/dwc3-exynos.h>
#include <linux/reboot.h>
#include <linux/suspend.h>
#include <linux/workqueue.h>

#include "core-exynos.h"
#include "exynos-otg.h"
#include "dwc3-exynos-ldo.h"

#define OTG_NO_CONNECT		0
#define OTG_CONNECT_ONLY	1
#define OTG_DEVICE_CONNECT	2
#define LINK_DEBUG_L		(0x0C)
#define LINK_DEBUG_H		(0x10)
#define BUS_ACTIVITY_CHECK	(0x3F << 16)
#define READ_TRANS_OFFSET	10

/* -------------------------------------------------------------------------- */
static int dwc3_otg_reboot_notify(struct notifier_block *nb, unsigned long event, void *buf);
static struct notifier_block dwc3_otg_reboot_notifier = {
	.notifier_call = dwc3_otg_reboot_notify,
};

static int dwc3_otg_statemachine(struct otg_fsm *fsm)
{
	struct usb_otg *otg = fsm->otg;
	struct dwc3_otg	*dotg = container_of(otg, struct dwc3_otg, otg);
	struct device *dev = dotg->dwc->dev;
	enum usb_otg_state prev_state = otg->state;
	int ret = 0;

	switch (otg->state) {
	case OTG_STATE_UNDEFINED:
		if (fsm->id)
			otg->state = OTG_STATE_B_IDLE;
		else
			otg->state = OTG_STATE_A_IDLE;
		break;
	case OTG_STATE_B_IDLE:
		if (!fsm->id) {
			otg->state = OTG_STATE_A_IDLE;
		} else if (fsm->b_sess_vld) {
			ret = dwc3_otg_start_gadget(fsm, 1);
			if (!ret)
				otg->state = OTG_STATE_B_PERIPHERAL;
			else
				dev_err(dev, "OTG SM: cannot start gadget\n");
		}
		break;
	case OTG_STATE_B_PERIPHERAL:
		if (!fsm->id || !fsm->b_sess_vld) {
			ret = dwc3_otg_start_gadget(fsm, 0);
			if (!ret)
				otg->state = OTG_STATE_B_IDLE;
			else
				dev_err(dev, "OTG SM: cannot stop gadget\n");
		}
		break;
	case OTG_STATE_A_IDLE:
		if (fsm->id) {
			otg->state = OTG_STATE_B_IDLE;
		} else {
			ret = dwc3_otg_start_host(fsm, 1);
			if (!ret) {
				otg->state = OTG_STATE_A_HOST;
			} else {
				dev_err(dev, "OTG SM: cannot start host\n");
			}
		}
		break;
	case OTG_STATE_A_HOST:
		if (fsm->id) {
			ret = dwc3_otg_start_host(fsm, 0);
			if (!ret)
				otg->state = OTG_STATE_A_IDLE;
			else
				dev_err(dev, "OTG SM: cannot stop host\n");
		}
		break;
	default:
		dev_err(dev, "OTG SM: invalid state\n");
	}

	if (!ret)
		ret = (otg->state != prev_state);

	dev_dbg(dev, "OTG SM: %s => %s\n", usb_otg_state_string(prev_state),
		(ret > 0) ? usb_otg_state_string(otg->state) : "(no change)");

	return ret;
}

/* -------------------------------------------------------------------------- */

static void dwc3_otg_set_mode(struct dwc3 *dwc, u32 mode)
{
	u32 reg;

	reg = dwc3_exynos_readl(dwc->regs, DWC3_GCTL);
	reg &= ~(DWC3_GCTL_PRTCAPDIR(DWC3_GCTL_PRTCAP_OTG));
	reg |= DWC3_GCTL_PRTCAPDIR(mode);
	dwc3_exynos_writel(dwc->regs, DWC3_GCTL, reg);
}

static void dwc3_otg_set_host_mode(struct dwc3_otg *dotg)
{
	struct dwc3 *dwc = dotg->dwc;
	u32 reg;

	/* Disable undefined length burst mode */
	reg = dwc3_exynos_readl(dwc->regs, DWC3_GSBUSCFG0);
	reg &= ~(DWC3_GSBUSCFG0_INCRBRSTEN);
	dwc3_exynos_writel(dwc->regs, DWC3_GSBUSCFG0, reg);

	dwc3_otg_set_mode(dwc, DWC3_GCTL_PRTCAP_HOST);
}

static void dwc3_otg_set_peripheral_mode(struct dwc3_otg *dotg)
{
	struct dwc3 *dwc = dotg->dwc;

	dwc3_otg_set_mode(dwc, DWC3_GCTL_PRTCAP_DEVICE);
}

void dwc3_otg_phy_tune(struct otg_fsm *fsm)
{
	struct usb_otg	*otg = fsm->otg;
	struct dwc3_otg	*dotg = container_of(otg, struct dwc3_otg, otg);
	struct dwc3	*dwc = dotg->dwc;

	exynos_usbdrd_phy_tune(dwc->usb2_generic_phy,
						dotg->otg.state);
#ifdef CONFIG_EXYNOS_USBDRD_PHY30
	exynos_usbdrd_phy_tune(dwc->usb3_generic_phy,
						dotg->otg.state);
#endif
}

int dwc3_otg_start_host(struct otg_fsm *fsm, int on)
{
	struct usb_otg	*otg = fsm->otg;
	struct dwc3_otg	*dotg = container_of(otg, struct dwc3_otg, otg);
	struct dwc3	*dwc = dotg->dwc;
	struct device	*dev = dotg->dwc->dev;
	struct dwc3_exynos *exynos = dotg->exynos;
	static struct usb_gadget_driver *temp_gadget_driver;
	struct usb_composite_driver *composite;
	int ret = 0;
	int ret1 = -1;
	int wait_counter = 0;

	__pm_stay_awake(dotg->wakelock);

	if (on) {
		if (!dwc3_otg_check_usb_suspend(exynos))
			dev_err(dev, "too long to wait for dwc3 suspended\n");

		dotg->otg_connection = 1;
		while (dwc->gadget_driver == NULL) {
			wait_counter++;
			msleep(20);

			if (wait_counter > 50) {
				dev_err(dev, "Can't wait host start!\n");
				break;
			}
		}

		if (!dwc->xhci) {
			ret = dwc3_exynos_host_init(exynos);
			if (ret) {
				dev_err(dev, "%s: failed to init dwc3 host\n", __func__);
				goto err1;
			}
		}

		/* To ignore gadget operation, it set gadget_driver to NULL */
		temp_gadget_driver = dwc->gadget_driver;
		dwc->gadget_driver = NULL;

		mutex_lock(&dotg->lock);
		exynos->need_dr_role = 1;

		ret = pm_runtime_get_sync(dev);
		if (ret < 0) {
			dev_err(dev, "failed to resume exynos device\n");
			pm_runtime_set_suspended(dev);
			mutex_unlock(&dotg->lock);
			goto err1;
		}
		exynos->need_dr_role = 0;

		/* To ignore gadget suspend/resume on host l2 suspend */
		exynos->dwc->current_dr_role = DWC3_EXYNOS_IGNORE_CORE_OPS;
		mutex_unlock(&dotg->lock);

		dwc3_otg_phy_tune(fsm);

		dwc3_exynos_core_init(dwc, exynos);
		dwc3_otg_set_host_mode(dotg);

		ret = platform_device_add(dwc->xhci);
		if (ret) {
			dev_err(dev, "%s: cannot add xhci\n", __func__);
			goto err1;
		}

	} else {
		dotg->otg_connection = 0;

		if (!dwc->xhci) {
			dev_err(dev, "%s: stop USB host without xhci device\n",
				__func__);
			return -EINVAL;
		}

		if (dotg->dwc3_suspended) {
			dev_dbg(dev, "wait resume completion\n");
			ret1 = wait_for_completion_timeout(&dotg->resume_cmpl,
							msecs_to_jiffies(5000));
		}

		if (temp_gadget_driver) {
			composite = to_cdriver(temp_gadget_driver);
			if (composite && composite->gadget_driver.udc_name)
				dwc->gadget_driver = temp_gadget_driver;
		}

		dwc3_exynos_host_exit(exynos);
		dwc->xhci = NULL;
err1:
		mutex_lock(&dotg->lock);
		exynos->dwc->current_dr_role = DWC3_GCTL_PRTCAP_DEVICE;
		pm_runtime_put_sync_suspend(dev);
		mutex_unlock(&dotg->lock);
	}
	__pm_relax(dotg->wakelock);
	return ret;
}

int dwc3_otg_start_gadget(struct otg_fsm *fsm, int on)
{
	struct usb_otg	*otg = fsm->otg;
	struct dwc3_otg	*dotg = container_of(otg, struct dwc3_otg, otg);
	struct dwc3	*dwc = dotg->dwc;
	struct dwc3_exynos *exynos = dotg->exynos;
	struct device	*dev = dotg->dwc->dev;
	int ret = 0;
	int wait_counter = 0;
	u32 evt_count, evt_buf_cnt;

	if (!dwc->gadget) {
		dev_err(dev, "%s does not have any gadget\n", __func__);
		return -EINVAL;
	}

	if (on) {
		__pm_stay_awake(dotg->wakelock);
		if (!dwc3_otg_check_usb_suspend(exynos))
			dev_err(dev, "too long to wait for dwc3 suspended\n");

		while (dwc->gadget_driver == NULL) {
			wait_counter++;
			usleep_range(100, 200);

			if (wait_counter > 500) {
				dev_err(dev, "Can't wait gadget start!\n");
				break;
			}
		}

		mutex_lock(&dotg->lock);
		exynos->need_dr_role = 1;
		dwc->connected = true;

		ret = pm_runtime_get_sync(dev);
		if (ret < 0) {
			dev_err(dev, "failed to resume exynos device\n");
			pm_runtime_set_suspended(dev);
		}
		exynos->need_dr_role = 0;
		mutex_unlock(&dotg->lock);

		dwc3_otg_phy_tune(fsm);
		dwc3_exynos_core_init(dwc, exynos);

		/* connect gadget */
		usb_udc_vbus_handler(dwc->gadget, true);

		exynos->gadget_state = true;
		dwc3_otg_set_peripheral_mode(dotg);
	} else {
		evt_buf_cnt = dwc->ev_buf->count;

		/* Wait until gadget stop */
		wait_counter = 0;
		evt_count = dwc3_readl(dwc->regs, DWC3_GEVNTCOUNT(0));
		evt_count &= DWC3_GEVNTCOUNT_MASK;
		while (evt_count || evt_buf_cnt) {
			wait_counter++;
			mdelay(20);

			if (wait_counter > 20) {
				dev_err(dev, "Can't wait event buffer empty!\n");
				break;
			}
			evt_count = dwc3_readl(dwc->regs, DWC3_GEVNTCOUNT(0));
			evt_count &= DWC3_GEVNTCOUNT_MASK;
			evt_buf_cnt = dwc->ev_buf->count;
		}
		dev_dbg(dev, "%s, evt compl wait cnt = %d\n",
			 __func__, wait_counter);

		/* disconnect gadget */
		usb_udc_vbus_handler(dwc->gadget, false);

		if (exynos->config.is_not_vbus_pad && exynos_usbdrd_get_ldo_status() &&
				!dotg->in_shutdown)
			dwc3_exynos_gadget_disconnect_proc(dwc);

		if (exynos->extra_delay)
			msleep(100);

		if (!dwc3_otg_check_usb_activity(exynos))
			dev_err(dev, "too long to suspend after cable plug-out\n");

		mutex_lock(&dotg->lock);
		pm_runtime_put_sync_suspend(dev);
		mutex_unlock(&dotg->lock);

		exynos->gadget_state = false;

		__pm_relax(dotg->wakelock);
	}

	return 0;
}

/* -------------------------------------------------------------------------- */

void dwc3_otg_run_sm(struct otg_fsm *fsm)
{
	struct dwc3_otg	*dotg = container_of(fsm, struct dwc3_otg, fsm);
	int		state_changed;

	/* Prevent running SM on early system resume */
	if (!dotg->ready)
		return;

	mutex_lock(&fsm->lock);

	do {
		state_changed = dwc3_otg_statemachine(fsm);
	} while (state_changed > 0);

	mutex_unlock(&fsm->lock);
}

/**
 * dwc3_otg_start
 * @dwc: pointer to our controller context structure
 */
int dwc3_otg_start(struct dwc3 *dwc, struct dwc3_exynos *exynos)
{
	struct dwc3_otg	*dotg = exynos->dotg;
	struct otg_fsm	*fsm = &dotg->fsm;

	/* B-device by default */
	fsm->id = 1;
	fsm->b_sess_vld = 0;

	dotg->ready = 1;

	dwc3_otg_run_sm(fsm);

	return 0;
}

/* -------------------------------------------------------------------------- */
static struct device_node *exynos_dwusb_parse_dt(void)
{
	struct device_node *np = NULL;

	np = of_find_compatible_node(NULL, NULL, "samsung,exynos9-dwusb");
	if (!np) {
		pr_err("%s: failed to get the usbdrd node\n", __func__);
		goto err;
	}
	return np;
err:
	return NULL;
}

static struct dwc3_exynos *exynos_dwusb_get_struct(void)
{
	struct device_node *np = NULL;
	struct platform_device *pdev = NULL;
	struct device *dev;
	struct dwc3_exynos *exynos;

	np = exynos_dwusb_parse_dt();
	if (np) {
		pdev = of_find_device_by_node(np);
		dev = &pdev->dev;
		of_node_put(np);
		if (pdev) {
			exynos = dev->driver_data;
			return exynos;
		}
	}

	pr_err("%s: failed to get the platform_device\n", __func__);
	return NULL;
}

int dwc3_otg_host_enable(bool enabled)
{
	struct dwc3_exynos *exynos;
	struct otg_fsm  *fsm;

	exynos = exynos_dwusb_get_struct();
	if (!exynos) {
		pr_err("%s: error exynos_dwusb_get_struct\n", __func__);
		return -ENODEV;
	}

	fsm = &exynos->dotg->fsm;
	fsm->id = !enabled;

	dev_dbg(exynos->dev, "%s: %s host\n", __func__, enabled ? "enable" : "disable");
	dwc3_otg_run_sm(fsm);

	return 0;
}
EXPORT_SYMBOL_GPL(dwc3_otg_host_enable);

bool dwc3_otg_check_usb_suspend(struct dwc3_exynos *exynos)
{
	int wait_counter = 0;
	bool exynos_suspend, dwc_suspend;

	do {
		exynos_suspend = (pm_runtime_suspend(exynos->dev) &
				  (atomic_read(&exynos->dev->power.usage_count) < 1));
		dwc_suspend = (pm_runtime_suspend(exynos->dwc->dev) &
			       (atomic_read(&exynos->dwc->dev->power.usage_count) < 1));

		if (exynos_suspend && dwc_suspend)
			break;

		wait_counter++;
		msleep(20);
	} while (wait_counter < DWC3_EXYNOS_MAX_WAIT_COUNT);

	return wait_counter < DWC3_EXYNOS_MAX_WAIT_COUNT;
}

bool dwc3_otg_check_usb_activity(struct dwc3_exynos *exynos)
{
	int wait_counter = 0;

	do {
		if ((atomic_read(&exynos->dwc->dev->power.usage_count)) < 2)
			break;

		wait_counter++;
		msleep(20);
	} while (wait_counter < DWC3_EXYNOS_DISCONNECT_COUNT);

	return wait_counter < DWC3_EXYNOS_DISCONNECT_COUNT;
}

static int dwc3_otg_reboot_notify(struct notifier_block *nb, unsigned long event, void *buf)
{
	struct dwc3_exynos *exynos;
	struct dwc3_otg *dotg;

	exynos = exynos_dwusb_get_struct();
	if (!exynos)
		return -ENODEV;

	dotg = exynos->dotg;

	switch (event) {
	case SYS_HALT:
	case SYS_RESTART:
	case SYS_POWER_OFF:
		exynos->dwc->current_dr_role = DWC3_EXYNOS_IGNORE_CORE_OPS;
		dotg->in_shutdown = true;
		break;
	}

	return 0;
}

u32 dwc3_otg_is_connect(void)
{
	struct dwc3_exynos *exynos;
	struct dwc3_otg *dotg;

	exynos = exynos_dwusb_get_struct();
	if (!exynos || !exynos->dotg) {
		pr_err("[%s] error\n", __func__);
		return -ENODEV;
	}
	dotg = exynos->dotg;

	if (!dotg->otg_connection)
		return OTG_NO_CONNECT;
	else
		return OTG_DEVICE_CONNECT;
}
EXPORT_SYMBOL_GPL(dwc3_otg_is_connect);

int dwc3_otg_get_idle_ip_index(void)
{
	struct dwc3_exynos *exynos;

	exynos = exynos_dwusb_get_struct();

	if (exynos == NULL)
		return -ENODEV;

	return exynos->idle_ip_index;
}
EXPORT_SYMBOL_GPL(dwc3_otg_get_idle_ip_index);

static int dwc3_otg_pm_notifier(struct notifier_block *nb,
		unsigned long action, void *nb_data)
{
	struct dwc3_otg *dotg
		= container_of(nb, struct dwc3_otg, pm_nb);

	switch (action) {
	case PM_SUSPEND_PREPARE:
		dotg->dwc3_suspended = 1;
		reinit_completion(&dotg->resume_cmpl);
		break;
	case PM_POST_SUSPEND:
		dotg->dwc3_suspended = 0;
		complete(&dotg->resume_cmpl);
		break;
	default:
		break;
	}
	return NOTIFY_OK;
}

static int psy_changed(struct notifier_block *nb, unsigned long evt, void *ptr)
{
	struct dwc3_otg *dotg = container_of(nb, struct dwc3_otg, psy_notifier);
	struct power_supply *psy = ptr;

	if (!strstr(psy->desc->name, "usb") || evt != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_OK;

	if (dotg->dwc->gadget->state == USB_STATE_CONFIGURED && !dotg->usb_charged) {
		dotg->usb_charged = true;
		if (dotg->dwc->speed >= DWC3_DSTS_SUPERSPEED) {
			if (dotg->pm_qos_int_usb3_val) {
				dev_dbg(dotg->dwc->dev, "pm_qos set value = %d\n",
					dotg->pm_qos_int_usb3_val);
				exynos_pm_qos_update_request(&dotg->pm_qos_int_req,
							     dotg->pm_qos_int_usb3_val);
			}
		} else {
			if (dotg->pm_qos_int_usb2_val) {
				dev_dbg(dotg->dwc->dev, "pm_qos set value = %d\n",
					dotg->pm_qos_int_usb2_val);
				exynos_pm_qos_update_request(&dotg->pm_qos_int_req,
							     dotg->pm_qos_int_usb2_val);
			}
		}
	} else if (dotg->dwc->gadget->state != USB_STATE_CONFIGURED && dotg->usb_charged) {
		dotg->usb_charged = false;
		dev_dbg(dotg->dwc->dev, "clear pm_qos value\n");
		if (dotg->pm_qos_int_usb2_val || dotg->pm_qos_int_usb3_val)
			exynos_pm_qos_update_request(&dotg->pm_qos_int_req, 0);
	}

	return NOTIFY_OK;
}

static void dwc3_exynos_rsw_work(struct work_struct *w)
{
	struct dwc3_otg *dotg = container_of(w, struct dwc3_otg, work);

	dwc3_otg_run_sm(&dotg->fsm);
}

int dwc3_exynos_otg_init(struct dwc3 *dwc, struct dwc3_exynos *exynos)
{
	struct dwc3_otg *dotg;
	int ret = 0;

	dotg = devm_kzalloc(dwc->dev, sizeof(struct dwc3_otg), GFP_KERNEL);
	if (!dotg)
		return -ENOMEM;

	dotg->dwc = dwc;
	dotg->exynos = exynos;

	ret = of_property_read_u32(exynos->dev->of_node, "usb-pm-qos-usb2-int",
				   &dotg->pm_qos_int_usb2_val);

	ret = of_property_read_u32(exynos->dev->of_node, "usb-pm-qos-usb3-int",
				   &dotg->pm_qos_int_usb3_val);

	if (ret < 0) {
		dev_err(dwc->dev, "couldn't read usb-pm-qos-int %s node, error = %d\n",
			dwc->dev->of_node->name, ret);
		dotg->pm_qos_int_usb2_val = 0;
		dotg->pm_qos_int_usb3_val = 0;
	} else {
		exynos_pm_qos_add_request(&dotg->pm_qos_int_req,
					  PM_QOS_DEVICE_THROUGHPUT, 0);
	}

	dotg->otg.state = OTG_STATE_UNDEFINED;
	dotg->in_shutdown = false;

	mutex_init(&dotg->fsm.lock);
	dotg->fsm.otg = &dotg->otg;

	INIT_WORK(&dotg->work, dwc3_exynos_rsw_work);

	dotg->wakelock = wakeup_source_register(dwc->dev, "dwc3-otg");

	mutex_init(&dotg->lock);

	init_completion(&dotg->resume_cmpl);
	dotg->dwc3_suspended = 0;
	dotg->pm_nb.notifier_call = dwc3_otg_pm_notifier;
	register_pm_notifier(&dotg->pm_nb);

	ret = register_reboot_notifier(&dwc3_otg_reboot_notifier);
	if (ret)
		dev_err(dwc->dev, "failed register reboot notifier\n");

	dotg->psy_notifier.notifier_call = psy_changed;
	ret = power_supply_reg_notifier(&dotg->psy_notifier);
	if (ret)
		dev_err(dwc->dev, "failed register power supply notifier\n");

	/* Make dwc3_otg accessible after member variable initialization completes */
	exynos->dotg = dotg;

	dev_dbg(dwc->dev, "otg_init done\n");

	return 0;
}

void dwc3_exynos_otg_exit(struct dwc3 *dwc, struct dwc3_exynos *exynos)
{
	struct dwc3_otg *dotg = exynos->dotg;

	unregister_pm_notifier(&dotg->pm_nb);
	cancel_work_sync(&dotg->work);
	wakeup_source_unregister(dotg->wakelock);
	dotg->otg.state = OTG_STATE_UNDEFINED;
	kfree(dotg);
	exynos->dotg = NULL;
}
