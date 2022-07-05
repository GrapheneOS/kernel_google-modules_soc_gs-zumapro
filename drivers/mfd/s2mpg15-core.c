// SPDX-License-Identifier: GPL-2.0+
/*
 * s2mpg15.c
 *
 * Copyright (C) 2022 Samsung Electronics
 *
 * mfd core driver for the s2mpg15
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/mfd/core.h>
#include <linux/mfd/samsung/s2mpg15.h>
#include <linux/mfd/samsung/s2mpg15-register.h>
#include <linux/regulator/machine.h>
#include <linux/rtc.h>
#include <soc/google/acpm_mfd.h>

#if IS_ENABLED(CONFIG_OF)
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif /* CONFIG_OF */

static struct device_node *acpm_mfd_node;

static struct mfd_cell s2mpg15_devs[] = {
	{
		.name = "s2mpg15-regulator",
	},
	{
		.name = "s2mpg15-meter",
	},
	{
		.name = "s2mpg15_gpio",
	},
};

int s2mpg15_read_reg(struct i2c_client *i2c, u8 reg, u8 *dest)
{
	struct s2mpg15_dev *s2mpg15 = i2c_get_clientdata(i2c);
	u8 channel = 1;
	int ret;

	mutex_lock(&s2mpg15->i2c_lock);
	ret = exynos_acpm_read_reg(acpm_mfd_node, channel,
				   i2c->addr, reg, dest);
	mutex_unlock(&s2mpg15->i2c_lock);
	if (ret)
		pr_err("[%s] acpm ipc fail!\n", __func__);
	return ret;
}
EXPORT_SYMBOL_GPL(s2mpg15_read_reg);

int s2mpg15_bulk_read(struct i2c_client *i2c, u8 reg, int count, u8 *buf)
{
	struct s2mpg15_dev *s2mpg15 = i2c_get_clientdata(i2c);
	u8 channel = 1;
	int ret;

	mutex_lock(&s2mpg15->i2c_lock);
	ret = exynos_acpm_bulk_read(acpm_mfd_node, channel,
				    i2c->addr, reg, count, buf);
	mutex_unlock(&s2mpg15->i2c_lock);
	if (ret)
		pr_err("[%s] acpm ipc fail!\n", __func__);
	return ret;
}
EXPORT_SYMBOL_GPL(s2mpg15_bulk_read);

int s2mpg15_write_reg(struct i2c_client *i2c, u8 reg, u8 value)
{
	struct s2mpg15_dev *s2mpg15 = i2c_get_clientdata(i2c);
	u8 channel = 1;
	int ret;

	mutex_lock(&s2mpg15->i2c_lock);
	ret = exynos_acpm_write_reg(acpm_mfd_node, channel,
				    i2c->addr, reg, value);
	mutex_unlock(&s2mpg15->i2c_lock);
	if (ret)
		pr_err("[%s] acpm ipc fail!\n", __func__);
	return ret;
}
EXPORT_SYMBOL_GPL(s2mpg15_write_reg);

int s2mpg15_bulk_write(struct i2c_client *i2c, u8 reg, int count, u8 *buf)
{
	struct s2mpg15_dev *s2mpg15 = i2c_get_clientdata(i2c);
	u8 channel = 1;
	int ret;

	mutex_lock(&s2mpg15->i2c_lock);
	ret = exynos_acpm_bulk_write(acpm_mfd_node, channel,
				     i2c->addr, reg, count, buf);
	mutex_unlock(&s2mpg15->i2c_lock);
	if (ret)
		pr_err("[%s] acpm ipc fail!\n", __func__);
	return ret;
}
EXPORT_SYMBOL_GPL(s2mpg15_bulk_write);

int s2mpg15_update_reg(struct i2c_client *i2c, u8 reg, u8 val, u8 mask)
{
	struct s2mpg15_dev *s2mpg15 = i2c_get_clientdata(i2c);
	u8 channel = 1;
	int ret;

	mutex_lock(&s2mpg15->i2c_lock);
	ret = exynos_acpm_update_reg(acpm_mfd_node, channel,
				     i2c->addr, reg, val, mask);
	mutex_unlock(&s2mpg15->i2c_lock);
	if (ret)
		pr_err("[%s] acpm ipc fail!\n", __func__);
	return ret;
}
EXPORT_SYMBOL_GPL(s2mpg15_update_reg);

#if IS_ENABLED(CONFIG_OF)
static int of_s2mpg15_dt(struct device *dev,
			 struct s2mpg15_platform_data *pdata,
			 struct s2mpg15_dev *s2mpg15)
{
	struct device_node *np = dev->of_node;
	const char *status;

	if (!np)
		return -EINVAL;

	acpm_mfd_node = np;

	status = of_get_property(np, "s2mpg15,wakeup", NULL);
	if (!status)
		return -EINVAL;
	pdata->wakeup = !strcmp(status, "enabled") || !strcmp(status, "okay");

	return 0;
}
#else
static int of_s2mpg15_dt(struct device *dev,
			 struct s2mpg15_platform_data *pdata)
{
	return 0;
}
#endif /* CONFIG_OF */

static int s2mpg15_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *dev_id)
{
	struct s2mpg15_dev *s2mpg15;
	struct s2mpg15_platform_data *pdata = i2c->dev.platform_data;
	u8 reg_data;
	int ret = 0;

	dev_dbg(&i2c->dev, "%s i2c probe\n", S2MPG15_MFD_DEV_NAME);

	s2mpg15 = devm_kzalloc(&i2c->dev, sizeof(*s2mpg15), GFP_KERNEL);
	if (!s2mpg15)
		return -ENOMEM;

	pdata = devm_kzalloc(&i2c->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	ret = of_s2mpg15_dt(&i2c->dev, pdata, s2mpg15);
	if (ret < 0) {
		dev_err(&i2c->dev, "Failed to get device of_node\n");
		return ret;
	}
	i2c->dev.platform_data = pdata;

	pdata->irq_base = devm_irq_alloc_descs(&i2c->dev, -1, 0,
					       S2MPG15_IRQ_NR, -1);
	if (pdata->irq_base < 0) {
		dev_err(&i2c->dev, "%s devm_irq_alloc_descs Fail! ret(%d)\n",
			__func__, pdata->irq_base);
		return -EINVAL;
	}

	s2mpg15->dev = &i2c->dev;
	i2c->addr = I2C_ADDR_TOP;
	s2mpg15->i2c = i2c;
	s2mpg15->device_type = S2MPG15X;
	s2mpg15->pdata = pdata;
	s2mpg15->wakeup = pdata->wakeup;
	s2mpg15->irq_base = pdata->irq_base;

	mutex_init(&s2mpg15->i2c_lock);

	s2mpg15->pmic = i2c_new_dummy_device(i2c->adapter, I2C_ADDR_PMIC);
	s2mpg15->meter = i2c_new_dummy_device(i2c->adapter, I2C_ADDR_METER);
	s2mpg15->wlwp = i2c_new_dummy_device(i2c->adapter, I2C_ADDR_WLWP);
	s2mpg15->gpio = i2c_new_dummy_device(i2c->adapter, I2C_ADDR_GPIO);
	s2mpg15->mt_trim = i2c_new_dummy_device(i2c->adapter, I2C_ADDR_MT_TRIM);
	s2mpg15->pm_trim1 = i2c_new_dummy_device(i2c->adapter, I2C_ADDR_PM_TRIM1);
	s2mpg15->pm_trim2 = i2c_new_dummy_device(i2c->adapter, I2C_ADDR_PM_TRIM2);

	i2c_set_clientdata(s2mpg15->i2c, s2mpg15);
	i2c_set_clientdata(s2mpg15->pmic, s2mpg15);
	i2c_set_clientdata(s2mpg15->meter, s2mpg15);
	i2c_set_clientdata(s2mpg15->wlwp, s2mpg15);
	i2c_set_clientdata(s2mpg15->gpio, s2mpg15);
	i2c_set_clientdata(s2mpg15->mt_trim, s2mpg15);
	i2c_set_clientdata(s2mpg15->pm_trim1, s2mpg15);
	i2c_set_clientdata(s2mpg15->pm_trim2, s2mpg15);

	if (s2mpg15_read_reg(i2c, S2MPG15_COMMON_CHIPID, &reg_data) < 0) {
		dev_warn(s2mpg15->dev, "device not found on this channel\n");
		ret = -ENODEV;
		goto err_w_lock;
	}
	s2mpg15->pmic_rev = reg_data;

	dev_info(s2mpg15->dev, "device found: rev.0x%02x\n", s2mpg15->pmic_rev);

	ret = s2mpg15_notifier_init(s2mpg15);
	if (ret < 0) {
		dev_err(s2mpg15->dev, "s2mpg15_notifier_init fail\n");
		goto err_w_lock;
	}

	ret = mfd_add_devices(s2mpg15->dev, -1, s2mpg15_devs,
			      ARRAY_SIZE(s2mpg15_devs), NULL, 0, NULL);
	if (ret < 0)
		goto err_mfd;

	ret = device_init_wakeup(s2mpg15->dev, pdata->wakeup);
	if (ret < 0) {
		dev_err(s2mpg15->dev, "device_init_wakeup fail(%d)\n", ret);
		goto err_mfd;
	}

	return ret;

err_mfd:
	mfd_remove_devices(s2mpg15->dev);
err_w_lock:
	mutex_destroy(&s2mpg15->i2c_lock);
	return ret;
}

static int s2mpg15_i2c_remove(struct i2c_client *i2c)
{
	struct s2mpg15_dev *s2mpg15 = i2c_get_clientdata(i2c);

	if (s2mpg15->pdata->wakeup)
		device_init_wakeup(s2mpg15->dev, false);
	mfd_remove_devices(s2mpg15->dev);
	i2c_unregister_device(s2mpg15->i2c);

	return 0;
}

static const struct i2c_device_id s2mpg15_i2c_id[] = {
	{ S2MPG15_MFD_DEV_NAME, TYPE_S2MPG15 }, {} };

MODULE_DEVICE_TABLE(i2c, s2mpg15_i2c_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id s2mpg15_i2c_dt_ids[] = {
	{ .compatible = "samsung,s2mpg15mfd" },
	{},
};
#endif /* CONFIG_OF */

#if IS_ENABLED(CONFIG_PM)
static int s2mpg15_suspend(struct device *dev)
{
	return 0;
}

static int s2mpg15_resume(struct device *dev)
{
	return 0;
}
#else
#define s2mpg15_suspend NULL
#define s2mpg15_resume NULL
#endif /* CONFIG_PM */

const struct dev_pm_ops s2mpg15_pm = {
	.suspend_late = s2mpg15_suspend,
	.resume_early = s2mpg15_resume,
};

static struct i2c_driver s2mpg15_i2c_driver = {
	.driver = {
		   .name = S2MPG15_MFD_DEV_NAME,
		   .owner = THIS_MODULE,
#if IS_ENABLED(CONFIG_PM)
		   .pm = &s2mpg15_pm,
#endif /* CONFIG_PM */
#if IS_ENABLED(CONFIG_OF)
		   .of_match_table = s2mpg15_i2c_dt_ids,
#endif /* CONFIG_OF */
		   .suppress_bind_attrs = true,
		    },
	.probe = s2mpg15_i2c_probe,
	.remove = s2mpg15_i2c_remove,
	.id_table = s2mpg15_i2c_id,
};

static int __init s2mpg15_i2c_init(void)
{
	return i2c_add_driver(&s2mpg15_i2c_driver);
}

/* init early so consumer devices can complete system boot */
subsys_initcall(s2mpg15_i2c_init);

static void __exit s2mpg15_i2c_exit(void)
{
	i2c_del_driver(&s2mpg15_i2c_driver);
}

module_exit(s2mpg15_i2c_exit);

MODULE_DESCRIPTION("s2mpg15 multi-function core driver");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
