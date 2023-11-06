// SPDX-License-Identifier: GPL-2.0
/*
 *
 * Copyright (C) 2021 Samsung Electronics Co., Ltd.
 * http://www.samsungsemi.com/
 *
 * Core file for Samsung EUSB Repeater driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "eusb_repeater.h"

struct eusb_repeater_data *g_tud;

int eusb_repeater_i2c_write(struct eusb_repeater_data *tud, u8 reg, u8 *data, int len)
{
	u8 buf[I2C_WRITE_BUFFER_SIZE + 1];
	int ret;
	unsigned char retry;
	struct i2c_msg msg;

	if (len > I2C_WRITE_BUFFER_SIZE) {
		dev_err(&tud->client->dev, "%s: len is larger than buffer size\n", __func__);
		return -EINVAL;
	}

	buf[0] = reg;
	memcpy(buf + 1, data, len);

	msg.addr = tud->client->addr;
	msg.flags = 0;
	msg.len = len + 1;
	msg.buf = buf;
	mutex_lock(&tud->i2c_mutex);
	for (retry = 0; retry < TUSB_I2C_RETRY_CNT; retry++) {
		ret = i2c_transfer(tud->client->adapter, &msg, 1);
		if (ret == 1)
			break;

		usleep_range(1 * 1000, 1 * 1000);

		if (retry > 1) {
			dev_err(&tud->client->dev, "%s: I2C retry %d\n", __func__, retry + 1);
			tud->comm_err_count++;
		}
	}

	mutex_unlock(&tud->i2c_mutex);

	if (ret == 1)
		return 0;

	return -EIO;
}

int eusb_repeater_i2c_read(struct eusb_repeater_data *tud, u8 reg, u8 *data, int len)
{
	u8 buf[4];
	int ret;
	unsigned char retry;
	struct i2c_msg msg[2];

	if (!len) {
		dev_err(&tud->client->dev,
			"%s: I2c message length is wrong!\n", __func__);
		goto err;
	}

	buf[0] = reg;

	msg[0].addr = tud->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = buf;

	msg[1].addr = tud->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = data;

	mutex_lock(&tud->i2c_mutex);

	for (retry = 0; retry < TUSB_I2C_RETRY_CNT; retry++) {
		ret = i2c_transfer(tud->client->adapter, msg, 2);
		if (ret == 2)
			break;
		usleep_range(1 * 1000, 1 * 1000);

		if (retry > 1) {
			dev_err(&tud->client->dev, "%s: I2C retry %d\n", __func__, retry + 1);
			tud->comm_err_count++;
		}
	}

	mutex_unlock(&tud->i2c_mutex);

	return ret;
err:
	return -EIO;
}

static int eusb_repeater_write_reg(struct eusb_repeater_data *tud, u8 reg, u8 *data, int len)
{
	int ret = 0;

	ret = eusb_repeater_i2c_write(tud, reg, data, len);
	if (ret < 0) {
		dev_err(tud->dev, "%s: reg(0x%x), ret(%d)\n",
			__func__, reg, ret);
	}
	return ret;
}

static int eusb_repeater_read_reg(struct eusb_repeater_data *tud, u8 reg, u8 *data, int len)
{
	int ret = 0;

	ret = eusb_repeater_i2c_read(tud, reg, data, len);
	if (ret < 0) {
		dev_err(tud->dev, "%s: reg(0x%x), ret(%d)\n",
			__func__, reg, ret);
		return ret;
	}
	ret &= 0xff;

	return ret;
}

static int eusb_repeater_fill_tune_param(struct eusb_repeater_data *tud,
				struct device_node *node)
{
	struct device *dev = tud->dev;
	struct device_node *child = NULL;
	struct eusb_repeater_tune_param *tune_param;
	size_t size = sizeof(struct eusb_repeater_tune_param);
	int ret;
	u32 res[4];
	u32 idx = 0;
	const char *name;

	ret = of_property_read_u32_array(node, "repeater_tune_cnt", &res[0], 1);
	tud->tune_cnt = res[0];

	dev_dbg(dev, "%s repeater tune cnt = %d\n", __func__, res[0]);

	tune_param = devm_kzalloc(dev, size * (res[0] + 1), GFP_KERNEL);
	if (!tune_param)
		return -ENOMEM;

	tud->tune_param = tune_param;

	for_each_child_of_node(node, child) {
		ret = of_property_read_string(child, "tune_name", &name);
		if (ret == 0) {
			memcpy(tune_param[idx].name, name, strlen(name));
		} else {
			dev_err(dev, "failed to read tune name from %s node\n", child->name);
			return ret;
		}
		ret = of_property_read_u32_array(child, "tune_value", res, 4);
		if (ret == 0) {
			tud->tune_param[idx].reg = res[0];
			tud->tune_param[idx].value = res[1];
			tud->tune_param[idx].shift = res[2];
			tud->tune_param[idx].mask = res[3];
		} else {
			dev_err(dev, "failed to read tune value from %s node\n", child->name);
			return -EINVAL;
		}
		dev_dbg(dev, "%s, tune name = %s, param = 0x%x, 0x%x, 0x%x, 0x%x\n",
			 __func__, tud->tune_param[idx].name,
			 tud->tune_param[idx].reg, tud->tune_param[idx].value,
			 tud->tune_param[idx].shift, tud->tune_param[idx].mask);

		idx++;
	}

	tune_param[idx].value = EXYNOS_USB_TUNE_LAST;

	return 0;
}

static int eusb_repeater_ctrl(int value)
{
	struct eusb_repeater_data *tud = g_tud;
	int ret = 0;

	if (!ret)
		tud->ctrl_sel_status = value;

	if (ret)
		dev_err(tud->dev, "Failed to select %s state\n", value ? "power on" : "power off");

	return ret;
}

static ssize_t
eusb_repeater_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct eusb_repeater_data *tud = dev_get_drvdata(dev);
	int len = 0, i;

	len += snprintf(buf + len, PAGE_SIZE, "\t==== Print Repeater Tune Value ====\n");
	len += snprintf(buf + len, PAGE_SIZE, "Tune value count : %d\n", tud->tune_cnt);

	for (i = 0; i < tud->tune_cnt; i++) {
		len += snprintf(buf + len, PAGE_SIZE, "%s\t\t\t: 0x%x\n",
				tud->tune_param[i].name,
				tud->tune_param[i].value);
	}

	return len;
}

static ssize_t
eusb_repeater_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	char tune_name[30];
	u32 tune_val;
	struct eusb_repeater_data *tud = dev_get_drvdata(dev);
	int ret, i;
	u8 read_data, write_data;

	if (sscanf(buf, "%29s %x", tune_name, &tune_val) != 2)
		return -EINVAL;

	if (tud->ctrl_sel_status == false) {
		eusb_repeater_ctrl(true);
		mdelay(3);
	}

	for (i = 0; i < tud->tune_cnt; i++) {
		if (!strncmp(tud->tune_param[i].name, tune_name,
			strlen(tud->tune_param[i].name))) {
			write_data = (u8)tune_val;
			ret = eusb_repeater_write_reg(tud, (u8)tud->tune_param[i].reg,
						 &write_data, 1);
			if (ret < 0)
				dev_info(&tud->client->dev, "%s: i2c write error\n", __func__);

			ret = eusb_repeater_read_reg(tud, (u8)tud->tune_param[i].reg,
						&read_data, 1);
			if (ret < 0)
				dev_info(&tud->client->dev, "%s: i2c read error\n", __func__);

			tud->tune_param[i].value = read_data;

			dev_info(&tud->client->dev, "%s, 0x%x = 0x%x\n", tud->tune_param[i].name,
				 tud->tune_param[i].reg, read_data);

		}
	}

	return n;
}
static DEVICE_ATTR_RW(eusb_repeater);

static struct attribute *eusb_repeater_attrs[] = {
	&dev_attr_eusb_repeater.attr,
	NULL
};
ATTRIBUTE_GROUPS(eusb_repeater);

static int eusb_repeater_parse_dt(struct device *dev, struct eusb_repeater_data *tud,
				     struct eusb_repeater_plat_data *pdata)
{
	struct device_node *np = dev->of_node;
	struct device_node *tune_node;
	int ret = 0;

	if (np == NULL) {
		dev_err(dev, "%s np is NULL\n", __func__);
		return -1;
	}

	tune_node = of_parse_phandle(dev->of_node, "repeater_tune_param", 0);
	if (tune_node != NULL) {
		ret = eusb_repeater_fill_tune_param(tud, tune_node);
		if (ret < 0) {
			dev_err(dev, "can't fill repeater tuning param\n");
			return -EINVAL;
		}
	} else
		dev_info(dev, "don't need repeater tuning param\n");

	return 0;
}

static const struct of_device_id eusb_repeater_match_table[] = {
		{ .compatible = "samsung,eusb-repeater",},
		{},
};

int eusb_repeater_power_off(void)
{
	struct eusb_repeater_data *tud = g_tud;

	if (!tud)
		return -EEXIST;

	eusb_repeater_ctrl(false);
	return 0;
}
EXPORT_SYMBOL_GPL(eusb_repeater_power_off);

int eusb_repeater_power_on(void)
{
	struct eusb_repeater_data *tud = g_tud;
	u8 read_data, write_data, shift, mask;
	int ret, i;

	if (!tud)
		return -EEXIST;

	eusb_repeater_ctrl(true);

	mdelay(3);

	ret = eusb_repeater_read_reg(tud, 0x50,
				&read_data, 1);
	if (ret < 0) {
		dev_info(tud->dev, "%s: i2c read error\n", __func__);
		goto err;
	}
	dev_info(tud->dev, "usb: %s : check eusb reg(0x50) = %x\n",
		__func__, read_data);

	for (i = 0; i < tud->tune_cnt; i++) {
		ret = eusb_repeater_read_reg(tud, tud->tune_param[i].reg,
					     &read_data, 1);
		if (ret < 0) {
			dev_err(tud->dev, "%s: i2c read error\n", __func__);
			goto err;
		}
		write_data = (u8)tud->tune_param[i].value;
		shift = (u8)tud->tune_param[i].shift;
		mask = (u8)tud->tune_param[i].mask;
		write_data = (read_data & ~(mask << shift)) | ((write_data & mask) << shift);

		ret = eusb_repeater_write_reg(tud, (u8)tud->tune_param[i].reg,
					      &write_data, 1);
		if (ret < 0) {
			dev_err(tud->dev, "%s: i2c write error\n", __func__);
			goto err;
		}

		ret = eusb_repeater_read_reg(tud, tud->tune_param[i].reg,
					     &read_data, 1);
		if (ret < 0) {
			dev_err(tud->dev, "%s: i2c read error\n", __func__);
			goto err;
		}
		tud->tune_param[i].value = (read_data >> shift) & mask;
		dev_dbg(tud->dev, "%s, %s: 0x%x=0x%x\n",  __func__,
			 tud->tune_param[i].name, tud->tune_param[i].reg,
			 tud->tune_param[i].value);

	}

	return 0;
err:
	return ret;

}
EXPORT_SYMBOL_GPL(eusb_repeater_power_on);

static int eusb_repeater_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct device_node *of_node = client->dev.of_node;
	struct eusb_repeater_data *tud;
	struct eusb_repeater_plat_data *pdata = client->dev.platform_data;
	int ret = 0;

	tud = kzalloc(sizeof(*tud), GFP_KERNEL);
	if (tud == NULL) {
		ret = -ENOMEM;
		goto err_repeater_nomem;
	}
	tud->dev = &client->dev;

	if (of_node) {
		pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			ret = -ENOMEM;
			goto err_repeater_func;
		}

		ret = eusb_repeater_parse_dt(&client->dev, tud, pdata);
		if (ret < 0)
			goto err_parse_dt;
	} else {
		pdata = client->dev.platform_data;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s: EIO err!\n", __func__);
		ret = -ENOMEM;
		goto err_parse_dt;
	}

	tud->client = client;
	tud->pdata = pdata;
	g_tud = tud;

	tud->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(tud->pinctrl)) {
		dev_err(&client->dev, "failed to allocate pinctrl ret: %ld\n", PTR_ERR(tud->pinctrl));
		ret = PTR_ERR(tud->pinctrl);
	} else {
		tud->init_state = pinctrl_lookup_state(tud->pinctrl, "init_state");
		if (IS_ERR(tud->init_state)) {
			dev_err(&client->dev, "failed to allocate pinctrl ret: %ld\n",
				PTR_ERR(tud->init_state));
			ret = PTR_ERR(tud->init_state);
			tud->pinctrl = NULL;
			tud->init_state = NULL;
		} else
			pinctrl_select_state(tud->pinctrl, tud->init_state);
	}

	i2c_set_clientdata(client, tud);
	mutex_init(&tud->i2c_mutex);

	/*
	 * eUSB repeater control should be done with ldo on.
	 * So control will be done according to utmi_init/exit.
	 * eusb_repeater_power_on();
	 */

	return 0;

err_parse_dt:
	devm_kfree(&client->dev, pdata);
err_repeater_func:
	kfree(tud);
err_repeater_nomem:
	dev_info(&client->dev, "%s: err = %d\n", __func__, ret);

	return ret;
}

static const struct i2c_device_id eusb_repeater_id[] = {
	{"eusb-repeater", 0},
	{}
};

static void eusb_repeater_shutdown(struct i2c_client *client)
{
}

static int eusb_repeater_remove(struct i2c_client *client)
{
	struct eusb_repeater_data *tud = i2c_get_clientdata(client);

	mutex_destroy(&tud->i2c_mutex);
	return 0;
}


#if IS_ENABLED(CONFIG_PM)
static int eusb_repeater_suspend(struct device *dev)
{
	return 0;
}

static int eusb_repeater_resume(struct device *dev)
{
	return 0;
}
#else
#define eusb_repeater_suspend NULL
#define eusb_repeater_resume NULL
#endif

#ifdef CONFIG_PM
static const struct dev_pm_ops eusb_repeater_pm_ops = {
	.suspend = eusb_repeater_suspend,
	.resume = eusb_repeater_resume,
};
#endif

static struct i2c_driver eusb_repeater_driver = {
	.probe  = eusb_repeater_probe,
	.remove = eusb_repeater_remove,
	.shutdown   = eusb_repeater_shutdown,
	.id_table   = eusb_repeater_id,
	.driver = {
		.name = "eusb-repeater",
		.owner = THIS_MODULE,
		.dev_groups = eusb_repeater_groups,
		.pm = &eusb_repeater_pm_ops,
		.of_match_table = eusb_repeater_match_table,
	},
};

static int __init eusb_repeater_init(void)
{
	return i2c_add_driver(&eusb_repeater_driver);
}

static void __exit eusb_repeater_exit(void)
{
	i2c_del_driver(&eusb_repeater_driver);
}
module_init(eusb_repeater_init);
module_exit(eusb_repeater_exit);

MODULE_DESCRIPTION("Samsung eUSB Repeater Driver");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");

