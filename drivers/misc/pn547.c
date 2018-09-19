/*
 * Copyright (C) 2010 Trusted Logic S.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/pn547.h>
#include <linux/clk.h>
#include <linux/device.h>

#define MAX_NORMAL_FRAME_SIZE	(255 + 3)
#define MAX_FIRMDL_FRAME_SIZE	(1023 + 5)

struct pn547_dev {
	wait_queue_head_t	read_wq;
	struct mutex		read_mutex;
	struct device		*dev;
	struct i2c_client	*client;
	struct miscdevice	pn547_device;
	struct pinctrl		*pinctrl;
	enum pn547_state	state;
	unsigned int 		ven_gpio;
	unsigned int 		firm_gpio;
	unsigned int		irq_gpio;
	bool			irq_enabled;
	spinlock_t		irq_enabled_lock;
	bool			irq_wake_up;
};

static int pn547_pinctrl_config(struct pn547_dev *pn547_dev, uint8_t active)
{
	struct pinctrl_state *state;
	const char *name = active ? "pn547-active" : "pn547-inactive";

	state = pinctrl_lookup_state(pn547_dev->pinctrl, name);
	if (IS_ERR(state)) {
		dev_err(pn547_dev->dev,
				"%s: pinctrl lookup state failed\n",
				__func__);

		return PTR_ERR(state);
	}

	return pinctrl_select_state(pn547_dev->pinctrl, state);
}

static void pn547_disable_irq(struct pn547_dev *pn547_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&pn547_dev->irq_enabled_lock, flags);
	if (pn547_dev->irq_enabled) {
		disable_irq_nosync(pn547_dev->client->irq);
		pn547_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&pn547_dev->irq_enabled_lock, flags);
}

static void pn547_enable_irq(struct pn547_dev *pn547_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&pn547_dev->irq_enabled_lock, flags);
	if (!pn547_dev->irq_enabled) {
		pn547_dev->irq_enabled = true;
		enable_irq(pn547_dev->client->irq);
	}
	spin_unlock_irqrestore(&pn547_dev->irq_enabled_lock, flags);
}

static irqreturn_t pn547_dev_irq_handler(int irq, void *dev_id)
{
	struct pn547_dev *pn547_dev = dev_id;

	if (device_may_wakeup(&pn547_dev->client->dev))
		pm_wakeup_event(&pn547_dev->client->dev, 2000);

	pn547_disable_irq(pn547_dev);

	/* Wake up waiting readers */
	wake_up(&pn547_dev->read_wq);

	return IRQ_HANDLED;
}

static ssize_t pn547_dev_read(struct file *filp, char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn547_dev *pn547_dev = filp->private_data;
	char tmp[MAX_FIRMDL_FRAME_SIZE];
	int ret, maxlen;
	bool fwdl;

	fwdl = pn547_dev->state == PN547_STATE_FWDL;
	maxlen = fwdl ? MAX_FIRMDL_FRAME_SIZE : MAX_NORMAL_FRAME_SIZE;
	if (count > maxlen)
		count = maxlen;

	dev_dbg(pn547_dev->dev, "%s: reading %zu bytes.\n", __func__, count);

	mutex_lock(&pn547_dev->read_mutex);

	if (!gpio_get_value(pn547_dev->irq_gpio)) {
		if (filp->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto fail;
		}

		while (1) {
			if (!pn547_dev->irq_enabled) {
				pn547_dev->irq_enabled = true;
				enable_irq(pn547_dev->client->irq);
			}

			ret = wait_event_interruptible(pn547_dev->read_wq,
					!pn547_dev->irq_enabled);

			pn547_disable_irq(pn547_dev);

			if (ret)
				goto fail;

			if (gpio_get_value(pn547_dev->irq_gpio))
				break;

			dev_err_ratelimited(&pn547_dev->client->dev,
			"%s: gpio is low, no need to read data\n", __func__);
		}
	}

	/* Read data */
	ret = i2c_master_recv(pn547_dev->client, tmp, count);

	if (ret < 0) {
		dev_err(pn547_dev->dev, "%s: i2c_master_recv returned %d\n",
				__func__, ret);
		return ret;
	}
	if (ret > count) {
		dev_err(pn547_dev->dev, "%s: received too many bytes from i2c (%d)\n",
				__func__, ret);
		return -EIO;
	}
	if (copy_to_user(buf, tmp, ret)) {
		dev_err(pn547_dev->dev, "%s: failed to copy to user space\n",
				__func__);
		return -EFAULT;
	}

	mutex_unlock(&pn547_dev->read_mutex);

	return ret;

fail:
	mutex_unlock(&pn547_dev->read_mutex);
	return ret;
}

static ssize_t pn547_dev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn547_dev  *pn547_dev;
	char tmp[MAX_FIRMDL_FRAME_SIZE];
	int ret, retry, maxlen;
	bool fwdl;

	pn547_dev = filp->private_data;

	fwdl = pn547_dev->state == PN547_STATE_FWDL;
	maxlen = fwdl ? MAX_FIRMDL_FRAME_SIZE : MAX_NORMAL_FRAME_SIZE;
	if (count > maxlen)
		count = maxlen;

	if (copy_from_user(tmp, buf, count)) {
		dev_err(pn547_dev->dev, "%s: failed to copy from user space\n",
				__func__);
		return -EFAULT;
	}

	dev_dbg(pn547_dev->dev, "%s: writing %zu bytes.\n", __func__, count);

	/* Write data */
	for (retry = 0; retry < 5; retry++) {
		ret = i2c_master_send(pn547_dev->client, tmp, count);
		if (ret < 0) {
			dev_err(&pn547_dev->client->dev,
				"%s: write failed, maybe in standby mode - retry(%d)\n",
				 __func__, retry);
			usleep_range(1000, 1100);
		} else if (ret == count)
			break;
	}

	if (ret != count) {
		dev_err(pn547_dev->dev, "%s: i2c_master_send returned %d\n",
				__func__, ret);
		ret = -EIO;
	}

	return ret;
}

static int pn547_dev_open(struct inode *inode, struct file *filp)
{
	struct pn547_dev *pn547_dev = container_of(filp->private_data,
				struct pn547_dev, pn547_device);

	filp->private_data = pn547_dev;

	dev_dbg(&pn547_dev->client->dev,
			"%s: %d,%d\n", __func__, imajor(inode), iminor(inode));

	return 0;
}

static long pn547_dev_ioctl(struct file *filp, unsigned int cmd,
				unsigned long arg)
{
	struct pn547_dev *pn547_dev = filp->private_data;
	int ret = 0, state;

	/* Activate pinctrl before bit banging. */
	ret = pn547_pinctrl_config(pn547_dev, 1);
	if (ret) {
		dev_err(pn547_dev->dev,
				"%s: pinctrl failed on chip configuration %d\n",
				__func__, ret);
		goto err;
	}

	switch (cmd) {
	case PN547_SET_PWR:
		if (arg == 2) {
			/* power on with firmware download (requires hw reset)
			 */
			gpio_set_value(pn547_dev->ven_gpio, 1);
			msleep(10);
			gpio_set_value(pn547_dev->firm_gpio, 1);
			msleep(10);
			gpio_set_value(pn547_dev->ven_gpio, 0);
			msleep(10);
			gpio_set_value(pn547_dev->ven_gpio, 1);
			msleep(10);
			state = PN547_STATE_FWDL;
		} else if (arg == 1) {
			/* power on */
			pn547_enable_irq(pn547_dev);
			gpio_set_value(pn547_dev->firm_gpio, 0);
			gpio_set_value(pn547_dev->ven_gpio, 1);
			msleep(10);
			state = PN547_STATE_ON;
		} else  if (arg == 0) {
			/* power off */
			pn547_disable_irq(pn547_dev);
			gpio_set_value(pn547_dev->firm_gpio, 0);
			gpio_set_value(pn547_dev->ven_gpio, 0);
			msleep(60);
			state = PN547_STATE_OFF;

			/* Suspend pinctrl when PN547 is turned off. */
			ret = pn547_pinctrl_config(pn547_dev, 0);
			if (ret) {
				dev_err(pn547_dev->dev,
						"%s: pinctrl failed on PN547_STATE_OFF %d\n",
					__func__, ret);
				goto err;
			}
		} else {
			dev_err(pn547_dev->dev, "%s: bad ioctl %lu\n",
					__func__, arg);
			return -EINVAL;
		}
		break;
	default:
		dev_err(pn547_dev->dev, "%s: bad ioctl %u\n", __func__, cmd);
		return -EINVAL;
	}

	if (pn547_dev->state != state)
		pn547_dev->state = state;

	return 0;

err:
	return ret;
}

static const struct file_operations pn547_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read  = pn547_dev_read,
	.write = pn547_dev_write,
	.open = pn547_dev_open,
	.unlocked_ioctl = pn547_dev_ioctl,
};

static int pn547_parse_dt(struct device *dev,
			 struct pn547_i2c_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	int ret = 0;

	ret = of_get_named_gpio(np, "nxp,irq_gpio", 0);
	if (ret < 0) {
		dev_err(dev, "failed to get \"nxp,irq_gpio\"\n");
		goto err;
	}
	pdata->irq_gpio = ret;

	ret = of_get_named_gpio(np, "nxp,dwld_en", 0);
	if (ret < 0) {
		dev_err(dev, "failed to get \"nxp,dwld_en\"\n");
		goto err;
	}
	pdata->firm_gpio = ret;

	ret = of_get_named_gpio(np, "nxp,ven", 0);
	if (ret < 0) {
		dev_err(dev, "failed to get \"nxp,ven\"\n");
		goto err;
	}
	pdata->ven_gpio = ret;

err:
	return ret;
}

static void pn547_pinctrl_destroy(struct pn547_dev *pn547_dev)
{
	int ret = 0;

	ret = pn547_pinctrl_config(pn547_dev, 0);
	if (ret)
		dev_err(pn547_dev->dev, "%s: pinctrl failed on destroy %d\n",
			__func__, ret);

	devm_pinctrl_put(pn547_dev->pinctrl);
}

static struct clk *nfc_clk;
static int pn547_clk_enable(struct device *dev)
{
	int ret = -1;

	nfc_clk = clk_get(dev, "nfc_clk");
	if (IS_ERR(nfc_clk)) {
		dev_err(dev, "%s: failed to get nfc_clk\n", __func__);
		return ret;
	}

	ret = clk_prepare(nfc_clk);
	if (ret) {
		dev_err(dev, "%s: failed to prepare nfc_clk\n", __func__);
		return ret;
	}

	return ret;
}

static void pn547_clk_disable(void)
{
	clk_unprepare(nfc_clk);
	clk_put(nfc_clk);
	nfc_clk = NULL;
}

static int pn547_gpio_request(struct device *dev,
				struct pn547_i2c_platform_data *pdata)
{
	int ret;

	ret = gpio_request(pdata->irq_gpio, "pn547_irq");
	if (ret)
		goto err_irq;
	ret = gpio_direction_input(pdata->irq_gpio);
	if (ret)
		goto err_irq;

	gpio_free(pdata->firm_gpio);
	ret = gpio_request(pdata->firm_gpio, "pn547_fw");
	if (ret)
		goto err_fwdl_en;
	ret = gpio_direction_output(pdata->firm_gpio, 0);
	if (ret)
		goto err_fwdl_en;

	ret = gpio_request(pdata->ven_gpio, "pn547_ven");
	if (ret)
		goto err_ven;
	ret = gpio_direction_output(pdata->ven_gpio, 0);
	if (ret)
		goto err_ven;

	return 0;

err_ven:
	gpio_free(pdata->firm_gpio);
err_fwdl_en:
	gpio_free(pdata->irq_gpio);
err_irq:
	dev_err(dev, "%s: gpio request err %d\n", __func__, ret);
	return ret;
}

static void pn547_gpio_release(struct pn547_i2c_platform_data *pdata)
{
	gpio_free(pdata->ven_gpio);
	gpio_free(pdata->irq_gpio);
	gpio_free(pdata->firm_gpio);
	pn547_clk_disable();
}

static int pn547_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret = 0;
	struct pn547_i2c_platform_data *platform_data;
	struct pn547_dev *pn547_dev;
	struct pinctrl *pinctrl;

	dev_dbg(&client->dev, "%s: enter\n", __func__);

	platform_data = kzalloc(sizeof(struct pn547_i2c_platform_data),
			GFP_KERNEL);
	if (!platform_data) {
		ret = -ENOMEM;
		goto err_platform_data;
	}

	if (platform_data == NULL) {
		ret = -ENODEV;
		goto err_platform_data;
	}

	pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(pinctrl)) {
		dev_err(&client->dev, "devm_pinctrl_get error\n");
		goto err_pinctrl;
	}

	ret = pn547_parse_dt(&client->dev, platform_data);
	if (ret < 0) {
		dev_err(&client->dev, "failed to parse device tree: %d\n", ret);
		goto err_parse_dt;
	}

	ret = pn547_gpio_request(&client->dev, platform_data);
	if (ret) {
		dev_err(&client->dev, "failed to request gpio\n");
		goto err_gpio_request;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s: need I2C_FUNC_I2C\n", __func__);
		ret = -ENODEV;
		goto err_i2c;
	}

	pn547_dev = kzalloc(sizeof(*pn547_dev), GFP_KERNEL);
	if (pn547_dev == NULL) {
		dev_err(&client->dev,
				"failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_exit;
	}

	pn547_dev->irq_gpio = platform_data->irq_gpio;
	pn547_dev->ven_gpio = platform_data->ven_gpio;
	pn547_dev->firm_gpio = platform_data->firm_gpio;
	pn547_dev->client = client;
	pn547_dev->dev = &client->dev;
	pn547_dev->pinctrl = pinctrl;
	pn547_dev->state = PN547_STATE_UNKNOWN;

	/* Initialise mutex and work queue */
	init_waitqueue_head(&pn547_dev->read_wq);
	mutex_init(&pn547_dev->read_mutex);
	spin_lock_init(&pn547_dev->irq_enabled_lock);

	pn547_dev->pn547_device.minor = MISC_DYNAMIC_MINOR;
	pn547_dev->pn547_device.name = PN547_DEVICE_NAME;
	pn547_dev->pn547_device.fops = &pn547_dev_fops;

	ret = misc_register(&pn547_dev->pn547_device);
	if (ret) {
		dev_err(&client->dev, "%s: misc_register failed\n", __func__);
		goto err_misc_register;
	}

	ret = pn547_clk_enable(&client->dev);

	/* NFC IRQ */
	pn547_dev->irq_enabled = true;
	ret = request_irq(client->irq, pn547_dev_irq_handler,
			  IRQF_TRIGGER_RISING, client->name, pn547_dev);
	if (ret) {
		dev_err(&client->dev, "%s: request_irq failed\n", __func__);
		goto err_request_irq_failed;
	}
	pn547_disable_irq(pn547_dev);

	device_init_wakeup(&client->dev, true);
	device_set_wakeup_capable(&client->dev, true);
	i2c_set_clientdata(client, pn547_dev);
	pn547_dev->irq_wake_up = false;

	dev_err(&client->dev, "%s: probing pn547 exited successfully\n",
				__func__);
	return 0;

err_request_irq_failed:
	misc_deregister(&pn547_dev->pn547_device);
err_misc_register:
	mutex_destroy(&pn547_dev->read_mutex);
	kfree(pn547_dev);
err_exit:
err_i2c:
	pn547_gpio_release(platform_data);
err_gpio_request:
err_parse_dt:
	devm_pinctrl_put(pinctrl);
err_pinctrl:
	kzfree(platform_data);
err_platform_data:
	dev_err(&client->dev,
	"%s: probing pn547 failed, check hardware\n", __func__);
	return ret;
}

static int pn547_remove(struct i2c_client *client)
{
	struct pn547_dev *pn547_dev;

	pn547_dev = i2c_get_clientdata(client);
	free_irq(client->irq, pn547_dev);
	misc_deregister(&pn547_dev->pn547_device);
	mutex_destroy(&pn547_dev->read_mutex);
	pn547_pinctrl_destroy(pn547_dev);
	gpio_free(pn547_dev->irq_gpio);
	gpio_free(pn547_dev->ven_gpio);
	gpio_free(pn547_dev->firm_gpio);

	kfree(pn547_dev);

	return 0;
}

static int pn547_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pn547_dev *pn547_dev = i2c_get_clientdata(client);

	if (device_may_wakeup(&client->dev) && pn547_dev->irq_enabled) {
		if (!enable_irq_wake(client->irq))
			pn547_dev->irq_wake_up = true;
	}

	return 0;
}

static int pn547_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pn547_dev *pn547_dev = i2c_get_clientdata(client);

	if (device_may_wakeup(&client->dev) && pn547_dev->irq_wake_up) {
		if (!disable_irq_wake(client->irq))
			pn547_dev->irq_wake_up = false;
	}

	return 0;
}

static const struct i2c_device_id pn547_id[] = {
	{ PN547_DEVICE_NAME, 0 },
	{ },
};

static struct of_device_id pn547_match_table[] = {
	{ .compatible = "nxp,pn547", },
	{ },
};

static const struct dev_pm_ops pn547_pm_ops = {
	.suspend = pn547_suspend,
	.resume  = pn547_resume,
};

static struct i2c_driver pn547_driver = {
	.id_table = pn547_id,
	.probe = pn547_probe,
	.remove = pn547_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = PN547_DEVICE_NAME,
		.of_match_table = pn547_match_table,
		.pm = &pn547_pm_ops,
	},
};

module_i2c_driver(pn547_driver);

MODULE_AUTHOR("Sylvain Fonteneau");
MODULE_DESCRIPTION("NFC PN547 driver");
MODULE_LICENSE("GPL");
