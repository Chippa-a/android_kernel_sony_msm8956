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
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/pn547.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>

#define MAX_NORMAL_FRAME_SIZE	(255 + 3)
#define MAX_FIRMDL_FRAME_SIZE	(1023 + 5)

struct pn547_dev {
	wait_queue_head_t	read_wq;
	struct mutex		read_mutex;
	struct i2c_client	*client;
	struct miscdevice	pn547_device;
	unsigned int 		ven_gpio;
	unsigned int 		firm_gpio;
	bool				irq_enabled;
	spinlock_t			irq_enabled_lock;
	unsigned int		irq_gpio;
	unsigned int 		irq_wakeup_state;
	struct pinctrl		*pinctrl;
	enum pn547_state	state;
};

static int pn547_parse_dt(struct device *dev, struct pn547_dev *pdata)
{
	struct device_node *np = dev->of_node;
	int ret = 0;

	ret = of_get_named_gpio(np, "nxp,irq_gpio", 0);
	if (ret < 0) {
		pr_err("failed to get \"nxp,irq_gpio\"\n");
		goto err;
	}
	pdata->irq_gpio = ret;

	ret = of_get_named_gpio(np, "nxp,dwld_en", 0);
	if (ret < 0) {
		pr_err("failed to get \"nxp,dwld_en\"\n");
		goto err;
	}
	pdata->firm_gpio = ret;

	ret = of_get_named_gpio(np, "nxp,ven", 0);
	if (ret < 0) {
		pr_err("failed to get \"nxp,ven\"\n");
		goto err;
	}
	pdata->ven_gpio = ret;

err:
	return ret;
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

static irqreturn_t pn547_dev_irq_handler(int irq, void *dev_id)
{
	struct pn547_dev *pn547_dev = dev_id;

	if (!gpio_get_value(pn547_dev->irq_gpio)) {
		return IRQ_HANDLED;
	}

	pn547_disable_irq(pn547_dev);

	/* Wake up waiting readers */
	wake_up(&pn547_dev->read_wq);

	return IRQ_HANDLED;
}

static ssize_t pn547_dev_read(struct file *filp, char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn547_dev *pn547_dev = filp->private_data;
	char *tmp;
	int ret, maxlen;
	bool fwdl;

	fwdl = pn547_dev->state == PN547_STATE_FWDL;
	maxlen = fwdl ? MAX_FIRMDL_FRAME_SIZE : MAX_NORMAL_FRAME_SIZE;

	if (count > maxlen)
		count = maxlen;

	tmp = kzalloc(count * sizeof(char), GFP_KERNEL);
	if (tmp == NULL) {
		pr_err("%s: Failed to allocate %s tmp\n", __func__, tmp);
		ret = -EFAULT;
		goto fail;
	}

	mutex_lock(&pn547_dev->read_mutex);

	if (!gpio_get_value(pn547_dev->irq_gpio)) {
		if (filp->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto fail;
		}

		pn547_dev->irq_enabled = true;
		enable_irq(pn547_dev->client->irq);
		ret = wait_event_interruptible(pn547_dev->read_wq,
				gpio_get_value(pn547_dev->irq_gpio));

		pn547_disable_irq(pn547_dev);

		if (ret)
			goto fail;
	}

	/* Read data */
	ret = i2c_master_recv(pn547_dev->client, tmp, count);
	mutex_unlock(&pn547_dev->read_mutex);

	if (ret < 0) {
		pr_err("%s: i2c_master_recv returned %d\n", __func__, ret);
		goto exit;
	}

	if (ret > count) {
		pr_err("%s: received too many bytes from i2c (%d)\n",
			__func__, ret);
		ret = -EIO;
		goto exit;
	}

	if (copy_to_user(buf, tmp, ret)) {
		pr_err("%s : failed to copy to user space\n", __func__);
		ret = -EFAULT;
		goto exit;
	}

exit:
	kfree(tmp);
	return ret;

fail:
	kfree(tmp);
	mutex_unlock(&pn547_dev->read_mutex);
	return ret;
}

static ssize_t pn547_dev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn547_dev  *pn547_dev;
	char *tmp;
	int ret, retry = 5, maxlen;
	bool fwdl;

	pn547_dev = filp->private_data;

	fwdl = pn547_dev->state == PN547_STATE_FWDL;
	maxlen = fwdl ? MAX_FIRMDL_FRAME_SIZE : MAX_NORMAL_FRAME_SIZE;

	if (count > maxlen)
		count = maxlen;

	tmp = kzalloc(count * sizeof(char), GFP_KERNEL);
	if (tmp == NULL) {
		pr_err("%s: Failed to allocate %s tmp\n", __func__, tmp);
		ret = -EFAULT;
		goto exit;
	}

	if (copy_from_user(tmp, buf, count)) {
		pr_err("%s : failed to copy from user space\n", __func__);
		ret = -EFAULT;
		goto exit;
	}

	/* Write data */
	do {
		retry--;
		ret = i2c_master_send(pn547_dev->client, tmp, count);
		if (ret == count)
			break;
		usleep_range(6000, 10000);
	} while (retry);

	if (ret != count) {
		pr_err("%s : i2c_master_send returned %d\n", __func__, ret);
		ret = -EIO;
		goto exit;
	}

exit:
	kfree(tmp);
	return ret;
}

static int pn547_dev_open(struct inode *inode, struct file *filp)
{
	struct pn547_dev *pn547_dev = container_of(filp->private_data,
						struct pn547_dev,
						pn547_device);

	filp->private_data = pn547_dev;

	pr_err("%s : %d,%d\n", __func__, imajor(inode), iminor(inode));

	return 0;
}

static int pn547_pinctrl_config(struct pn547_dev *dev, uint8_t active)
{
	struct pinctrl_state *state;
	const char *name = active ? "pn547-active" : "pn547-inactive";

	state = pinctrl_lookup_state(dev->pinctrl, name);
	if (IS_ERR(state)) {
		pr_err("%s: pinctrl lookup state failed\n", __func__);
		return PTR_ERR(state);
	}

	return pinctrl_select_state(dev->pinctrl, state);
}

static long pn547_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int state, ret = 0;
	struct pn547_dev *pn547_dev = filp->private_data;

	// Activate pinctrl
	ret = pn547_pinctrl_config(pn547_dev, 1);
	if (ret)
		pr_err("%s: pinctrl failed ON chip\n", __func__);

	switch (cmd) {
	case PN547_SET_PWR:
		if (arg == 2) {
			/* power on with firmware download (requires hw reset)
			 */
			gpio_set_value(pn547_dev->ven_gpio, 1);
			gpio_set_value(pn547_dev->firm_gpio, 1);
			msleep(10);
			gpio_set_value(pn547_dev->ven_gpio, 0);
			msleep(50);
			gpio_set_value(pn547_dev->ven_gpio, 1);
			msleep(10);
			state = PN547_STATE_FWDL;
		} else if (arg == 1) {
			/* power on */
			if (pn547_dev->irq_wakeup_state == 0){
				irq_set_irq_wake(pn547_dev->client->irq, 1);
				pn547_dev->irq_wakeup_state = 1;
			}
			gpio_set_value(pn547_dev->firm_gpio, 0);
			gpio_set_value(pn547_dev->ven_gpio, 1);
			msleep(10);
			state = PN547_STATE_ON;
		} else  if (arg == 0) {
			/* power off */
			if (pn547_dev->irq_wakeup_state == 1){	
				irq_set_irq_wake(pn547_dev->client->irq, 0);
				pn547_dev->irq_wakeup_state = 0;
			}
			gpio_set_value(pn547_dev->firm_gpio, 0);
			gpio_set_value(pn547_dev->ven_gpio, 0);
			msleep(10);
			state = PN547_STATE_OFF;
			ret = pn547_pinctrl_config(pn547_dev, 0);
			if (ret)
				pr_err("%s: pinctrl failed OFF chip\n", __func__);
		} else {
			pr_err("%s bad arg %lu\n", __func__, arg);
			return -EINVAL;
		}
		break;
	default:
		pr_err("%s bad ioctl %u\n", __func__, cmd);
		return -EINVAL;
	}

	if (pn547_dev->state != state)
		pn547_dev->state = state;

	return 0;
}

static const struct file_operations pn547_dev_fops = {
	.owner	= THIS_MODULE,
	.llseek	= no_llseek,
	.read	= pn547_dev_read,
	.write	= pn547_dev_write,
	.open	= pn547_dev_open,
	.unlocked_ioctl  = pn547_dev_ioctl,
};

static int pn547_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{	
	struct pn547_dev *pn547_dev;
	struct clk *nfc_clk = NULL;
	struct pinctrl *pinctrl;
	int ret = 0;

	pn547_dev = kzalloc(sizeof(*pn547_dev), GFP_KERNEL);
	if (pn547_dev == NULL) {
		dev_err(&client->dev,
				"failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_exit;
	}

	pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(pinctrl)) {
		dev_err(&client->dev, "devm_pinctrl_get error\n");
		ret = PTR_ERR(pinctrl);
	}

	pn547_dev->client = client;
	ret = pn547_parse_dt(&client->dev, pn547_dev);
	if (ret < 0) {
		dev_err(&client->dev, "failed to parse device tree: %d\n", ret);
		devm_pinctrl_put(pinctrl);
		goto err_parse_dt;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s: i2c check failed\n", __func__);
		return -ENODEV;
	}

	nfc_clk = clk_get(&client->dev, "nfc_clk");
	if (IS_ERR(nfc_clk)) {
		dev_err(&client->dev, "Couldn't get nfc_clk\n");
		goto err_clk;
	}
	ret = clk_prepare_enable(nfc_clk);
	if (ret) {
		dev_err(&client->dev, "nfc_clk enable is failed\n");
		goto err_clk_enable;
	}

	pn547_dev->state = PN547_STATE_UNKNOWN;
	pn547_dev->pinctrl = pinctrl;

	/* init mutex and queues */
	init_waitqueue_head(&pn547_dev->read_wq);
	mutex_init(&pn547_dev->read_mutex);
	spin_lock_init(&pn547_dev->irq_enabled_lock);

	pn547_dev->pn547_device.minor = MISC_DYNAMIC_MINOR;
	pn547_dev->pn547_device.name = PN547_DEVICE_NAME;
	pn547_dev->pn547_device.fops = &pn547_dev_fops;

	ret = misc_register(&pn547_dev->pn547_device);
	if (ret) {
		pr_err("%s : misc_register failed\n", __FILE__);
		goto err_misc_register;
	}

	ret = gpio_request(pn547_dev->irq_gpio, "nfc_int");
	if (ret)
		return -ENODEV;

	ret = gpio_request(pn547_dev->ven_gpio, "nfc_ven");
	if (ret)
		goto err_ven;

	ret = gpio_request(pn547_dev->firm_gpio, "nfc_firm");
	if (ret)
		goto err_firm;

	gpio_direction_input(pn547_dev->irq_gpio);
	gpio_direction_output(pn547_dev->firm_gpio, 0);
	gpio_direction_output(pn547_dev->ven_gpio, 0);
	
	client->irq = gpio_to_irq(pn547_dev->irq_gpio);
	pr_info("%s : requesting IRQ %d\n", __func__, client->irq);
	pn547_dev->irq_wakeup_state = 0;
	pn547_dev->irq_enabled = true;

	ret = request_irq(client->irq, pn547_dev_irq_handler,
			  IRQF_TRIGGER_HIGH, client->name, pn547_dev);
	if (ret) {
		dev_err(&client->dev, "request_irq failed\n");
		goto err_request_irq_failed;
	}

	pn547_disable_irq(pn547_dev);

	i2c_set_clientdata(client, pn547_dev);

	return 0;

err_request_irq_failed:
	misc_deregister(&pn547_dev->pn547_device);
err_misc_register:
	mutex_destroy(&pn547_dev->read_mutex);
	kfree(pn547_dev);
err_exit:
	gpio_free(pn547_dev->firm_gpio);
err_firm:
	gpio_free(pn547_dev->ven_gpio);
err_ven:
	gpio_free(pn547_dev->irq_gpio);
err_clk_enable:
	clk_put(nfc_clk);
err_clk:
err_parse_dt:
	kfree(pn547_dev);

	return ret;
}

static int pn547_remove(struct i2c_client *client)
{
	struct pn547_dev *pn547_dev;
	int ret = 0;

	pn547_dev = i2c_get_clientdata(client);
	free_irq(client->irq, pn547_dev);
	ret = pn547_pinctrl_config(pn547_dev, 0);
	if (ret)
		pr_err("%s: pinctrl failed on remove\n", __func__);
	devm_pinctrl_put(pn547_dev->pinctrl);
	misc_deregister(&pn547_dev->pn547_device);
	mutex_destroy(&pn547_dev->read_mutex);
	gpio_free(pn547_dev->irq_gpio);
	gpio_free(pn547_dev->ven_gpio);
	gpio_free(pn547_dev->firm_gpio);
	kfree(pn547_dev);

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

static struct i2c_driver pn547_driver = {
	.id_table	= pn547_id,
	.probe		= pn547_probe,
	.remove		= pn547_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= PN547_DEVICE_NAME,
		.of_match_table = pn547_match_table,
	},
};

static int __init pn547_dev_init(void)
{
	pr_info("Loading pn547 driver\n");
	return i2c_add_driver(&pn547_driver);
}
module_init(pn547_dev_init);

static void __exit pn547_dev_exit(void)
{
	pr_info("Unloading pn547 driver\n");
	i2c_del_driver(&pn547_driver);
}
module_exit(pn547_dev_exit);

MODULE_AUTHOR("Ernest Li");
MODULE_DESCRIPTION("NFC PN547 driver");
MODULE_LICENSE("GPL");
