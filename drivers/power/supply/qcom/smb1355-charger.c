/* Copyright (c) 2016-2017 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "SMB1355: %s: " fmt, __func__

#include <linux/device.h>
#include <linux/regmap.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/iio/consumer.h>
#include <linux/platform_device.h>
#include <linux/qpnp/qpnp-revid.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/power_supply.h>
#include <linux/pmic-voter.h>

#define SMB1355_DEFAULT_FCC_UA 1000000

/* SMB1355 registers, different than mentioned in smb-reg.h */

#define CHGR_BASE	0x1000
#define BATIF_BASE	0x1200
#define USBIN_BASE	0x1300
#define MISC_BASE	0x1600

#define BATTERY_STATUS_2_REG			(CHGR_BASE + 0x0B)
#define DISABLE_CHARGING_BIT			BIT(3)

#define BATTERY_STATUS_3_REG			(CHGR_BASE + 0x0C)
#define BATT_GT_PRE_TO_FAST_BIT			BIT(4)
#define ENABLE_CHARGING_BIT			BIT(3)

#define CHGR_CFG2_REG				(CHGR_BASE + 0x51)
#define CHG_EN_SRC_BIT				BIT(7)
#define CHG_EN_POLARITY_BIT			BIT(6)

#define CFG_REG					(CHGR_BASE + 0x53)
#define CHG_OPTION_PIN_TRIM_BIT			BIT(7)
#define BATN_SNS_CFG_BIT			BIT(4)
#define CFG_TAPER_DIS_AFVC_BIT			BIT(3)
#define BATFET_SHUTDOWN_CFG_BIT			BIT(2)
#define VDISCHG_EN_CFG_BIT			BIT(1)
#define VCHG_EN_CFG_BIT				BIT(0)

#define FAST_CHARGE_CURRENT_CFG_REG		(CHGR_BASE + 0x61)
#define FAST_CHARGE_CURRENT_SETTING_MASK	GENMASK(7, 0)

#define CHGR_BATTOV_CFG_REG			(CHGR_BASE + 0x70)
#define BATTOV_SETTING_MASK			GENMASK(7, 0)

#define TEMP_COMP_STATUS_REG			(MISC_BASE + 0x07)
#define SKIN_TEMP_RST_HOT_BIT			BIT(6)
#define SKIN_TEMP_UB_HOT_BIT			BIT(5)
#define SKIN_TEMP_LB_HOT_BIT			BIT(4)
#define DIE_TEMP_TSD_HOT_BIT			BIT(3)
#define DIE_TEMP_RST_HOT_BIT			BIT(2)
#define DIE_TEMP_UB_HOT_BIT			BIT(1)
#define DIE_TEMP_LB_HOT_BIT			BIT(0)

#define BARK_BITE_WDOG_PET_REG			(MISC_BASE + 0x43)
#define BARK_BITE_WDOG_PET_BIT			BIT(0)

#define WD_CFG_REG				(MISC_BASE + 0x51)
#define WATCHDOG_TRIGGER_AFP_EN_BIT		BIT(7)
#define BARK_WDOG_INT_EN_BIT			BIT(6)
#define BITE_WDOG_INT_EN_BIT			BIT(5)
#define WDOG_IRQ_SFT_BIT			BIT(2)
#define WDOG_TIMER_EN_ON_PLUGIN_BIT		BIT(1)
#define WDOG_TIMER_EN_BIT			BIT(0)

#define SNARL_BARK_BITE_WD_CFG_REG		(MISC_BASE + 0x53)
#define BITE_WDOG_DISABLE_CHARGING_CFG_BIT	BIT(7)
#define SNARL_WDOG_TIMEOUT_MASK			GENMASK(6, 4)
#define BARK_WDOG_TIMEOUT_MASK			GENMASK(3, 2)
#define BITE_WDOG_TIMEOUT_MASK			GENMASK(1, 0)

struct smb_chg_param {
	const char	*name;
	u16		reg;
	int		min_u;
	int		max_u;
	int		step_u;
};

struct smb_params {
	struct smb_chg_param	fcc;
	struct smb_chg_param	ov;
};

static struct smb_params v1_params = {
	.fcc		= {
		.name	= "fast charge current",
		.reg	= FAST_CHARGE_CURRENT_CFG_REG,
		.min_u	= 0,
		.max_u	= 6000000,
		.step_u	= 25000,
	},
	.ov		= {
		.name	= "battery over voltage",
		.reg	= CHGR_BATTOV_CFG_REG,
		.min_u	= 2450000,
		.max_u	= 5000000,
		.step_u	= 10000,
	},
};

struct smb_irq_info {
	const char		*name;
	const irq_handler_t	handler;
	const bool		wake;
	int			irq;
};

struct smb_iio {
	struct iio_channel	*temp_chan;
	struct iio_channel	*temp_max_chan;
};

struct smb1355 {
	struct device		*dev;
	char			*name;
	struct regmap		*regmap;

	struct smb_params	param;
	struct smb_iio		iio;

	struct mutex		write_lock;

	struct power_supply	*parallel_psy;
	struct pmic_revid_data	*pmic_rev_id;
};

static bool is_secure(struct smb1355 *chip, int addr)
{
	/* assume everything above 0xA0 is secure */
	return (addr & 0xFF) >= 0xA0;
}

static int smb1355_read(struct smb1355 *chip, u16 addr, u8 *val)
{
	unsigned int temp;
	int rc;

	rc = regmap_read(chip->regmap, addr, &temp);
	if (rc >= 0)
		*val = (u8)temp;

	return rc;
}

static int smb1355_masked_write(struct smb1355 *chip, u16 addr, u8 mask, u8 val)
{
	int rc;

	mutex_lock(&chip->write_lock);
	if (is_secure(chip, addr)) {
		rc = regmap_write(chip->regmap, (addr & 0xFF00) | 0xD0, 0xA5);
		if (rc < 0)
			goto unlock;
	}

	rc = regmap_update_bits(chip->regmap, addr, mask, val);

unlock:
	mutex_unlock(&chip->write_lock);
	return rc;
}

static int smb1355_write(struct smb1355 *chip, u16 addr, u8 val)
{
	int rc;

	mutex_lock(&chip->write_lock);

	if (is_secure(chip, addr)) {
		rc = regmap_write(chip->regmap, (addr & ~(0xFF)) | 0xD0, 0xA5);
		if (rc < 0)
			goto unlock;
	}

	rc = regmap_write(chip->regmap, addr, val);

unlock:
	mutex_unlock(&chip->write_lock);
	return rc;
}

static int smb1355_set_charge_param(struct smb1355 *chip,
			struct smb_chg_param *param, int val_u)
{
	int rc;
	u8 val_raw;

	if (val_u > param->max_u || val_u < param->min_u) {
		pr_err("%s: %d is out of range [%d, %d]\n",
			param->name, val_u, param->min_u, param->max_u);
		return -EINVAL;
	}

	val_raw = (val_u - param->min_u) / param->step_u;

	rc = smb1355_write(chip, param->reg, val_raw);
	if (rc < 0) {
		pr_err("%s: Couldn't write 0x%02x to 0x%04x rc=%d\n",
			param->name, val_raw, param->reg, rc);
		return rc;
	}

	return rc;
}

static int smb1355_get_charge_param(struct smb1355 *chip,
			struct smb_chg_param *param, int *val_u)
{
	int rc;
	u8 val_raw;

	rc = smb1355_read(chip, param->reg, &val_raw);
	if (rc < 0) {
		pr_err("%s: Couldn't read from 0x%04x rc=%d\n",
			param->name, param->reg, rc);
		return rc;
	}

	*val_u = val_raw * param->step_u + param->min_u;

	return rc;
}

static irqreturn_t smb1355_handle_chg_state_change(int irq, void *data)
{
	struct smb1355 *chip = data;

	if (chip->parallel_psy)
		power_supply_changed(chip->parallel_psy);

	return IRQ_HANDLED;
}

static irqreturn_t smb1355_handle_wdog_bark(int irq, void *data)
{
	struct smb1355 *chip = data;
	int rc;

	rc = smb1355_write(chip, BARK_BITE_WDOG_PET_REG,
					BARK_BITE_WDOG_PET_BIT);
	if (rc < 0)
		pr_err("Couldn't pet the dog rc=%d\n", rc);

	return IRQ_HANDLED;
}

/*****************************
 * PARALLEL PSY REGISTRATION *
 *****************************/

static enum power_supply_property smb1355_parallel_props[] = {
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_PIN_ENABLED,
	POWER_SUPPLY_PROP_INPUT_SUSPEND,
	POWER_SUPPLY_PROP_CHARGER_TEMP,
	POWER_SUPPLY_PROP_CHARGER_TEMP_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_PARALLEL_MODE,
	POWER_SUPPLY_PROP_CONNECTOR_HEALTH,
};

static int smb1355_get_prop_batt_charge_type(struct smb1355 *chip,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smb1355_read(chip, BATTERY_STATUS_3_REG, &stat);
	if (rc < 0) {
		pr_err("Couldn't read SMB1355_BATTERY_STATUS_3 rc=%d\n", rc);
		return rc;
	}

	if (stat & ENABLE_CHARGING_BIT) {
		if (stat & BATT_GT_PRE_TO_FAST_BIT)
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
		else
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	} else {
		val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
	}

	return rc;
}

static int smb1355_get_parallel_charging(struct smb1355 *chip, int *disabled)
{
	int rc;
	u8 cfg2;

	rc = smb1355_read(chip, CHGR_CFG2_REG, &cfg2);
	if (rc < 0) {
		pr_err("Couldn't read en_cmg_reg rc=%d\n", rc);
		return rc;
	}

	if (cfg2 & CHG_EN_SRC_BIT)
		*disabled = 0;
	else
		*disabled = 1;

	return 0;
}

static int smb1355_get_prop_connector_health(struct smb1355 *chip)
{
	u8 temp;
	int rc;

	rc = smb1355_read(chip, TEMP_COMP_STATUS_REG, &temp);
	if (rc < 0) {
		pr_err("Couldn't read comp stat reg rc = %d\n", rc);
		return POWER_SUPPLY_HEALTH_UNKNOWN;
	}

	if (temp & SKIN_TEMP_RST_HOT_BIT)
		return POWER_SUPPLY_HEALTH_OVERHEAT;

	if (temp & SKIN_TEMP_UB_HOT_BIT)
		return POWER_SUPPLY_HEALTH_HOT;

	if (temp & SKIN_TEMP_LB_HOT_BIT)
		return POWER_SUPPLY_HEALTH_WARM;

	return POWER_SUPPLY_HEALTH_COOL;
}


static int smb1355_get_prop_charger_temp(struct smb1355 *chip,
				union power_supply_propval *val)
{
	int rc;

	if (!chip->iio.temp_chan ||
		PTR_ERR(chip->iio.temp_chan) == -EPROBE_DEFER)
		chip->iio.temp_chan = devm_iio_channel_get(chip->dev,
						"charger_temp");

	if (IS_ERR(chip->iio.temp_chan))
		return PTR_ERR(chip->iio.temp_chan);

	rc = iio_read_channel_processed(chip->iio.temp_chan, &val->intval);
	val->intval /= 100;
	return rc;
}

static int smb1355_get_prop_charger_temp_max(struct smb1355 *chip,
				union power_supply_propval *val)
{
	int rc;

	if (!chip->iio.temp_max_chan ||
		PTR_ERR(chip->iio.temp_max_chan) == -EPROBE_DEFER)
		chip->iio.temp_max_chan = devm_iio_channel_get(chip->dev,
							"charger_temp_max");
	if (IS_ERR(chip->iio.temp_max_chan))
		return PTR_ERR(chip->iio.temp_max_chan);

	rc = iio_read_channel_processed(chip->iio.temp_max_chan, &val->intval);
	val->intval /= 100;
	return rc;
}

static int smb1355_parallel_get_prop(struct power_supply *psy,
				     enum power_supply_property prop,
				     union power_supply_propval *val)
{
	struct smb1355 *chip = power_supply_get_drvdata(psy);
	u8 stat;
	int rc = 0;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		rc = smb1355_get_prop_batt_charge_type(chip, val);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		rc = smb1355_read(chip, BATTERY_STATUS_3_REG, &stat);
		if (rc >= 0)
			val->intval = (bool)(stat & ENABLE_CHARGING_BIT);
		break;
	case POWER_SUPPLY_PROP_PIN_ENABLED:
		rc = smb1355_read(chip, BATTERY_STATUS_2_REG, &stat);
		if (rc >= 0)
			val->intval = !(stat & DISABLE_CHARGING_BIT);
		break;
	case POWER_SUPPLY_PROP_CHARGER_TEMP:
		rc = smb1355_get_prop_charger_temp(chip, val);
		break;
	case POWER_SUPPLY_PROP_CHARGER_TEMP_MAX:
		rc = smb1355_get_prop_charger_temp_max(chip, val);
		break;
	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
		rc = smb1355_get_parallel_charging(chip, &val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		rc = smb1355_get_charge_param(chip, &chip->param.ov,
						&val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		rc = smb1355_get_charge_param(chip, &chip->param.fcc,
						&val->intval);
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = chip->name;
		break;
	case POWER_SUPPLY_PROP_PARALLEL_MODE:
		val->intval = POWER_SUPPLY_PL_USBMID_USBMID;
		break;
	case POWER_SUPPLY_PROP_CONNECTOR_HEALTH:
		val->intval = smb1355_get_prop_connector_health(chip);
		break;
	default:
		pr_err_ratelimited("parallel psy get prop %d not supported\n",
			prop);
		return -EINVAL;
	}

	if (rc < 0) {
		pr_debug("Couldn't get prop %d rc = %d\n", prop, rc);
		return -ENODATA;
	}

	return rc;
}

static int smb1355_set_parallel_charging(struct smb1355 *chip, bool disable)
{
	int rc;

	rc = smb1355_masked_write(chip, WD_CFG_REG, WDOG_TIMER_EN_BIT,
				 disable ? 0 : WDOG_TIMER_EN_BIT);
	if (rc < 0) {
		pr_err("Couldn't %s watchdog rc=%d\n",
		       disable ? "disable" : "enable", rc);
		disable = true;
	}

	/*
	 * Configure charge enable for high polarity and
	 * When disabling charging set it to cmd register control(cmd bit=0)
	 * When enabling charging set it to pin control
	 */
	rc = smb1355_masked_write(chip, CHGR_CFG2_REG,
			CHG_EN_POLARITY_BIT | CHG_EN_SRC_BIT,
			disable ? 0 : CHG_EN_SRC_BIT);
	if (rc < 0) {
		pr_err("Couldn't configure charge enable source rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static int smb1355_parallel_set_prop(struct power_supply *psy,
				     enum power_supply_property prop,
				     const union power_supply_propval *val)
{
	struct smb1355 *chip = power_supply_get_drvdata(psy);
	int rc = 0;

	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
		rc = smb1355_set_parallel_charging(chip, (bool)val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		rc = smb1355_set_charge_param(chip, &chip->param.ov,
						val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		rc = smb1355_set_charge_param(chip, &chip->param.fcc,
						val->intval);
		break;
	default:
		pr_debug("parallel power supply set prop %d not supported\n",
			prop);
		return -EINVAL;
	}

	return rc;
}

static int smb1355_parallel_prop_is_writeable(struct power_supply *psy,
					      enum power_supply_property prop)
{
	return 0;
}

static struct power_supply_desc parallel_psy_desc = {
	.name			= "parallel",
	.type			= POWER_SUPPLY_TYPE_PARALLEL,
	.properties		= smb1355_parallel_props,
	.num_properties		= ARRAY_SIZE(smb1355_parallel_props),
	.get_property		= smb1355_parallel_get_prop,
	.set_property		= smb1355_parallel_set_prop,
	.property_is_writeable	= smb1355_parallel_prop_is_writeable,
};

static int smb1355_init_parallel_psy(struct smb1355 *chip)
{
	struct power_supply_config parallel_cfg = {};

	parallel_cfg.drv_data = chip;
	parallel_cfg.of_node = chip->dev->of_node;

	/* change to smb1355's property list */
	parallel_psy_desc.properties = smb1355_parallel_props;
	parallel_psy_desc.num_properties = ARRAY_SIZE(smb1355_parallel_props);
	chip->parallel_psy = devm_power_supply_register(chip->dev,
						   &parallel_psy_desc,
						   &parallel_cfg);
	if (IS_ERR(chip->parallel_psy)) {
		pr_err("Couldn't register parallel power supply\n");
		return PTR_ERR(chip->parallel_psy);
	}

	return 0;
}

/***************************
 * HARDWARE INITIALIZATION *
 ***************************/

static int smb1355_init_hw(struct smb1355 *chip)
{
	int rc;

	/* enable watchdog bark and bite interrupts, and disable the watchdog */
	rc = smb1355_masked_write(chip, WD_CFG_REG, WDOG_TIMER_EN_BIT
			| WDOG_TIMER_EN_ON_PLUGIN_BIT | BITE_WDOG_INT_EN_BIT
			| BARK_WDOG_INT_EN_BIT,
			BITE_WDOG_INT_EN_BIT | BARK_WDOG_INT_EN_BIT);
	if (rc < 0) {
		pr_err("Couldn't configure the watchdog rc=%d\n", rc);
		return rc;
	}

	/* disable charging when watchdog bites */
	rc = smb1355_masked_write(chip, SNARL_BARK_BITE_WD_CFG_REG,
				 BITE_WDOG_DISABLE_CHARGING_CFG_BIT,
				 BITE_WDOG_DISABLE_CHARGING_CFG_BIT);
	if (rc < 0) {
		pr_err("Couldn't configure the watchdog bite rc=%d\n", rc);
		return rc;
	}

	/* disable parallel charging path */
	rc = smb1355_set_parallel_charging(chip, true);
	if (rc < 0) {
		pr_err("Couldn't disable parallel path rc=%d\n", rc);
		return rc;
	}

	/* initialize FCC to 0 */
	rc = smb1355_set_charge_param(chip, &chip->param.fcc, 0);
	if (rc < 0) {
		pr_err("Couldn't set 0 FCC rc=%d\n", rc);
		return rc;
	}

	/* enable parallel current sensing */
	rc = smb1355_masked_write(chip, CFG_REG,
				 VCHG_EN_CFG_BIT, VCHG_EN_CFG_BIT);
	if (rc < 0) {
		pr_err("Couldn't enable parallel current sensing rc=%d\n",
			rc);
		return rc;
	}

	return 0;
}

/**************************
 * INTERRUPT REGISTRATION *
 **************************/
static struct smb_irq_info smb1355_irqs[] = {
	[0] = {
		.name		= "wdog-bark",
		.handler	= smb1355_handle_wdog_bark,
	},
	[1] = {
		.name		= "chg-state-change",
		.handler	= smb1355_handle_chg_state_change,
		.wake		= true,
	},
};

static int smb1355_get_irq_index_byname(const char *irq_name)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(smb1355_irqs); i++) {
		if (strcmp(smb1355_irqs[i].name, irq_name) == 0)
			return i;
	}

	return -ENOENT;
}

static int smb1355_request_interrupt(struct smb1355 *chip,
				struct device_node *node,
				const char *irq_name)
{
	int rc = 0, irq, irq_index;

	irq = of_irq_get_byname(node, irq_name);
	if (irq < 0) {
		pr_err("Couldn't get irq %s byname\n", irq_name);
		return irq;
	}

	irq_index = smb1355_get_irq_index_byname(irq_name);
	if (irq_index < 0) {
		pr_err("%s is not a defined irq\n", irq_name);
		return irq_index;
	}

	if (!smb1355_irqs[irq_index].handler)
		return 0;

	rc = devm_request_threaded_irq(chip->dev, irq, NULL,
				smb1355_irqs[irq_index].handler,
				IRQF_ONESHOT, irq_name, chip);
	if (rc < 0) {
		pr_err("Couldn't request irq %d rc=%d\n", irq, rc);
		return rc;
	}

	if (smb1355_irqs[irq_index].wake)
		enable_irq_wake(irq);

	return rc;
}

static int smb1355_request_interrupts(struct smb1355 *chip)
{
	struct device_node *node = chip->dev->of_node;
	struct device_node *child;
	int rc = 0;
	const char *name;
	struct property *prop;

	for_each_available_child_of_node(node, child) {
		of_property_for_each_string(child, "interrupt-names",
					prop, name) {
			rc = smb1355_request_interrupt(chip, child, name);
			if (rc < 0) {
				pr_err("Couldn't request interrupt %s rc=%d\n",
					name, rc);
				return rc;
			}
		}
	}

	return rc;
}

/*********
 * PROBE *
 *********/
static const struct of_device_id match_table[] = {
	{
		.compatible	= "qcom,smb1355",
	},
	{ },
};

static int smb1355_probe(struct platform_device *pdev)
{
	struct smb1355 *chip;
	const struct of_device_id *id;
	int rc = 0;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &pdev->dev;
	chip->param = v1_params;
	chip->name = "smb1355";
	mutex_init(&chip->write_lock);

	chip->regmap = dev_get_regmap(chip->dev->parent, NULL);
	if (!chip->regmap) {
		pr_err("parent regmap is missing\n");
		return -EINVAL;
	}

	id = of_match_device(of_match_ptr(match_table), chip->dev);
	if (!id) {
		pr_err("Couldn't find a matching device\n");
		return -ENODEV;
	}

	platform_set_drvdata(pdev, chip);

	rc = smb1355_init_hw(chip);
	if (rc < 0) {
		pr_err("Couldn't initialize hardware rc=%d\n", rc);
		goto cleanup;
	}

	rc = smb1355_init_parallel_psy(chip);
	if (rc < 0) {
		pr_err("Couldn't initialize parallel psy rc=%d\n", rc);
		goto cleanup;
	}

	rc = smb1355_request_interrupts(chip);
	if (rc < 0) {
		pr_err("Couldn't request interrupts rc=%d\n", rc);
		goto cleanup;
	}

	pr_info("%s probed successfully\n", chip->name);
	return rc;

cleanup:
	platform_set_drvdata(pdev, NULL);
	return rc;
}

static int smb1355_remove(struct platform_device *pdev)
{
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct platform_driver smb1355_driver = {
	.driver	= {
		.name		= "qcom,smb1355-charger",
		.owner		= THIS_MODULE,
		.of_match_table	= match_table,
	},
	.probe	= smb1355_probe,
	.remove	= smb1355_remove,
};
module_platform_driver(smb1355_driver);

MODULE_DESCRIPTION("QPNP SMB1355 Charger Driver");
MODULE_LICENSE("GPL v2");
