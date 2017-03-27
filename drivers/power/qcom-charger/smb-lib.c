/* Copyright (c) 2016 The Linux Foundation. All rights reserved.
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

#include <linux/device.h>
#include <linux/regmap.h>
#include <linux/iio/consumer.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/irq.h>
#include "smb-lib.h"
#include "smb-reg.h"
#include "pmic-voter.h"

#define smblib_dbg(chg, reason, fmt, ...)			\
	do {							\
		if (*chg->debug_mask & (reason))		\
			dev_info(chg->dev, fmt, ##__VA_ARGS__);	\
		else						\
			dev_dbg(chg->dev, fmt, ##__VA_ARGS__);	\
	} while (0)

static bool is_secure(struct smb_charger *chg, int addr)
{
	/* assume everything above 0xC0 is secure */
	return (bool)((addr & 0xFF) >= 0xC0);
}

int smblib_read(struct smb_charger *chg, u16 addr, u8 *val)
{
	unsigned int temp;
	int rc = 0;

	rc = regmap_read(chg->regmap, addr, &temp);
	if (rc >= 0)
		*val = (u8)temp;

	return rc;
}

int smblib_masked_write(struct smb_charger *chg, u16 addr, u8 mask, u8 val)
{
	int rc = 0;

	mutex_lock(&chg->write_lock);
	if (is_secure(chg, addr)) {
		rc = regmap_write(chg->regmap, (addr & 0xFF00) | 0xD0, 0xA5);
		if (rc < 0)
			goto unlock;
	}

	rc = regmap_update_bits(chg->regmap, addr, mask, val);

unlock:
	mutex_unlock(&chg->write_lock);
	return rc;
}

int smblib_write(struct smb_charger *chg, u16 addr, u8 val)
{
	int rc = 0;

	mutex_lock(&chg->write_lock);

	if (is_secure(chg, addr)) {
		rc = regmap_write(chg->regmap, (addr & ~(0xFF)) | 0xD0, 0xA5);
		if (rc < 0)
			goto unlock;
	}

	rc = regmap_write(chg->regmap, addr, val);

unlock:
	mutex_unlock(&chg->write_lock);
	return rc;
}

static int smblib_get_step_charging_adjustment(struct smb_charger *chg,
					       int *cc_offset)
{
	int step_state;
	int rc;
	u8 stat;

	if (!chg->step_chg_enabled) {
		*cc_offset = 0;
		return 0;
	}

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return rc;
	}

	step_state = (stat & STEP_CHARGING_STATUS_MASK) >> 3;
	rc = smblib_get_charge_param(chg, &chg->param.step_cc_delta[step_state],
				     cc_offset);

	return rc;
}

static void smblib_fcc_split_ua(struct smb_charger *chg, int total_fcc,
			int *master_ua, int *slave_ua)
{
	int rc, cc_reduction_ua = 0;
	int step_cc_delta;
	int master_percent = min(max(*chg->pl.master_percent, 0), 100);
	union power_supply_propval pval = {0, };
	int effective_fcc;

	/*
	 * if master_percent is 0, s/w will configure master's fcc to zero and
	 * slave's fcc to the max. However since master's fcc is zero it
	 * disables its own charging and as a result the slave's charging is
	 * disabled via the fault line.
	 */

	rc = smblib_get_prop_batt_health(chg, &pval);
	if (rc == 0) {
		if (pval.intval == POWER_SUPPLY_HEALTH_WARM
			|| pval.intval == POWER_SUPPLY_HEALTH_COOL) {
			rc = smblib_get_charge_param(chg,
					&chg->param.jeita_cc_comp,
					&cc_reduction_ua);
			if (rc < 0) {
				dev_err(chg->dev, "Could not get jeita comp, rc=%d\n",
					rc);
				cc_reduction_ua = 0;
			}
		}
	}

	rc = smblib_get_step_charging_adjustment(chg, &step_cc_delta);
	if (rc < 0)
		step_cc_delta = 0;

	/*
	 * During JEITA condition and with step_charging enabled, PMI will
	 * pick the lower of the two value: (FCC - JEITA current compensation)
	 * or (FCC + step_charging current delta)
	 */

	effective_fcc = min(max(0, total_fcc - cc_reduction_ua),
			    max(0, total_fcc + step_cc_delta));
	*master_ua = (effective_fcc * master_percent) / 100;
	*slave_ua = (effective_fcc - *master_ua) * chg->pl.taper_percent / 100;
	*master_ua = max(0, *master_ua + total_fcc - effective_fcc);
}

/********************
 * REGISTER GETTERS *
 ********************/

int smblib_get_charge_param(struct smb_charger *chg,
			    struct smb_chg_param *param, int *val_u)
{
	int rc = 0;
	u8 val_raw;

	rc = smblib_read(chg, param->reg, &val_raw);
	if (rc < 0) {
		dev_err(chg->dev, "%s: Couldn't read from 0x%04x rc=%d\n",
			param->name, param->reg, rc);
		return rc;
	}

	if (param->get_proc)
		*val_u = param->get_proc(param, val_raw);
	else
		*val_u = val_raw * param->step_u + param->min_u;
	smblib_dbg(chg, PR_REGISTER, "%s = %d (0x%02x)\n",
		   param->name, *val_u, val_raw);

	return rc;
}

int smblib_get_usb_suspend(struct smb_charger *chg, int *suspend)
{
	int rc = 0;
	u8 temp;

	rc = smblib_read(chg, USBIN_CMD_IL_REG, &temp);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read USBIN_CMD_IL rc=%d\n", rc);
		return rc;
	}
	*suspend = temp & USBIN_SUSPEND_BIT;

	return rc;
}

struct apsd_result {
	const char * const name;
	const u8 bit;
	const enum power_supply_type pst;
};

static const struct apsd_result const smblib_apsd_results[] = {
	{"UNKNOWN", 0, POWER_SUPPLY_TYPE_UNKNOWN},
	{"SDP", SDP_CHARGER_BIT, POWER_SUPPLY_TYPE_USB},
	{"CDP", CDP_CHARGER_BIT, POWER_SUPPLY_TYPE_USB_CDP},
	{"DCP", DCP_CHARGER_BIT, POWER_SUPPLY_TYPE_USB_DCP},
	{"OCP", OCP_CHARGER_BIT, POWER_SUPPLY_TYPE_USB_DCP},
	{"FLOAT", FLOAT_CHARGER_BIT, POWER_SUPPLY_TYPE_USB_DCP},
	{"HVDCP2", DCP_CHARGER_BIT | QC_2P0_BIT, POWER_SUPPLY_TYPE_USB_HVDCP},
	{"HVDCP3", DCP_CHARGER_BIT | QC_3P0_BIT, POWER_SUPPLY_TYPE_USB_HVDCP_3},
};

static const struct apsd_result *smblib_get_apsd_result(struct smb_charger *chg)
{
	int rc, i;
	u8 stat;

	rc = smblib_read(chg, APSD_STATUS_REG, &stat);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read APSD_STATUS rc=%d\n", rc);
		return &smblib_apsd_results[0];
	}
	smblib_dbg(chg, PR_REGISTER, "APSD_STATUS = 0x%02x\n", stat);

	if (!(stat & APSD_DTC_STATUS_DONE_BIT))
		return &smblib_apsd_results[0];

	rc = smblib_read(chg, APSD_RESULT_STATUS_REG, &stat);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read APSD_RESULT_STATUS rc=%d\n",
			rc);
		return &smblib_apsd_results[0];
	}
	stat &= APSD_RESULT_STATUS_MASK;

	for (i = 0; i < ARRAY_SIZE(smblib_apsd_results); i++) {
		if (smblib_apsd_results[i].bit == stat)
			return &smblib_apsd_results[i];
	}

	dev_err(chg->dev, "Couldn't find an APSD result for 0x%02x\n", stat);
	return &smblib_apsd_results[0];
}


/********************
 * REGISTER SETTERS *
 ********************/

int smblib_set_charge_param(struct smb_charger *chg,
			    struct smb_chg_param *param, int val_u)
{
	int rc = 0;
	u8 val_raw;

	if (param->set_proc) {
		rc = param->set_proc(param, val_u, &val_raw);
		if (rc < 0)
			return -EINVAL;
	} else {
		if (val_u > param->max_u || val_u < param->min_u) {
			dev_err(chg->dev, "%s: %d is out of range [%d, %d]\n",
				param->name, val_u, param->min_u, param->max_u);
			return -EINVAL;
		}

		val_raw = (val_u - param->min_u) / param->step_u;
	}

	rc = smblib_write(chg, param->reg, val_raw);
	if (rc < 0) {
		dev_err(chg->dev, "%s: Couldn't write 0x%02x to 0x%04x rc=%d\n",
			param->name, val_raw, param->reg, rc);
		return rc;
	}

	smblib_dbg(chg, PR_REGISTER, "%s = %d (0x%02x)\n",
		   param->name, val_u, val_raw);

	return rc;
}

static int step_charge_soc_update(struct smb_charger *chg, int capacity)
{
	int rc = 0;

	rc = smblib_set_charge_param(chg, &chg->param.step_soc, capacity);
	if (rc < 0) {
		dev_err(chg->dev, "Error in updating soc, rc=%d\n", rc);
		return rc;
	}

	rc = smblib_write(chg, STEP_CHG_SOC_VBATT_V_UPDATE_REG,
			STEP_CHG_SOC_VBATT_V_UPDATE_BIT);
	if (rc < 0) {
		dev_err(chg->dev,
			"Couldn't set STEP_CHG_SOC_VBATT_V_UPDATE_REG rc=%d\n",
			rc);
		return rc;
	}

	return rc;
}

int smblib_set_usb_suspend(struct smb_charger *chg, bool suspend)
{
	int rc = 0;

	rc = smblib_masked_write(chg, USBIN_CMD_IL_REG, USBIN_SUSPEND_BIT,
				 suspend ? USBIN_SUSPEND_BIT : 0);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't write %s to USBIN_SUSPEND_BIT rc=%d\n",
			suspend ? "suspend" : "resume", rc);

	return rc;
}

int smblib_set_dc_suspend(struct smb_charger *chg, bool suspend)
{
	int rc = 0;

	rc = smblib_masked_write(chg, DCIN_CMD_IL_REG, DCIN_SUSPEND_BIT,
				 suspend ? DCIN_SUSPEND_BIT : 0);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't write %s to DCIN_SUSPEND_BIT rc=%d\n",
			suspend ? "suspend" : "resume", rc);

	return rc;
}

#define MICRO_5V	5000000
#define MICRO_9V	9000000
#define MICRO_12V	12000000
static int smblib_set_usb_pd_allowed_voltage(struct smb_charger *chg,
					int min_allowed_uv, int max_allowed_uv)
{
	int rc;
	u8 allowed_voltage;

	if (min_allowed_uv == MICRO_5V && max_allowed_uv == MICRO_5V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_5V;
	} else if (min_allowed_uv == MICRO_9V && max_allowed_uv == MICRO_9V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_9V;
	} else if (min_allowed_uv == MICRO_12V && max_allowed_uv == MICRO_12V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_12V;
	} else if (min_allowed_uv < MICRO_9V && max_allowed_uv <= MICRO_9V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_5V_TO_9V;
	} else if (min_allowed_uv < MICRO_9V && max_allowed_uv <= MICRO_12V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_5V_TO_12V;
	} else if (min_allowed_uv < MICRO_12V && max_allowed_uv <= MICRO_12V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_9V_TO_12V;
	} else {
		dev_err(chg->dev, "invalid allowed voltage [%d, %d]\n",
			min_allowed_uv, max_allowed_uv);
		return -EINVAL;
	}

	rc = smblib_write(chg, USBIN_ADAPTER_ALLOW_CFG_REG, allowed_voltage);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't write 0x%02x to USBIN_ADAPTER_ALLOW_CFG rc=%d\n",
			allowed_voltage, rc);
		return rc;
	}

	return rc;
}

/********************
 * HELPER FUNCTIONS *
 ********************/

static int smblib_update_usb_type(struct smb_charger *chg)
{
	int rc = 0;
	const struct apsd_result *apsd_result;

	/* if PD is active, APSD is disabled so won't have a valid result */
	if (chg->pd_active)
		return rc;

	apsd_result = smblib_get_apsd_result(chg);
	chg->usb_psy_desc.type = apsd_result->pst;
	return rc;
}

static int smblib_detach_usb(struct smb_charger *chg)
{
	int rc;

	cancel_delayed_work_sync(&chg->hvdcp_detect_work);
	chg->usb_psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;

	/* reconfigure allowed voltage for HVDCP */
	rc = smblib_write(chg, USBIN_ADAPTER_ALLOW_CFG_REG,
			  USBIN_ADAPTER_ALLOW_5V_OR_9V_TO_12V);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't set USBIN_ADAPTER_ALLOW_5V_OR_9V_TO_12V rc=%d\n",
			rc);
		return rc;
	}

	chg->voltage_min_uv = MICRO_5V;
	chg->voltage_max_uv = MICRO_5V;

	/* clear USB ICL vote for PD_VOTER */
	rc = vote(chg->usb_icl_votable, PD_VOTER, false, 0);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't vote for USB ICL rc=%d\n",
			rc);
		return rc;
	}

	vote(chg->pd_allowed_votable, DEFAULT_VOTER, false, 0);

	return rc;
}

static int smblib_notifier_call(struct notifier_block *nb,
		unsigned long ev, void *v)
{
	struct power_supply *psy = v;
	struct smb_charger *chg = container_of(nb, struct smb_charger, nb);

	if (!strcmp(psy->desc->name, "bms")) {
		if (!chg->bms_psy)
			chg->bms_psy = psy;
		if (ev == PSY_EVENT_PROP_CHANGED && chg->batt_psy)
			schedule_work(&chg->bms_update_work);
	}

	if (!chg->pl.psy && !strcmp(psy->desc->name, "parallel")) {
		chg->pl.psy = psy;
		schedule_work(&chg->pl_detect_work);
	}

	return NOTIFY_OK;
}

static int smblib_register_notifier(struct smb_charger *chg)
{
	int rc;

	chg->nb.notifier_call = smblib_notifier_call;
	rc = power_supply_reg_notifier(&chg->nb);
	if (rc < 0) {
		pr_err("Couldn't register psy notifier rc = %d\n", rc);
		return rc;
	}

	return 0;
}

int smblib_mapping_soc_from_field_value(struct smb_chg_param *param,
					     int val_u, u8 *val_raw)
{
	if (val_u > param->max_u || val_u < param->min_u)
		return -EINVAL;

	*val_raw = val_u << 1;

	return 0;
}

int smblib_mapping_cc_delta_to_field_value(struct smb_chg_param *param,
					   u8 val_raw)
{
	int val_u  = val_raw * param->step_u + param->min_u;

	if (val_u > param->max_u)
		val_u -= param->max_u * 2;

	return val_u;
}

int smblib_mapping_cc_delta_from_field_value(struct smb_chg_param *param,
					     int val_u, u8 *val_raw)
{
	if (val_u > param->max_u || val_u < param->min_u - param->max_u)
		return -EINVAL;

	val_u += param->max_u * 2 - param->min_u;
	val_u %= param->max_u * 2;
	*val_raw = val_u / param->step_u;

	return 0;
}

/*********************
 * VOTABLE CALLBACKS *
 *********************/

static int smblib_usb_suspend_vote_callback(struct votable *votable, void *data,
			int suspend, const char *client)
{
	struct smb_charger *chg = data;

	return smblib_set_usb_suspend(chg, suspend);
}

static int smblib_dc_suspend_vote_callback(struct votable *votable, void *data,
			int suspend, const char *client)
{
	struct smb_charger *chg = data;

	if (suspend < 0)
		suspend = false;

	return smblib_set_dc_suspend(chg, suspend);
}

static int smblib_fcc_max_vote_callback(struct votable *votable, void *data,
			int fcc_ua, const char *client)
{
	struct smb_charger *chg = data;

	return vote(chg->fcc_votable, FCC_MAX_RESULT, true, fcc_ua);
}

static int smblib_fcc_vote_callback(struct votable *votable, void *data,
			int fcc_ua, const char *client)
{
	struct smb_charger *chg = data;
	int rc = 0;
	union power_supply_propval pval = {0, };
	int master_ua = fcc_ua, slave_ua;

	if (fcc_ua < 0) {
		smblib_dbg(chg, PR_MISC, "No Voter\n");
		return 0;
	}

	if (chg->mode == PARALLEL_MASTER
		&& !get_effective_result_locked(chg->pl_disable_votable)) {
		smblib_fcc_split_ua(chg, fcc_ua, &master_ua, &slave_ua);

		/*
		 * parallel charger is not disabled, implying that
		 * chg->pl.psy exists
		 */
		pval.intval = slave_ua;
		rc = power_supply_set_property(chg->pl.psy,
				POWER_SUPPLY_PROP_CURRENT_MAX, &pval);
		if (rc < 0) {
			dev_err(chg->dev, "Could not set parallel fcc, rc=%d\n",
				rc);
			return rc;
		}

		chg->pl.slave_fcc = slave_ua;
	}

	rc = smblib_set_charge_param(chg, &chg->param.fcc, master_ua);
	if (rc < 0) {
		dev_err(chg->dev, "Error in setting fcc, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

#define PARALLEL_FLOAT_VOLTAGE_DELTA_UV 50000
static int smblib_fv_vote_callback(struct votable *votable, void *data,
			int fv_uv, const char *client)
{
	struct smb_charger *chg = data;
	union power_supply_propval pval = {0, };
	int rc = 0;

	if (fv_uv < 0) {
		smblib_dbg(chg, PR_MISC, "No Voter\n");
		return 0;
	}

	rc = smblib_set_charge_param(chg, &chg->param.fv, fv_uv);
	if (rc < 0) {
		dev_err(chg->dev,
			"Couldn't set floating voltage rc=%d\n", rc);
		return rc;
	}

	if (chg->mode == PARALLEL_MASTER && chg->pl.psy) {
		pval.intval = fv_uv + PARALLEL_FLOAT_VOLTAGE_DELTA_UV;
		rc = power_supply_set_property(chg->pl.psy,
				POWER_SUPPLY_PROP_VOLTAGE_MAX, &pval);
		if (rc < 0) {
			dev_err(chg->dev,
				"Couldn't set float on parallel rc=%d\n", rc);
			return rc;
		}
	}

	return 0;
}

#define USBIN_25MA 25000
#define USBIN_100MA 100000
static int smblib_usb_icl_vote_callback(struct votable *votable, void *data,
			int icl_ua, const char *client)
{
	struct smb_charger *chg = data;
	int rc = 0;
	bool suspend;

	if (icl_ua < 0) {
		smblib_dbg(chg, PR_MISC, "No Voter hence suspending\n");
		icl_ua = 0;
	}

	suspend = (icl_ua < USBIN_25MA);
	if (suspend)
		goto suspend;

	if (chg->usb_psy_desc.type == POWER_SUPPLY_TYPE_USB)
		rc = smblib_masked_write(chg, USBIN_ICL_OPTIONS_REG,
				USB51_MODE_BIT,
				(icl_ua > USBIN_100MA) ? USB51_MODE_BIT : 0);
	else
		rc = smblib_set_charge_param(chg, &chg->param.usb_icl, icl_ua);

	if (rc < 0) {
		dev_err(chg->dev,
			"Couldn't set USB input current limit rc=%d\n", rc);
		return rc;
	}

suspend:
	rc = vote(chg->usb_suspend_votable, PD_VOTER, suspend, 0);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't %s input rc=%d\n",
			suspend ? "suspend" : "resume", rc);
		return rc;
	}

	return rc;
}

static int smblib_dc_icl_vote_callback(struct votable *votable, void *data,
			int icl_ua, const char *client)
{
	struct smb_charger *chg = data;
	int rc = 0;
	bool suspend;

	if (icl_ua < 0) {
		smblib_dbg(chg, PR_MISC, "No Voter hence suspending\n");
		icl_ua = 0;
	}

	suspend = (icl_ua < USBIN_25MA);
	if (suspend)
		goto suspend;

	rc = smblib_set_charge_param(chg, &chg->param.dc_icl, icl_ua);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't set DC input current limit rc=%d\n",
			rc);
		return rc;
	}

suspend:
	rc = vote(chg->dc_suspend_votable, USER_VOTER, suspend, 0);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't vote to %s DC rc=%d\n",
			suspend ? "suspend" : "resume", rc);
		return rc;
	}
	return rc;
}

static int smblib_awake_vote_callback(struct votable *votable, void *data,
			int awake, const char *client)
{
	struct smb_charger *chg = data;

	if (awake)
		pm_stay_awake(chg->dev);
	else
		pm_relax(chg->dev);

	return 0;
}

static int smblib_pl_disable_vote_callback(struct votable *votable, void *data,
			int pl_disable, const char *client)
{
	struct smb_charger *chg = data;
	union power_supply_propval pval = {0, };
	int rc;

	if (chg->mode != PARALLEL_MASTER || !chg->pl.psy)
		return 0;

	chg->pl.taper_percent = 100;
	rerun_election(chg->fv_votable);
	rerun_election(chg->fcc_votable);

	pval.intval = pl_disable;
	rc = power_supply_set_property(chg->pl.psy,
			POWER_SUPPLY_PROP_INPUT_SUSPEND, &pval);
	if (rc < 0) {
		dev_err(chg->dev,
			"Couldn't change slave suspend state rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static int smblib_chg_disable_vote_callback(struct votable *votable, void *data,
			int chg_disable, const char *client)
{
	struct smb_charger *chg = data;
	int rc;

	rc = smblib_masked_write(chg, CHARGING_ENABLE_CMD_REG,
				 CHARGING_ENABLE_CMD_BIT,
				 chg_disable ? 0 : CHARGING_ENABLE_CMD_BIT);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't %s charging rc=%d\n",
			chg_disable ? "disable" : "enable", rc);
		return rc;
	}

	return 0;
}
/*****************
 * OTG REGULATOR *
 *****************/

int smblib_vbus_regulator_enable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc = 0;

	rc = regmap_write(chg->regmap, CMD_OTG_REG, OTG_EN_BIT);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't enable OTG regulator rc=%d\n", rc);

	return rc;
}

int smblib_vbus_regulator_disable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc = 0;

	rc = regmap_write(chg->regmap, CMD_OTG_REG, 0);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't disable OTG regulator rc=%d\n", rc);

	return rc;
}

int smblib_vbus_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc = 0;
	u8 cmd;

	rc = smblib_read(chg, CMD_OTG_REG, &cmd);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read CMD_OTG rc=%d", rc);
		return rc;
	}

	return (cmd & OTG_EN_BIT) ? 1 : 0;
}

/*******************
 * VCONN REGULATOR *
 * *****************/

int smblib_vconn_regulator_enable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc = 0;

	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 VCONN_EN_VALUE_BIT, VCONN_EN_VALUE_BIT);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't enable vconn regulator rc=%d\n",
			rc);

	return rc;
}

int smblib_vconn_regulator_disable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc = 0;

	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 VCONN_EN_VALUE_BIT, 0);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't disable vconn regulator rc=%d\n",
			rc);

	return rc;
}

int smblib_vconn_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc = 0;
	u8 cmd;

	rc = smblib_read(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG, &cmd);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read TYPE_C_INTRPT_ENB_SOFTWARE_CTRL rc=%d\n",
			rc);
		return rc;
	}

	return (cmd & VCONN_EN_VALUE_BIT) ? 1 : 0;
}

/********************
 * BATT PSY GETTERS *
 ********************/

int smblib_get_prop_input_suspend(struct smb_charger *chg,
				  union power_supply_propval *val)
{
	val->intval = get_client_vote(chg->usb_suspend_votable, USER_VOTER) &&
			get_client_vote(chg->dc_suspend_votable, USER_VOTER);
	return 0;
}

int smblib_get_prop_batt_present(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, BATIF_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read BATIF_INT_RT_STS rc=%d\n",
			rc);
		return rc;
	}

	val->intval = !(stat & (BAT_THERM_OR_ID_MISSING_RT_STS_BIT
					| BAT_TERMINAL_MISSING_RT_STS_BIT));

	return rc;
}

int smblib_get_prop_batt_capacity(struct smb_charger *chg,
				  union power_supply_propval *val)
{
	int rc = -EINVAL;

	if (chg->fake_capacity >= 0) {
		val->intval = chg->fake_capacity;
		return 0;
	}

	if (chg->bms_psy)
		rc = power_supply_get_property(chg->bms_psy,
				POWER_SUPPLY_PROP_CAPACITY, val);
	return rc;
}

int smblib_get_prop_batt_status(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;
	union power_supply_propval pval = {0, };

	smblib_get_prop_input_suspend(chg, &pval);
	if (pval.intval) {
		val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		return rc;
	}

	rc = smblib_read(chg, POWER_PATH_STATUS_REG, &stat);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read POWER_PATH_STATUS rc=%d\n",
			rc);
		return rc;
	}

	if (!(stat & (USE_USBIN_BIT | USE_DCIN_BIT)) ||
				!(stat & VALID_INPUT_POWER_SOURCE_BIT)) {
		val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		return rc;
	}

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "BATTERY_CHARGER_STATUS_1 = 0x%02x\n",
		 stat);

	stat = stat & BATTERY_CHARGER_STATUS_MASK;
	if (stat >= COMPLETED_CHARGE)
		val->intval = POWER_SUPPLY_STATUS_FULL;
	else
		val->intval = POWER_SUPPLY_STATUS_CHARGING;

	return rc;
}

int smblib_get_prop_batt_charge_type(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "BATTERY_CHARGER_STATUS_1 = 0x%02x\n",
		   stat);

	switch (stat & BATTERY_CHARGER_STATUS_MASK) {
	case TRICKLE_CHARGE:
	case PRE_CHARGE:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		break;
	case FAST_CHARGE:
	case FULLON_CHARGE:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
		break;
	case TAPER_CHARGE:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_TAPER;
		break;
	default:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
	}

	return rc;
}

int smblib_get_prop_batt_health(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_2_REG, &stat);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read BATTERY_CHARGER_STATUS_2 rc=%d\n",
			rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "BATTERY_CHARGER_STATUS_2 = 0x%02x\n",
		   stat);

	if (stat & CHARGER_ERROR_STATUS_BAT_OV_BIT) {
		dev_err(chg->dev, "battery over-voltage\n");
		val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		goto done;
	}

	if (stat & BAT_TEMP_STATUS_TOO_COLD_BIT)
		val->intval = POWER_SUPPLY_HEALTH_COLD;
	else if (stat & BAT_TEMP_STATUS_TOO_HOT_BIT)
		val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (stat & BAT_TEMP_STATUS_COLD_SOFT_LIMIT_BIT)
		val->intval = POWER_SUPPLY_HEALTH_COOL;
	else if (stat & BAT_TEMP_STATUS_HOT_SOFT_LIMIT_BIT)
		val->intval = POWER_SUPPLY_HEALTH_WARM;
	else
		val->intval = POWER_SUPPLY_HEALTH_GOOD;

done:
	return rc;
}

int smblib_get_prop_system_temp_level(struct smb_charger *chg,
				union power_supply_propval *val)
{
	val->intval = chg->system_temp_level;
	return 0;
}

/***********************
 * BATTERY PSY SETTERS *
 ***********************/

int smblib_set_prop_input_suspend(struct smb_charger *chg,
				  const union power_supply_propval *val)
{
	int rc;

	rc = vote(chg->usb_suspend_votable, USER_VOTER, (bool)val->intval, 0);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't vote to %s USB rc=%d\n",
			(bool)val->intval ? "suspend" : "resume", rc);
		return rc;
	}

	rc = vote(chg->dc_suspend_votable, USER_VOTER, (bool)val->intval, 0);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't vote to %s DC rc=%d\n",
			(bool)val->intval ? "suspend" : "resume", rc);
		return rc;
	}

	power_supply_changed(chg->batt_psy);
	return rc;
}

int smblib_set_prop_batt_capacity(struct smb_charger *chg,
				  const union power_supply_propval *val)
{
	chg->fake_capacity = val->intval;

	power_supply_changed(chg->batt_psy);

	return 0;
}

int smblib_set_prop_system_temp_level(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	if (val->intval < 0)
		return -EINVAL;

	if (chg->thermal_levels <= 0)
		return -EINVAL;

	if (val->intval > chg->thermal_levels)
		return -EINVAL;

	chg->system_temp_level = val->intval;
	if (chg->system_temp_level == chg->thermal_levels)
		return vote(chg->chg_disable_votable, THERMAL_DAEMON, true, 0);

	vote(chg->chg_disable_votable, THERMAL_DAEMON, false, 0);
	if (chg->system_temp_level == 0)
		return vote(chg->fcc_votable, THERMAL_DAEMON, false, 0);

	vote(chg->fcc_votable, THERMAL_DAEMON, true,
			chg->thermal_mitigation[chg->system_temp_level]);
	return 0;
}

/*******************
 * DC PSY GETTERS *
 *******************/

int smblib_get_prop_dc_present(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc = 0;
	u8 stat;

	rc = smblib_read(chg, DC_INT_RT_STS_REG, &stat);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read DC_INT_RT_STS_REG rc=%d\n",
			rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "DC_INT_RT_STS_REG = 0x%02x\n",
		   stat);

	val->intval = (bool)(stat & DCIN_PLUGIN_RT_STS_BIT);

	return rc;
}

int smblib_get_prop_dc_online(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	int rc = 0;
	u8 stat;

	if (get_client_vote(chg->dc_suspend_votable, USER_VOTER)) {
		val->intval = false;
		return rc;
	}

	rc = smblib_read(chg, POWER_PATH_STATUS_REG, &stat);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read POWER_PATH_STATUS rc=%d\n",
			rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "POWER_PATH_STATUS = 0x%02x\n",
		   stat);

	val->intval = (stat & USE_DCIN_BIT) &&
		      (stat & VALID_INPUT_POWER_SOURCE_BIT);

	return rc;
}

int smblib_get_prop_dc_current_max(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	val->intval = get_effective_result_locked(chg->dc_icl_votable);
	return 0;
}

/*******************
 * USB PSY SETTERS *
 * *****************/

int smblib_set_prop_dc_current_max(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc;

	rc = vote(chg->dc_icl_votable, USER_VOTER, true, val->intval);
	return rc;
}

/*******************
 * USB PSY GETTERS *
 *******************/

int smblib_get_prop_usb_present(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc = 0;
	u8 stat;

	rc = smblib_read(chg, TYPE_C_STATUS_4_REG, &stat);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read TYPE_C_STATUS_4 rc=%d\n",
			rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_STATUS_4 = 0x%02x\n",
		   stat);

	val->intval = (bool)(stat & CC_ATTACHED_BIT);

	return rc;
}

int smblib_get_prop_usb_online(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	int rc = 0;
	u8 stat;

	if (get_client_vote(chg->usb_suspend_votable, USER_VOTER)) {
		val->intval = false;
		return rc;
	}

	rc = smblib_read(chg, POWER_PATH_STATUS_REG, &stat);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read POWER_PATH_STATUS rc=%d\n",
			rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "POWER_PATH_STATUS = 0x%02x\n",
		   stat);

	val->intval = (stat & USE_USBIN_BIT) &&
		      (stat & VALID_INPUT_POWER_SOURCE_BIT);

	return rc;
}

int smblib_get_prop_usb_voltage_now(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	int rc = 0;

	rc = smblib_get_prop_usb_present(chg, val);
	if (rc < 0 || !val->intval)
		return rc;

	if (!chg->iio.usbin_v_chan ||
		PTR_ERR(chg->iio.usbin_v_chan) == -EPROBE_DEFER)
		chg->iio.usbin_v_chan = iio_channel_get(chg->dev, "usbin_v");

	if (IS_ERR(chg->iio.usbin_v_chan))
		return PTR_ERR(chg->iio.usbin_v_chan);

	return iio_read_channel_processed(chg->iio.usbin_v_chan, &val->intval);
}

int smblib_get_prop_usb_current_max(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	val->intval = get_effective_result_locked(chg->usb_icl_votable);
	return 0;
}

int smblib_get_prop_usb_current_now(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	int rc = 0;

	rc = smblib_get_prop_usb_present(chg, val);
	if (rc < 0 || !val->intval)
		return rc;

	if (!chg->iio.usbin_i_chan ||
		PTR_ERR(chg->iio.usbin_i_chan) == -EPROBE_DEFER)
		chg->iio.usbin_i_chan = iio_channel_get(chg->dev, "usbin_i");

	if (IS_ERR(chg->iio.usbin_i_chan))
		return PTR_ERR(chg->iio.usbin_i_chan);

	return iio_read_channel_processed(chg->iio.usbin_i_chan, &val->intval);
}

int smblib_get_prop_charger_temp(struct smb_charger *chg,
				 union power_supply_propval *val)
{
	int rc;

	if (!chg->iio.temp_chan ||
		PTR_ERR(chg->iio.temp_chan) == -EPROBE_DEFER)
		chg->iio.temp_chan = iio_channel_get(chg->dev, "charger_temp");

	if (IS_ERR(chg->iio.temp_chan))
		return PTR_ERR(chg->iio.temp_chan);

	rc = iio_read_channel_processed(chg->iio.temp_chan, &val->intval);
	val->intval /= 100;
	return rc;
}

int smblib_get_prop_charger_temp_max(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	int rc;

	if (!chg->iio.temp_max_chan ||
		PTR_ERR(chg->iio.temp_max_chan) == -EPROBE_DEFER)
		chg->iio.temp_max_chan = iio_channel_get(chg->dev,
							 "charger_temp_max");
	if (IS_ERR(chg->iio.temp_max_chan))
		return PTR_ERR(chg->iio.temp_max_chan);

	rc = iio_read_channel_processed(chg->iio.temp_max_chan, &val->intval);
	val->intval /= 100;
	return rc;
}

int smblib_get_prop_typec_cc_orientation(struct smb_charger *chg,
					 union power_supply_propval *val)
{
	int rc = 0;
	u8 stat;

	rc = smblib_read(chg, TYPE_C_STATUS_4_REG, &stat);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read TYPE_C_STATUS_4 rc=%d\n",
			rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_STATUS_4 = 0x%02x\n",
		   stat);

	if (stat & CC_ATTACHED_BIT)
		val->intval = (bool)(stat & CC_ORIENTATION_BIT) + 1;
	else
		val->intval = 0;

	return rc;
}

static const char * const smblib_typec_mode_name[] = {
	[POWER_SUPPLY_TYPEC_NONE]		  = "NONE",
	[POWER_SUPPLY_TYPEC_SOURCE_DEFAULT]	  = "SOURCE_DEFAULT",
	[POWER_SUPPLY_TYPEC_SOURCE_MEDIUM]	  = "SOURCE_MEDIUM",
	[POWER_SUPPLY_TYPEC_SOURCE_HIGH]	  = "SOURCE_HIGH",
	[POWER_SUPPLY_TYPEC_NON_COMPLIANT]	  = "NON_COMPLIANT",
	[POWER_SUPPLY_TYPEC_SINK]		  = "SINK",
	[POWER_SUPPLY_TYPEC_SINK_POWERED_CABLE]   = "SINK_POWERED_CABLE",
	[POWER_SUPPLY_TYPEC_SINK_DEBUG_ACCESSORY] = "SINK_DEBUG_ACCESSORY",
	[POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER]   = "SINK_AUDIO_ADAPTER",
	[POWER_SUPPLY_TYPEC_POWERED_CABLE_ONLY]   = "POWERED_CABLE_ONLY",
};

static int smblib_get_prop_ufp_mode(struct smb_charger *chg)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, TYPE_C_STATUS_1_REG, &stat);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read TYPE_C_STATUS_1 rc=%d\n", rc);
		return POWER_SUPPLY_TYPEC_NONE;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_STATUS_1 = 0x%02x\n", stat);

	switch (stat) {
	case 0:
		return POWER_SUPPLY_TYPEC_NONE;
	case UFP_TYPEC_RDSTD_BIT:
		return POWER_SUPPLY_TYPEC_SOURCE_DEFAULT;
	case UFP_TYPEC_RD1P5_BIT:
		return POWER_SUPPLY_TYPEC_SOURCE_MEDIUM;
	case UFP_TYPEC_RD3P0_BIT:
		return POWER_SUPPLY_TYPEC_SOURCE_HIGH;
	default:
		break;
	}

	return POWER_SUPPLY_TYPEC_NON_COMPLIANT;
}

static int smblib_get_prop_dfp_mode(struct smb_charger *chg)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, TYPE_C_STATUS_2_REG, &stat);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read TYPE_C_STATUS_2 rc=%d\n", rc);
		return POWER_SUPPLY_TYPEC_NONE;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_STATUS_2 = 0x%02x\n", stat);

	switch (stat & DFP_TYPEC_MASK) {
	case DFP_RA_RA_BIT:
		return POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER;
	case DFP_RD_RD_BIT:
		return POWER_SUPPLY_TYPEC_SINK_DEBUG_ACCESSORY;
	case DFP_RD_RA_VCONN_BIT:
		return POWER_SUPPLY_TYPEC_SINK_POWERED_CABLE;
	case DFP_RD_OPEN_BIT:
		return POWER_SUPPLY_TYPEC_SINK;
	case DFP_RA_OPEN_BIT:
		return POWER_SUPPLY_TYPEC_POWERED_CABLE_ONLY;
	default:
		break;
	}

	return POWER_SUPPLY_TYPEC_NONE;
}

int smblib_get_prop_typec_mode(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, TYPE_C_STATUS_4_REG, &stat);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read TYPE_C_STATUS_4 rc=%d\n", rc);
		val->intval = POWER_SUPPLY_TYPEC_NONE;
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_STATUS_4 = 0x%02x\n", stat);

	if (!(stat & TYPEC_DEBOUNCE_DONE_STATUS_BIT)) {
		val->intval = POWER_SUPPLY_TYPEC_NONE;
		return rc;
	}

	if (stat & UFP_DFP_MODE_STATUS_BIT)
		val->intval = smblib_get_prop_dfp_mode(chg);
	else
		val->intval = smblib_get_prop_ufp_mode(chg);

	return rc;
}

int smblib_get_prop_typec_power_role(struct smb_charger *chg,
				     union power_supply_propval *val)
{
	int rc = 0;
	u8 ctrl;

	rc = smblib_read(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG, &ctrl);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read TYPE_C_INTRPT_ENB_SOFTWARE_CTRL rc=%d\n",
			rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_INTRPT_ENB_SOFTWARE_CTRL = 0x%02x\n",
		   ctrl);

	if (ctrl & TYPEC_DISABLE_CMD_BIT) {
		val->intval = POWER_SUPPLY_TYPEC_PR_NONE;
		return rc;
	}

	switch (ctrl & (DFP_EN_CMD_BIT | UFP_EN_CMD_BIT)) {
	case 0:
		val->intval = POWER_SUPPLY_TYPEC_PR_DUAL;
		break;
	case DFP_EN_CMD_BIT:
		val->intval = POWER_SUPPLY_TYPEC_PR_SOURCE;
		break;
	case UFP_EN_CMD_BIT:
		val->intval = POWER_SUPPLY_TYPEC_PR_SINK;
		break;
	default:
		val->intval = POWER_SUPPLY_TYPEC_PR_NONE;
		dev_err(chg->dev, "unsupported power role 0x%02lx\n",
			ctrl & (DFP_EN_CMD_BIT | UFP_EN_CMD_BIT));
		return -EINVAL;
	}

	return rc;
}

int smblib_get_prop_pd_allowed(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	val->intval = get_effective_result_locked(chg->pd_allowed_votable);
	return 0;
}

int smblib_get_prop_input_current_settled(struct smb_charger *chg,
					  union power_supply_propval *val)
{
	return smblib_get_charge_param(chg, &chg->param.icl_stat, &val->intval);
}

/*******************
 * USB PSY SETTERS *
 * *****************/

int smblib_set_prop_usb_current_max(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc;

	rc = vote(chg->usb_icl_votable, PD_VOTER, true, val->intval);
	return rc;
}

int smblib_set_prop_typec_power_role(struct smb_charger *chg,
				     const union power_supply_propval *val)
{
	int rc = 0;
	u8 power_role;

	switch (val->intval) {
	case POWER_SUPPLY_TYPEC_PR_NONE:
		power_role = TYPEC_DISABLE_CMD_BIT;
		break;
	case POWER_SUPPLY_TYPEC_PR_DUAL:
		power_role = 0;
		break;
	case POWER_SUPPLY_TYPEC_PR_SINK:
		power_role = UFP_EN_CMD_BIT;
		break;
	case POWER_SUPPLY_TYPEC_PR_SOURCE:
		power_role = DFP_EN_CMD_BIT;
		break;
	default:
		dev_err(chg->dev, "power role %d not supported\n", val->intval);
		return -EINVAL;
	}

	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 TYPEC_POWER_ROLE_CMD_MASK, power_role);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't write 0x%02x to TYPE_C_INTRPT_ENB_SOFTWARE_CTRL rc=%d\n",
			power_role, rc);
		return rc;
	}

	return rc;
}

int smblib_set_prop_usb_voltage_min(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc, min_uv;

	min_uv = min(val->intval, chg->voltage_max_uv);
	rc = smblib_set_usb_pd_allowed_voltage(chg, min_uv,
					       chg->voltage_max_uv);
	if (rc < 0) {
		dev_err(chg->dev, "invalid max voltage %duV rc=%d\n",
			val->intval, rc);
		return rc;
	}

	chg->voltage_min_uv = val->intval;
	return rc;
}

int smblib_set_prop_usb_voltage_max(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc, max_uv;

	max_uv = max(val->intval, chg->voltage_min_uv);
	rc = smblib_set_usb_pd_allowed_voltage(chg, chg->voltage_min_uv,
					       max_uv);
	if (rc < 0) {
		dev_err(chg->dev, "invalid min voltage %duV rc=%d\n",
			val->intval, rc);
		return rc;
	}

	chg->voltage_max_uv = val->intval;
	return rc;
}

int smblib_set_prop_pd_active(struct smb_charger *chg,
			      const union power_supply_propval *val)
{
	int rc;

	if (!get_effective_result(chg->pd_allowed_votable)) {
		dev_err(chg->dev, "PD is not allowed\n");
		return -EINVAL;
	}

	rc = smblib_masked_write(chg, USBIN_OPTIONS_1_CFG_REG,
				 AUTO_SRC_DETECT_BIT,
				 val->intval ? 0 : AUTO_SRC_DETECT_BIT);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't %s APSD rc=%d\n",
			val->intval ? "disable" : "enable", rc);
		return rc;
	}

	vote(chg->pd_allowed_votable, PD_VOTER, val->intval, 0);

	chg->pd_active = (bool)val->intval;
	smblib_update_usb_type(chg);
	return rc;
}

/**********************
 * INTERRUPT HANDLERS *
 **********************/

irqreturn_t smblib_handle_debug(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_chg_state_change(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	union power_supply_propval pval = {0, };
	int rc;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	if (chg->mode != PARALLEL_MASTER)
		return IRQ_HANDLED;

	rc = smblib_get_prop_batt_charge_type(chg, &pval);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't get batt charge type rc=%d\n", rc);
		return IRQ_HANDLED;
	}

	if (pval.intval == POWER_SUPPLY_CHARGE_TYPE_FAST)
		vote(chg->pl_disable_votable, CHG_STATE_VOTER, false, 0);

	if (pval.intval == POWER_SUPPLY_CHARGE_TYPE_TAPER
		&& !get_effective_result_locked(chg->pl_disable_votable)) {
		cancel_delayed_work_sync(&chg->pl_taper_work);
		schedule_delayed_work(&chg->pl_taper_work, 0);
	}

	rc = smblib_get_prop_batt_status(chg, &pval);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't get batt status type rc=%d\n", rc);
		return IRQ_HANDLED;
	}
	if (pval.intval == POWER_SUPPLY_STATUS_FULL)
		vote(chg->pl_disable_votable, TAPER_END_VOTER, false, 0);

	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_step_chg_state_change(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	if (chg->step_chg_enabled)
		rerun_election(chg->fcc_votable);

	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_step_chg_soc_update_fail(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	if (chg->step_chg_enabled)
		rerun_election(chg->fcc_votable);

	return IRQ_HANDLED;
}

#define STEP_SOC_REQ_MS	3000
irqreturn_t smblib_handle_step_chg_soc_update_request(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc;
	union power_supply_propval pval = {0, };

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	if (!chg->bms_psy) {
		schedule_delayed_work(&chg->step_soc_req_work,
				      msecs_to_jiffies(STEP_SOC_REQ_MS));
		return IRQ_HANDLED;
	}

	rc = smblib_get_prop_batt_capacity(chg, &pval);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get batt capacity rc=%d\n", rc);
	else
		step_charge_soc_update(chg, pval.intval);

	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_batt_temp_changed(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	rerun_election(chg->fcc_votable);
	power_supply_changed(chg->batt_psy);
	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_batt_psy_changed(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	smblib_handle_debug(irq, data);
	power_supply_changed(chg->batt_psy);
	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_usb_psy_changed(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	smblib_handle_debug(irq, data);
	power_supply_changed(chg->usb_psy);
	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_usb_plugin(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc;
	u8 stat;

	/* fetch the DPDM regulator */
	if (!chg->dpdm_reg && of_get_property(chg->dev->of_node,
					      "dpdm-supply", NULL)) {
		chg->dpdm_reg = devm_regulator_get(chg->dev, "dpdm");
		if (IS_ERR(chg->dpdm_reg)) {
			dev_err(chg->dev, "Couldn't get dpdm regulator rc=%ld\n",
				PTR_ERR(chg->dpdm_reg));
			chg->dpdm_reg = NULL;
		}
	}

	if (!chg->dpdm_reg)
		goto skip_dpdm_float;

	rc = smblib_read(chg, USBIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read USB_INT_RT_STS rc=%d\n", rc);
		return IRQ_HANDLED;
	}

	chg->vbus_present = (bool)(stat & USBIN_PLUGIN_RT_STS_BIT);

	if (chg->vbus_present) {
		if (!regulator_is_enabled(chg->dpdm_reg)) {
			smblib_dbg(chg, PR_MISC, "enabling DPDM regulator\n");
			rc = regulator_enable(chg->dpdm_reg);
			if (rc < 0)
				dev_err(chg->dev, "Couldn't enable dpdm regulator rc=%d\n",
					rc);
		}
	} else {
		if (regulator_is_enabled(chg->dpdm_reg)) {
			smblib_dbg(chg, PR_MISC, "disabling DPDM regulator\n");
			rc = regulator_disable(chg->dpdm_reg);
			if (rc < 0)
				dev_err(chg->dev, "Couldn't disable dpdm regulator rc=%d\n",
					rc);
		}
	}

skip_dpdm_float:
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s %s\n",
		   irq_data->name, chg->vbus_present ? "attached" : "detached");
	return IRQ_HANDLED;
}

#define MICRO_5P5V		5500000
#define USB_WEAK_INPUT_MA	1500000
static bool is_icl_pl_ready(struct smb_charger *chg)
{
	union power_supply_propval pval = {0, };
	int icl_ma;
	int rc;

	rc = smblib_get_prop_usb_voltage_now(chg, &pval);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't get prop usb voltage rc=%d\n", rc);
		return false;
	}

	if (pval.intval <= MICRO_5P5V) {
		rc = smblib_get_charge_param(chg,
					&chg->param.icl_stat, &icl_ma);
		if (rc < 0) {
			dev_err(chg->dev, "Couldn't get ICL status rc=%d\n",
				rc);
			return false;
		}

		if (icl_ma < USB_WEAK_INPUT_MA)
			return false;
	}

	/*
	 * Always enable parallel charging when USB INPUT is higher than 5V
	 * regardless of the AICL results. Assume chargers above 5V are strong
	 */

	return true;
}

irqreturn_t smblib_handle_icl_change(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	if (chg->mode == PARALLEL_MASTER)
		vote(chg->pl_disable_votable, USBIN_ICL_VOTER,
					!is_icl_pl_ready(chg), 0);

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	return IRQ_HANDLED;
}

static void smblib_handle_slow_plugin_timeout(struct smb_charger *chg,
					      bool rising)
{
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: slow-plugin-timeout %s\n",
		   rising ? "rising" : "falling");
}

static void smblib_handle_sdp_enumeration_done(struct smb_charger *chg,
					       bool rising)
{
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: sdp-enumeration-done %s\n",
		   rising ? "rising" : "falling");
}

static void smblib_handle_adaptive_voltage_done(struct smb_charger *chg,
						bool rising)
{
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: adaptive-voltage-done %s\n",
		   rising ? "rising" : "falling");
}

/* triggers when HVDCP 3.0 authentication has finished */
static void smblib_handle_hvdcp_3p0_auth_done(struct smb_charger *chg,
					      bool rising)
{
	const struct apsd_result *apsd_result;

	if (!rising)
		return;

	/* the APSD done handler will set the USB supply type */
	apsd_result = smblib_get_apsd_result(chg);
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: hvdcp-3p0-auth-done rising; %s detected\n",
		   apsd_result->name);
}

/* triggers when HVDCP is detected */
static void smblib_handle_hvdcp_detect_done(struct smb_charger *chg,
					    bool rising)
{
	if (!rising)
		return;

	/* the APSD done handler will set the USB supply type */
	cancel_delayed_work_sync(&chg->hvdcp_detect_work);
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: hvdcp-detect-done %s\n",
		   rising ? "rising" : "falling");
}

#define HVDCP_DET_MS 2500
static void smblib_handle_apsd_done(struct smb_charger *chg, bool rising)
{
	int rc;
	const struct apsd_result *apsd_result;

	if (!rising)
		return;

	apsd_result = smblib_get_apsd_result(chg);
	switch (apsd_result->bit) {
	case SDP_CHARGER_BIT:
	case CDP_CHARGER_BIT:
	case OCP_CHARGER_BIT:
	case FLOAT_CHARGER_BIT:
		vote(chg->pd_allowed_votable, DEFAULT_VOTER, true, 0);
		break;
	case DCP_CHARGER_BIT:
		schedule_delayed_work(&chg->hvdcp_detect_work,
				      msecs_to_jiffies(HVDCP_DET_MS));
		break;
	default:
		break;
	}

	rc = smblib_update_usb_type(chg);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't update usb type rc=%d\n", rc);

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: apsd-done rising; %s detected\n",
		   apsd_result->name);
}

irqreturn_t smblib_handle_usb_source_change(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc = 0;
	u8 stat;

	rc = smblib_read(chg, APSD_STATUS_REG, &stat);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read APSD_STATUS rc=%d\n", rc);
		return IRQ_HANDLED;
	}
	smblib_dbg(chg, PR_REGISTER, "APSD_STATUS = 0x%02x\n", stat);

	smblib_handle_apsd_done(chg,
		(bool)(stat & APSD_DTC_STATUS_DONE_BIT));

	smblib_handle_hvdcp_detect_done(chg,
		(bool)(stat & QC_CHARGER_BIT));

	smblib_handle_hvdcp_3p0_auth_done(chg,
		(bool)(stat & QC_AUTH_DONE_STATUS_BIT));

	smblib_handle_adaptive_voltage_done(chg,
		(bool)(stat & VADP_CHANGE_DONE_AFTER_AUTH_BIT));

	smblib_handle_sdp_enumeration_done(chg,
		(bool)(stat & ENUMERATION_DONE_BIT));

	smblib_handle_slow_plugin_timeout(chg,
		(bool)(stat & SLOW_PLUGIN_TIMEOUT_BIT));

	power_supply_changed(chg->usb_psy);

	return IRQ_HANDLED;
}

static void smblib_handle_typec_cc(struct smb_charger *chg, bool attached)
{
	int rc;

	if (!attached) {
		rc = smblib_detach_usb(chg);
		if (rc < 0)
			dev_err(chg->dev, "Couldn't detach USB rc=%d\n", rc);
	}

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: CC %s\n",
		   attached ? "attached" : "detached");
}

static void smblib_handle_typec_debounce_done(struct smb_charger *chg,
					      bool rising, bool sink_attached)
{
	int rc;
	union power_supply_propval pval = {0, };

	/* allow PD for attached sinks */
	if (rising && sink_attached)
		vote(chg->pd_allowed_votable, DEFAULT_VOTER, true, 0);

	rc = smblib_get_prop_typec_mode(chg, &pval);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get prop typec mode rc=%d\n", rc);

	/*
	 * vote to enable parallel charging if a source is attached, and disable
	 * otherwise
	 */
	vote(chg->pl_disable_votable, TYPEC_SRC_VOTER,
					!rising || sink_attached, 0);

	if (!rising || sink_attached) {
		/* icl votes to disable parallel charging */
		vote(chg->pl_disable_votable, USBIN_ICL_VOTER, true, 0);
		/* reset taper_end voter here */
		vote(chg->pl_disable_votable, TAPER_END_VOTER, false, 0);
	}

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: debounce-done %s; Type-C %s detected\n",
		   rising ? "rising" : "falling",
		   smblib_typec_mode_name[pval.intval]);
}

irqreturn_t smblib_handle_usb_typec_change(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc;
	u8 stat;

	rc = smblib_read(chg, TYPE_C_STATUS_4_REG, &stat);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read TYPE_C_STATUS_4 rc=%d\n",
			rc);
		return IRQ_HANDLED;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_STATUS_4 = 0x%02x\n", stat);

	if (stat & TYPEC_VBUS_ERROR_STATUS_BIT) {
		dev_err(chg->dev, "IRQ: vbus-error rising\n");
		return IRQ_HANDLED;
	}

	smblib_handle_typec_cc(chg,
			(bool)(stat & CC_ATTACHED_BIT));
	smblib_handle_typec_debounce_done(chg,
			(bool)(stat & TYPEC_DEBOUNCE_DONE_STATUS_BIT),
			(bool)(stat & UFP_DFP_MODE_STATUS_BIT));

	power_supply_changed(chg->usb_psy);

	return IRQ_HANDLED;
}

/***************
 * Work Queues *
 ***************/

static void smblib_hvdcp_detect_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
					       hvdcp_detect_work.work);
	const struct apsd_result *apsd_result;

	apsd_result = smblib_get_apsd_result(chg);
	if (apsd_result->bit &&
			!(apsd_result->bit & (QC_2P0_BIT | QC_3P0_BIT))) {
		vote(chg->pd_allowed_votable, DEFAULT_VOTER, true, 0);
		power_supply_changed(chg->usb_psy);
	}
}

static void bms_update_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						bms_update_work);
	power_supply_changed(chg->batt_psy);
}

static void step_soc_req_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						step_soc_req_work.work);
	union power_supply_propval pval = {0, };
	int rc;

	rc = smblib_get_prop_batt_capacity(chg, &pval);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't get batt capacity rc=%d\n", rc);
		return;
	}

	step_charge_soc_update(chg, pval.intval);
}

static void smblib_pl_detect_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						pl_detect_work);

	if (!get_effective_result_locked(chg->pl_disable_votable))
		rerun_election(chg->pl_disable_votable);
}

#define MINIMUM_PARALLEL_FCC_UA		500000
#define PL_TAPER_WORK_DELAY_MS		100
#define TAPER_RESIDUAL_PERCENT		75
static void smblib_pl_taper_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						pl_taper_work.work);
	union power_supply_propval pval = {0, };
	int rc;

	if (chg->pl.slave_fcc < MINIMUM_PARALLEL_FCC_UA) {
		vote(chg->pl_disable_votable, TAPER_END_VOTER, true, 0);
		goto done;
	}

	rc = smblib_get_prop_batt_charge_type(chg, &pval);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't get batt charge type rc=%d\n", rc);
		goto done;
	}

	if (pval.intval == POWER_SUPPLY_CHARGE_TYPE_TAPER) {
		vote(chg->awake_votable, PL_VOTER, true, 0);
		/* Reduce the taper percent by 25 percent */
		chg->pl.taper_percent = chg->pl.taper_percent
					* TAPER_RESIDUAL_PERCENT / 100;
		rerun_election(chg->fcc_votable);
		schedule_delayed_work(&chg->pl_taper_work,
				msecs_to_jiffies(PL_TAPER_WORK_DELAY_MS));
		return;
	}

	/*
	 * Master back to Fast Charge, get out of this round of taper reduction
	 */
done:
	vote(chg->awake_votable, PL_VOTER, false, 0);
}

static int smblib_create_votables(struct smb_charger *chg)
{
	int rc = 0;

	chg->usb_suspend_votable = create_votable("USB_SUSPEND", VOTE_SET_ANY,
					smblib_usb_suspend_vote_callback,
					chg);
	if (IS_ERR(chg->usb_suspend_votable)) {
		rc = PTR_ERR(chg->usb_suspend_votable);
		return rc;
	}

	chg->dc_suspend_votable = create_votable("DC_SUSPEND", VOTE_SET_ANY,
					smblib_dc_suspend_vote_callback,
					chg);
	if (IS_ERR(chg->dc_suspend_votable)) {
		rc = PTR_ERR(chg->dc_suspend_votable);
		return rc;
	}

	chg->fcc_max_votable = create_votable("FCC_MAX", VOTE_MAX,
					smblib_fcc_max_vote_callback,
					chg);
	if (IS_ERR(chg->fcc_max_votable)) {
		rc = PTR_ERR(chg->fcc_max_votable);
		return rc;
	}

	chg->fcc_votable = create_votable("FCC", VOTE_MIN,
					smblib_fcc_vote_callback,
					chg);
	if (IS_ERR(chg->fcc_votable)) {
		rc = PTR_ERR(chg->fcc_votable);
		return rc;
	}

	chg->fv_votable = create_votable("FV", VOTE_MAX,
					smblib_fv_vote_callback,
					chg);
	if (IS_ERR(chg->fv_votable)) {
		rc = PTR_ERR(chg->fv_votable);
		return rc;
	}

	chg->usb_icl_votable = create_votable("USB_ICL", VOTE_MIN,
					smblib_usb_icl_vote_callback,
					chg);
	if (IS_ERR(chg->usb_icl_votable)) {
		rc = PTR_ERR(chg->usb_icl_votable);
		return rc;
	}

	chg->dc_icl_votable = create_votable("DC_ICL", VOTE_MIN,
					smblib_dc_icl_vote_callback,
					chg);
	if (IS_ERR(chg->dc_icl_votable)) {
		rc = PTR_ERR(chg->dc_icl_votable);
		return rc;
	}

	chg->pd_allowed_votable = create_votable("PD_ALLOWED", VOTE_SET_ANY,
					NULL, NULL);
	if (IS_ERR(chg->pd_allowed_votable)) {
		rc = PTR_ERR(chg->pd_allowed_votable);
		return rc;
	}

	chg->awake_votable = create_votable("AWAKE", VOTE_SET_ANY,
					smblib_awake_vote_callback,
					chg);
	if (IS_ERR(chg->awake_votable)) {
		rc = PTR_ERR(chg->awake_votable);
		return rc;
	}

	chg->pl_disable_votable = create_votable("PL_DISABLE", VOTE_SET_ANY,
					smblib_pl_disable_vote_callback,
					chg);
	if (IS_ERR(chg->pl_disable_votable)) {
		rc = PTR_ERR(chg->pl_disable_votable);
		return rc;
	}

	chg->chg_disable_votable = create_votable("CHG_DISABLE", VOTE_SET_ANY,
					smblib_chg_disable_vote_callback,
					chg);
	if (IS_ERR(chg->chg_disable_votable)) {
		rc = PTR_ERR(chg->chg_disable_votable);
		return rc;
	}

	return rc;
}

static void smblib_destroy_votables(struct smb_charger *chg)
{
	if (chg->usb_suspend_votable)
		destroy_votable(chg->usb_suspend_votable);
	if (chg->dc_suspend_votable)
		destroy_votable(chg->dc_suspend_votable);
	if (chg->fcc_max_votable)
		destroy_votable(chg->fcc_max_votable);
	if (chg->fcc_votable)
		destroy_votable(chg->fcc_votable);
	if (chg->fv_votable)
		destroy_votable(chg->fv_votable);
	if (chg->usb_icl_votable)
		destroy_votable(chg->usb_icl_votable);
	if (chg->dc_icl_votable)
		destroy_votable(chg->dc_icl_votable);
	if (chg->pd_allowed_votable)
		destroy_votable(chg->pd_allowed_votable);
	if (chg->awake_votable)
		destroy_votable(chg->awake_votable);
	if (chg->pl_disable_votable)
		destroy_votable(chg->pl_disable_votable);
}

static void smblib_iio_deinit(struct smb_charger *chg)
{
	if (!IS_ERR_OR_NULL(chg->iio.temp_chan))
		iio_channel_release(chg->iio.temp_chan);
	if (!IS_ERR_OR_NULL(chg->iio.temp_max_chan))
		iio_channel_release(chg->iio.temp_max_chan);
	if (!IS_ERR_OR_NULL(chg->iio.usbin_i_chan))
		iio_channel_release(chg->iio.usbin_i_chan);
	if (!IS_ERR_OR_NULL(chg->iio.usbin_v_chan))
		iio_channel_release(chg->iio.usbin_v_chan);
}

int smblib_init(struct smb_charger *chg)
{
	int rc = 0;

	mutex_init(&chg->write_lock);
	INIT_WORK(&chg->bms_update_work, bms_update_work);
	INIT_WORK(&chg->pl_detect_work, smblib_pl_detect_work);
	INIT_DELAYED_WORK(&chg->hvdcp_detect_work, smblib_hvdcp_detect_work);
	INIT_DELAYED_WORK(&chg->pl_taper_work, smblib_pl_taper_work);
	INIT_DELAYED_WORK(&chg->step_soc_req_work, step_soc_req_work);
	chg->fake_capacity = -EINVAL;

	switch (chg->mode) {
	case PARALLEL_MASTER:
		rc = smblib_create_votables(chg);
		if (rc < 0) {
			dev_err(chg->dev, "Couldn't create votables rc=%d\n",
				rc);
			return rc;
		}

		chg->bms_psy = power_supply_get_by_name("bms");
		chg->pl.psy = power_supply_get_by_name("parallel");

		rc = smblib_register_notifier(chg);
		if (rc < 0) {
			dev_err(chg->dev,
				"Couldn't register notifier rc=%d\n", rc);
			return rc;
		}

		break;
	case PARALLEL_SLAVE:
		break;
	default:
		dev_err(chg->dev, "Unsupported mode %d\n", chg->mode);
		return -EINVAL;
	}

	return rc;
}

int smblib_deinit(struct smb_charger *chg)
{
	switch (chg->mode) {
	case PARALLEL_MASTER:
		power_supply_unreg_notifier(&chg->nb);
		smblib_destroy_votables(chg);
		break;
	case PARALLEL_SLAVE:
		break;
	default:
		dev_err(chg->dev, "Unsupported mode %d\n", chg->mode);
		return -EINVAL;
	}

	smblib_iio_deinit(chg);

	return 0;
}
