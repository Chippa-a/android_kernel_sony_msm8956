/* Copyright (c) 2018 The Linux Foundation. All rights reserved.
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
#ifndef __QG_BATTERY_PROFILE_H__
#define __QG_BATTERY_PROFILE_H__

int qg_batterydata_init(struct device_node *node);
void qg_batterydata_exit(void);
int lookup_soc_ocv(u32 *soc, u32 ocv_uv, int batt_temp, u8 lookup);

#endif /* __QG_BATTERY_PROFILE_H__ */
