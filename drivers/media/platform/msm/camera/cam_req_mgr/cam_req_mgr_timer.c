/* Copyright (c) 2016-2017, The Linux Foundation. All rights reserved.
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

#include "cam_req_mgr_timer.h"

void crm_timer_reset(struct cam_req_mgr_timer *crm_timer)
{
	if (!crm_timer)
		return;
	CRM_DBG("Starting timer to fire in %d ms. (jiffies=%lu)\n",
		crm_timer->expires, jiffies);
	mod_timer(&crm_timer->sys_timer,
		(jiffies + msecs_to_jiffies(crm_timer->expires)));
}

void crm_timer_callback(unsigned long data)
{
	struct cam_req_mgr_timer *timer = (struct cam_req_mgr_timer *)data;

	if (!timer) {
		CRM_ERR("NULL timer");
		return;
	}
	CRM_DBG("timer %pK parent %pK", timer, timer->parent);
	crm_timer_reset(timer);
}

void crm_timer_modify(struct cam_req_mgr_timer *crm_timer,
	int32_t expires)
{
	CRM_DBG("new time %d", expires);
	if (crm_timer) {
		crm_timer->expires = expires;
		crm_timer_reset(crm_timer);
	}
}

int crm_timer_init(struct cam_req_mgr_timer **timer,
	int32_t expires, void *parent, void (*timer_cb)(unsigned long))
{
	int                       ret = 0;
	struct cam_req_mgr_timer *crm_timer = NULL;

	CRM_DBG("init timer %d %pK", expires, *timer);
	if (*timer == NULL) {
		crm_timer = (struct cam_req_mgr_timer *)
			kzalloc(sizeof(struct cam_req_mgr_timer), GFP_KERNEL);
		if (!crm_timer) {
			ret = -ENOMEM;
			goto end;
		}

		if (timer_cb != NULL)
			crm_timer->timer_cb = timer_cb;
		else
			crm_timer->timer_cb = crm_timer_callback;

		crm_timer->expires = expires;
		crm_timer->parent = parent;
		setup_timer(&crm_timer->sys_timer,
			crm_timer->timer_cb, (unsigned long)crm_timer);
		crm_timer_reset(crm_timer);
		*timer = crm_timer;
	} else {
		CRM_WARN("Timer already exists!!");
		ret = -EINVAL;
	}
end:
	return ret;
}
void crm_timer_exit(struct cam_req_mgr_timer **crm_timer)
{
	CRM_DBG("destroy timer %pK", *crm_timer);
	if (*crm_timer) {
		del_timer(&(*crm_timer)->sys_timer);
		kfree(*crm_timer);
		*crm_timer = NULL;
	}
}

