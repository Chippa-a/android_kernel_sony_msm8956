/* Copyright (c) 2017, The Linux Foundation. All rights reserved.
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

#ifndef _CAM_DEBUG_UTIL_H_
#define _CAM_DEBUG_UTIL_H_

#define CAM_CDM    (1 << 0)
#define CAM_CORE   (1 << 1)
#define CAM_CPAS   (1 << 2)
#define CAM_ISP    (1 << 3)
#define CAM_CRM    (1 << 4)
#define CAM_SENSOR (1 << 5)
#define CAM_SMMU   (1 << 6)
#define CAM_SYNC   (1 << 7)
#define CAM_ICP    (1 << 8)
#define CAM_JPEG   (1 << 9)
#define CAM_FD     (1 << 10)
#define CAM_LRME   (1 << 11)
#define CAM_FLASH  (1 << 12)

#define STR_BUFFER_MAX_LENGTH  1024

enum cam_debug_level {
	CAM_LEVEL_INFO,
	CAM_LEVEL_WARN,
	CAM_LEVEL_ERR,
	CAM_LEVEL_DBG,
};

/*
 *  cam_debug_log()
 *
 * @brief     :  Get the Module name from module ID and print
 *               respective debug logs
 *
 * @module_id :  Respective Module ID which is calling this function
 * @dbg_level :  Debug level from cam_module_debug_level enum entries
 * @func      :  Function which is calling to print logs
 * @line      :  Line number associated with the function which is calling
 *               to print log
 * @fmt       :  Formatted string which needs to be print in the log
 *
 */
void cam_debug_log(unsigned int module_id, enum cam_debug_level dbg_level,
	const char *func, const int line, const char *fmt, ...);

/*
 * cam_get_module_name()
 *
 * @brief     :  Get the module name from module ID
 *
 * @module_id :  Module ID which is using this function
 */
const char *cam_get_module_name(unsigned int module_id);

/*
 * CAM_ERR
 * @brief    :  This Macro will print error logs
 *
 * @__module :  Respective module id which is been calling this Macro
 * @fmt      :  Formatted string which needs to be print in log
 * @args     :  Arguments which needs to be print in log
 */
#define CAM_ERR(__module, fmt, args...)                            \
	cam_debug_log(__module, CAM_LEVEL_ERR, __func__, __LINE__, fmt, ##args)

/*
 * CAM_WARN
 * @brief    :  This Macro will print warning logs
 *
 * @__module :  Respective module id which is been calling this Macro
 * @fmt      :  Formatted string which needs to be print in log
 * @args     :  Arguments which needs to be print in log
 */
#define CAM_WARN(__module, fmt, args...)                           \
	cam_debug_log(__module, CAM_LEVEL_WARN, __func__, __LINE__, fmt, ##args)

/*
 * CAM_INFO
 * @brief    :  This Macro will print Information logs
 *
 * @__module :  Respective module id which is been calling this Macro
 * @fmt      :  Formatted string which needs to be print in log
 * @args     :  Arguments which needs to be print in log
 */
#define CAM_INFO(__module, fmt, args...)                           \
	cam_debug_log(__module, CAM_LEVEL_INFO, __func__, __LINE__, fmt, ##args)

/*
 * CAM_DBG
 * @brief :     This Macro will print debug logs when enabled using GROUP
 *
 * @__module :  Respective module id which is been calling this Macro
 * @fmt      :  Formatted string which needs to be print in log
 * @args     :  Arguments which needs to be print in log
 */
#define CAM_DBG(__module, fmt, args...)                            \
	cam_debug_log(__module, CAM_LEVEL_DBG, __func__, __LINE__, fmt, ##args)

/*
 * CAM_ERR_RATE_LIMIT
 * @brief :     This Macro will prevent error print logs with ratelimit
 */
#define CAM_ERR_RATE_LIMIT(__module, fmt, args...)                 \
	pr_err_ratelimited("CAM_ERR: %s: %s: %d\n" fmt,            \
		cam_get_module_name(__module), __func__,  __LINE__, ##args)

#endif /* _CAM_DEBUG_UTIL_H_ */
