// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2011, 2014-2016, 2018, The Linux Foundation. All rights reserved.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

/*mike_zhu 20200407 add for B200407-1964 start*/
int msm_show_resume_irq_mask = 1;
/*mike_zhu 20200407 add for B200407-1964 end*/

module_param_named(
	debug_mask, msm_show_resume_irq_mask, int, 0664);
