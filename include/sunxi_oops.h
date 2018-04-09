/* Copyright (C)
* 2014 - AllwinnerTech
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
*
*/

/**
* @file sunxi_oops.h
* @brief: hooks for oops,
*         print more information and enable sdcard jtag.
* @author AllwinnerTech
* @version v0.2
* @date 2017-5-8
*/


#ifndef __SUNXI_OOPS_H__
#define __SUNXI_OOPS_H__

#include <linux/io.h>
#include <linux/sys_config.h>
#include <linux/pinctrl/pinconf-sunxi.h>
#include <linux/pinctrl/consumer.h>
#include <linux/clk.h>
#include <linux/of.h>

#ifdef CONFIG_SUNXI_OOPS_HOOK

#ifdef CONFIG_ARCH_SUN50IW6P1
#define JTAG_MS "PF0"
#define JTAG_CK	"PF5"
#define JTAG_DO	"PF3"
#define JTAG_DI	"PF1"
#define CONFIG_SUNXI_CARD_JTAG
#endif

extern unsigned int cpufreq_get(unsigned int cpu);
extern unsigned long dramfreq_get(void);

void sunxi_oops_hook(void)
{
	struct clk *clk_cpu, *clk_gpu;
	unsigned int pll;

	/*	jtag_ms
	*	jtag_ck
	*	jtag_do
	*	jtag_di
	*/
#ifdef CONFIG_SUNXI_CARD_JTAG
	unsigned long config;
	config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_FUNC, 3);
	pin_config_set(SUNXI_PINCTRL, JTAG_MS, config);
	pin_config_set(SUNXI_PINCTRL, JTAG_CK, config);
	pin_config_set(SUNXI_PINCTRL, JTAG_DO, config);
	pin_config_set(SUNXI_PINCTRL, JTAG_DI, config);
#endif

	/*
	*		CPU FREQ
	*/
	pll = 0;
#ifdef CONFIG_CPU_FREQ
	pll = cpufreq_get(0);
#endif
	if (pll >= 1000)
		printk(KERN_EMERG "sunxi oops: cpu frequency: %d MHz\n", pll/1000);
	else {
		clk_cpu = clk_get(NULL, "pll_cpu");
		if (!IS_ERR_OR_NULL(clk_cpu)) {
			pll = clk_get_rate(clk_cpu);
		}
		printk(KERN_EMERG "sunxi oops: cpu frequency: %d MHz\n",
			pll >= 1000000 ? pll/1000000 : 0);
	}

	/*
	*		DRAM FREQ
	*/
	pll = 0;
#ifdef CONFIG_ARM_SUNXI_DRAM_DEVFREQ
	pll = dramfreq_get();
#endif
	printk(KERN_EMERG "sunxi oops: ddr frequency: %d MHz\n",
		pll >= 1000 ? pll/1000 : 0);

	/*
	*		GPU FREQ
	*/
	clk_gpu = clk_get(NULL, "gpu");
	pll = 0;
	if (!IS_ERR_OR_NULL(clk_gpu)) {
		pll = clk_get_rate(clk_gpu);
	}
	printk(KERN_EMERG "sunxi oops: gpu frequency: %d MHz\n",
		pll >= 1000000 ? pll/1000000 : 0);

	/*
	*		CPU OR GPU TEMP
	*/
	printk(KERN_EMERG "sunxi oops: cpu temperature: %d \n", 0);
}

#endif	/* CONFIG_SUNXI_OOPS_HOOK */

#endif	/* __SUNXI_OOPS_H__ */
