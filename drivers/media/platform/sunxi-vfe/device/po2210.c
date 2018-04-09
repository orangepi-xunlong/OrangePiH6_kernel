/* **************************************************************************************
 *po2210.c
 *A V4L2 driver for PO2210N cameras
 *Copyright (c) 2014 by Allwinnertech Co., Ltd.http://www.allwinnertech.com
 *	Version		Author		Date				Description
 *	1.0						2016/9/23		PO2210N PARALLEL YUV sensor Support
 *  1.0                     2016/1101       20161101.ccf
 *  1.0                     2016/1124       720P_161124.ccf 1080P_161124.ccf
 ****************************************************************************************
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/clk.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-mediabus.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include "camera.h"

MODULE_AUTHOR("richard.liu");
MODULE_DESCRIPTION("A low-level driver for po2210n sensors");
MODULE_LICENSE("GPL");

/*for internel driver debug*/
#define DEV_DBG_EN      1
#ifdef DEV_DBG_EN
#define vfe_dev_dbg(x, arg...)	printk(KERN_ERR"[PO2210N]"x,	##arg)
#else
#define vfe_dev_dbg(x, arg...) do { } while (0)
#endif

#define vfe_dev_err(x, arg...)	printk(KERN_ERR"[PO2210N]"x,	##arg)
#define vfe_dev_print(x, arg...) printk(KERN_ERR"[PO2210N]"x,	##arg)
#define LOG_ERR_RET(x)  { \
							int ret;  \
							ret = x; \
							if (ret < 0) {\
								vfe_dev_err("error at %s\n", __func__);  \
								return ret; \
							} \
						}

/*define module timing*/
#define MCLK              (27*1000*1000)
#define VREF_POL          V4L2_MBUS_VSYNC_ACTIVE_HIGH

#define HREF_POL          V4L2_MBUS_HSYNC_ACTIVE_HIGH
#define CLK_POL           V4L2_MBUS_PCLK_SAMPLE_RISING
#define V4L2_IDENT_SENSOR 0x2210

/*define the voltage level of control signal*/
#define CSI_STBY_ON     1
#define CSI_STBY_OFF    0
#define CSI_RST_ON      0
#define CSI_RST_OFF     1
#define CSI_PWR_ON      1
#define CSI_PWR_OFF     0

#define SENSOR_NAME "po2210n"
#define regval_list reg_list_a8_d8

#define REG_DLY  0xff
#define FLIP_H_V  0x00

#define SENSOR_FRAME_RATE 30
#define SENSOR_FRAME_RATE_15FPS 15
#define USE_CSI_ISP_FUNCTION  1


/*
 * The po2210n sits on i2c with ID 0xEE
 */
#define I2C_ADDR 0xEE
static struct v4l2_subdev *glb_sd;

static u32 g_denominator = 30;


/*
 * Information we maintain about a known sensor.
 */

struct cfg_array { /* coming later */
	struct regval_list *regs;
	int size;
};

static inline struct sensor_info *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct sensor_info, sd);
}

/*
 * The default register settings
 *
 */
static struct regval_list sensor_default_regs[] = {
	 /*30fps*/
	{0x03, 0x00},
	{0x3C, 0xC4},	/*# Internal DVDD OFF, bgrcon_15 = 000b (1.35V)*/
	{0x04, 0x01},	/*# chip_mode*/
	{0x06, 0x08},	/*# framewidth_h*/
	{0x07, 0x97},	/*# framewidth_l*/
	{0x24, 0x0A},	/*# clkdiv1*/
	{0x25, 0x22},	/*# clkdiv2*/
	{0x2F, 0x01},	/*# pad_control7        (01)*/
	{0x2A, 0xF3},	/*# pad_control2        (00)  //max drive //0x43*/
	{0x2B, 0xFC},	/*# pad_control3        (00)  //max drive //0x9C*/
	{0x2E, 0x03},	/*# pad_control6        (00)*/
	{0x30, 0xFF},	/*# pad_control8        (00)*/
	{0x31, 0xFF},	/*# pad_control9        (00)*/
	{0x87, 0x88},	/*# led_control*/
	{0x40, 0x0B},	/*# pll_m_cnt */
	{0x41, 0x02},	/*# pll_r_cnt*/
	{0x3F, 0x40},	/*# pll_control1*/
	{0x03, 0x01},	/*############## Start Settings ################*/
	{0x16, 0x04},	/*# led_dsel*/
	{0xB7, 0x30},	/*# adcoffset*/
	{0x03, 0x02},
	{0x2B, 0x14},	/*# dpc_offset*/
	{0x03, 0x01},	/*##################################### blacksun */
	{0x1E, 0x0E},	/*# bsmode off*/
	{0x26, 0x04},	/*# blacksunth_h*/
	{0x03, 0x01},	/*# Limiter reference fitting due to gain*/
	{0xF6, 0x0E},	/*# bs_ofst0*/
	{0xF7, 0x14},	/*# bs_ofst1*/
	{0xF8, 0x24},	/*# bs_ofst2 */
	{0xF9, 0x26},	/*# bs_ofst3*/
	{0xFA, 0x26},	/*# bs_ofst4*/
	{0xFB, 0x26},	/*# bs_ofst5*/
	{0xFC, 0x26},	/*# bs_ofst6*/
	{0xFD, 0x26},	/*# bs_ofst_max*/
	{0xFE, 0x00},	/*# bs_ofst_min*/
	{0x03, 0x00},	/*##################################### cds v1.1*/
	{0x35, 0x08},	/*# pixelbias (01)*/
	{0x36, 0x04},	/*# compbias (02)*/
	{0x03, 0x01},
	{0x19, 0xC4},	/*# ramppclk_sel*/
	{0x1C, 0x11},	/*# ramp speed X1, adc speed X1*/
	{0x03, 0x01},
	{0x57, 0x08},
	{0x58, 0x7F},
	{0x59, 0x08},
	{0x5A, 0x96},
	{0x53, 0x00},
	{0x54, 0x02},
	{0x55, 0x08},
	{0x56, 0x7F},
	{0x67, 0x00},
	{0x68, 0x54},
	{0x69, 0x00},
	{0x6A, 0x5E},
	{0x5B, 0x00},
	{0x5C, 0x00},
	{0x5D, 0x08},
	{0x5E, 0x7F},
	{0x5F, 0x00},
	{0x60, 0x00},
	{0x61, 0x00},
	{0x62, 0x50},
	{0x99, 0x00},
	{0x9A, 0x54},
	{0x9B, 0x08},
	{0x9C, 0x7F},
	{0x6F, 0x00},
	{0x70, 0x00},
	{0x71, 0x05},
	{0x72, 0x7A},
	{0x73, 0x00},
	{0x74, 0x00},
	{0x75, 0x05},
	{0x76, 0x78},
	{0x77, 0x08},
	{0x78, 0x95},
	{0x79, 0x08},
	{0x7A, 0x96},
	{0x8F, 0x00},
	{0x90, 0x52},
	{0x8B, 0x00},
	{0x8C, 0x64},
	{0x8D, 0x08},
	{0x8E, 0x6A},
	{0x87, 0x08},
	{0x88, 0x48},
	{0x89, 0x08},
	{0x8A, 0x7C},
	{0x95, 0x08},
	{0x96, 0x80},
	{0x97, 0x08},
	{0x98, 0x8F},
	{0x91, 0x08},
	{0x92, 0x80},
	{0x93, 0x08},
	{0x94, 0x97},
	{0x7F, 0x08},
	{0x80, 0x80},
	{0x81, 0x08},
	{0x82, 0x8F},
	{0x83, 0x08},
	{0x84, 0x80},

	{0x85, 0x08},
	{0x86, 0x8F},
	{0xB9, 0x08},
	{0xBA, 0x80},
	{0xBB, 0x08},
	{0xBC, 0x8F},
	{0xA1, 0x0B},
	{0xA2, 0x84},
	{0x36, 0x00},
	{0x37, 0xBE},
	{0x38, 0x08},
	{0x39, 0x4E},
	{0x7B, 0x00},
	{0x7C, 0x00},
	{0x7D, 0x05},
	{0x7E, 0x7C},
	{0x3E, 0x00},
	{0x3F, 0xBE},
	{0x40, 0x08},
	{0x41, 0x4E},
	{0x03, 0x00},	/*##################################### ablc*/
	{0x38, 0x90},	/*# analog_control_02*/
	{0x3D, 0x2F},	/*# analog_control_07*/
	{0x03, 0x01},	/*# bank B*/
	{0x1F, 0x51},	/*# bayer_control_10*/
	{0x20, 0xA8},	/*# Median value for filter and Average value for selection*/
	{0xA3, 0xE0},	/*# blc_top_th*/
	{0xA4, 0x70},	/*# blc_bot_th*/
	{0xA5, 0x02},	/*# ablc_update*/
	{0x03, 0x04},
	{0x06, 0xA1},	/*# auto_control_3[0] : ablc fitting enable*/
	{0x03, 0x04},	/*# fitting x reference*/
	{0xC7, 0x00},	/*# overOBP_xref0*/
	{0xC8, 0x08},	/*# overOBP_xref1*/
	{0xC9, 0x1E},	/*# overOBP_xref2*/
	{0xCA, 0x32},	/*# overOBP_xref3*/
	{0xCB, 0x58},	/*# overOBP_xref4*/
	{0x03, 0x03},	/*# fitting y reference*/
	{0xDC, 0x00},	/*# overOBP_yref0*/
	{0xDD, 0x16},	/*# overOBP_yref1*/
	{0xDE, 0x1B},	/*# overOBP_yref2*/
	{0xE0, 0x25},	/*# overOBP_yref3*/
	{0xE1, 0x30},	/*# overOBP_yref4*/
	{0x03, 0x03},	/*##################################### intp*/
	{0x30, 0x00},	/*# intp_w0       (10)*/
	{0x31, 0xFF},	/*# intp_x0       (00)*/
	{0x32, 0x40},	/*# intp_slope    (40)*/
	{0x03, 0x02},
	{0x05, 0xFB},	/*# [4] edge_blf_mode : 0=new, 1=old  FB*/
	{0x03, 0x03},
	{0x33, 0x00},	/*# blf_w0_ref0   00*/
	{0x34, 0x40},	/*# blf_w0_ref1   00*/
	{0x35, 0x40},	/*# blf_w0_ref2   00*/
	{0x37, 0x20},	/*# blf_x0    20*/
	{0x38, 0x40},	/*# blf_slope 40*/
	{0x39, 0x7F},	/*# blf_c0    80  7F*/
	{0x3A, 0x78},	/*# blf_c1    60  78*/
	{0x3B, 0x63},	/*# blf_c2    40  63*/
	{0x3C, 0x3F},	/*# blf_c3    20  2F*/
	{0x3D, 0x2B},	/*# blf_c4    10  0B*/
	{0x3E, 0x18},	/*# blf_c5    08  00*/
	{0x03, 0x09},	/*########################### sc*/
	{0x04, 0x03},	/*#    acce_ctrl_0 [1]:acce enable, [0]:histogram enable (00)*/
	{0x6D, 0x04},	/*# ac_ctrl_0 [2]:AE relate mode*/
	{0x49, 0x30},	/*# ce_th      (20)*/
	{0x4A, 0x10},	/*# ce_x0      (40)*/
	{0x4B, 0x40},	/*# ce_slope   (40)*/
	{0xAD, 0x08},	/*#08 # lpf_w1 (08)*/
	{0xAE, 0x10},	/*#10 # lpf_w2 (18)*/
	{0xAF, 0x20},	/*#20 # lpf_w3 (40)*/
	{0xB0, 0x10},	/*#10 # lpf_w4 (18)*/
	{0xB1, 0x08},	/*#08 # lpf_w5 (08)*/
	{0xB2, 0x04},	/*# ac_offset*/
	{0xB3, 0x60},	/*# max_ac_gain0*/
	{0xB4, 0x60},	/*# max_ac_gain1*/
	{0xB5, 0x40},	/*# max_ac_gain2*/
	{0xB7, 0x40},	/*# min_ac_gain*/
	{0xB8, 0x03},	/*# ac_speed*/
	{0xB9, 0x02},	/*# ac_lock*/
	{0xBB, 0x04},	/*# ac_frame*/
	{0x8E, 0x00},	/*# ac_cv_w0*/
	{0x8F, 0x04},	/*# ac_cv_w1*/
	{0x90, 0x06},	/*# ac_cv_w2*/
	{0x91, 0x06},	/*# ac_cv_w3*/
	{0x92, 0x04},	/*# ac_cv_w4*/
	{0x93, 0x03},	/*# ac_cv_w5*/
	{0x94, 0x01},	/*# ac_cv_w6*/
	{0x95, 0x00},	/*# ac_cv_w7*/

	{0x03, 0x00},	/*#################### tune*/
	{REG_DLY, FLIP_H_V},	/*x05,0x00},	//# Mir*/
	{0x05, 0x00},   /*# mirror*/

	{0x4A, 0x08},	/*# FlkCtl*/
	{0x54, 0x01},	/*# fd_period_b*/
	{0x55, 0x51},
	{0x56, 0xA7},
    {0x03, 0x01},	/*# ----------- B*/
    {0x16, 0x04},	/*#Variable frame rate control*/
	{0x03, 0x02},	/*# ----------- C*/

	{0x04, 0xF5},	/*#ISP function control*/
	{0x05, 0xFB},
	{0x08, 0x00},

	{0x09, 0x01},
	{0x0B, 0x82},

	{0x1E, 0x03},   /*#lens gain fitting*/
	{0x1F, 0x03},

	{0x33, 0x32},	/*# ccr*/
	{0x34, 0x87},
	{0x35, 0x8B},
	{0x36, 0x8F},
	{0x37, 0x3F},
	{0x38, 0x8F},
	{0x39, 0x92},
	{0x3A, 0x85},
	{0x3B, 0x37},
	{0x3D, 0x00},	/*# yGm1  #gamma curve fitting    20161124*/
	{0x3E, 0x03},
	{0x3F, 0x0C},
	{0x40, 0x19},
	{0x41, 0x26},
	{0x42, 0x3F},
	{0x43, 0x52},
	{0x44, 0x6E},
	{0x45, 0x82},
	{0x46, 0xA1},
	{0x47, 0xB9},
	{0x48, 0xCE},
	{0x49, 0xE0},
	{0x4A, 0xF0},
	{0x4B, 0xFF},
	{0x4C, 0x00},	/*# yGm2  #gamma curve fitting*/
	{0x4D, 0x27},
	{0x4E, 0x36},
	{0x4F, 0x40},
	{0x50, 0x49},
	{0x51, 0x58},
	{0x52, 0x64},
	{0x53, 0x78},
	{0x54, 0x89},
	{0x55, 0xA4},
	{0x56, 0xBB},
	{0x57, 0xCF},
	{0x58, 0xE0},
	{0x59, 0xF1},
	{0x5A, 0xFF},

	{0x5B, 0x00},	/*# cGm1  #rgb gamma1 curve*/
	{0x5C, 0x03},
	{0x5D, 0x0C},
	{0x5E, 0x19},
	{0x5F, 0x26},
	{0x60, 0x3F},
	{0x61, 0x52},
	{0x62, 0x6E},
	{0x63, 0x82},
	{0x64, 0xA1},
	{0x65, 0xB9},
	{0x66, 0xCE},
	{0x67, 0xE0},
	{0x68, 0xF0},
	{0x69, 0xFF},

	{0x6A, 0x00},  /*#gamma curve fitting*/
	{0x6B, 0x02},
	{0x6C, 0x07},
	{0x6D, 0x0F},
	{0x6E, 0x17},
	{0x6F, 0x25},
	{0x70, 0x31},
	{0x71, 0x46},
	{0x72, 0x58},
	{0x73, 0x79},
	{0x74, 0x97},
	{0x75, 0xB3},
	{0x76, 0xCE},
	{0x77, 0xE7},
	{0x78, 0xFF},

	{0x80, 0x2A},  /*#Color saturation*/
	{0x81, 0x86},
	{0x82, 0x04},
	{0x83, 0x28},

	{0x95, 0x00},	/*# Bri #ybrightness*/
	{0x96, 0x00},
	{0x97, 0x0C},
	{0xB3, 0x00},
	{0xB4, 0x05},
	{0xB5, 0x07},
	{0xB6, 0x84},
	{0xB7, 0x00},
	{0xB8, 0xC8},
	{0xB9, 0x04},
	{0xBA, 0x48},
	{0xBB, 0x01},
	{0xBC, 0x2C},
	{0xBD, 0x06},
	{0xBE, 0x64},
	{0xBF, 0x01},
	{0xC0, 0x7C},
	{0xC1, 0x03},
	{0xC2, 0x5C},
	{0xC7, 0x00},
	{0xC8, 0x05},
	{0xC9, 0x07},
	{0xCA, 0x84},
	{0xCB, 0x00},
	{0xCC, 0x78},
	{0xCD, 0x03},
	{0xCE, 0xD4},
	{0x03, 0x03},	/*# ----------- D*/

	{0x04, 0x02},
	{0x05, 0x00},
	{0x06, 0x08},
	{0x07, 0x00},
	{0x08, 0x00},
	{0x09, 0x00},

	{0x0A, 0x3E},
	{0x0B, 0x5D},
	{0x0C, 0x6C},
	{0x0D, 0x38},	/*# user cs        #Color saturation weight*/
	{0x1A, 0x10},	/*# dpc_p*/
	{0x1B, 0x10},
	{0x1C, 0x50},
	{0x1E, 0x00},	/*# dpc_n*/
	{0x1F, 0x08},
	{0x20, 0x18},
	{0x24, 0x1C},	/*# hf_dir_max*/
	{0x25, 0x1C},
	{0x26, 0x7F},
	{0x28, 0x08},	/*# intp_dir_th*/
	{0x29, 0x08},
	{0x2A, 0x7F},
	{0x3F, 0x18},	/*# blf_darkness*/
	{0x40, 0x18},
	{0x41, 0x18},
	{0x48, 0x30},	/*# e_gm_curve*/
	{0x49, 0x2A},
	{0x4A, 0x27},
	{0x4B, 0x2A},
	{0x4C, 0x27},
	{0x4D, 0x24},
	{0x4E, 0x28},
	{0x4F, 0x00},	/*# e_gm*/
	{0x50, 0x00},
	{0x51, 0x00},
	{0x53, 0x10},	/*# lf  edge_gain_lf*/
	{0x54, 0x10},
	{0x55, 0x10},
	{0x57, 0x10},	/*# ghf   #edge_gain_ghf*/
	{0x58, 0x10},
	{0x59, 0x10},
	{0x5B, 0x10},	/*# ehf  #edge_gain_ehf*/
	{0x5C, 0x10},
	{0x5D, 0x10},
	{0x60, 0x04},	/*# ec_pth*/
	{0x61, 0x04},
	{0x62, 0x04},
	{0x64, 0x04},	/*# ec_mth*/
	{0x65, 0x04},
	{0x66, 0x04},
	{0x68, 0x40},	/*# ec_pmax*/
	{0x69, 0x10},
	{0x6A, 0x10},
	{0x6C, 0x40},	/*# ec_mmax*/
	{0x6D, 0x20},
	{0x6E, 0x20},
	{0x70, 0x40},	/*# ec_pgain*/
	{0x71, 0x40},	/*# ec_mgain*/
	{0x72, 0x00},
	{0x73, 0x10},
	{0x74, 0x20},
	{0x7A, 0x40},	/*# y_weight*/
	{0x7B, 0x40},
	{0x7C, 0x48},
	{0x7E, 0x00},	/*# ccr*/
	{0x7F, 0x01},
	{0x80, 0x02},
	{0x82, 0x00},	/*# dc*/
	{0x83, 0x00},
	{0x84, 0x00},
	{0x86, 0x16},	/*# dc_y1 dark_dc_y1*/
	{0x87, 0x16},
	{0x88, 0x18},
	{0x8A, 0xF0},
	{0x93, 0x08},   /*#Vector control*/
	{0x94, 0x14},	/*# vec_sample_range*/
	{0x95, 0x1A},
	{0x98, 0x1B},
	{0x99, 0x27},
	{0x9C, 0x35},
	{0x9D, 0x42},
	{0xA0, 0x44},
	{0xA1, 0x62},
	{0xA4, 0x64},
	{0xA5, 0x72},
	{0xA8, 0x76},
	{0xA9, 0x80},
	{0xAC, 0x90},	/*# vec_h&s_a*/
	{0xAD, 0x88},	/*# vec_h&s_b*/
	{0xAE, 0x88},	/*# vec_h&s_c*/
	{0xAF, 0x04},
	{0xB0, 0x04},
	{0xB1, 0x04},
	{0xB2, 0x07},
	{0xB3, 0x01},
	{0xB4, 0x00},
	{0xB5, 0x04},
	{0xB6, 0x00},
	{0xB7, 0x04},
	{0xB8, 0x82},
	{0xB9, 0x05},
	{0xBA, 0x01},
	{0xBB, 0x04},
	{0xBC, 0x04},
	{0xBD, 0x04},
	{0xBE, 0x92},
	{0xBF, 0x8C},
	{0xC0, 0x00},
	{0xC1, 0x04},
	{0xC2, 0x00},
	{0xC3, 0x06},
	{0xC4, 0x85},
	{0xC5, 0x85},
	{0xC6, 0x07},
	{0xC7, 0x04},
	{0xC8, 0x04},
	{0xC9, 0x04},
	{0xCA, 0x86},
	{0xCB, 0x84},
	{0xCC, 0x00},
	{0xCD, 0x04},
	{0xCE, 0x04},
	{0xCF, 0x04},
	{0xE8, 0x40},	/*# slope2  #ycont_slope2*/
	{0xE9, 0x44},
	{0xEA, 0x44},
	{0x03, 0x04},	/*# ----------- E*/

	{0x05, 0x7F},   /*#enable lens/cs fitting*/

	{0x06, 0xA1},
	{0x12, 0x04},	/*# ExpFrmH  Auto exposure control */
	{0x13, 0x5E},
	{0x14, 0x04},
	{0x15, 0x5E},
	{0x16, 0x04},
	{0x17, 0x5E},
	{0x1B, 0x00},	/*# Gain*/
	{0x1C, 0x3D},
	{0x1D, 0x24},
	{0x1E, 0x00},
	{0x1F, 0x3D},
	{0x20, 0x24},
	{0x2C, 0x66},
	{0x30, 0x08},
	{0x31, 0x08},
	{0x32, 0x10},
	{0x33, 0x10},
	{0x34, 0x28},
	{0x3B, 0x48},	/*# yTg   Y target control*/
	{0x3C, 0x50},
	{0x3D, 0x44},
	{0x3E, 0x48},
	{0x3F, 0x50},
	{0x40, 0x44},
	{0x41, 0x00},	/*# yt_xref1*/
	{0x42, 0x00},
	{0x43, 0x14},
	{0x44, 0x00},	/*# yt_xref2*/
	{0x45, 0x02},
	{0x46, 0xE8},
	{0x47, 0x00},	/*# yt_xref3*/
	{0x48, 0x45},
	{0x49, 0xE0},
	{0x4A, 0x00},	/*# yt_xref4*/
	{0x4B, 0x8B},
	{0x4C, 0xC0},
	{0x55, 0x04},	/*# ae_speed_u*/
	{0x56, 0x04},	/*# ae_speed_d*/
	{0x57, 0x0C},	/*# ae_lock*/
	{0x5C, 0x00},	/*# awb area*/
	{0x5D, 0x40},
	{0x5E, 0xA0},
	{0x5F, 0x01},
	{0x60, 0x02},
	{0x61, 0x50},
	{0x62, 0x02},
	{0x63, 0x00},
	{0x64, 0x04},
	{0x65, 0x6E},
	{0x66, 0x45},
	{0x67, 0x27},
	{0x68, 0x4F},
	{0x69, 0x64},
	{0x6A, 0xC4},
	{0x6B, 0x0A},
	{0x6C, 0x46},
	{0x6D, 0x32},
	{0x6E, 0x78},
	{0x6F, 0x37},
	{0x70, 0xAF},
	{0x71, 0x32},
	{0x72, 0x23},
	{0x73, 0x78},	/*# rg_ratio_b  #awb rg/bg ratio fitting*/
	{0x74, 0x80},
	{0x75, 0x7C},	/*# rg_ratio_c*/
	{0x76, 0x80},
	{0x77, 0x80},
	{0x78, 0x80},	/*# bg_ratio_c*/
	{0x7E, 0x08},	/*# awb_lock*/
	{0x7F, 0x04},	/*# awb_speed*/
	{0x8D, 0xF1},
	{0x8F, 0x00},
	{0x90, 0x78},	/*# group1:yGm,cGm,de-color*/
	{0x91, 0x01},	/*# group1:wWeight*/
	{0x94, 0x00},
	{0x95, 0x00},
	{0x96, 0x00},   /*Darkness Mode Control*/
	{0x98, 0x00},  	/*# xref1*/
	{0x99, 0x03},
	{0x9A, 0x00},
	{0x9B, 0x09},
	{0x9C, 0x00},
	{0x9D, 0x10},
	{0x9E, 0x00},	/*# xref2*/
	{0x9F, 0x02},
	{0xA0, 0x00},
	{0xA1, 0x05},
	{0xA2, 0x00},
	{0xA3, 0x07},
	{0xBA, 0x10},
};

static struct regval_list sensor_15fps_regs[] = {
	{0x03, 0x00},
	{0x3C, 0xC4},	/*# Internal DVDD OFF, bgrcon_15 = 000b (1.35V)*/
	{0x04, 0x01},	/*# chip_mode*/
	{0x06, 0x08},	/*# framewidth_h*/
	{0x07, 0x97},	/*# framewidth_l*/
	{0x24, 0x0A},	/*# clkdiv1*/
	{0x25, 0x22},	/*# clkdiv2*/
	{0x2F, 0x01},	/*# pad_control7        (01)*/
	{0x2A, 0x43},	/*# pad_control2        (00)*/
	{0x2B, 0x9C},	/*# pad_control3        (00)*/
	{0x2E, 0x03},	/*# pad_control6        (00)*/
	{0x30, 0xFF},	/*# pad_control8        (00)*/
	{0x31, 0xFF},	/*# pad_control9        (00)*/
	{0x87, 0x88},	/*# led_control*/
	{0x40, 0x0B},	/*# pll_m_cnt*/
	{0x41, 0x04},	/*# pll_r_cnt*/
	{0x3F, 0x40},	/*# pll_control1*/
	{0x03, 0x01},	/*############## Start Settings ################*/
	{0x16, 0x04},	/*# led_dsel*/
	{0xB7, 0x30},	/*# adcoffset*/
	{0x03, 0x02},
	{0x2B, 0x14},	/*# dpc_offset*/
	{0x03, 0x01},	/*##################################### blacksun*/
	{0x1E, 0x0E},	/*# bsmode off*/
	{0x26, 0x04},	/*# blacksunth_h*/
	{0x03, 0x01},	/*# Limiter reference fitting due to gain*/
	{0xF6, 0x0E},	/*# bs_ofst0*/
	{0xF7, 0x14},	/*# bs_ofst1*/
	{0xF8, 0x24},	/*# bs_ofst2*/
	{0xF9, 0x26},	/*# bs_ofst3*/
	{0xFA, 0x26},	/*# bs_ofst4*/
	{0xFB, 0x26},	/*# bs_ofst5*/
	{0xFC, 0x26},	/*# bs_ofst6*/
	{0xFD, 0x26},	/*# bs_ofst_max*/
	{0xFE, 0x00},	/*# bs_ofst_min*/
	{0x03, 0x00},	/*##################################### cds v1.1*/
	{0x35, 0x08},	/*# pixelbias (01)*/
	{0x36, 0x04},	/*# compbias (02)*/
	{0x03, 0x01},
	{0x19, 0xC4},	/*# ramppclk_sel*/
	{0x1C, 0x11},	/*# ramp speed X1, adc speed X1*/
	{0x03, 0x01},
	{0x57, 0x08},
	{0x58, 0x7F},
	{0x59, 0x08},
	{0x5A, 0x96},
	{0x53, 0x00},
	{0x54, 0x02},
	{0x55, 0x08},
	{0x56, 0x7F},
	{0x67, 0x00},
	{0x68, 0x54},
	{0x69, 0x00},
	{0x6A, 0x5E},
	{0x5B, 0x00},
	{0x5C, 0x00},
	{0x5D, 0x08},
	{0x5E, 0x7F},
	{0x5F, 0x00},
	{0x60, 0x00},
	{0x61, 0x00},
	{0x62, 0x50},
	{0x99, 0x00},
	{0x9A, 0x54},
	{0x9B, 0x08},
	{0x9C, 0x7F},
	{0x6F, 0x00},
	{0x70, 0x00},
	{0x71, 0x05},
	{0x72, 0x7A},
	{0x73, 0x00},
	{0x74, 0x00},
	{0x75, 0x05},
	{0x76, 0x78},
	{0x77, 0x08},
	{0x78, 0x95},
	{0x79, 0x08},
	{0x7A, 0x96},
	{0x8F, 0x00},
	{0x90, 0x52},
	{0x8B, 0x00},
	{0x8C, 0x64},
	{0x8D, 0x08},
	{0x8E, 0x6A},
	{0x87, 0x08},
	{0x88, 0x48},
	{0x89, 0x08},
	{0x8A, 0x7C},
	{0x95, 0x08},
	{0x96, 0x80},
	{0x97, 0x08},
	{0x98, 0x8F},
	{0x91, 0x08},
	{0x92, 0x80},
	{0x93, 0x08},
	{0x94, 0x97},
	{0x7F, 0x08},
	{0x80, 0x80},
	{0x81, 0x08},
	{0x82, 0x8F},
	{0x83, 0x08},
	{0x84, 0x80},
	{0x85, 0x08},
	{0x86, 0x8F},
	{0xB9, 0x08},
	{0xBA, 0x80},
	{0xBB, 0x08},
	{0xBC, 0x8F},
	{0xA1, 0x0B},
	{0xA2, 0x84},
	{0x36, 0x00},
	{0x37, 0xBE},
	{0x38, 0x08},
	{0x39, 0x4E},
	{0x7B, 0x00},
	{0x7C, 0x00},
	{0x7D, 0x05},
	{0x7E, 0x7C},
	{0x3E, 0x00},
	{0x3F, 0xBE},
	{0x40, 0x08},
	{0x41, 0x4E},
	{0x03, 0x00},	/*##################################### ablc*/
	{0x38, 0x90},	/*# analog_control_02*/
	{0x3D, 0x2F},	/*# analog_control_07*/
	{0x03, 0x01},	/*# bank B*/
	{0x1F, 0x51},	/*# bayer_control_10*/
	{0x20, 0xA8},	/*# Median value for filter and Average value for selection*/
	{0xA3, 0xE0},	/*# blc_top_th*/
	{0xA4, 0x70},	/*# blc_bot_th*/
	{0xA5, 0x02},	/*# ablc_update*/
	{0x03, 0x04},
	{0x06, 0xA1},	/*# auto_control_3[0] : ablc fitting enable*/
	{0x03, 0x04},	/*# fitting x reference*/
	{0xC7, 0x00},	/*# overOBP_xref0*/
	{0xC8, 0x08},	/*# overOBP_xref1*/
	{0xC9, 0x1E},	/*# overOBP_xref2*/
	{0xCA, 0x32},	/*# overOBP_xref3*/
	{0xCB, 0x58},	/*# overOBP_xref4*/
	{0x03, 0x03},	/*# fitting y reference*/
	{0xDC, 0x00},	/*# overOBP_yref0*/
	{0xDD, 0x16},	/*# overOBP_yref1*/
	{0xDE, 0x1B},	/*# overOBP_yref2*/
	{0xE0, 0x25},	/*# overOBP_yref3*/
	{0xE1, 0x30},	/*# overOBP_yref4*/
	{0x03, 0x03},	/*##################################### intp*/
	{0x30, 0x00},	/*# intp_w0       (10)*/
	{0x31, 0xFF},	/*# intp_x0       (00)*/
	{0x32, 0x40},	/*# intp_slope    (40)*/
	{0x03, 0x02},
	{0x05, 0xFB},	/*# [4] edge_blf_mode : 0=new, 1=old  FB*/
	{0x03, 0x03},
	{0x33, 0x00},	/*# blf_w0_ref0   00*/
	{0x34, 0x40},	/*# blf_w0_ref1   00*/
	{0x35, 0x40},	/*# blf_w0_ref2   00*/
	{0x37, 0x20},	/*# blf_x0    20*/
	{0x38, 0x40},	/*# blf_slope 40*/
	{0x39, 0x7F},	/*# blf_c0    80  7F*/
	{0x3A, 0x78},	/*# blf_c1    60  78*/
	{0x3B, 0x63},	/*# blf_c2    40  63*/
	{0x3C, 0x3F},	/*# blf_c3    20  2F*/
	{0x3D, 0x2B},	/*# blf_c4    10  0B*/
	{0x3E, 0x18},	/*# blf_c5    08  00*/
	{0x03, 0x09},	/*########################### sc */
	{0x04, 0x03},	/*#    acce_ctrl_0 [1]:acce enable, [0]:histogram enable (00)*/
	{0x6D, 0x04},	/*# ac_ctrl_0 [2]:AE relate mode*/
	{0x49, 0x30},	/*# ce_th      (20)*/
	{0x4A, 0x10},	/*# ce_x0      (40)*/
	{0x4B, 0x40},	/*# ce_slope   (40)*/
	{0xAD, 0x08},	/*#08 # lpf_w1 (08)*/
	{0xAE, 0x10},	/*#10 # lpf_w2 (18)*/
	{0xAF, 0x20},	/*#20 # lpf_w3 (40)*/
	{0xB0, 0x10},	/*#10 # lpf_w4 (18)*/
	{0xB1, 0x08},	/*#08 # lpf_w5 (08)*/
	{0xB2, 0x04},	/*# ac_offset*/
	{0xB3, 0x60},	/*# max_ac_gain0*/
	{0xB4, 0x60},	/*# max_ac_gain1*/
	{0xB5, 0x40},	/*# max_ac_gain2*/
	{0xB7, 0x40},	/*# min_ac_gain*/
	{0xB8, 0x03},	/*# ac_speed*/
	{0xB9, 0x02},	/*# ac_lock*/
	{0xBB, 0x04},	/*# ac_frame*/
	{0x8E, 0x00},	/*# ac_cv_w0 */
	{0x8F, 0x04},	/*# ac_cv_w1 */
	{0x90, 0x06},	/*# ac_cv_w2 */
	{0x91, 0x06},	/*# ac_cv_w3 */
	{0x92, 0x04},	/*# ac_cv_w4 */
	{0x93, 0x03},	/*# ac_cv_w5 */
	{0x94, 0x01},	/*# ac_cv_w6 */
	{0x95, 0x00},	/*# ac_cv_w7 */
	{0x03, 0x00},	/*#################### tune*/
	{REG_DLY, FLIP_H_V},	/*x05,0x00	//# Mir*/
	{0x05, 0x00},   /*# mirror*/

	{0x4A, 0x08},	/*# FlkCtl*/
	{0x54, 0x00},	/*# fd_period_b*/
	{0x55, 0xA8},
	{0x56, 0xD3},
    {0x03, 0x01},	/*# ----------- B*/
    {0x16, 0x04},	/*#Variable frame rate control*/
	{0x03, 0x02},	/*# ----------- C*/
	{0x04, 0xF5},	/*#ISP function control*/
	{0x08, 0x00},

	{0x05, 0xFB},
	{0x09, 0x01},
	{0x0B, 0x82},

	{0x1E, 0x03},   /*#lens gain fitting*/
	{0x1F, 0x03},

	{0x33, 0x32},	/*# ccr*/
	{0x34, 0x87},
	{0x35, 0x8B},
	{0x36, 0x8F},
	{0x37, 0x3F},
	{0x38, 0x8F},
	{0x39, 0x92},
	{0x3A, 0x85},
	{0x3B, 0x37},
	{0x3D, 0x00},	/*# yGm1  #gamma curve fitting*/
	{0x3E, 0x03},
	{0x3F, 0x0C},
	{0x40, 0x19},
	{0x41, 0x26},
	{0x42, 0x3F},
	{0x43, 0x52},
	{0x44, 0x6E},
	{0x45, 0x82},
	{0x46, 0xA1},
	{0x47, 0xB9},
	{0x48, 0xCE},
	{0x49, 0xE0},
	{0x4A, 0xF0},
	{0x4B, 0xFF},

	{0x4C, 0x00},	/*# yGm2  #gamma curve fitting*/
	{0x4D, 0x27},
	{0x4E, 0x36},
	{0x4F, 0x40},
	{0x50, 0x49},
	{0x51, 0x58},
	{0x52, 0x64},
	{0x53, 0x78},
	{0x54, 0x89},
	{0x55, 0xA4},
	{0x56, 0xBB},
	{0x57, 0xCF},
	{0x58, 0xE0},
	{0x59, 0xF1},

	{0x5B, 0x00},	/*# cGm1  #rgb gamma1 curve*/
	{0x5C, 0x03},
	{0x5D, 0x0C},
	{0x5E, 0x19},
	{0x5F, 0x26},
	{0x60, 0x3F},
	{0x61, 0x52},
	{0x62, 0x6E},
	{0x63, 0x82},
	{0x64, 0xA1},
	{0x65, 0xB9},
	{0x66, 0xCE},
	{0x67, 0xE0},
	{0x68, 0xF0},
	{0x69, 0xFF},

	{0x6A, 0x00},  /*#gamma curve fitting*/
	{0x6B, 0x02},
	{0x6C, 0x07},
	{0x6D, 0x0F},
	{0x6E, 0x17},
	{0x6F, 0x25},
	{0x70, 0x31},
	{0x71, 0x46},
	{0x72, 0x58},
	{0x73, 0x79},
	{0x74, 0x97},
	{0x75, 0xB3},
	{0x76, 0xCE},
	{0x77, 0xE7},
	{0x78, 0xFF},

	{0x80, 0x2A},  /*#Color saturation*/
	{0x81, 0x86},
	{0x82, 0x04},
	{0x83, 0x28},

	{0x95, 0x00},	/*# Bri #ybrightness*/
	{0x96, 0x00},

	{0x97, 0x0C},
	{0xB3, 0x00},
	{0xB4, 0x05},
	{0xB5, 0x07},
	{0xB6, 0x84},
	{0xB7, 0x00},
	{0xB8, 0xC8},
	{0xB9, 0x04},
	{0xBA, 0x48},
	{0xBB, 0x01},
	{0xBC, 0x2C},
	{0xBD, 0x06},
	{0xBE, 0x64},
	{0xBF, 0x01},
	{0xC0, 0x7C},
	{0xC1, 0x03},
	{0xC2, 0x5C},
	{0xC7, 0x00},
	{0xC8, 0x05},
	{0xC9, 0x07},
	{0xCA, 0x84},
	{0xCB, 0x00},
	{0xCC, 0x78},
	{0xCD, 0x03},
	{0xCE, 0xD4},
	{0x03, 0x03},	/*# ----------- D*/

	{0x04, 0x02},
	{0x05, 0x00},
	{0x06, 0x08},
	{0x07, 0x00},
	{0x08, 0x00},
	{0x09, 0x00},

	{0x0A, 0x3E},
	{0x0B, 0x5D},
	{0x0C, 0x6C},
	{0x0D, 0x38},	/*# user cs     #Color saturation weight*/
	{0x1A, 0x10},	/*# dpc_p*/
	{0x1B, 0x10},
	{0x1C, 0x50},
	{0x1E, 0x00},	/*# dpc_n*/
	{0x1F, 0x08},
	{0x20, 0x18},
	{0x24, 0x1C},	/*# hf_dir_max*/
	{0x25, 0x1C},
	{0x26, 0x7F},
	{0x28, 0x08},	/* intp_dir_th*/
	{0x29, 0x08},
	{0x2A, 0x7F},

	{0x3F, 0x18},	/*# blf_darkness*/
	{0x40, 0x18},
	{0x41, 0x18},
	{0x48, 0x30},	/*# e_gm_curve*/
	{0x49, 0x2A},
	{0x4A, 0x27},
	{0x4B, 0x2A},
	{0x4C, 0x27},
	{0x4D, 0x24},

	{0x4E, 0x28},
	{0x4F, 0x00},	/*# e_gm*/
	{0x50, 0x00},
	{0x51, 0x00},

	{0x53, 0x10},	/*# lf  edge_gain_lf*/
	{0x54, 0x10},
	{0x55, 0x10},
	{0x57, 0x10},	/*# ghf   #edge_gain_ghf*/
	{0x58, 0x10},
	{0x59, 0x10},
	{0x5B, 0x10},	/*# ehf  #edge_gain_ehf*/
	{0x5C, 0x10},

	{0x5D, 0x10},
	{0x60, 0x04},	/*# ec_pth*/
	{0x61, 0x04},
	{0x62, 0x04},
	{0x64, 0x04},	/*# ec_mth*/
	{0x65, 0x04},
	{0x66, 0x04},
	{0x68, 0x40},	/*# ec_pmax*/
	{0x69, 0x10},
	{0x6A, 0x10},
	{0x6C, 0x40},	/*# ec_mmax*/
	{0x6D, 0x20},
	{0x6E, 0x20},
	{0x70, 0x40},	/*# ec_pgain*/
	{0x71, 0x40},	/*# ec_mgain*/
	{0x72, 0x00},
	{0x73, 0x10},
	{0x74, 0x20},
	{0x7A, 0x40},	/*# y_weight*/
	{0x7B, 0x40},
	{0x7C, 0x48},
	{0x7E, 0x00},	/*# ccr*/
	{0x7F, 0x01},
	{0x80, 0x02},
	{0x82, 0x00},	/*# dc*/
	{0x83, 0x00},
	{0x84, 0x00},
	{0x86, 0x16},	/*# dc_y1 dark_dc_y1*/
	{0x87, 0x16},
	{0x88, 0x18},
	{0x8A, 0xF0},
	{0x93, 0x08},
	{0x94, 0x14},	/*# vec_sample_range*/
	{0x95, 0x1A},
	{0x98, 0x1B},
	{0x99, 0x27},
	{0x9C, 0x35},
	{0x9D, 0x42},
	{0xA0, 0x44},
	{0xA1, 0x62},
	{0xA4, 0x64},
	{0xA5, 0x72},
	{0xA8, 0x76},
	{0xA9, 0x80},
	{0xAC, 0x90},	/*# vec_h&s_a*/
	{0xAD, 0x88},	/*# vec_h&s_b*/
	{0xAE, 0x88},	/*# vec_h&s_c*/
	{0xAF, 0x04},
	{0xB0, 0x04},
	{0xB1, 0x04},
	{0xB2, 0x07},
	{0xB3, 0x01},
	{0xB4, 0x00},
	{0xB5, 0x04},
	{0xB6, 0x00},
	{0xB7, 0x04},
	{0xB8, 0x82},
	{0xB9, 0x05},
	{0xBA, 0x01},
	{0xBB, 0x04},
	{0xBC, 0x04},
	{0xBD, 0x04},
	{0xBE, 0x92},
	{0xBF, 0x8C},
	{0xC0, 0x00},
	{0xC1, 0x04},
	{0xC2, 0x00},
	{0xC3, 0x06},
	{0xC4, 0x85},
	{0xC5, 0x85},
	{0xC6, 0x07},
	{0xC7, 0x04},
	{0xC8, 0x04},
	{0xC9, 0x04},
	{0xCA, 0x86},
	{0xCB, 0x84},
	{0xCC, 0x00},
	{0xCD, 0x04},
	{0xCE, 0x04},
	{0xCF, 0x04},
	{0xE8, 0x40},	/*# slope2  #ycont_slope2*/
	{0xE9, 0x44},
	{0xEA, 0x44},
	{0x03, 0x04},	/*# ----------- E*/

	{0x05, 0x7F},   /*#enable lens/cs fitting*/
	{0x06, 0xA1},
	{0x12, 0x04},	/*# ExpFrmH*/
	{0x13, 0x5E},
	{0x14, 0x04},
	{0x15, 0x5E},
	{0x16, 0x04},
	{0x17, 0x5E},
	{0x1B, 0x00},	/*# Gain*/
	{0x1C, 0x2B},
	{0x1D, 0xAC},
	{0x1E, 0x00},
	{0x1F, 0x2B},
	{0x20, 0xAC},
	{0x2C, 0x66},
	{0x30, 0x08},
	{0x31, 0x08},
	{0x32, 0x10},
	{0x33, 0x10},
	{0x34, 0x28},
	{0x3B, 0x48},	/*# yTg  #Y target control*/
	{0x3C, 0x50},
	{0x3D, 0x44},
	{0x3E, 0x48},
	{0x3F, 0x50},
	{0x40, 0x44},
	{0x41, 0x00},	/*# yt_xref1*/
	{0x42, 0x00},
	{0x43, 0x14},
	{0x44, 0x00},	/*# yt_xref2*/
	{0x45, 0x02},
	{0x46, 0xE8},
	{0x47, 0x00},	/*# yt_xref3*/
	{0x48, 0x45},
	{0x49, 0xE0},
	{0x4A, 0x00},	/*# yt_xref4*/
	{0x4B, 0x8B},
	{0x4C, 0xC0},
	{0x55, 0x04},	/*# ae_speed_u*/
	{0x56, 0x04},	/*# ae_speed_d*/
	{0x57, 0x0C},	/*# ae_lock*/
	{0x5C, 0x00},	/*# awb area*/
	{0x5D, 0x40},
	{0x5E, 0xA0},
	{0x5F, 0x01},
	{0x60, 0x02},
	{0x61, 0x50},
	{0x62, 0x02},
	{0x63, 0x00},
	{0x64, 0x04},
	{0x65, 0x6E},
	{0x66, 0x45},
	{0x67, 0x27},
	{0x68, 0x4F},
	{0x69, 0x64},
	{0x6A, 0xC4},
	{0x6B, 0x0A},
	{0x6C, 0x46},
	{0x6D, 0x32},
	{0x6E, 0x78},
	{0x6F, 0x37},
	{0x70, 0xAF},
	{0x71, 0x32},
	{0x72, 0x23},
	{0x73, 0x78},	/*# rg_ratio_b*/
	{0x74, 0x80},
	{0x75, 0x7C},	/*# rg_ratio_b*/
	{0x76, 0x80},
	{0x77, 0x80},	/*# rg_ratio_c*/
	{0x78, 0x80},	/*# bg_ratio_c*/
	{0x7E, 0x08},	/*# awb_lock*/
	{0x7F, 0x04},	/*# awb_speed*/
	{0x8D, 0xF1},
	{0x8F, 0x00},
	{0x90, 0x78},	/*# group1:yGm,cGm,de-color*/
	{0x91, 0x01},	/*# group1:wWeight*/
	{0x94, 0x00},
	{0x95, 0x00},
	{0x96, 0x00},   /*Darkness Mode Control*/
	{0x98, 0x00},  	/*# xref1*/
	{0x99, 0x03},
	{0x9A, 0x00},
	{0x9B, 0x09},
	{0x9C, 0x00},
	{0x9D, 0x10},
	{0x9E, 0x00},	/*# xref2*/
	{0x9F, 0x02},
	{0xA0, 0x00},
	{0xA1, 0x05},
	{0xA2, 0x00},
	{0xA3, 0x07},
	{0xBA, 0x10},
};

#if	0
/*
 * The white balance settings
 * Here only tune the R G B channel gain.
 * The white balance enalbe bit is modified in sensor_s_autowb and sensor_s_wb
 */
static struct regval_list sensor_wb_manual[] = {

};

static struct regval_list sensor_wb_auto_regs[] = {

};

static struct regval_list sensor_wb_incandescence_regs[] = {

};

static struct regval_list sensor_wb_fluorescent_regs[] = {

};

static struct regval_list sensor_wb_tungsten_regs[] = {

};

static struct regval_list sensor_wb_horizon[] = {

};

static struct regval_list sensor_wb_daylight_regs[] = {

};

static struct regval_list sensor_wb_flash[] = {

};

static struct regval_list sensor_wb_cloud_regs[] = {

};

static struct regval_list sensor_wb_shade[] = {

};

static struct regval_list sensor_brightness_neg4_regs[] = {

};

static struct regval_list sensor_brightness_neg3_regs[] = {

};

static struct regval_list sensor_brightness_neg2_regs[] = {

};

static struct regval_list sensor_brightness_neg1_regs[] = {

};

static struct regval_list sensor_brightness_zero_regs[] = {

};

static struct regval_list sensor_brightness_pos1_regs[] = {

};

static struct regval_list sensor_brightness_pos2_regs[] = {

};

static struct regval_list sensor_brightness_pos3_regs[] = {

};

static struct regval_list sensor_brightness_pos4_regs[] = {

};

static struct cfg_array sensor_brightness[] = {
  {
	.regs = sensor_brightness_neg4_regs,
	.size = ARRAY_SIZE(sensor_brightness_neg4_regs),
  },
  {
	.regs = sensor_brightness_neg3_regs,
	.size = ARRAY_SIZE(sensor_brightness_neg3_regs),
  },
  {
	.regs = sensor_brightness_neg2_regs,
	.size = ARRAY_SIZE(sensor_brightness_neg2_regs),
  },
  {
	.regs = sensor_brightness_neg1_regs,
	.size = ARRAY_SIZE(sensor_brightness_neg1_regs),
  },
  {
	.regs = sensor_brightness_zero_regs,
	.size = ARRAY_SIZE(sensor_brightness_zero_regs),
  },
  {
	.regs = sensor_brightness_pos1_regs,
	.size = ARRAY_SIZE(sensor_brightness_pos1_regs),
  },
  {
	.regs = sensor_brightness_pos2_regs,
	.size = ARRAY_SIZE(sensor_brightness_pos2_regs),
  },
  {
	.regs = sensor_brightness_pos3_regs,
	.size = ARRAY_SIZE(sensor_brightness_pos3_regs),
  },
  {
	.regs = sensor_brightness_pos4_regs,
	.size = ARRAY_SIZE(sensor_brightness_pos4_regs),
  },
};

static struct regval_list sensor_contrast_neg4_regs[] = {

};

static struct regval_list sensor_contrast_neg3_regs[] = {

};

static struct regval_list sensor_contrast_neg2_regs[] = {

};

static struct regval_list sensor_contrast_neg1_regs[] = {

};

static struct regval_list sensor_contrast_zero_regs[] = {

};

static struct regval_list sensor_contrast_pos1_regs[] = {

};

static struct regval_list sensor_contrast_pos2_regs[] = {

};

static struct regval_list sensor_contrast_pos3_regs[] = {

};

static struct regval_list sensor_contrast_pos4_regs[] = {

};

static struct cfg_array sensor_contrast[] = {
  {
	.regs = sensor_contrast_neg4_regs,
	.size = ARRAY_SIZE(sensor_contrast_neg4_regs),
  },
  {
	.regs = sensor_contrast_neg3_regs,
	.size = ARRAY_SIZE(sensor_contrast_neg3_regs),
  },
  {
	.regs = sensor_contrast_neg2_regs,
	.size = ARRAY_SIZE(sensor_contrast_neg2_regs),
  },
  {
	.regs = sensor_contrast_neg1_regs,
	.size = ARRAY_SIZE(sensor_contrast_neg1_regs),
  },
  {
	.regs = sensor_contrast_zero_regs,
	.size = ARRAY_SIZE(sensor_contrast_zero_regs),
  },
  {
	.regs = sensor_contrast_pos1_regs,
	.size = ARRAY_SIZE(sensor_contrast_pos1_regs),
  },
  {
	.regs = sensor_contrast_pos2_regs,
	.size = ARRAY_SIZE(sensor_contrast_pos2_regs),
  },
  {
	.regs = sensor_contrast_pos3_regs,
	.size = ARRAY_SIZE(sensor_contrast_pos3_regs),
  },
  {
	.regs = sensor_contrast_pos4_regs,
	.size = ARRAY_SIZE(sensor_contrast_pos4_regs),
  },
};

static struct regval_list sensor_saturation_neg4_regs[] = {

};

static struct regval_list sensor_saturation_neg3_regs[] = {

};

static struct regval_list sensor_saturation_neg2_regs[] = {

};

static struct regval_list sensor_saturation_neg1_regs[] = {

};

static struct regval_list sensor_saturation_zero_regs[] = {

};

static struct regval_list sensor_saturation_pos1_regs[] = {

};

static struct regval_list sensor_saturation_pos2_regs[] = {

};

static struct regval_list sensor_saturation_pos3_regs[] = {

};

static struct regval_list sensor_saturation_pos4_regs[] = {

};

static struct cfg_array sensor_saturation[] = {
  {
	.regs = sensor_saturation_neg4_regs,
	.size = ARRAY_SIZE(sensor_saturation_neg4_regs),
  },
  {
	.regs = sensor_saturation_neg3_regs,
	.size = ARRAY_SIZE(sensor_saturation_neg3_regs),
  },
  {
	.regs = sensor_saturation_neg2_regs,
	.size = ARRAY_SIZE(sensor_saturation_neg2_regs),
  },
  {
	.regs = sensor_saturation_neg1_regs,
	.size = ARRAY_SIZE(sensor_saturation_neg1_regs),
  },
  {
	.regs = sensor_saturation_zero_regs,
	.size = ARRAY_SIZE(sensor_saturation_zero_regs),
  },
  {
	.regs = sensor_saturation_pos1_regs,
	.size = ARRAY_SIZE(sensor_saturation_pos1_regs),
  },
  {
	.regs = sensor_saturation_pos2_regs,
	.size = ARRAY_SIZE(sensor_saturation_pos2_regs),
  },
  {
	.regs = sensor_saturation_pos3_regs,
	.size = ARRAY_SIZE(sensor_saturation_pos3_regs),
  },
  {
	.regs = sensor_saturation_pos4_regs,
	.size = ARRAY_SIZE(sensor_saturation_pos4_regs),
  },
};

/*
 * The exposure target setttings
 */
static struct regval_list sensor_ev_neg4_regs[] = {

};

static struct regval_list sensor_ev_neg3_regs[] = {

};

static struct regval_list sensor_ev_neg2_regs[] = {

};

static struct regval_list sensor_ev_neg1_regs[] = {

};

static struct regval_list sensor_ev_zero_regs[] = {

};

static struct regval_list sensor_ev_pos1_regs[] = {

};

static struct regval_list sensor_ev_pos2_regs[] = {

};

static struct regval_list sensor_ev_pos3_regs[] = {

};

static struct regval_list sensor_ev_pos4_regs[] = {

};

static struct cfg_array sensor_ev[] = {
  {
	.regs = sensor_ev_neg4_regs,
	.size = ARRAY_SIZE(sensor_ev_neg4_regs),
  },
  {
	.regs = sensor_ev_neg3_regs,
	.size = ARRAY_SIZE(sensor_ev_neg3_regs),
  },
  {
	.regs = sensor_ev_neg2_regs,
	.size = ARRAY_SIZE(sensor_ev_neg2_regs),
  },
  {
	.regs = sensor_ev_neg1_regs,
	.size = ARRAY_SIZE(sensor_ev_neg1_regs),
  },
  {
	.regs = sensor_ev_zero_regs,
	.size = ARRAY_SIZE(sensor_ev_zero_regs),
  },
  {
	.regs = sensor_ev_pos1_regs,
	.size = ARRAY_SIZE(sensor_ev_pos1_regs),
  },
  {
	.regs = sensor_ev_pos2_regs,
	.size = ARRAY_SIZE(sensor_ev_pos2_regs),
  },
  {
	.regs = sensor_ev_pos3_regs,
	.size = ARRAY_SIZE(sensor_ev_pos3_regs),
  },
  {
	.regs = sensor_ev_pos4_regs,
	.size = ARRAY_SIZE(sensor_ev_pos4_regs),
  },
};
#endif

/*
 * Here we'll try to encapsulate the changes for just the output
 * video format.
 *
 */
static struct regval_list sensor_fmt_yuv422_yuyv[] = {

};

static struct regval_list sensor_fmt_yuv422_yvyu[] = {

};

static struct regval_list sensor_fmt_yuv422_vyuy[] = {

};

static struct regval_list sensor_fmt_yuv422_uyvy[] = {

};

static int sensor_read(struct v4l2_subdev *sd, unsigned char reg,
	unsigned char *value)
{
	int ret = 0;
	int cnt = 0;

	ret = cci_read_a8_d8(sd, reg, value);
	while (ret != 0 && cnt < 8) {
		ret = cci_read_a8_d8(sd, reg, value);
		cnt++;
	}
	if (cnt > 0) {
		vfe_dev_print("sensor read retry=%d,reg=0x%x\n", cnt, reg);
	}

	return ret;
}

static int sensor_write(struct v4l2_subdev *sd, unsigned char reg,
    unsigned char value)
{
	int ret = 0;
	int cnt = 0;

	ret = cci_write_a8_d8(sd, reg, value);
	while (ret != 0 && cnt < 8) {
		ret = cci_write_a8_d8(sd, reg, value);
		cnt++;
	}
	if (cnt > 0) {
		vfe_dev_err("sensor write retry=[%d]\n", cnt);
	}

	return ret;
}

static int sensor_write_array(struct v4l2_subdev *sd, struct regval_list *regs, int array_size)
{
	int i = 0;

	if (!regs)
		return -EINVAL;

	while (i < array_size) {
		if (regs->addr == REG_DLY)
			msleep(regs->data);
		else {
		/*printk("write 0x%x=0x%x\n", regs->addr, regs->data);*/
		LOG_ERR_RET(sensor_write(sd, regs->addr, regs->data))
	}
	i++;
	regs++;
	}

	return 0;
}

/* stuff about exposure when capturing image and video*/
static int sensor_g_hflip(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	unsigned char rdval;

	LOG_ERR_RET(sensor_write(sd, 0x03, 0x00)); /*bank A*/
	LOG_ERR_RET(sensor_read(sd, 0x05, &rdval))

	rdval &= 1;

	*value = rdval;
	info->hflip = *value;

	return 0;
}

static int sensor_s_hflip(struct v4l2_subdev *sd, int value)
{
	struct sensor_info *info = to_state(sd);
	unsigned char rdval;

	if (info->hflip == value)
		return 0;
	LOG_ERR_RET(sensor_write(sd, 0x03, 0x00)); /*bank A*/
	LOG_ERR_RET(sensor_read(sd, 0x05, &rdval))

	switch (value) {
	case 0:
		rdval &= 0xfe;
		break;
	case 1:
		rdval |= 0x01;
		break;
	default:
		vfe_dev_err("warning sensor_s_hflip\n");
		return -EINVAL;
	}

	info->hflip = value;

	return 0;
}

static int sensor_g_vflip(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	unsigned char rdval;

	LOG_ERR_RET(sensor_write(sd, 0x03, 0x00)); /*bank A*/
	LOG_ERR_RET(sensor_read(sd, 0x05, &rdval))

	rdval &= (1<<1);
	rdval >>= 1;

	*value = rdval;
	info->vflip = *value;

	return 0;
}

static int sensor_s_vflip(struct v4l2_subdev *sd, int value)
{
	struct sensor_info *info = to_state(sd);
	unsigned char rdval;

	if (info->vflip == value)
		return 0;

	LOG_ERR_RET(sensor_write(sd, 0x03, 0x00)); /*bank A*/
	LOG_ERR_RET(sensor_read(sd, 0x05, &rdval))

	switch (value) {
	case 0:
		rdval &= 0xfd;
		break;
	case 1:
		rdval |= 0x02;
		break;
	default:
		vfe_dev_err("warning sensor_s_vflip\n");
		return -EINVAL;
	}

	info->vflip = value;

	return 0;
}

static int sensor_g_autogain(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}

static int sensor_s_autogain(struct v4l2_subdev *sd, int value)
{
	return 0;
}

static int sensor_g_autoexp(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}

static int sensor_s_autoexp(struct v4l2_subdev *sd,
    enum v4l2_exposure_auto_type value)
{
	return 0;
}

static int sensor_g_autowb(struct v4l2_subdev *sd, int *value)
{
	return 0;
}

static int sensor_s_autowb(struct v4l2_subdev *sd, int value)
{
	return 0;
}

static int sensor_g_hue(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}

static int sensor_s_hue(struct v4l2_subdev *sd, int value)
{
	return 0;
}

static int sensor_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}

static int sensor_s_gain(struct v4l2_subdev *sd, int value)
{
	return 0;
}

/* *********************************************end of ******************************************** */

static int sensor_g_brightness(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);

	*value = info->brightness;

	return 0;
}

static int sensor_s_brightness(struct v4l2_subdev *sd, int value)
{
	struct sensor_info *info = to_state(sd);

	if (info->brightness == value)
		return 0;

	if (value < -4 || value > 4)
		return -ERANGE;

	info->brightness = value;

	return 0;
}

static int sensor_g_contrast(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);

	*value = info->contrast;

	return 0;
}

static int sensor_s_contrast(struct v4l2_subdev *sd, int value)
{
	struct sensor_info *info = to_state(sd);

	if (info->contrast == value)
		return 0;

	if (value < -4 || value > 4)
		return -ERANGE;

	info->contrast = value;

	return 0;
}

static int sensor_g_saturation(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);

	*value = info->saturation;

	return 0;
}

static int sensor_s_saturation(struct v4l2_subdev *sd, int value)
{
	struct sensor_info *info = to_state(sd);

	if (info->saturation == value)
		return 0;

	if (value < -4 || value > 4)
		return -ERANGE;

	info->saturation = value;

	return 0;
}

static int sensor_g_exp_bias(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);

	*value = info->exp_bias;
	return 0;
}

static int sensor_s_exp_bias(struct v4l2_subdev *sd, int value)
{
	struct sensor_info *info = to_state(sd);

	if (info->exp_bias == value)
		return 0;

	if (value < -4 || value > 4)
		return -ERANGE;

	info->exp_bias = value;

	return 0;
}

static int sensor_g_wb(struct v4l2_subdev *sd, int *value)
{
	struct sensor_info *info = to_state(sd);
	enum v4l2_auto_n_preset_white_balance *wb_type = (enum v4l2_auto_n_preset_white_balance *)value;

	*wb_type = info->wb;

	return 0;
}

static int sensor_s_wb(struct v4l2_subdev *sd,
    enum v4l2_auto_n_preset_white_balance value)
{
	struct sensor_info *info = to_state(sd);

	if (info->capture_mode == V4L2_MODE_IMAGE)
		return 0;

	if (info->wb == value)
		return 0;

	if (value == V4L2_WHITE_BALANCE_AUTO)
		info->autowb = 1;
	else
		info->autowb = 0;

	info->wb = value;

	return 0;
}

static int sensor_g_flash_mode(struct v4l2_subdev *sd,
    __s32 *value)
{
	return 0;
}

static int sensor_s_flash_mode(struct v4l2_subdev *sd,
    enum v4l2_flash_led_mode value)
{
	return 0;
}

static int sensor_power(struct v4l2_subdev *sd, int on)
{
	int ret;
	ret = 0;

	switch (on) {
	case CSI_SUBDEV_STBY_ON:
		vfe_dev_dbg("CSI_SUBDEV_STBY_ON!\n");
		cci_lock(sd);
		vfe_set_mclk(sd, OFF);
		usleep_range(10000, 12000);
		cci_unlock(sd);
		break;
	case CSI_SUBDEV_STBY_OFF:
		vfe_dev_dbg("CSI_SUBDEV_STBY_OFF!\n");
		cci_lock(sd);
		/*active mclk before stadby out*/
		vfe_set_mclk_freq(sd, MCLK);
		vfe_set_mclk(sd, ON);
		usleep_range(10000, 12000);
		cci_unlock(sd);
		break;
	case CSI_SUBDEV_PWR_ON:
		vfe_dev_dbg("CSI_SUBDEV_PWR_ON!\n");
		cci_lock(sd);
		vfe_gpio_set_status(sd, PWDN, 1);
		vfe_gpio_set_status(sd, RESET, 1);
		vfe_gpio_set_status(sd, POWER_EN, 1);
		vfe_gpio_write(sd, RESET, 0);
		usleep_range(10000, 12000);
		vfe_gpio_write(sd, POWER_EN, CSI_GPIO_HIGH);
		usleep_range(10000, 12000);

		vfe_set_mclk_freq(sd, MCLK);
		vfe_set_mclk(sd, ON);
		usleep_range(10000, 12000);

		vfe_gpio_write(sd, RESET, 1);
		usleep_range(10000, 12000);
		cci_unlock(sd);
		break;
	case CSI_SUBDEV_PWR_OFF:
		vfe_dev_dbg("CSI_SUBDEV_PWR_OFF!\n");
		cci_lock(sd);
		vfe_gpio_set_status(sd, RESET, 1);
		vfe_gpio_write(sd, RESET, CSI_GPIO_LOW);
		vfe_set_mclk(sd, OFF);
		/*power supply off*/
		vfe_gpio_write(sd, POWER_EN, CSI_GPIO_LOW);
		usleep_range(10000, 12000);
		/*set the io to hi-z*/
		vfe_gpio_set_status(sd, RESET, 0);/*set the gpio to input*/
		vfe_gpio_set_status(sd, POWER_EN, 0);
		cci_unlock(sd);
		break;
	default:
	return -EINVAL;
  }

	return 0;
}

static int sensor_reset(struct v4l2_subdev *sd, u32 val)
{
	switch (val) {
	case 0:
		vfe_gpio_write(sd, RESET, CSI_RST_OFF);
		usleep_range(10000, 12000);
		break;
	case 1:
		vfe_gpio_write(sd, RESET, CSI_RST_ON);
		usleep_range(10000, 12000);
		break;
	default:
		return -EINVAL;
  }

	return 0;
}

static int sensor_detect(struct v4l2_subdev *sd)
{
	unsigned char rdval;

	msleep(30);
	sensor_read(sd, 0x00, &rdval);

	if (rdval != 0x22) {
		vfe_dev_err("%s detect warning,chid should be 0x22 \n", __FUNCTION__);
		msleep(10);
		sensor_read(sd, 0x00, &rdval);
		if (rdval == 0x22) {
			vfe_dev_print("%s detect ok\n", __FUNCTION__);
		}
		return 0;
	} else
		vfe_dev_print("%s detect ok\n", __FUNCTION__);
	msleep(10);

	return 0;
}

static int sensor_init(struct v4l2_subdev *sd, u32 val)
{
	int ret;
	struct sensor_info *info = to_state(sd);

	ret = sensor_detect(sd);
	if (ret) {
		vfe_dev_err("chip found is not an target chip.\n");
		return ret;
	}

	info->focus_status = 0;
	info->low_speed = 0;
	info->width = 0;
	info->height = 0;
	info->brightness = 0;
	info->contrast = 0;
	info->saturation = 0;
	info->hue = 0;
	info->hflip = 0;
	info->vflip = 0;
	info->gain = 0;
	info->autogain = 1;
	info->exp_bias = 0;
	info->autoexp = 1;
	info->autowb = 1;
	info->wb = V4L2_WHITE_BALANCE_AUTO;
	info->clrfx = V4L2_COLORFX_NONE;
	info->band_filter = V4L2_CID_POWER_LINE_FREQUENCY_50HZ;
	info->tpf.numerator = 1;
	info->tpf.denominator = 30;/* 30fps */
	g_denominator = info->tpf.denominator;

	ret = sensor_write_array(sd, sensor_default_regs, ARRAY_SIZE(sensor_default_regs));
	if (ret < 0) {
		vfe_dev_err("write sensor_default_regs error\n");
		return ret;
	}

	if (info->stby_mode == 0)
		info->init_first_flag = 0;
	info->preview_first_flag = 1;

	return 0;
}

static long sensor_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	int ret = 0;
	struct sensor_info *info = to_state(sd);

	switch (cmd) {
	case GET_CURRENT_WIN_CFG:
      if (info->current_wins != NULL) {
		memcpy(arg, info->current_wins, sizeof(struct sensor_win_size));
		ret = 0;
      } else {
		vfe_dev_err("empty wins!\n");
		ret = -1;
      }
		break;
	default:
		vfe_dev_err("empty window!\n");
		return -EINVAL;
	}

	return ret;
}

/*
 * Store information about the video data format.
 */
static struct sensor_format_struct {
	__u8 *desc;
	enum v4l2_mbus_pixelcode mbus_code;
	struct regval_list *regs;
	int	regs_size;
	int bpp;/* Bytes per pixel */
} sensor_formats[] = {
	{
		.desc		= "YUYV 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_YUYV8_2X8,
		.regs 		= sensor_fmt_yuv422_yuyv,
		.regs_size = ARRAY_SIZE(sensor_fmt_yuv422_yuyv),
		.bpp		= 2,
	},
	{
		.desc		= "YVYU 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_YVYU8_2X8,
		.regs 		= sensor_fmt_yuv422_yvyu,
		.regs_size = ARRAY_SIZE(sensor_fmt_yuv422_yvyu),
		.bpp		= 2,
	},
	{
		.desc		= "VYUY 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_VYUY8_2X8,
		.regs 		= sensor_fmt_yuv422_vyuy,
		.regs_size = ARRAY_SIZE(sensor_fmt_yuv422_vyuy),
		.bpp		= 2,
	},
	{
		.desc		= "UYVY 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_UYVY8_2X8,
		.regs 		= sensor_fmt_yuv422_uyvy,
		.regs_size = ARRAY_SIZE(sensor_fmt_yuv422_uyvy),
		.bpp		= 2,
	},
};

#define N_FMTS ARRAY_SIZE(sensor_formats)

#ifdef	USE_CSI_ISP_FUNCTION
/*if use USE_CSI_ISP_FUNCTION csi0_dev0_isp_used must set 1 */
/* 30%  ,¼´ÉÏÏÂ×óÓÒ¸÷ÇÐ 15% */
static struct sensor_win_size sensor_win_sizes[] = {
 {
    .width      = 1920,/*1344,*/
    .height     = 1080,/*756,*/
    .hoffset    = 0,/*288,*/
    .voffset    = 0,/*162,*/
    .regs       = sensor_15fps_regs,
    .regs_size  = ARRAY_SIZE(sensor_15fps_regs),
    .set_size   = NULL,
  },
  {
    .width      = 1280,
    .height     = 720,
	.hoffset	= 288,
	.voffset	= 162,
    .width_input      = 1344,
    .height_input     = 756,
    .regs       = sensor_default_regs,
    .regs_size  = ARRAY_SIZE(sensor_default_regs), /*30fps*/
    .set_size   = NULL,
  },
  {
    .width      = 640,
    .height     = 480,
    .hoffset    = 240,
    .voffset    = 0,
    .width_input      = 1440,
    .height_input     = 1080,
    .regs       = sensor_default_regs,
    .regs_size  = ARRAY_SIZE(sensor_default_regs),
    .set_size   = NULL,
  },
  {
    .width      = 640,
    .height     = 360,
	.hoffset	= 288,
	.voffset	= 162,
    .width_input      = 1344,
    .height_input     = 756,
    .regs       = sensor_default_regs,
    .regs_size  = ARRAY_SIZE(sensor_default_regs),
    .set_size   = NULL,
  },
};


#define N_WIN_SIZES (ARRAY_SIZE(sensor_win_sizes))

static struct sensor_win_size sensor_win_sizes_15fps[] = {

  {
    .width      = 1920,
    .height     = 1080,
    .hoffset    = 0,
    .voffset    = 0,
    .regs       = sensor_15fps_regs,
    .regs_size  = ARRAY_SIZE(sensor_15fps_regs),
    .set_size   = NULL,
  },
  {
    .width      = 1280,
    .height     = 720,
    .hoffset    = 288,
    .voffset    = 162,
    .width_input      = 1344,
    .height_input     = 756,
    .regs       = sensor_15fps_regs,
    .regs_size  = ARRAY_SIZE(sensor_15fps_regs),
    .set_size   = NULL,
  },
  {
    .width      = 640,
    .height     = 480,
    .hoffset    = 240,
    .voffset    = 0,
    .width_input      = 1440,
    .height_input     = 1080,
    .regs       = sensor_15fps_regs,
    .regs_size  = ARRAY_SIZE(sensor_15fps_regs),
    .set_size   = NULL,
   },
  {
    .width      = 640,
    .height     = 360,
    .hoffset    = 288,
    .voffset    = 162,
    .width_input      = 1344,
    .height_input     = 756,
    .regs       = sensor_15fps_regs,
    .regs_size  = ARRAY_SIZE(sensor_15fps_regs),
    .set_size   = NULL,
  },
};

#else /* use sensor crop function and csi0_dev0_isp_used must set 0*/

static struct sensor_win_size sensor_win_sizes[] = {

  {
    .width      = 1920,
    .height     = 1080,
    .hoffset    = 0,
    .voffset    = 0,
    .regs       = sensor_15fps_regs,
    .regs_size  = ARRAY_SIZE(sensor_15fps_regs),
    .set_size   = NULL,
  },
  {
    .width      = 1280,
    .height     = 720,
    .hoffset    = 0,
    .voffset    = 0,
    .regs       = sensor_default_regs,
    .regs_size  = ARRAY_SIZE(sensor_default_regs),
    .set_size   = NULL,
  },
  {
    .width      = 640,
    .height     = 480,
    .hoffset    = 240,
    .voffset    = 0,
    .regs       = sensor_default_regs,
    .regs_size  = ARRAY_SIZE(sensor_default_regs),
    .set_size   = NULL,
  },
    {
    .width      = 640,
    .height     = 360,
    .hoffset    = 0,
    .voffset    = 0,
    .regs       = sensor_default_regs,
    .regs_size  = ARRAY_SIZE(sensor_default_regs),
    .set_size   = NULL,
  },
};

#define N_WIN_SIZES (ARRAY_SIZE(sensor_win_sizes))

static struct sensor_win_size sensor_win_sizes_15fps[] = {
  {
    .width      = 1920,
    .height     = 1080,
    .hoffset    = 0,
    .voffset    = 0,
    .regs       = sensor_15fps_regs,
    .regs_size  = ARRAY_SIZE(sensor_15fps_regs),
    .set_size   = NULL,
  },
  {
    .width      = 1280,
    .height     = 720,
    .hoffset    = 0,
    .voffset    = 0,
    .regs       = sensor_15fps_regs,
    .regs_size  = ARRAY_SIZE(sensor_15fps_regs),
    .set_size   = NULL,
  },
  {
    .width      = 640,
    .height     = 480,
    .hoffset    = 240,
    .voffset    = 0,
    .regs       = sensor_15fps_regs,
    .regs_size  = ARRAY_SIZE(sensor_15fps_regs),
    .set_size   = NULL,
  },
  {
    .width      = 640,
    .height     = 360,
    .hoffset    = 0,
    .voffset    = 0,
    .regs       = sensor_15fps_regs,
    .regs_size  = ARRAY_SIZE(sensor_15fps_regs),
    .set_size   = NULL,
  },
};


#endif

static int sensor_enum_fmt(struct v4l2_subdev *sd, unsigned index,
		enum v4l2_mbus_pixelcode *code)
{
	if (index >= N_FMTS)
		return -EINVAL;

	*code = sensor_formats[index].mbus_code;
	return 0;
}

static int sensor_enum_size(struct v4l2_subdev *sd,
		struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index > N_WIN_SIZES-1)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	if (g_denominator == 30) {
		fsize->discrete.width = sensor_win_sizes[fsize->index].width;
		fsize->discrete.height = sensor_win_sizes[fsize->index].height;
	} else {
		fsize->discrete.width = sensor_win_sizes_15fps[fsize->index].width;
		fsize->discrete.height = sensor_win_sizes_15fps[fsize->index].height;
	}

	return 0;
}

static int sensor_try_fmt_internal(struct v4l2_subdev *sd,
    struct v4l2_mbus_framefmt *fmt,
    struct sensor_format_struct **ret_fmt,
    struct sensor_win_size **ret_wsize)
{
	int index;
	struct sensor_win_size *wsize;
	struct sensor_info *info = to_state(sd);

	for (index = 0; index < N_FMTS; index++) {
		if (sensor_formats[index].mbus_code == fmt->code)
			break;
	}

	if (index >= N_FMTS)
		return -EINVAL;

	if (ret_fmt != NULL)
		*ret_fmt = sensor_formats + index;
	fmt->field = V4L2_FIELD_NONE;

	if (SENSOR_FRAME_RATE_15FPS == info->tpf.denominator) {
	  /*
	   * Round requested image size down to the nearest
	   * we support, but not below the smallest.
	   */

	 for (wsize = sensor_win_sizes_15fps; wsize < sensor_win_sizes_15fps + N_WIN_SIZES;
	       wsize++)
		if (fmt->width >= wsize->width && fmt->height >= wsize->height)
			break;

		if (wsize >= sensor_win_sizes_15fps + N_WIN_SIZES)
			wsize--;/* Take the smallest one */
	} else {
	  for (wsize = sensor_win_sizes; wsize < sensor_win_sizes + N_WIN_SIZES;
	       wsize++)
		if (fmt->width >= wsize->width && fmt->height >= wsize->height)
			break;

		if (wsize >= sensor_win_sizes + N_WIN_SIZES)
			wsize--;
	}
	if (ret_wsize != NULL)
		*ret_wsize = wsize;

	fmt->width = wsize->width;
	fmt->height = wsize->height;
    info->current_wins = wsize;
	g_denominator = info->tpf.denominator;

	return 0;
}

static int sensor_try_fmt(struct v4l2_subdev *sd,
		struct v4l2_mbus_framefmt *fmt)
{
	return sensor_try_fmt_internal(sd, fmt, NULL, NULL);
}

static int sensor_g_mbus_config(struct v4l2_subdev *sd,
			struct v4l2_mbus_config *cfg)
{
	cfg->type = V4L2_MBUS_PARALLEL;
	cfg->flags = V4L2_MBUS_MASTER | VREF_POL | HREF_POL | CLK_POL;

	return 0;
}

static int sensor_s_fmt(struct v4l2_subdev *sd,
			struct v4l2_mbus_framefmt *fmt)
{
	int ret;
	struct sensor_format_struct *sensor_fmt;
	struct sensor_win_size *wsize;
	struct sensor_info *info = to_state(sd);

	vfe_dev_dbg("sensor_s_fmt\n");

	ret = sensor_try_fmt_internal(sd, fmt, &sensor_fmt, &wsize);
	if (ret)
		return ret;

	if (info->capture_mode == V4L2_MODE_VIDEO) {
		/*video */
	} else if (info->capture_mode == V4L2_MODE_IMAGE) {
		/*image */
	}

  sensor_write_array(sd, sensor_fmt->regs, sensor_fmt->regs_size);

  if (wsize->regs)
    LOG_ERR_RET(sensor_write_array(sd, wsize->regs, wsize->regs_size))

	if (wsize->set_size)
		LOG_ERR_RET(wsize->set_size(sd))

	sensor_s_hflip(sd, info->hflip);
	sensor_s_vflip(sd, info->vflip);

	info->fmt = sensor_fmt;
	info->width = wsize->width;
	info->height = wsize->height;
	if (info->capture_mode == V4L2_MODE_IMAGE) {
		vfe_dev_print("s_fmt image width = %d, height = %d\n", wsize->width,
		      wsize->height);
		return 0;
	}

	msleep(500);
	vfe_dev_print("s_fmt set width = %d, height = %d\n", wsize->width,
		      wsize->height);

	return 0;
}

/*
 * Implement G/S_PARM.  There is a "high quality" mode we could try
 * to do someday; for now, we just do the frame rate tweak.
 */
static int sensor_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	struct sensor_info *info = to_state(sd);

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(cp, 0, sizeof(struct v4l2_captureparm));
	cp->capability = V4L2_CAP_TIMEPERFRAME;
	cp->capturemode = info->capture_mode;

	cp->timeperframe.numerator = info->tpf.numerator;
	cp->timeperframe.denominator = info->tpf.denominator;

	return 0;
}

static int sensor_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	struct v4l2_fract *tpf = &cp->timeperframe;
	struct sensor_info *info = to_state(sd);

	vfe_dev_dbg("sensor_s_parm\n");

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	if (info->tpf.numerator == 0) {
		vfe_dev_err("sensor_s_parm numerator error \n");
		return -EINVAL;
	}

	info->capture_mode = cp->capturemode;
	info->tpf.denominator = cp->timeperframe.denominator;

	if (info->capture_mode == V4L2_MODE_IMAGE) {
		vfe_dev_dbg("capture mode is not video mode,can not set frame rate!\n");
		return 0;
	}

	if (tpf->numerator == 0 || tpf->denominator == 0) {
		tpf->numerator = 1;
		tpf->denominator = SENSOR_FRAME_RATE;/* Reset to full rate */
		vfe_dev_err("sensor frame rate reset to full rate!\n");
	}
	vfe_dev_dbg("set frame rate %d\n", tpf->denominator/tpf->numerator);

	if (info->tpf.denominator <= 0 || info->tpf.denominator > 30)
		info->tpf.denominator = 30;

	g_denominator = info->tpf.denominator;

   return 0;
}

/*
 * Code for dealing with controls.
 * fill with different sensor module
 * different sensor module has different settings here
 * if not support the follow function ,retrun -EINVAL
 */

/* *********************************************begin of ******************************************** */
static int sensor_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	/* Fill in min, max, step and default value for these controls. */
	/* see include/linux/videodev2.h for details */

	switch (qc->id) {
	case V4L2_CID_VFLIP:
	case V4L2_CID_HFLIP:
		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 0);

	case V4L2_CID_EXPOSURE:
	case V4L2_CID_AUTO_EXPOSURE_BIAS:
		return v4l2_ctrl_query_fill(qc, -4, 4, 1, 0);
	case V4L2_CID_EXPOSURE_AUTO:
		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 0);
	case V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE:
		return v4l2_ctrl_query_fill(qc, 0, 9, 1, 1);
	case V4L2_CID_AUTO_WHITE_BALANCE:
		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 1);
	case V4L2_CID_COLORFX:
		return v4l2_ctrl_query_fill(qc, 0, 15, 1, 0);
	case V4L2_CID_FLASH_LED_MODE:
		return v4l2_ctrl_query_fill(qc, 0, 4, 1, 0);
	}
	return -EINVAL;
}

static int sensor_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	vfe_dev_print("sensor_g_ctrl ctrl->id=0x%8x\n", ctrl->id);
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return sensor_g_brightness(sd, &ctrl->value);
	case V4L2_CID_CONTRAST:
		return sensor_g_contrast(sd, &ctrl->value);
	case V4L2_CID_SATURATION:
		return sensor_g_saturation(sd, &ctrl->value);
	case V4L2_CID_HUE:
		return sensor_g_hue(sd, &ctrl->value);
	case V4L2_CID_VFLIP:
		return sensor_g_vflip(sd, &ctrl->value);
	case V4L2_CID_HFLIP:
		return sensor_g_hflip(sd, &ctrl->value);
	case V4L2_CID_GAIN:
		return sensor_g_gain(sd, &ctrl->value);
	case V4L2_CID_AUTOGAIN:
		return sensor_g_autogain(sd, &ctrl->value);
	case V4L2_CID_EXPOSURE:
	case V4L2_CID_AUTO_EXPOSURE_BIAS:
		return sensor_g_exp_bias(sd, &ctrl->value);
	case V4L2_CID_EXPOSURE_AUTO:
		return sensor_g_autoexp(sd, &ctrl->value);
	case V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE:
		return sensor_g_wb(sd, &ctrl->value);
	case V4L2_CID_AUTO_WHITE_BALANCE:
		return sensor_g_autowb(sd, &ctrl->value);
	case V4L2_CID_FLASH_LED_MODE:
		return sensor_g_flash_mode(sd, &ctrl->value);
	default:
		return 0;
	}

	return -EINVAL;
}

static int sensor_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct v4l2_queryctrl qc;
	int ret;

	vfe_dev_print("sensor_s_ctrl ctrl->id=0x%8x\n", ctrl->id);
	qc.id = ctrl->id;
	ret = sensor_queryctrl(sd, &qc);
	if (ret < 0)
		return ret;

	if (qc.type == V4L2_CTRL_TYPE_MENU ||
		qc.type == V4L2_CTRL_TYPE_INTEGER ||
		qc.type == V4L2_CTRL_TYPE_BOOLEAN) {
	  if (ctrl->value < qc.minimum || ctrl->value > qc.maximum)
	    return -ERANGE;
	}

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return sensor_s_brightness(sd, ctrl->value);
	case V4L2_CID_CONTRAST:
		return sensor_s_contrast(sd, ctrl->value);
	case V4L2_CID_SATURATION:
		return sensor_s_saturation(sd, ctrl->value);
	case V4L2_CID_HUE:
		return sensor_s_hue(sd, ctrl->value);
	case V4L2_CID_VFLIP:
		return sensor_s_vflip(sd, ctrl->value);
	case V4L2_CID_HFLIP:
		return sensor_s_hflip(sd, ctrl->value);
	case V4L2_CID_GAIN:
		return sensor_s_gain(sd, ctrl->value);
	case V4L2_CID_AUTOGAIN:
		return sensor_s_autogain(sd, ctrl->value);
	case V4L2_CID_EXPOSURE:
	case V4L2_CID_AUTO_EXPOSURE_BIAS:
		return sensor_s_exp_bias(sd, ctrl->value);
	case V4L2_CID_EXPOSURE_AUTO:
		return sensor_s_autoexp(sd, (enum v4l2_exposure_auto_type) ctrl->value);
	case V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE:
		return sensor_s_wb(sd, (enum v4l2_auto_n_preset_white_balance) ctrl->value);
	case V4L2_CID_AUTO_WHITE_BALANCE:
		return sensor_s_autowb(sd, ctrl->value);
	case V4L2_CID_FLASH_LED_MODE:
		return sensor_s_flash_mode(sd, (enum v4l2_flash_led_mode) ctrl->value);
	default:
		return 0;
	}

	return -EINVAL;
}

static int sensor_g_chip_ident(struct v4l2_subdev *sd,
			       struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_SENSOR, 0);
}

/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops sensor_core_ops = {
	.g_chip_ident = sensor_g_chip_ident,
	.g_ctrl = sensor_g_ctrl,
	.s_ctrl = sensor_s_ctrl,
	.queryctrl = sensor_queryctrl,
	.reset = sensor_reset,
	.init = sensor_init,
	.s_power = sensor_power,
	.ioctl = sensor_ioctl,
};

static const struct v4l2_subdev_video_ops sensor_video_ops = {
	.enum_mbus_fmt = sensor_enum_fmt,
	.enum_framesizes = sensor_enum_size,
	.try_mbus_fmt = sensor_try_fmt,
	.s_mbus_fmt = sensor_s_fmt,
	.s_parm = sensor_s_parm,
	.g_parm = sensor_g_parm,
	.g_mbus_config = sensor_g_mbus_config,
};

static const struct v4l2_subdev_ops sensor_ops = {
	.core = &sensor_core_ops,
	.video = &sensor_video_ops,
};

/* ----------------------------------------------------------------------- */
static struct cci_driver cci_drv = {
	.name = SENSOR_NAME,
	.addr_width = CCI_BITS_8,
	.data_width = CCI_BITS_8,
};

static int sensor_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	struct sensor_info *info;
	info = kzalloc(sizeof(struct sensor_info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
	sd = &info->sd;
	glb_sd = sd;
	cci_dev_probe_helper(sd, client, &sensor_ops, &cci_drv);

	info->fmt = &sensor_formats[0];
	info->af_first_flag = 1;
	info->init_first_flag = 1;
	info->auto_focus = 0;
	return 0;
}

static int sensor_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd;
	sd = cci_dev_remove_helper(client, &cci_drv);
	kfree(to_state(sd));
	return 0;
}

static const struct i2c_device_id sensor_id[] = {
	{SENSOR_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct i2c_driver sensor_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = SENSOR_NAME,
		   },
	.probe = sensor_probe,
	.remove = sensor_remove,
	.id_table = sensor_id,
};

static __init int init_sensor(void)
{
	return cci_dev_init_helper(&sensor_driver);
}

static __exit void exit_sensor(void)
{
	cci_dev_exit_helper(&sensor_driver);
}

module_init(init_sensor);
module_exit(exit_sensor);
