/* Copyright Statement:
*
* This software/firmware and related documentation ("MediaTek Software") are
* protected under relevant copyright laws. The information contained herein
* is confidential and proprietary to MediaTek Inc. and/or its licensors.
* Without the prior written permission of MediaTek inc. and/or its licensors,
* any reproduction, modification, use or disclosure of MediaTek Software,
* and information contained herein, in whole or in part, shall be strictly prohibited.
*/
/* MediaTek Inc. (C) 2015. All rights reserved.
*
* BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
* THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
* RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
* AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
* NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
* SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
* SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
* THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
* THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
* CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
* SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
* STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
* CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
* AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
* OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
* MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
* The following software/firmware and/or related documentation ("MediaTek Software")
* have been modified by MediaTek Inc. All revisions are subject to any receiver\'s
* applicable license agreements with MediaTek Inc.
*/

#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#else
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <asm-generic/gpio.h>

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#endif
#endif

#include "lcm_drv.h"
#include "lcm_driver_gpio.h"
/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */
#define FRAME_WIDTH  (1024)
#define FRAME_HEIGHT (600)

#define GPIO_OUT_ONE  1
#define GPIO_OUT_ZERO 0


/* --------------------------------------------------------------------------- */
/* Local Variables */
/* --------------------------------------------------------------------------- */
static LCM_UTIL_FUNCS lcm_util = { 0 };

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))
#define REGFLAG_END_OF_TABLE    0xFFF   // END OF REGISTERS MARKER
#define REGFLAG_DELAY       0XFFE
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	  lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define LCM_DSI_CMD_MODE  0
#define dsi_set_cmdq(pdata, queue_size, force_update) \
		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)

/* --------------------------------------------------------------------------- */
/* Local Functions */
/* --------------------------------------------------------------------------- */
struct LCM_setting_table
{
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

#if 0
static struct LCM_setting_table lcm_initialization_setting[] =
	{
	{0x89,01,{0xD5}},
	{0xa8,01,{0x08}},
	{0x11,01,{0x00}},
	{REGFLAG_DELAY,120,{}},
	{0x29,01,{0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
	};
#endif
#if 0
static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for(i = 0; i < count; i++) {

        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd) {

            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;

            case REGFLAG_END_OF_TABLE :
                break;

            default:
                dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
        }
    }

}
#endif
static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{
#ifdef BUILD_LK
	mt_set_gpio_mode(GPIO, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO, output);
#else
	gpio_set_value(GPIO, output);
#endif
}

static void lcm_init_power(void)
{
#ifdef BUILD_LK
	printf("[LK/LCM] lcm_init_power() enter\n");

	SET_RESET_PIN(0);
	MDELAY(20);

	lcm_set_gpio_output(GPIO_LCD_PWR_, GPIO_OUT_ONE);
	MDELAY(20);

	mt6392_upmu_set_rg_vgp2_vosel(3);
	mt6392_upmu_set_rg_vgp2_en(0x1);

	SET_RESET_PIN(1);
	MDELAY(20);
#else
	lcm_vgp_supply_enable();
	MDELAY(20);

	lcm_set_gpio_output(GPIO_LCD_RST_, GPIO_OUT_ONE);
	MDELAY(10);

	lcm_set_gpio_output(GPIO_LCD_RST_, GPIO_OUT_ZERO);
	MDELAY(10);

	lcm_set_gpio_output(GPIO_LCD_RST_, GPIO_OUT_ONE);
	MDELAY(10);

	lcm_set_gpio_output(GPIO_LCD_PWR_, GPIO_OUT_ONE);

	MDELAY(10);

	{
		unsigned int data_array[16];
		data_array[0] = 0xD6892300;
		dsi_set_cmdq(data_array, 1, 1);

		data_array[0] = 0x03B92300;
		dsi_set_cmdq(data_array, 1, 1);

		data_array[0] = 0x07BB2300;
		dsi_set_cmdq(data_array, 1, 1);

		data_array[0] = 0x1A952300;
		dsi_set_cmdq(data_array, 1, 1);

		data_array[0] = 0xD5892300;
		dsi_set_cmdq(data_array, 1, 1);

		data_array[0] = 0x08A82300;
		dsi_set_cmdq(data_array, 1, 1);

		data_array[0] = 0x00110500;
		dsi_set_cmdq(data_array, 1, 1);
		MDELAY(120);

		data_array[0] = 0x00290500;
		dsi_set_cmdq(data_array, 1, 1);
		MDELAY(20);
	}
	MDELAY(10);

	#endif
}

static void lcm_suspend_power(void)
{
#ifndef BUILD_LK
	pr_err("[Kernel/LCM] lcm_suspend_power() enter wangpei\n");

	{
		unsigned int data_array[16];
		data_array[0] = 0x00280500;
		dsi_set_cmdq(data_array, 1, 1);
		MDELAY(60);

		data_array[0] = 0x00100500;
		dsi_set_cmdq(data_array, 1, 1);
		MDELAY(120);
	}

	lcm_set_gpio_output(GPIO_LCD_PWR_, GPIO_OUT_ZERO);
	MDELAY(100);

	lcm_set_gpio_output(GPIO_LCD_RST_, GPIO_OUT_ZERO);
	MDELAY(10);

	lcm_vgp_supply_disable();
	MDELAY(10);

#endif
}

static void lcm_resume_power(void)
{
#ifndef BUILD_LK
	pr_err("[Kernel/LCM] lcm_resume_power() enter\n");

	lcm_vgp_supply_enable();
	MDELAY(20);

	lcm_set_gpio_output(GPIO_LCD_RST_, GPIO_OUT_ONE);
	MDELAY(20);

	lcm_set_gpio_output(GPIO_LCD_RST_, GPIO_OUT_ZERO);
	MDELAY(30);

	lcm_set_gpio_output(GPIO_LCD_RST_, GPIO_OUT_ONE);
	MDELAY(50);
	lcm_set_gpio_output(GPIO_LCD_PWR_, GPIO_OUT_ONE);
	MDELAY(10);


	{
		unsigned int data_array[16];
		data_array[0] = 0xD6892300;
		dsi_set_cmdq(data_array, 1, 1);

		data_array[0] = 0x03B92300;
		dsi_set_cmdq(data_array, 1, 1);

		data_array[0] = 0x07BB2300;
		dsi_set_cmdq(data_array, 1, 1);

		data_array[0] = 0x1A952300;
		dsi_set_cmdq(data_array, 1, 1);

		data_array[0] = 0xD5892300;
		dsi_set_cmdq(data_array, 1, 1);

		data_array[0] = 0x08A82300;
		dsi_set_cmdq(data_array, 1, 1);

		data_array[0] = 0x00110500;
		dsi_set_cmdq(data_array, 1, 1);
		MDELAY(120);

		data_array[0] = 0x00290500;
		dsi_set_cmdq(data_array, 1, 1);
		MDELAY(20);
	}


#endif
}

/* --------------------------------------------------------------------------- */
/* LCM Driver Implementations */
/* --------------------------------------------------------------------------- */
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
		memset(params, 0, sizeof(LCM_PARAMS));
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;
#if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
#else
		params->dsi.mode   = BURST_VDO_MODE;	//SYNC_EVENT_VDO_MODE;		//SYNC_PULSE_VDO_MODE;
#endif
		// DSI
		/* Command mode setting */
		// Three lane or Four lane
		params->dsi.LANE_NUM				= LCM_THREE_LANE;

		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		// Not support in MT6573
		params->dsi.packet_size=256;

		// Video mode setting
		params->dsi.intermediat_buffer_num = 0;

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		params->dsi.word_count=FRAME_WIDTH*3;

		params->dsi.vertical_sync_active				= 1;
		params->dsi.vertical_backporch					= 23;
		params->dsi.vertical_frontporch					= 12;
		params->dsi.vertical_active_line				= FRAME_HEIGHT;

		params->dsi.horizontal_sync_active				= 10;
		params->dsi.horizontal_backporch				= 160;
		params->dsi.horizontal_frontporch				= 160;
		params->dsi.horizontal_blanking_pixel = 60;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

		// Bit rate calculation
		// Every lane speed
		//params->dsi.pll_div1=0;				// div1=0,1,2,3;div1_real=1,2,4,4
		//params->dsi.pll_div2=0;				// div2=0,1,2,3;div1_real=1,2,4,4
		//params->dsi.fbk_div =0x12;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)
		params->dsi.cont_clock 	= 1;
		params->dsi.ssc_disable = 0;
		params->dsi.PLL_CLOCK 	= 210;
		params->dsi.HS_TRAIL = 5;
		params->dsi.HS_PRPR = 4;

		params->dsi.esd_check_enable = 1;
                params->dsi.customization_esd_check_enable = 1;
                params->dsi.lcm_esd_check_table[0].cmd = 0xA8;
                params->dsi.lcm_esd_check_table[0].count = 1;
                params->dsi.lcm_esd_check_table[0].para_list[0] = 0x08;

}


static void lcm_init_lcm(void)
{
#ifdef BUILD_LK
	printf("[LK/LCM] lcm_init() enter\n");
#else
	pr_err("[Kernel/LCM] lcm_init() enter\n");
#endif
}



static void lcm_suspend(void)
{
#ifdef BUILD_LK
	printf("[LK/LCM] lcm_suspend() enter\n");
#else
	pr_err("[Kernel/LCM] lcm_suspend() enter\n");
#endif
}



static void lcm_resume(void)
{
#ifdef BUILD_LK
	printf("[LK/LCM] lcm_resume() enter\n");
#else
	pr_err("[Kernel/LCM] lcm_resume() enter\n");
#endif
}


static unsigned int lcm_ata_check(unsigned char *buffer)
{
	return 0;
}

LCM_DRIVER HX8282_wsvga_dsi_vdo_lcm_drv = {
	.name = "HX8282_wsvga_txd",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init_lcm,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
	.ata_check = lcm_ata_check,
};

