/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/string.h>
#include <linux/regulator/consumer.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/syscore_ops.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#else
#include <string.h>
#endif

#include "lcm_drv.h"

#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#endif

#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/gpio.h>
#include <linux/device.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#endif

static unsigned int GPIO_LCD_PWR_EN;
//tlc
/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */
void lcm_get_gpio_infor(void)
{
	static struct device_node *node;


	node = of_find_compatible_node(NULL, NULL, "mediatek,mt8173-lcm");
	if (node == NULL) {
		//printk("tlc_null lcm gpio info - get node failed\n");
	 }
	 else
	 {	

	GPIO_LCD_PWR_EN = of_get_named_gpio(node, "gpio_lcm_pwr_en", 0);

}
	
}






static int lcm_driver_probe(struct device *dev, void const *data)
{

	
	lcm_get_gpio_infor();
    
	return 0;
	
}



static const struct of_device_id lcm_platform_of_match[] = {
	{
		.compatible = "mediatek,mt8173-lcm"
	}, {
	}
};

MODULE_DEVICE_TABLE(of, platform_of_match);

static int lcm_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;

	id = of_match_node(lcm_platform_of_match, pdev->dev.of_node);
	
	if (!id)
		return -ENODEV;

	return lcm_driver_probe(&pdev->dev, id->data);
}

static struct platform_driver lcm_driver = {
	.probe = lcm_platform_probe,
	.driver = {
		   .name = "nt71397_dsi_vdo_kd101n46_wuxga",
		   .owner = THIS_MODULE,
		   .of_match_table = lcm_platform_of_match,
		   },
};

static int __init lcm_init(void)
{
	
	if (platform_driver_register(&lcm_driver)) {
		pr_err("LCM: failed to register this driver!\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit lcm_exit(void)
{
	platform_driver_unregister(&lcm_driver);
	pr_debug("LCM: Unregister this driver done\n");
}

late_initcall(lcm_init);
module_exit(lcm_exit);
MODULE_AUTHOR("mediatek");
MODULE_DESCRIPTION("LCM display subsystem driver");
MODULE_LICENSE("GPL");
#define FRAME_HEIGHT (1920)
#define FRAME_WIDTH  (1200)



#define GPIO_OUT_ONE 1
#define GPIO_OUT_ZERO 0
/* --------------------------------------------------------------------------- */
/* Local Variables */
/* --------------------------------------------------------------------------- */

static LCM_UTIL_FUNCS lcm_util = {
	.set_reset_pin = NULL,
	.udelay = NULL,
	.mdelay = NULL,
};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

/* --------------------------------------------------------------------------- */
/* Local Functions */
/* --------------------------------------------------------------------------- */

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)	 lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)									 lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)				 lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)						  lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)


#define REGFLAG_DELAY                                                                   0xFE
#define REGFLAG_END_OF_TABLE                                                            0xFD   // END OF REGISTE

#define   LCM_DSI_CMD_MODE							0

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_sleep_out_setting[] = {
	/* Sleep Out */
	{0x11, 0, {} },
	{REGFLAG_DELAY, 120, {} },

	/* Display ON */
	{0x29, 0, {} },
	{REGFLAG_DELAY, 30, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	/* Sleep Mode On */
	{0x28, 0, {} },	
	{0x10, 0, {} },
	{REGFLAG_DELAY, 120, {} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
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
                                MDELAY(10);
        }
    }
}

/* --------------------------------------------------------------------------- */
/* LCM Driver Implementations */
/* --------------------------------------------------------------------------- */
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{
	gpio_direction_output(GPIO, output);
	gpio_set_value(GPIO, output);

}



static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode = CMD_MODE;
#else
	params->dsi.mode = BURST_VDO_MODE;//SYNC_EVENT_VDO_MODE;//BURST_VDO_MODE; //SYNC_PULSE_VDO_MODE;
#endif

	params->dsi.LANE_NUM = LCM_FOUR_LANE;

	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	params->dsi.packet_size = 256;
	//params->dsi.intermediat_buffer_num = 0;
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.word_count = FRAME_WIDTH * 3;

	params->dsi.vertical_sync_active = 5;
	params->dsi.vertical_backporch = 2;
	params->dsi.vertical_frontporch = 13;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 16;
	params->dsi.horizontal_backporch = 8;
	params->dsi.horizontal_frontporch = 116;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

	
	params->dsi.ssc_disable = 1;
	params->dsi.cont_clock = 1;
	params->dsi.PLL_CLOCK = 519;
	params->dsi.clk_lp_per_line_enable = 1; 

}

static void lcm_init_lcm(void)
{
	 lcm_get_gpio_infor();

    lcm_set_gpio_output(GPIO_LCD_PWR_EN, GPIO_OUT_ONE);
	
	MDELAY(20);

	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);

}

static void lcm_suspend(void)
{

	
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
    MDELAY(20);
   

	lcm_set_gpio_output(GPIO_LCD_PWR_EN, GPIO_OUT_ZERO);
	MDELAY(10);

}

static void lcm_resume(void)
{
	
	lcm_set_gpio_output(GPIO_LCD_PWR_EN, GPIO_OUT_ONE);
	MDELAY(20);

    push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);	

	 
}



LCM_DRIVER cm_n070ice_dsi_vdo_mt8173_lcm_drv = {
	.name = "nt71397_dsi_vdo_kd101n46_wuxga",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init_lcm,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
};
