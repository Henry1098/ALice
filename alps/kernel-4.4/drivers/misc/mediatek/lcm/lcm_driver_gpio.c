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
#ifndef BUILD_LK

unsigned int GPIO_LCD_PWR_;
unsigned int GPIO_LCD_RST_;
unsigned int GPIO_LCD_ID1_;

static struct regulator *lcm_vgp;
/* get LDO supply */
 int lcm_get_vgp_supply(struct device *dev)
{
	int ret;
	struct regulator *lcm_vgp_ldo;

	/* printk("LCM: lcm_get_vgp_supply is going\n"); */

	lcm_vgp_ldo = devm_regulator_get(dev, "reg-lcm");
	if (IS_ERR(lcm_vgp_ldo)) {
		ret = PTR_ERR(lcm_vgp_ldo);
		dev_err(dev, "failed to get reg-lcm LDO, %d\n", ret);
		return ret;
	}

	pr_debug("LCM: lcm get supply ok.\n");

	/* get current voltage settings */
	ret = regulator_get_voltage(lcm_vgp_ldo);
	pr_debug("lcm LDO voltage = %d in LK stage\n", ret);

	lcm_vgp = lcm_vgp_ldo;

	return ret;
}


 int lcm_vgp_supply_enable(void)
{
	int ret;
	unsigned int volt;

	/* printk("LCM: lcm_vgp_supply_enable\n"); */

	if (lcm_vgp == NULL)
		return 0;

	/* printk("LCM: set regulator voltage lcm_vgp voltage to 3.3V\n"); */
	/* set voltage to 1.8V */
	ret = regulator_set_voltage(lcm_vgp, 1800000, 1800000);
	if (ret != 0) {
		pr_err("LCM: lcm failed to set lcm_vgp voltage: %d\n", ret);
		return ret;
	}

	/* get voltage settings again */
	volt = regulator_get_voltage(lcm_vgp);
	/*
	if (volt == 1800000)
		printk("LCM: check regulator voltage=1800000 pass!\n");
	else
		printk("LCM: check regulator voltage=1800000 fail! (voltage: %d)\n", volt);
	*/
	ret = regulator_enable(lcm_vgp);
	if (ret != 0) {
		pr_err("LCM: Failed to enable lcm_vgp: %d\n", ret);
		return ret;
	}

	return ret;
}



 int lcm_vgp_supply_disable(void)
{
	int ret = 0;
	unsigned int isenable;

	if (lcm_vgp == NULL)
		return 0;

	/* disable regulator */
	isenable = regulator_is_enabled(lcm_vgp);

	/* printk("LCM: lcm query regulator enable status[%d]\n", isenable); */

	if (isenable) {
		ret = regulator_disable(lcm_vgp);
		if (ret != 0) {
			pr_err("LCM: lcm failed to disable lcm_vgp: %d\n", ret);
			return ret;
		}
		/* verify */
		isenable = regulator_is_enabled(lcm_vgp);
		if (!isenable)
			pr_err("LCM: lcm regulator disable pass\n");
	}

	return ret;
}


 void lcm_request_gpio_control(struct device *dev)
{
	GPIO_LCD_PWR_ = of_get_named_gpio(dev->of_node, "gpio_lcd_pwr", 0);
	gpio_request(GPIO_LCD_PWR_, "GPIO_LCD_PWR");
	pr_err("wangpei_GPIO_LCD_PWR_=%d\n",GPIO_LCD_PWR_);

	GPIO_LCD_RST_ = of_get_named_gpio(dev->of_node, "gpio_lcd_rst", 0);
	gpio_request(GPIO_LCD_RST_, "GPIO_LCD_RST");
	pr_err("wangpei_GGPIO_LCD_RST_=%d\n",GPIO_LCD_RST_);

	GPIO_LCD_ID1_=of_get_named_gpio(dev->of_node, "gpio_lcd_id1", 0);
	gpio_request(GPIO_LCD_ID1_, "GPIO_LCD_ID1");
	pr_err("wangpei_GPIO_LCD_PWR_=%d\n",GPIO_LCD_ID1_);
}


 int lcm_driver_probe(struct device *dev, void const *data)
{
	/* printk("LCM: lcm_driver_probe\n"); */

	lcm_request_gpio_control(dev);
	pr_err("wangpei_GPIO_!!!!!!!!!!!!!!!!!!1\n");
	lcm_get_vgp_supply(dev);
	lcm_vgp_supply_enable();

	return 0;
}

static const struct of_device_id lcm_platform_of_match[] = {
	{
		.compatible = "mediatek-lcm",
		.data = 0,
	}, {
		/* sentinel */
	}
};

MODULE_DEVICE_TABLE(of, platform_of_match);

static int lcm_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;
	pr_err("wangpei_Glcm_platform_probe\n");
	id = of_match_node(lcm_platform_of_match, pdev->dev.of_node);
	
	
	if (!id)
		return -ENODEV;
	pr_err("wangpei_idd\n");

	return lcm_driver_probe(&pdev->dev, id->data);
}

static struct platform_driver lcm_driver = {
	.probe = lcm_platform_probe,
	.driver = {
		   .name = "mediatek-lcm",
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
}
late_initcall(lcm_init);
module_exit(lcm_exit);
MODULE_AUTHOR("mediatek");
MODULE_DESCRIPTION("LCM display subsystem driver");
MODULE_LICENSE("GPL");
#endif
