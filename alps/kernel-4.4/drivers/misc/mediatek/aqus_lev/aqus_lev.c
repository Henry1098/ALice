#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#ifdef CONFIG_OF
#include <linux/of_fdt.h>
#endif
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/hardware_info.h>
char SKU_name[35];
static int SKU_probe(struct platform_device *dev)
{
  
   hardwareinfo_set_prop(HARDWARE_BORD_ID_INFO,SKU_name);
   return 0;
}

static int SKU_remove(struct platform_device *dev)
{
	return 0;
}



#if defined(CONFIG_OF) 
static const struct of_device_id sku_of_match[] = {
	{.compatible = "Lenovo,Aqus",},
	{},
};

MODULE_DEVICE_TABLE(of, sku_of_match);
#endif

static struct platform_driver SKU_driver = {
	.probe = SKU_probe,
	.remove = SKU_remove,
	.driver = {
		   .name = "Aqus",
#ifdef CONFIG_OF
		   .of_match_table = sku_of_match,
#endif
		   },
};

static int __init SKU_init(void)
{
	int ret;

	printk("SKU_init\n");

	ret = platform_driver_register(&SKU_driver);
	if (ret) {
		printk("[SKU_init] Unable to register driver (%d)\n", ret);
		return ret;
	}
		return 0;
}

static void __exit SKU_exit(void)
{
}
module_init(SKU_init);
module_exit(SKU_exit);

#ifdef CONFIG_OF
static int __init dt_get_SKU(unsigned long node, const char *uname, int depth, void *data)
{
	char *ptr = NULL,*br_ptr = NULL ,*q = NULL;

	if (depth != 1 || (strcmp(uname, "chosen") != 0 && strcmp(uname, "chosen@0") != 0))
		return 0;

	ptr = (char *)of_get_flat_dt_prop(node, "bootargs", NULL);
	if (ptr) {
        br_ptr = strstr(ptr, "SKU=");
		if (br_ptr != 0) {
			br_ptr = br_ptr+4;
			q = br_ptr;
			while(*q != ' ' && *q != '\0') q++;
			memset(SKU_name, 0, sizeof(SKU_name));
			strncpy(SKU_name, (const char*)br_ptr, (int)(q-br_ptr));
	   }
	}else{
	   printk("dt_get_SKU error!\n");
	}

	/* break now */
	return 1;
}
#endif

static int __init sku_lg_init(void)
{
#ifdef CONFIG_OF
	int rc;
	rc = of_scan_flat_dt(dt_get_SKU, NULL);
#endif
   return 0;
}

early_initcall(sku_lg_init);
