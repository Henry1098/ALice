#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/byteorder/generic.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
//#include <linux/rtpm_prio.h>
#include <linux/of_fdt.h>
#include "mtk_gsl3670.h"
#include "tpd.h"
//#include "include/tpd_custom_gsl915.h" 

#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_irq.h>
//#include <ext_wd_drv.h>
#include <linux/dma-mapping.h>
#include <mt-plat/mtk_boot_common.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/hardware_info.h>


#define GSL_DEBUG
//#define GSL_MONITOR

#define GSL3670_NAME	"GSL3670"
#define GSLX680_NAME	"gslX680"
#define GSLX680_ADDR	0x40
#define MAX_FINGERS	  	10
#define MAX_CONTACTS	10
#define DMA_TRANS_LEN	0x20
#define SMBUS_TRANS_LEN	0x01
#define GSL_PAGE_REG		0xf0
//#define ADD_I2C_DEVICE_ANDROID_4_0
//#define HIGH_SPEED_I2C
//#define FILTER_POINT
#ifdef FILTER_POINT
#define FILTER_MAX	9
#endif
//#define GSL_GESTURE
#ifdef GSL_GESTURE
static int gsl_gesture_flag = 1;
static struct input_dev *gsl_power_idev;
static int gsl_lcd_flag = 0;
static char gsl_gesture_c = 0;
extern void gsl_FunIICRead(unsigned int ReadIICInt(unsigned int *data,unsigned int addr,unsigned int len));
extern void gsl_GestureExternInt(unsigned int *model,int len);
extern int gsl_obtain_gesture(void);
unsigned int gsl_model_extern[]={
	0x10,0x56,
	0x08000f00,0x0f0c2f1f,0x12125040,0x13127060,0x16149181,0x1a18b1a1,0x1c1bd2c2,0x201df2e2,
	0x3324f7fe,0x4f41e7ef,0x6d5ed8df,0x8a7bc8d0,0xa698b7c0,0xc3b4a6af,0xe0d2959d,0xffee848d,
	0x10,0x57,
	0x00062610,0x03015c41,0x06049277,0x0f09c8ad,0x2918f7e0,0x5142e4fb,0x685eb2cb,0x77707c97,
	0x857d9177,0x978dc5ab,0xb4a1f3de,0xdbcbd5ec,0xebe4a2bd,0xf4f06c87,0xfaf73651,0xfffd001b,
	
	0x10,0x49,
	0x0e00f4ff,0x2f1ee4ec,0x4f3ed4dc,0x6f5fc4cc,0x8f7fb3bc,0xae9ea3ab,0xcebe949b,0xf0df858d,
	0xf0ff707a,0xcfdf6268,0xadbe525a,0x8d9e434a,0x6d7d353c,0x4c5c262e,0x2c3c151e,0x0c3c000b,
	0x10,0x49,
	0x775df6f9,0xab91e6ef,0xdac3cedb,0xf9eda2b9,0xfdff6d88,0xf1f93a53,0xcce21424,0x9ab50209,
	0x65800101,0x354d1409,0x0f1f3a25,0x01056e53,0x0100a288,0x1407d3bc,0x4128f1e4,0x765bfffb,	
	
};

#define  GSL_TPD_GES_COUNT  14
static int tpd_keys_gesture[GSL_TPD_GES_COUNT] ={KEY_POWER, \
												TPD_GES_KEY_DOUBLECLICK,\
												TPD_GES_KEY_LEFT,\
												TPD_GES_KEY_RIGHT,\
												TPD_GES_KEY_UP,\
												TPD_GES_KEY_DOWN,\
												TPD_GES_KEY_O,\
												TPD_GES_KEY_W,  \
												TPD_GES_KEY_M,\
												TPD_GES_KEY_E,\
												TPD_GES_KEY_C,\
												TPD_GES_KEY_S,\
												TPD_GES_KEY_V,\
												TPD_GES_KEY_Z};

#endif

#define TPD_PROC_DEBUG
#ifdef TPD_PROC_DEBUG
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/seq_file.h>  //lzk
//static struct proc_dir_entry *gsl_config_proc = NULL;
#define GSL_CONFIG_PROC_FILE "gsl_config"
#define CONFIG_LEN 31
static char gsl_read[CONFIG_LEN];
static u8 gsl_data_proc[8] = {0};
static u8 gsl_proc_flag = 0;
#endif

#define GSL_TEST_TP //wu add rawdata test 2015.11.5
#ifdef GSL_TEST_TP
extern void gsl_write_test_config(unsigned int cmd,int value);
extern unsigned int gsl_read_test_config(unsigned int cmd);
extern int gsl_obtain_array_data_ogv(unsigned short*ogv,int i_max,int j_max);
extern int gsl_obtain_array_data_dac(unsigned short  *dac,int i_max,int j_max);
extern int gsl_tp_module_test(char *buf,int size);
//static int open_short_row;
//static int open_short_col;
#define GSL_PARENT_PROC_NAME "touchscreen"
#define GSL_OPENHSORT_PROC_NAME "ctp_openshort_test"
#define GSL_RAWDATA_PROC_NAME "ctp_rawdata"
#endif


static char tp_string_version[40];


//#define GSL_NOID_VERSION
#define TPD_PROXIMITY
#ifdef TPD_PROXIMITY
#include <hwmsensor.h>
#include <sensors_io.h>
#include <alsps.h>
//#include <linux/hardware_info.h>
//#include <hwmsen_dev.h>
//#include "inc/aal_control.h"
//#include "hwmsensor.h"
//#include "sensors_io.h"
//#include "hwmsen_helper.h"
#include <linux/io.h>
#include <linux/wakelock.h>
//#include <alsps.h>
#include <linux/types.h>
#include <linux/ioctl.h>
static u8 tpd_proximity_flag = 0; //flag whether start alps
static u8 tpd_proximity_detect = 1;//0-->close ; 1--> far away
//static struct wake_lock ps_lock;
//static u8 gsl_psensor_data[8]={0};
//static u8 tdp_proximity_enabled = 1;
extern int ps_flush_report(void);
static int ps_flush(void);
struct gsl_priv {
	struct work_struct eint_work;
	ulong enable;		/*enable mask */
	ulong pending_intr;	/*pending interrupt */

	/*early suspend */
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_drv;
#endif
	bool ps_flush;

};
	static struct gsl_priv *gsl_obj;
enum {
	CMC_BIT_ALS = 1,
	CMC_BIT_PS = 2,
} CMC_BIT;
#endif

static int tpd_flag = 0;
static int tpd_halt=0;
static char eint_flag = 0;
extern struct tpd_device *tpd;
static struct i2c_client *i2c_client = NULL;
static struct task_struct *thread = NULL;
#ifdef GSL_MONITOR
static struct delayed_work gsl_monitor_work;
static struct workqueue_struct *gsl_monitor_workqueue = NULL;
static char int_1st[4] = {0};
static char int_2nd[4] = {0};
static char dac_counter = 0;
static char b0_counter = 0;
static char bc_counter = 0;
static char i2c_lock_flag = 0;
#endif
static char SKU_name[35];
static unsigned int lcd_product=0;
static int of_get_gsl6730_platform_data(struct device *dev);	

unsigned int gsl_touch_irq = 0;
static irqreturn_t tpd_eint_interrupt_handler(int irq, void *dev_id);
typedef unsigned char kal_uint8;
typedef unsigned short kal_uint16;
typedef unsigned int kal_uint32;

static kal_uint32 id_sign[MAX_CONTACTS+1] = {0};
static kal_uint8 id_state_flag[MAX_CONTACTS+1] = {0};
static kal_uint8 id_state_old_flag[MAX_CONTACTS+1] = {0};
static kal_uint16 x_old[MAX_CONTACTS+1] = {0};
static kal_uint16 y_old[MAX_CONTACTS+1] = {0};
static kal_uint16 x_new = 0;
static kal_uint16 y_new = 0;
#define TP_POWER MT6323_POWER_LDO_VGP1

//#define TPD_HAVE_BUTTON
//#define TPD_KEY_COUNT	3
//#define TPD_KEYS		{KEY_MENU, KEY_HOMEPAGE, KEY_BACK/*, KEY_SEARCH*/}
/* {button_center_x, button_center_y, button_width, button_height*/
//#ifdef GSL_XCL_TP
//#define TPD_KEYS_DIM	{{150, 1058, 60, 50},{330, 1058, 60, 50},{450, 1058, 60, 50}/*,{470, 2048, 60, 50}*/}
//#else
//#define TPD_KEYS_DIM	{{60, 2100, 40, 40},{200, 2100, 40, 40},{450, 2100, 40, 40}/*,{470, 2048, 60, 50}*/}
//#endif

static DECLARE_WAIT_QUEUE_HEAD(waiter);

#ifdef GSL_DEBUG 
#define print_info(fmt, args...)   \
        do{                              \
                printk(fmt, ##args);     \
        }while(0)
#else
#define print_info(fmt, args...)
#endif

//#ifdef TPD_HAVE_BUTTON 
//static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
//static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
//#endif

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT]   = TPD_WARP_END;
#endif
#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif

static void startup_chip(struct i2c_client *client)
{
	u8 write_buf = 0x00;

	i2c_smbus_write_i2c_block_data(client, 0xe0, 1, &write_buf); 	
#ifdef GSL_NOID_VERSION
	gsl_DataInit(gsl_config_data_id);
#endif

	msleep(10);		
}
static int __init dt_get_SKU(unsigned long node, const char *uname, int depth, void *data)
{
	char *ptr = NULL,*br_ptr = NULL ,*q = NULL;

	if (depth != 1 || (strcmp(uname, "chosen") != 0 && strcmp(uname, "chosen@0") != 0))
		return 0;

	ptr = (char *)of_get_flat_dt_prop(node, "bootargs", NULL);
	if (ptr) {
        br_ptr = strstr(ptr, "lcd_id=");
		if (br_ptr != 0) {
			q = br_ptr;
			while(*q != ' ' && *q != '\0') q++;
			memset(SKU_name, 0, sizeof(SKU_name));
			strncpy(SKU_name, (const char*)br_ptr, (int)(q-br_ptr));
			if(strstr(SKU_name,"lcd_id=1") != NULL)
				lcd_product = 1;  //tongxingda
			else
				lcd_product = 2;//xinyuan
	   }
	}else{
	   printk("dt_get_SKU error!\n");
	}

	/* break now */
	return 1;
}
static void reset_chip(struct i2c_client *client)
{
	u8 write_buf[4]	= {0};

	write_buf[0] = 0x88;
	i2c_smbus_write_i2c_block_data(client, 0xe0, 1, &write_buf[0]); 	
	msleep(20);

	write_buf[0] = 0x04;
	i2c_smbus_write_i2c_block_data(client, 0xe4, 1, &write_buf[0]); 	
	msleep(10);

	write_buf[0] = 0x00;
	write_buf[1] = 0x00;
	write_buf[2] = 0x00;
	write_buf[3] = 0x00;
	i2c_smbus_write_i2c_block_data(client, 0xbc, 4, write_buf); 	
	msleep(10);
}

static void clr_reg(struct i2c_client *client)
{
	char write_buf[4]	= {0};

	write_buf[0] = 0x88;
	i2c_smbus_write_i2c_block_data(client, 0xe0, 1, &write_buf[0]); 	
	msleep(20);

	write_buf[0] = 0x03;
	i2c_smbus_write_i2c_block_data(client, 0x80, 1, &write_buf[0]); 	
	msleep(5);
	
	write_buf[0] = 0x04;
	i2c_smbus_write_i2c_block_data(client, 0xe4, 1, &write_buf[0]); 	
	msleep(5);

	write_buf[0] = 0x00;
	i2c_smbus_write_i2c_block_data(client, 0xe0, 1, &write_buf[0]); 	
	msleep(20);
}

#ifdef HIGH_SPEED_I2C
static u32 gsl_read_interface(struct i2c_client *client, u8 reg, u8 *buf, u32 num)
{
	struct i2c_msg xfer_msg[2];

	xfer_msg[0].addr = client->addr;
	xfer_msg[0].len = 1;
	xfer_msg[0].flags = client->flags & I2C_M_TEN;
	xfer_msg[0].buf = &reg;
	xfer_msg[0].timing = 400;

	xfer_msg[1].addr = client->addr;
	xfer_msg[1].len = num;
	xfer_msg[1].flags |= I2C_M_RD;
	xfer_msg[1].buf = buf;
	xfer_msg[1].timing = 400;

	if (reg < 0x80) {
		i2c_transfer(client->adapter, xfer_msg, ARRAY_SIZE(xfer_msg));
		msleep(5);
	}

	return i2c_transfer(client->adapter, xfer_msg, ARRAY_SIZE(xfer_msg)) == ARRAY_SIZE(xfer_msg) ? 0 : -EFAULT;
}

static u32 gsl_write_interface(struct i2c_client *client, const u8 reg, u8 *buf, u32 num)
{
	struct i2c_msg xfer_msg[1];

	buf[0] = reg;

	xfer_msg[0].addr = client->addr;
	xfer_msg[0].len = num + 1;
	xfer_msg[0].flags = client->flags & I2C_M_TEN;
	xfer_msg[0].buf = buf;
	xfer_msg[0].timing = 400;

	return i2c_transfer(client->adapter, xfer_msg, 1) == 1 ? 0 : -EFAULT;
}

static __inline__ void fw2buf(u8 *buf, const u32 *fw)
{
	u32 *u32_buf = (int *)buf;
	*u32_buf = *fw;
}

static void gsl_load_fw(struct i2c_client *client)
{
	u8 buf[DMA_TRANS_LEN*4 + 1] = {0};
	u8 send_flag = 1;
	u8 *cur = buf + 1;
	u32 source_line = 0;
	u32 source_len;
	struct fw_data *ptr_fw;

	printk("=============gsl_load_fw start==============\n");

	ptr_fw = GSLX680_FW;
	source_len = ARRAY_SIZE(GSLX680_FW);
	for (source_line = 0; source_line < source_len; source_line++) 
	{
		/* init page trans, set the page val */
		if (GSL_PAGE_REG == ptr_fw[source_line].offset)
		{
			fw2buf(cur, &ptr_fw[source_line].val);
			gsl_write_interface(client, GSL_PAGE_REG, buf, 4);
			send_flag = 1;
		}
		else 
		{
			if (1 == send_flag % (DMA_TRANS_LEN < 0x20 ? DMA_TRANS_LEN : 0x20))
	    			buf[0] = (u8)ptr_fw[source_line].offset;

			fw2buf(cur, &ptr_fw[source_line].val);
			cur += 4;

			if (0 == send_flag % (DMA_TRANS_LEN < 0x20 ? DMA_TRANS_LEN : 0x20)) 
			{
	    			gsl_write_interface(client, buf[0], buf, cur - buf - 1);
	    			cur = buf + 1;
			}

			send_flag++;
		}
	}

	printk("=============gsl_load_fw end==============\n");

}
#else
// zhoulingbing add 0506
#if 0
typedef enum wd_restart_type {
	WD_TYPE_NORMAL,
	WD_TYPE_NOLOCK,	
}WD_RES_TYPE;

extern void mtk_wdt_restart(enum wd_restart_type type);

static void mtk_kick_wdt(void)
{
    mtk_wdt_restart(WD_TYPE_NORMAL);
    mtk_wdt_restart(WD_TYPE_NOLOCK);
}
#endif
//zhoulingbing add end
static void gsl_load_fw(struct i2c_client *client)
{
	u8 buf[SMBUS_TRANS_LEN*4] = {0};
	u8 reg = 0, send_flag = 1, cur = 0;
	
	unsigned int source_line = 0;
	unsigned int source_len = ARRAY_SIZE(GSLX680_FW);

	printk("=============gsl_load_fw start==============\n");

	for (source_line = 0; source_line < source_len; source_line++) 
	{
		//mtk_kick_wdt();  // zhoulingbing add 0506
		if(1 == SMBUS_TRANS_LEN)
		{
			reg = GSLX680_FW[source_line].offset;
			memcpy(&buf[0], &GSLX680_FW[source_line].val, 4);
			i2c_smbus_write_i2c_block_data(client, reg, 4, buf); 	
		}
		else
		{
			/* init page trans, set the page val */
			if (GSL_PAGE_REG == GSLX680_FW[source_line].offset)
			{
				buf[0] = (u8)(GSLX680_FW[source_line].val & 0x000000ff);
				i2c_smbus_write_i2c_block_data(client, GSL_PAGE_REG, 1, &buf[0]); 	
				send_flag = 1;
			}
			else 
			{
				if (1 == send_flag % (SMBUS_TRANS_LEN < 0x08 ? SMBUS_TRANS_LEN : 0x08))
					reg = GSLX680_FW[source_line].offset;

				memcpy(&buf[cur], &GSLX680_FW[source_line].val, 4);
				cur += 4;

				if (0 == send_flag % (SMBUS_TRANS_LEN < 0x08 ? SMBUS_TRANS_LEN : 0x08)) 
				{
					i2c_smbus_write_i2c_block_data(client, reg, SMBUS_TRANS_LEN*4, buf); 	
					cur = 0;
				}

				send_flag++;

			}
		}
	}

	printk("=============gsl_load_fw end==============\n");

}
#endif


#ifdef GSL_TEST_TP

void gsl_I2C_RTotal_Address(unsigned int addr,unsigned int *data)
{
	u8 tmp_buf[4]={0};	
	tmp_buf[3]=(u8)((addr/0x80)>>24);
	tmp_buf[2]=(u8)((addr/0x80)>>16);
	tmp_buf[1]=(u8)((addr/0x80)>>8);
	tmp_buf[0]=(u8)((addr/0x80));
	i2c_smbus_write_i2c_block_data(i2c_client, 0xf0, 4, tmp_buf);
//	i2c_smbus_read_i2c_block_data(i2c_client,(addr%0x80+8)&0x5c, 4, tmp_buf);
	i2c_smbus_read_i2c_block_data(i2c_client,addr%0x80, 4, tmp_buf);
	i2c_smbus_read_i2c_block_data(i2c_client,addr%0x80, 4, tmp_buf);

	*data = tmp_buf[0]|(tmp_buf[1]<<8)|(tmp_buf[2]<<16)|(tmp_buf[3]<<24);
}
EXPORT_SYMBOL(gsl_I2C_RTotal_Address);

void gsl_I2C_ROnePage(unsigned int addr, char *buf)
{
#if 0
	int i;
	u8 tmp_buf[4]={0};
	tmp_buf[3]=(u8)(addr>>24);
	tmp_buf[2]=(u8)(addr>>16);
	tmp_buf[1]=(u8)(addr>>8);
	tmp_buf[0]=(u8)(addr);
	i2c_smbus_write_i2c_block_data(i2c_client, 0xf0, 4, tmp_buf);
	for(i=0;i<32;i++)
	{
		//i2c_smbus_read_i2c_block_data(i2c_client, (i*4+8)&0x5c, 4, &buf[i*4]);
		i2c_smbus_read_i2c_block_data(i2c_client, i*4, 4, &buf[i*4]);
		//i2c_smbus_read_i2c_block_data(i2c_client, i*4, 4, &buf[i*4]);
	}
#endif

	int i=0;
	unsigned int data[32];

	for(i=0;i<32;i++)
		gsl_I2C_RTotal_Address(addr*0x80+i*4,data+i);

	memcpy(buf,data,128);
}
EXPORT_SYMBOL(gsl_I2C_ROnePage);
#endif

#ifdef GSL_TEST_TP
static ssize_t gsl_test_show(void)
{
	static int gsl_test_flag = 0; 
	char *tmp_buf;
	int err;
	int result = 0;
	printk("enter gsl_test_show start::gsl_test_flag  = %d\n",gsl_test_flag);
	if(gsl_test_flag == 1){
		return 0;	
	}
	gsl_test_flag = 1;
	tmp_buf = kzalloc(3*1024,GFP_KERNEL);
	if(!tmp_buf){
		printk(" kzalloc  kernel  fail\n");
		return 0;
		}
	err = gsl_tp_module_test(tmp_buf,3*1024);
	printk("enter gsl_test_show end\n");
	if(err > 0){
		printk("tp test pass\n");
		result = 1;
	}else{
		printk("tp test failure\n");
		result = 0;
	}
	kfree(tmp_buf);
	gsl_test_flag = 0; 
	return result;
}
#if 0
static s32 gsl_openshort_proc_write(struct file *filp, const char __user *userbuf,size_t count, loff_t *ppos)
{
	return -1;
}
#endif
static ssize_t gsl_openshort_proc_read(struct file *file, char __user *buf,size_t count, loff_t *ppos)
{
	//char *ptr = buf;
	int test_result  = 0;
	char result_buf[10]={0};
	int ret=0;
	
	if(*ppos)
	{
			printk("[%s]:tp test again return\n",__func__);
			return 0;
	}
	*ppos += 16;

	test_result = gsl_test_show();
	printk("[%s]:tp test result =%d \n",__func__,test_result);
	//memset(buf,'\0',16);
	if(1 == test_result)
	{
		printk("[%s]:tp test pass\n",__func__);
		sprintf(result_buf, "result=%d\n", 1);
		
	}
	else
	{
		printk("[%s]:tp test failure\n",__func__);
		sprintf(result_buf, "result=%d\n", 0);
	}
	if(copy_to_user(buf,result_buf,strlen(result_buf)+1))
	{

		printk("[%s]:copy to user fail\n",__func__);
		ret =-1;

	}
	return ret;
	//return test_result;
}
static ssize_t gsl_rawdata_proc_read(struct file *file, char __user *buf,size_t count, loff_t *ppos)
{
		
		int i,j,ret;
		static short* gsl_ogv;
		ssize_t read_buf_chars = 0; 
		static short* gsl_dac;

		
		gsl_ogv = kzalloc(24*14*2,GFP_KERNEL);

		gsl_dac = kzalloc(4*14*2,GFP_KERNEL);
		if(!gsl_ogv){
			return -1;
		}  
		if(*ppos)
		{
			printk("[%s]:tp test again return\n",__func__);
			return 0;
		}
		ret=gsl_test_show();
		gsl_obtain_array_data_ogv(gsl_ogv,24,14);
		for(i=0;i<24*14;i++)
		{
			read_buf_chars += sprintf(&(buf[read_buf_chars])," _%u_ ",gsl_ogv[i]);
			if(!((i+1)%14))
			{
				buf[read_buf_chars++] = '\n';
			}
		}
		for(i=0;i<24;i++)
		{
			for(j=0;j<14;j++){
			printk("%4d ",gsl_ogv[i*14+j]);
						}
			printk("\n");
		 }
		buf[read_buf_chars-1] = '\n';
		gsl_obtain_array_data_dac(gsl_dac,4,14);
		printk("gsl dac value:\n");
		for(i=0;i<4;i++)
		{
			for(j=0;j<14;j++)
			{
				read_buf_chars += sprintf(&(buf[read_buf_chars])," _%u_ ",gsl_dac[i*14+j]);
				if(!((i*14+j+1)%14))
				{
					buf[read_buf_chars++] = '\n';
				}
			}
		}
		for(i=0;i<4;i++)
		{
			for(j=0;j<14;j++){
			printk("%4d ",gsl_dac[i*14+j]);
						}
			printk("\n");
		 }
		printk("\n");
		*ppos += read_buf_chars; 
		return read_buf_chars; 
		
}
#if 0
static s32 gsl_rawdata_proc_write(struct file *filp, const char __user *userbuf,size_t count, loff_t *ppos)
{
	return -1;
}
#endif
static int gsl_server_list_rawdata_open(struct inode *inode,struct file *file)
{
	return 0;
	//return single_open(file,gsl_rawdata_proc_read,NULL);
}
static int gsl_server_list_test_open(struct inode *inode,struct file *file)
{
	return 0;
	//return single_open(file,gsl_openshort_proc_read,NULL);
}
static const struct file_operations gsl_seq_rawdata_fops = {
	.open = gsl_server_list_rawdata_open,
	.read = gsl_rawdata_proc_read,
	//.release = single_release,
	//.write = gsl_rawdata_proc_write,
	.owner = THIS_MODULE,
};
static const struct file_operations gsl_seq_openshort_fops = {
	.open = gsl_server_list_test_open,
	.read = gsl_openshort_proc_read,
	//.release = single_release,
	//.write = gsl_openshort_proc_write,
	.owner = THIS_MODULE,
};

void create_ctp_proc(void)
{
	struct proc_dir_entry *gsl_device_proc = NULL;
	struct proc_dir_entry *gsl_openshort_proc = NULL;
	struct proc_dir_entry *gsl_rawdata_proc = NULL;
	gsl_device_proc = proc_mkdir(GSL_PARENT_PROC_NAME, NULL);
	if(gsl_device_proc == NULL)
    	{
        	printk("gsl915: create parent_proc fail\n");
        	return;
    	}
	gsl_openshort_proc = proc_create(GSL_OPENHSORT_PROC_NAME, 0666, gsl_device_proc, &gsl_seq_openshort_fops);
    	if (gsl_openshort_proc == NULL)
    	{
        	printk("gsl915: create openshort_proc fail\n");
    	}
	gsl_rawdata_proc = proc_create(GSL_RAWDATA_PROC_NAME, 0777, gsl_device_proc, &gsl_seq_rawdata_fops);
    	if (gsl_rawdata_proc == NULL)
    	{
        	printk("gsl915: create ctp_rawdata_proc fail\n");
    	}
}
#endif

static int test_i2c(struct i2c_client *client)
{
	u8 read_buf = 0;
	u8 write_buf = 0x12;
	int ret, rc = 1;
	
	ret = i2c_smbus_read_i2c_block_data( client, 0xf0, 1, &read_buf );
	if  (ret  < 0)  
    		rc --;
	else
		printk("I read reg 0xf0 is %x\n", read_buf);

	msleep(2);
	ret = i2c_smbus_write_i2c_block_data( client, 0xf0, 1, &write_buf );
	if(ret  >=  0 )
		printk("I write reg 0xf0 0x12\n");
	
	msleep(2);
	ret = i2c_smbus_read_i2c_block_data( client, 0xf0, 1, &read_buf );
	if(ret <  0 )
		rc --;
	else
		printk("I read reg 0xf0 is 0x%x\n", read_buf);

	return rc;
}
static int init_chip(struct i2c_client *client)
{
	int rc;
	
		gpio_direction_output(tpd_rst_gpio_number, 0);//GSL_TP_GPIO_OUTPUT(GTP_RST_PORT, 0);
	msleep(20); 	
		gpio_direction_output(tpd_rst_gpio_number, 1);//GSL_TP_GPIO_OUTPUT(GTP_RST_PORT, 1);
	msleep(20); 		
	rc = test_i2c(client);
	if(rc < 0)
	{
		printk("------gslX3670 test_i2c error------\n");	
		return -1;
	}	
	clr_reg(client);
	reset_chip(client);
	gsl_load_fw(client);			
	startup_chip(client);
	reset_chip(client);
	startup_chip(client);	
	return 1;
}

#ifdef TPD_PROXIMITY
/* if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL */

static int tpd_get_ps_value(void)
{
	return tpd_proximity_detect;//send to OS to controll backlight on/off
}

static int tpd_enable_ps(int enable)
{
	u8 buf[4];
	if (enable) {
		//wake_lock(&ps_lock);

		buf[3] = 0x00;
		buf[2] = 0x00;
		buf[1] = 0x00;
		buf[0] = 0x4;
		i2c_smbus_write_i2c_block_data(i2c_client, 0xf0, 4, buf);
		buf[3] = 0x0;
		buf[2] = 0x0;
		buf[1] = 0x0;
		buf[0] = 0x2;
		i2c_smbus_write_i2c_block_data(i2c_client, 0x00, 4, buf);
		tpd_proximity_flag = 1;
		//add alps of function
		printk("tpd-ps function is on\n");
	}
	else 
	{
		tpd_proximity_flag = 0;
		//wake_unlock(&ps_lock);

		buf[3] = 0x00;
		buf[2] = 0x00;
		buf[1] = 0x00;
		buf[0] = 0x4;

		i2c_smbus_write_i2c_block_data(i2c_client, 0xf0, 4, buf);
		buf[3] =0x0;
		buf[2] = 0x0;
		buf[1] = 0x0;
		buf[0] = 0x0;
		i2c_smbus_write_i2c_block_data(i2c_client, 0x00, 4, buf);	
		printk("tpd-ps function is off\n");
	}
	return 0;
}
static int ps_open_report_data(int open)
{
	/* should queuq work to report event if  is_report_input_direct=true */
	return 0;
}

/* if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL */
static bool tp_is_resume =0;
static bool enable_tp_ps =0;

static int ps_enable_nodata(int en)
{
	//return 0;
	//int res = 0;
#ifdef CUSTOM_KERNEL_SENSORHUB
	SCP_SENSOR_HUB_DATA req;
	int len;
#endif				/* #ifdef CUSTOM_KERNEL_SENSORHUB */

	if(tp_is_resume)
	{
		printk("gsl_obj ps enable value = %d\n", en);
		if (en) {
			tpd_enable_ps(1);
			set_bit(CMC_BIT_PS, &gsl_obj->enable);
		} else {
			tpd_enable_ps(0);
			clear_bit(CMC_BIT_PS, &gsl_obj->enable);
		}
	}
	else
	{
		enable_tp_ps=1;
		printk("tp is not resume,stop enable ps.\n");
	}
	return 0;

}

static int ps_set_delay(u64 ns)
{
	return 0;
}

static int ps_get_data(int *value, int *status)
{
//	int err = 0;
	/* #ifdef CUSTOM_KERNEL_SENSORHUB */
	//mutex_lock(&Ltr559_lock);
	*value = tpd_get_ps_value();
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	printk("gsl_obj ps_get_data *value = %d\n", *value);

//#endif				/* #ifdef CUSTOM_KERNEL_SENSORHUB */
	//mutex_unlock(&Ltr559_lock);
	return 0;
}

static int ps_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return 0;
}

static int ps_flush(void)
{
	return 0;
}


#endif

static void check_mem_data(struct i2c_client *client)
{
	u8 read_buf[4]  = {0};
	
	msleep(30);
	i2c_smbus_read_i2c_block_data(client,0xb0, sizeof(read_buf), read_buf);
	printk("#########check mem read 0xb0 = %x %x %x %x #########\n", read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
	if (read_buf[3] != 0x5a || read_buf[2] != 0x5a || read_buf[1] != 0x5a || read_buf[0] != 0x5a)
	{
		printk("#########check mem read 0xb0 = %x %x %x %x #########\n", read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
		init_chip(client);
	}
	i2c_smbus_read_i2c_block_data(client,0xbc, sizeof(read_buf), read_buf);
	printk("#########check mem read 0xbc = %x %x %x %x #########\n", read_buf[3], read_buf[2], read_buf[1], read_buf[0]);

	i2c_smbus_read_i2c_block_data(client,0xb4, sizeof(read_buf), read_buf);
	printk("#########check mem read 0xb4 = %x %x %x %x #########\n", read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
	msleep(20);
	i2c_smbus_read_i2c_block_data(client,0xb4, sizeof(read_buf), read_buf);
	printk("#########check mem read 0xb4 = %x %x %x %x #########\n", read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
	read_buf[3]  =0x0;
	read_buf[2]  = 0x0;
	read_buf[1]  = 0x0;
	read_buf[0]  = 0x3;
	i2c_smbus_write_i2c_block_data(client,0xf0, sizeof(read_buf), read_buf);
	msleep(20);
	i2c_smbus_read_i2c_block_data(client,0x0, sizeof(read_buf), read_buf);
	msleep(20);
	i2c_smbus_read_i2c_block_data(client,0x0, sizeof(read_buf), read_buf);
	printk("#########check mem read 0x30 = %x %x %x %x #########\n", read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
}

#ifdef GSL_GESTURE
void gsl_key_report(int key_value)
{
#if 1
	input_report_key(tpd->dev, key_value, 1);
	input_sync(tpd->dev);	
	input_report_key(tpd->dev, key_value, 0);
	input_sync(tpd->dev);
#else
	input_report_key(tpd->dev, KEY_POWER, 1);
	input_sync(tpd->dev);
	input_report_key(tpd->dev, KEY_POWER, 0);
	input_sync(tpd->dev);
#endif
}


static unsigned int gsl_read_oneframe_data(unsigned int *data,unsigned int addr,unsigned int len)
{
    u8 buf[4];
    int i;
    printk("---gsl_read_oneframe_data---\n");
#if 0
    for(i=0;i<len/2;i++){
        buf[0] = ((addr+i*8)/0x80)&0xff;
        buf[1] = (((addr+i*8)/0x80)>>8)&0xff;
        buf[2] = (((addr+i*8)/0x80)>>16)&0xff;
        buf[3] = (((addr+i*8)/0x80)>>24)&0xff;
        i2c_smbus_write_i2c_block_data(i2c_client,0xf0,4,buf);

		if(i2c_smbus_read_i2c_block_data(i2c_client,(addr+i*8)%0x80,8,(char *)&data[i*2]) < 0)
		{
			printk("the first i2c read failed and need read again!\n");	
			i2c_smbus_read_i2c_block_data(i2c_client,(addr+i*8)%0x80,8,(char *)&data[i*2]);
		}		
		 printk("data[%d] = 0x%08x\n", i, data[i]);
	    }
	
    if(len%2){
        buf[0] = ((addr+len*4 - 4)/0x80)&0xff;
        buf[1] = (((addr+len*4 - 4)/0x80)>>8)&0xff;
        buf[2] = (((addr+len*4 - 4)/0x80)>>16)&0xff;
        buf[3] = (((addr+len*4 - 4)/0x80)>>24)&0xff;
        i2c_smbus_write_i2c_block_data(i2c_client,0xf0,4,buf);
        i2c_smbus_read_i2c_block_data(i2c_client,(addr+len*4 - 4)%0x80,4,(char *)&data[len-1]);
        i2c_smbus_read_i2c_block_data(i2c_client,(addr+len*4 - 4)%0x80,4,(char *)&data[len-1]);
    }
#else	
	for(i=0;i<len;i++)
	{
		buf[0] = ((addr+i*4)/0x80)&0xff;
		buf[1] = (((addr+i*4)/0x80)>>8)&0xff;
		buf[2] = (((addr+i*4)/0x80)>>16)&0xff;
		buf[3] = (((addr+i*4)/0x80)>>24)&0xff;
		i2c_smbus_write_i2c_block_data(i2c_client, 0xf0, 4, buf);
		i2c_smbus_read_i2c_block_data(i2c_client, (((addr+i*4)%0x80+8)&0x5f), 4, (char *)&data[i]);
		i2c_smbus_read_i2c_block_data(i2c_client, (addr+i*4)%0x80, 4, (char *)&data[i]);
		i2c_smbus_read_i2c_block_data(i2c_client, (addr+i*4)%0x80, 4, (char *)&data[i]);
		//printk("----txp----20150416----data[%d] = 0x%08x\n", i, data[i]);
	}
#endif

    return len;
}


static ssize_t gsl_sysfs_tpgesture_show(struct device *dev,struct device_attribute *attr, char *buf)
{
    ssize_t len=0;
    sprintf(&buf[len],"%s\n","tp gesture is on/off:");
    len += (strlen("tp gesture is on/off:")+1);
    if(gsl_gesture_flag == 1){
        sprintf(&buf[len],"%s\n","  on  ");
        len += (strlen("  on  ")+1);
    }else if(gsl_gesture_flag == 0){
        sprintf(&buf[len],"%s\n","  off  ");
        len += (strlen("  off  ")+1);
    }

    sprintf(&buf[len],"%s\n","tp gesture:");
    len += (strlen("tp gesture:")+1);
    sprintf(&buf[len],"%c\n",gsl_gesture_c);
    len += 2;	
    return len;
}
//wuhao start
static ssize_t gsl_sysfs_tpgesturet_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
    char tmp_buf[16];

    if(copy_from_user(tmp_buf, buf, (count>16?16:count))){
        return -1;
    }
    if(buf[0] == '0'){
        //	gsl_gesture_flag = 0;
        printk("[GSL_GESTURE] gsl_sysfs_tpgesturet_store off.\n");
    }else if(buf[0] == '1'){
        gsl_gesture_flag = 1;
        printk("[GSL_GESTURE] gsl_sysfs_tpgesturet_store on.\n");
    }

    return count;
}
static DEVICE_ATTR(tpgesture, S_IRUGO|S_IWUSR, gsl_sysfs_tpgesture_show, gsl_sysfs_tpgesturet_store);
static void gsl_request_power_idev(void)
{
    struct input_dev *idev;
    int rc = 0;
    idev = input_allocate_device();


    if(!idev){
        return;
    }
    gsl_power_idev = idev;
    idev->name = "gsl_gesture";
    idev->id.bustype = BUS_I2C;
    input_set_capability(idev,EV_KEY,KEY_POWER);
    input_set_capability(idev,EV_KEY,KEY_END);

    rc = input_register_device(idev);
    if(rc){
        input_free_device(idev);
        gsl_power_idev = NULL;
    }
}
static unsigned int gsl_gesture_init(void)
{
    int ret;
    struct kobject *gsl_debug_kobj;
    gsl_debug_kobj = kobject_create_and_add("gsl_gesture", NULL) ;
    if (gsl_debug_kobj == NULL)
    {
        printk("%s: subsystem_register failed\n", __func__);
        return -ENOMEM;
    }
    ret = sysfs_create_file(gsl_debug_kobj, &dev_attr_tpgesture.attr);
    if (ret)
    {
        printk("%s: sysfs_create_version_file failed\n", __func__);
        return ret;
    }
    gsl_request_power_idev();
    printk("[GSL_GESTURE] gsl_gesture_init success.\n");
    return 1;
}

#endif

#if 0
#define GSL_CHIP_NAME	"gslx68x"
static ssize_t gsl_sysfs_version_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	//ssize_t len=0;
	u32 tmp;
	u8 buf_tmp[4];
	char *ptr = buf;
	ptr += sprintf(ptr,"sileadinc:");
	ptr += sprintf(ptr,GSL_CHIP_NAME);
#ifdef GSL_NOID_VERSION
	tmp = gsl_version_id();
	ptr += sprintf(ptr,":%08x:",tmp);
	//ptr += sprintf(ptr,"%08x:",
		//gsl_cfg_table[gsl_cfg_index].data_id[0]);
#endif
	buf_tmp[0]=0x3;buf_tmp[1]=0;buf_tmp[2]=0;buf_tmp[3]=0;
	i2c_smbus_write_i2c_block_data(i2c_client, 0xf0, 4, buf_tmp);
	i2c_smbus_read_i2c_block_data(i2c_client, 0x4, 4, buf_tmp);
		msleep(5);
	i2c_smbus_read_i2c_block_data(i2c_client, 0x4, 4, buf_tmp);
	ptr += sprintf(ptr,"%02x%02x%02x%02x\n",buf_tmp[3],buf_tmp[2],buf_tmp[1],buf_tmp[0]);
		
    	return (ptr-buf);
}

static DEVICE_ATTR(version, 0444, gsl_sysfs_version_show, NULL);
#endif
static struct attribute *gsl_attrs[] = {
	//&dev_attr_version.attr,
#ifdef GSL_GESTURE
	&dev_attr_gesture.attr,
#endif
	NULL
};
static const struct attribute_group gsl_attr_group = {
	.attrs = gsl_attrs,
};//hebufan
static unsigned int gsl_sysfs_init(void)
{
	int ret;
	struct kobject *gsl_debug_kobj;
	gsl_debug_kobj = kobject_create_and_add("gsl_touchscreen", NULL) ;
	if (gsl_debug_kobj == NULL)
	{
		printk("%s: subsystem_register failed\n", __func__);
		return -ENOMEM;
	}
#if 0
	ret = sysfs_create_file(gsl_debug_kobj, &dev_attr_version.attr);
	if (ret)
	{
		printk("%s: sysfs_create_version_file failed\n", __func__);
		return ret;
	}
#endif	
	#ifdef GSL_GESTURE
	ret = sysfs_create_file(gsl_debug_kobj, &dev_attr_gesture.attr);
	if (ret)
	{
		printk("%s: sysfs_create_gesture_file failed\n", __func__);
		return ret;
	}
	#endif
	return ret;
}


#ifdef TPD_PROC_DEBUG
static int char_to_int(char ch)
{
    if(ch>='0' && ch<='9')
        return (ch-'0');
    else
        return (ch-'a'+10);
}

//static int gsl_config_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
static int gsl_config_read_proc(struct seq_file *m,void *v)
{
	char temp_data[5] = {0};
	unsigned int tmp=0;
	
	if('v'==gsl_read[0]&&'s'==gsl_read[1])
	{
#ifdef GSL_NOID_VERSION
		tmp=gsl_version_id();
#else 
		tmp=0x20121215;
#endif
		seq_printf(m,"version:%x\n",tmp);
	}
	else if('r'==gsl_read[0]&&'e'==gsl_read[1])
	{
		if('i'==gsl_read[3])
		{
#ifdef GSL_NOID_VERSION 
			tmp=(gsl_data_proc[5]<<8) | gsl_data_proc[4];
			seq_printf(m,"gsl_config_data_id[%d] = ",tmp);
			if(tmp>=0&&tmp<512)
			seq_printf(m,"%d\n",gsl_config_data_id[tmp]); 
#endif
		}
		else 
		{
			i2c_smbus_write_i2c_block_data(i2c_client,0Xf0,4,&gsl_data_proc[4]);
			if(gsl_data_proc[0] < 0x80)
				i2c_smbus_read_i2c_block_data(i2c_client,gsl_data_proc[0],4,temp_data);
			i2c_smbus_read_i2c_block_data(i2c_client,gsl_data_proc[0],4,temp_data);
			seq_printf(m,"offset : {0x%02x,0x",gsl_data_proc[0]);
			seq_printf(m,"%02x",temp_data[3]);
			seq_printf(m,"%02x",temp_data[2]);
			seq_printf(m,"%02x",temp_data[1]);
			seq_printf(m,"%02x ,}\n",temp_data[0]);
		}
	}
	return 0;
}
static ssize_t gsl_config_write_proc(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	u8 buf[8] = {0};
	char temp_buf[CONFIG_LEN];
	char *path_buf;
	int tmp = 0;
	int tmp1 = 0;
	print_info("[tp-gsl][%s] \n",__func__);
	if(count > 512)
	{
        return -EFAULT;
	}
	path_buf=kzalloc(count,GFP_KERNEL);
	if(!path_buf)
	{
		printk("alloc path_buf memory error \n");
		return -1;
	}	
	//if(copy_from_user(path_buf, buffer, (count<CONFIG_LEN?count:CONFIG_LEN)))
	if(copy_from_user(path_buf, buffer, count))
	{
		print_info("copy from user fail\n");
		goto exit_write_proc_out;
	}
	memcpy(temp_buf,path_buf,(count<CONFIG_LEN?count:CONFIG_LEN));
	print_info("[tp-gsl][%s][%s]\n",__func__,temp_buf);
	
	buf[3]=char_to_int(temp_buf[14])<<4 | char_to_int(temp_buf[15]);	
	buf[2]=char_to_int(temp_buf[16])<<4 | char_to_int(temp_buf[17]);
	buf[1]=char_to_int(temp_buf[18])<<4 | char_to_int(temp_buf[19]);
	buf[0]=char_to_int(temp_buf[20])<<4 | char_to_int(temp_buf[21]);
	
	buf[7]=char_to_int(temp_buf[5])<<4 | char_to_int(temp_buf[6]);
	buf[6]=char_to_int(temp_buf[7])<<4 | char_to_int(temp_buf[8]);
	buf[5]=char_to_int(temp_buf[9])<<4 | char_to_int(temp_buf[10]);
	buf[4]=char_to_int(temp_buf[11])<<4 | char_to_int(temp_buf[12]);
	if('v'==temp_buf[0]&& 's'==temp_buf[1])//version //vs
	{
		memcpy(gsl_read,temp_buf,4);
		printk("gsl version\n");
	}
	else if('s'==temp_buf[0]&& 't'==temp_buf[1])//start //st
	{
		gsl_proc_flag = 1;
		reset_chip(i2c_client);
	}
	else if('e'==temp_buf[0]&&'n'==temp_buf[1])//end //en
	{
		msleep(20);
		reset_chip(i2c_client);
		startup_chip(i2c_client);
		gsl_proc_flag = 0;
	}
	else if('r'==temp_buf[0]&&'e'==temp_buf[1])//read buf //
	{
		memcpy(gsl_read,temp_buf,4);
		memcpy(gsl_data_proc,buf,8);
	}
	else if('w'==temp_buf[0]&&'r'==temp_buf[1])//write buf
	{
		i2c_smbus_write_i2c_block_data(i2c_client,buf[4],4,buf);
	}
#ifdef GSL_NOID_VERSION
	else if('i'==temp_buf[0]&&'d'==temp_buf[1])//write id config //
	{
		tmp1=(buf[7]<<24)|(buf[6]<<16)|(buf[5]<<8)|buf[4];
		tmp=(buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|buf[0];
		if(tmp1>=0 && tmp1<512)
		{
			gsl_config_data_id[tmp1] = tmp;
		}
	}
#endif
exit_write_proc_out:
	kfree(path_buf);
	return count;
}
static int gsl_server_list_open(struct inode *inode,struct file *file)
{
	return single_open(file,gsl_config_read_proc,NULL);
}
static const struct file_operations gsl_seq_fops = {
	.open = gsl_server_list_open,
	.read = seq_read,
	//.release = single_release,
	.write = gsl_config_write_proc,
	.owner = THIS_MODULE,
};
#endif


#ifdef FILTER_POINT
static void filter_point(u16 x, u16 y , u8 id)
{
	u16 x_err =0;
	u16 y_err =0;
	u16 filter_step_x = 0, filter_step_y = 0;
	
	id_sign[id] = id_sign[id] + 1;
	if(id_sign[id] == 1)
	{
		x_old[id] = x;
		y_old[id] = y;
	}
	
	x_err = x > x_old[id] ? (x -x_old[id]) : (x_old[id] - x);
	y_err = y > y_old[id] ? (y -y_old[id]) : (y_old[id] - y);

	if( (x_err > FILTER_MAX && y_err > FILTER_MAX/3) || (x_err > FILTER_MAX/3 && y_err > FILTER_MAX) )
	{
		filter_step_x = x_err;
		filter_step_y = y_err;
	}
	else
	{
		if(x_err > FILTER_MAX)
			filter_step_x = x_err; 
		if(y_err> FILTER_MAX)
			filter_step_y = y_err;
	}

	if(x_err <= 2*FILTER_MAX && y_err <= 2*FILTER_MAX)
	{
		filter_step_x >>= 2; 
		filter_step_y >>= 2;
	}
	else if(x_err <= 3*FILTER_MAX && y_err <= 3*FILTER_MAX)
	{
		filter_step_x >>= 1; 
		filter_step_y >>= 1;
	}	
	else if(x_err <= 4*FILTER_MAX && y_err <= 4*FILTER_MAX)
	{
		filter_step_x = filter_step_x*3/4; 
		filter_step_y = filter_step_y*3/4;
	}	
	
	x_new = x > x_old[id] ? (x_old[id] + filter_step_x) : (x_old[id] - filter_step_x);
	y_new = y > y_old[id] ? (y_old[id] + filter_step_y) : (y_old[id] - filter_step_y);

	x_old[id] = x_new;
	y_old[id] = y_new;
}
#else

static void record_point(u16 x, u16 y , u8 id)
{
	u16 x_err =0;
	u16 y_err =0;

	id_sign[id]=id_sign[id]+1;
	
	if(id_sign[id]==1){
		x_old[id]=x;
		y_old[id]=y;
	}

	x = (x_old[id] + x)/2;
	y = (y_old[id] + y)/2;
		
	if(x>x_old[id]){
		x_err=x -x_old[id];
	}
	else{
		x_err=x_old[id]-x;
	}

	if(y>y_old[id]){
		y_err=y -y_old[id];
	}
	else{
		y_err=y_old[id]-y;
	}

	if( (x_err > 3 && y_err > 1) || (x_err > 1 && y_err > 3) ){
		x_new = x;     x_old[id] = x;
		y_new = y;     y_old[id] = y;
	}
	else{
		if(x_err > 3){
			x_new = x;     x_old[id] = x;
		}
		else
			x_new = x_old[id];
		if(y_err> 3){
			y_new = y;     y_old[id] = y;
		}
		else
			y_new = y_old[id];
	}

	if(id_sign[id]==1){
		x_new= x_old[id];
		y_new= y_old[id];
	}
	
}
#endif

void tpd_down( int id, int x, int y, int p) 
{
	print_info("------tpd_down id: %d, x:%d, y:%d------ \n", id, x, y);
     //  luyuan input_mt_slot(tpd->dev, id);
	//luyuan input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER,true);
	input_report_key(tpd->dev, BTN_TOUCH, 1);
	input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id); 	//luyuan
	input_mt_sync(tpd->dev);//luyuan

	/*if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
	{   
	#ifdef TPD_HAVE_BUTTON 
	 print_info("------tpd_down id: %d, x:%d, y:%d---anjian by tony--- \n", id, x, y);
	 printk("------tpd_down id: %d, x:%d, y:%d---m706 by tony--- \n", id, x, y); 
		tpd_button(x, y, 1);  
	#endif
	}*/
}

void tpd_up(void) 
{
	print_info("------tpd_up------ \n");

	input_report_key(tpd->dev, BTN_TOUCH, 0);
	input_mt_sync(tpd->dev);
	
	/*if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
	{   
	#ifdef TPD_HAVE_BUTTON 
	    tpd_button(0, 0, 0);  
	#endif
	}*/
}

static void report_data_handle(void)
{
	u8 touch_data[MAX_FINGERS * 4 + 4] = {0};
	u8 buf[4] = {0};
	char point_num = 0;
	unsigned int x, y, temp_a, temp_b, i, id;

#ifdef GSL_NOID_VERSION
	struct gsl_touch_info cinfo;
	int tmp1 = 0;
#endif
		
#ifdef TPD_PROXIMITY
		//int err;
		struct hwm_sensor_data sensor_data;
	
	if (tpd_proximity_flag == 1)
        {
                i2c_smbus_read_i2c_block_data(i2c_client,0xac,4,buf);
                print_info("gslX680   buf[0] = %d buf[1] = %d,  buf[2] = %d  buf[3] = %d \n",buf[0],buf[1],buf[2],buf[3]);

                if (buf[0] == 1 && buf[1] == 0 && buf[2] == 0 && buf[3] == 0)
                {
                    tpd_proximity_detect = 0;
                }
                else
                {
                    tpd_proximity_detect = 1;
                }
                print_info("gslX680    ps change   tpd_proximity_detect = %d  \n",tpd_proximity_detect);
                //map and store data to hwm_sensor_data
                sensor_data.values[0] = tpd_get_ps_value();
                sensor_data.value_divide = 1;
                sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
                //let up layer to know
                if(ps_report_interrupt_data(tpd_get_ps_value()))
                {
                    print_info("call hwmsen_get_interrupt_data fail \n");
                }
            
        }
#endif

#ifdef GSL_MONITOR
	if(i2c_lock_flag != 0)
		return;
	else
		i2c_lock_flag = 1;
#endif

	
#ifdef TPD_PROC_DEBUG
    if(gsl_proc_flag == 1)
        return;
#endif

	i2c_smbus_read_i2c_block_data(i2c_client, 0x80, 4, &touch_data[0]);
	point_num = touch_data[0];
	if(point_num > 0)
		i2c_smbus_read_i2c_block_data(i2c_client, 0x84, 8, &touch_data[4]);
	if(point_num > 2)
		i2c_smbus_read_i2c_block_data(i2c_client, 0x8c, 8, &touch_data[12]);
	if(point_num > 4)
		i2c_smbus_read_i2c_block_data(i2c_client, 0x94, 8, &touch_data[20]);
	if(point_num > 6)
		i2c_smbus_read_i2c_block_data(i2c_client, 0x9c, 8, &touch_data[28]);
	if(point_num > 8)
		i2c_smbus_read_i2c_block_data(i2c_client, 0xa4, 8, &touch_data[36]);
	
#ifdef GSL_NOID_VERSION
	cinfo.finger_num = point_num;
	print_info("tp-gsl  finger_num = %d\n",cinfo.finger_num);
	for(i = 0; i < (point_num < MAX_CONTACTS ? point_num : MAX_CONTACTS); i ++)
	{
		temp_a = touch_data[(i + 1) * 4 + 3] & 0x0f;
		temp_b = touch_data[(i + 1) * 4 + 2];
		cinfo.x[i] = temp_a << 8 |temp_b;
		temp_a = touch_data[(i + 1) * 4 + 1];
		temp_b = touch_data[(i + 1) * 4 + 0];
		cinfo.y[i] = temp_a << 8 |temp_b;		
		cinfo.id[i] = ((touch_data[(i + 1) * 4 + 3] & 0xf0)>>4);
		print_info("tp-gsl  before: x[%d] = %d, y[%d] = %d, id[%d] = %d \n",i,cinfo.x[i],i,cinfo.y[i],i,cinfo.id[i]);
	}
	cinfo.finger_num = (touch_data[3]<<24)|(touch_data[2]<<16)|
		(touch_data[1]<<8)|touch_data[0];
	gsl_alg_id_main(&cinfo);
	tmp1=gsl_mask_tiaoping();
	print_info("[tp-gsl] tmp1=%x\n",tmp1);
	if(tmp1>0&&tmp1<0xffffffff)
	{
		buf[0]=0xa;buf[1]=0;buf[2]=0;buf[3]=0;
		i2c_smbus_write_i2c_block_data(i2c_client,0xf0,4,buf);
		buf[0]=(u8)(tmp1 & 0xff);
		buf[1]=(u8)((tmp1>>8) & 0xff);
		buf[2]=(u8)((tmp1>>16) & 0xff);
		buf[3]=(u8)((tmp1>>24) & 0xff);
		print_info("tmp1=%08x,buf[0]=%02x,buf[1]=%02x,buf[2]=%02x,buf[3]=%02x\n",
			tmp1,buf[0],buf[1],buf[2],buf[3]);
		i2c_smbus_write_i2c_block_data(i2c_client,0x8,4,buf);
	}
	point_num = cinfo.finger_num;
#endif

#ifdef GSL_GESTURE
    //printk("[GSL_GESTURE] tpd_halt =%d, gsl_gesture_flag = %d \n",tpd_halt, gsl_gesture_flag);
    if(gsl_gesture_flag == 1){
        int tmp_c;
        tmp_c = gsl_obtain_gesture();
        //printk("[GSL_GESTURE] tmp_c =%d \n",tmp_c);
        if(tpd_halt == 1) 
        {
            switch(tmp_c){
                case (int)'C':
                case (int)'E':
                case (int)'W':
                case (int)'O':
				case (int)'S':
                case (int)'M':
                case (int)'Z':
                case (int)'V':
                case (int)'*':
                case 0xa1fb:     //left
                case 0xa1fd:     //down
                case 0xa1fc:     //UP
                case 0xa1fa:     //right
                    gsl_gesture_c = (char)tmp_c;
                    //printk("gesture,gsl_lcd_flag=%d\n",gsl_lcd_flag);
                    if(0 == gsl_lcd_flag)
                    {
                    	#if 1//
						if((int)'C' == tmp_c)
							gsl_key_report(KEY_C);
						else if((int)'E' == tmp_c)
							gsl_key_report(KEY_E);
						else if((int)'W' == tmp_c)
							gsl_key_report(KEY_W);
						else if((int)'O' == tmp_c)
							gsl_key_report(KEY_O);
						else if((int)'S' == tmp_c)
							gsl_key_report(KEY_S);
						else if((int)'M' == tmp_c)
							gsl_key_report(KEY_M);
						else if((int)'Z' == tmp_c)
							gsl_key_report(KEY_Z);
						else if((int)'V' == tmp_c)
							gsl_key_report(KEY_V);
						else if((int)'*' == tmp_c)
							gsl_key_report(KEY_I);
						else if(0xa1fb == tmp_c)
							gsl_key_report(KEY_F);
						else if(0xa1fd == tmp_c)
							gsl_key_report(KEY_L);
						else if(0xa1fc == tmp_c)
							gsl_key_report(KEY_K);
						else if(0xa1fa == tmp_c)
							gsl_key_report(KEY_R);
						#else
                        printk("[GSL_GESTURE] input report KEY_POWER\n");
                        input_report_key(gsl_power_idev,KEY_POWER,1);
                        input_sync(gsl_power_idev);
                        input_report_key(gsl_power_idev,KEY_POWER,0);
                        input_sync(gsl_power_idev);
						#endif
                    }
                    //printk("[GSL_GESTURE] set gsl_lcd_flag = 1\n");
                    //gsl_lcd_flag = 1;			
                    break;
                default:
                    break;
            }
            goto i2c_lock;
        }
        
    }
#endif
	for(i = 1 ;i <= MAX_CONTACTS; i ++)
	{
		if(point_num == 0)
			id_sign[i] = 0;	
		id_state_flag[i] = 0;
	}
	for(i = 0; i < (point_num < MAX_FINGERS ? point_num : MAX_FINGERS); i ++)
	{
	#ifdef GSL_NOID_VERSION
		id = cinfo.id[i];
		x =  cinfo.x[i];
		y =  cinfo.y[i];
	#else
		id = touch_data[(i + 1) * 4 + 3] >> 4;
		temp_a = touch_data[(i + 1) * 4 + 3] & 0x0f;
		temp_b = touch_data[(i + 1) * 4 + 2];
		x = temp_a << 8 |temp_b;
		temp_a = touch_data[(i + 1) * 4 + 1];
		temp_b = touch_data[(i + 1) * 4 + 0];
		y = temp_a << 8 |temp_b;	
	#endif
	
		if(1 <= id && id <= MAX_CONTACTS)
		{
		#ifdef FILTER_POINT
			filter_point(x, y ,id);
		#else
			record_point(x, y , id);
		#endif
		//	tpd_down(id, x_new, y_new, 10);
			tpd_down(id, 600-y_new, x_new, 10);
			id_state_flag[id] = 1;
		}
	}
	for(i = 1; i <= MAX_CONTACTS; i ++)
	{	
		if( (0 == point_num) || ((0 != id_state_old_flag[i]) && (0 == id_state_flag[i])) )
		{
			id_sign[i]=0;
		}
		id_state_old_flag[i] = id_state_flag[i];
	}			
	if(0 == point_num)
	{

		tpd_up();
	}
	input_sync(tpd->dev);
#ifdef GSL_GESTURE	
i2c_lock:
	;
#endif	
#ifdef GSL_MONITOR
	i2c_lock_flag = 0;
#endif
}

#ifdef GSL_MONITOR
static void gsl_monitor_worker(void)
{
	u8 write_buf[4] = {0};
	u8 read_buf[4]  = {0};
	char init_chip_flag = 0;
	
	print_info("----------------gsl_monitor_worker-----------------\n");	
#ifdef TPD_PROC_DEBUG   
	if(1 == gsl_proc_flag)
		goto queue_monitor_work;
#endif

#ifdef GSL_GESTURE
	if((1 == gsl_gesture_flag)&&(0 == gsl_lcd_flag))
		goto queue_monitor_work;
#endif
	
	if(i2c_lock_flag != 0)
		goto queue_monitor_work;
	else
		i2c_lock_flag = 1;
	
	i2c_smbus_read_i2c_block_data(i2c_client, 0xb0, 4, read_buf);
	if(read_buf[3] != 0x5a || read_buf[2] != 0x5a || read_buf[1] != 0x5a || read_buf[0] != 0x5a)
		b0_counter ++;
	else
		b0_counter = 0;

	if(b0_counter > 1)
	{
		printk("======read 0xb0: %x %x %x %x ======\n",read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
		init_chip_flag = 1;
		b0_counter = 0;
		goto queue_monitor_init_chip;
	}

	i2c_smbus_read_i2c_block_data(i2c_client, 0xb4, 4, read_buf);	
	
	int_2nd[3] = int_1st[3];
	int_2nd[2] = int_1st[2];
	int_2nd[1] = int_1st[1];
	int_2nd[0] = int_1st[0];
	int_1st[3] = read_buf[3];
	int_1st[2] = read_buf[2];
	int_1st[1] = read_buf[1];
	int_1st[0] = read_buf[0];

	if (int_1st[3] == int_2nd[3] && int_1st[2] == int_2nd[2] &&int_1st[1] == int_2nd[1] && int_1st[0] == int_2nd[0]) 
	{
		printk("======int_1st: %x %x %x %x , int_2nd: %x %x %x %x ======\n",int_1st[3], int_1st[2], int_1st[1], int_1st[0], int_2nd[3], int_2nd[2],int_2nd[1],int_2nd[0]);
		init_chip_flag = 1;
		goto queue_monitor_init_chip;
	}
#if 1 //version 1.4.0 or later than 1.4.0 read 0xbc for esd checking
	i2c_smbus_read_i2c_block_data(i2c_client, 0xbc, 4, read_buf);
	if(read_buf[3] != 0 || read_buf[2] != 0 || read_buf[1] != 0 || read_buf[0] != 0)
		bc_counter++;
	else
		bc_counter = 0;
	if(bc_counter > 1)
	{
		printk("======read 0xbc: %x %x %x %x======\n",read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
		init_chip_flag = 1;
		bc_counter = 0;
	}
#else
	write_buf[3] = 0x01;
	write_buf[2] = 0xfe;
	write_buf[1] = 0x10;
	write_buf[0] = 0x00;
	i2c_smbus_write_i2c_block_data(i2c_client, 0xf0, 4, write_buf);
	i2c_smbus_read_i2c_block_data(i2c_client, 0x10, 4, read_buf);
	i2c_smbus_read_i2c_block_data(i2c_client, 0x10, 4, read_buf);
	
	if(read_buf[3] < 10 && read_buf[2] < 10 && read_buf[1] < 10 && read_buf[0] < 10)
		dac_counter ++;
	else
		dac_counter = 0;

	if(dac_counter > 1) 
	{
		printk("======read DAC1_0: %x %x %x %x ======\n",read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
		init_chip_flag = 1;
		dac_counter = 0;
	}
#endif
queue_monitor_init_chip:
	if(init_chip_flag)
		init_chip(i2c_client);
	//	tpd_enable_ps(1);
	i2c_lock_flag = 0;
	
queue_monitor_work:	
	queue_delayed_work(gsl_monitor_workqueue, &gsl_monitor_work, 100);
}
#endif

static int touch_event_handler(void *unused)
{
	struct sched_param param = { .sched_priority = 4 };
	sched_setscheduler(current, SCHED_RR, &param);
	
	do
	{
		//enable_irq(gsl_touch_irq);
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, tpd_flag != 0);
		tpd_flag = 0;
		TPD_DEBUG_SET_TIME;
		set_current_state(TASK_RUNNING);
		print_info("===touch_event_handler, task running===\n");

		eint_flag = 0;
		report_data_handle();
		
	} while (!kthread_should_stop());
	
	return 0;
}

 static irqreturn_t tpd_eint_interrupt_handler(int irq, void *dev_id)
{
	print_info("gsl------------------===tpd irq interrupt===\n");

	eint_flag = 1;
	tpd_flag=1; 
	wake_up_interruptible(&waiter);
	return IRQ_HANDLED;
}

static int tpd_irq_registration(void)
{
	struct device_node *node = NULL;
	int ret = 0;

	node = of_find_compatible_node(NULL, NULL, "GSL3670");
	if (node) {
		/*touch_irq = gpio_to_irq(tpd_int_gpio_number);*/
		gsl_touch_irq = irq_of_parse_and_map(node, 0);
		ret = request_irq(gsl_touch_irq, tpd_eint_interrupt_handler,
					IRQF_TRIGGER_FALLING, GSL3670_NAME, NULL);
		printk("tpd touch_irq = %d",gsl_touch_irq);
		if (ret > 0)
			TPD_DMESG("tpd request_irq IRQ LINE NOT AVAILABLE!.");
	} else {
		TPD_DMESG("[%s] tpd request_irq can not find touch eint device node!.", __func__);
	}
	return 0;
}


static int tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {
	int err = 0;
	int ret = 0;
	u8 gls_version[4]={0};
	//char buffer[5];
	//int status=0;
//	int  gsl_key_set=0;
#ifdef TPD_PROXIMITY
	struct gsl_priv *obj;
	//struct hwmsen_object obj_ps;
	//struct als_control_path als_ctl = { 0 };
	//struct als_data_path als_data = { 0 };
	struct ps_control_path ps_ctl = { 0 };
	struct ps_data_path ps_data = { 0 };
		obj = kzalloc(sizeof(*obj), GFP_KERNEL);
		gsl_obj = obj;

#endif
	
	printk("==tpd_i2c_probe==\n");

	of_get_gsl6730_platform_data(&client->dev);
	
	err = gpio_request_one(tpd_rst_gpio_number, GPIOF_OUT_INIT_LOW,
				 "touchp_reset");

	if (err < 0) {
		printk("Unable to request gpio reset_pin\n");
		return -1;
	}
	
	gpio_direction_output(tpd_rst_gpio_number,0);//GSL_TP_GPIO_OUTPUT(GTP_RST_PORT, 0);
	msleep(100);

	
	err = gpio_request_one(tpd_int_gpio_number, GPIOF_IN,
					 "tpd_int");
		if (err < 0) {
			printk("Unable to request gpio int_pin\n");
			gpio_free(tpd_rst_gpio_number);
			return -1;
		}

	gpio_direction_output(tpd_rst_gpio_number,1);//GSL_TP_GPIO_OUTPUT(GTP_RST_PORT, 1);
	msleep(50);
	gpio_direction_input(tpd_int_gpio_number);//GSL_TP_GPIO_AS_INT(GTP_INT_PORT);

	i2c_client = client;	
  	ret = init_chip(i2c_client);
	if(ret < 0)
	{
		printk("gslx680 init error,exit!!");
		//return -1;
		goto out;
	}
	check_mem_data(i2c_client);
	
	//ret=regulator_enable(tpd->reg);  //enable regulator
	//if (ret)
	//	TPD_DMESG("regulator_enable() failed!\n");
	
	tpd_irq_registration();
	disable_irq(gsl_touch_irq);
	
	tpd_load_status = 1;
	thread = kthread_run(touch_event_handler, 0, GSL3670_NAME);
	if (IS_ERR(thread)) {
		err = PTR_ERR(thread);
		TPD_DMESG(GSL3670_NAME " failed to create kernel thread: %d\n", err);
	}

#ifdef GSL_TEST_TP
	create_ctp_proc();
#endif

#ifdef GSL_GESTURE
	for(gsl_key_set=0;gsl_key_set<GSL_TPD_GES_COUNT;gsl_key_set++)	
	input_set_capability(tpd->dev, EV_KEY,  tpd_keys_gesture[gsl_key_set]);
    gsl_FunIICRead(gsl_read_oneframe_data);
    gsl_GestureExternInt(gsl_model_extern,sizeof(gsl_model_extern)/sizeof(unsigned int)/18);
#endif

#ifdef GSL_MONITOR
	printk( "tpd_i2c_probe () : queue gsl_monitor_workqueue\n");

	INIT_DELAYED_WORK(&gsl_monitor_work, gsl_monitor_worker);
	gsl_monitor_workqueue = create_singlethread_workqueue("gsl_monitor_workqueue");
	queue_delayed_work(gsl_monitor_workqueue, &gsl_monitor_work, 1000);
#endif

#ifdef TPD_PROC_DEBUG
	#if 0
		gsl_config_proc = create_proc_entry(GSL_CONFIG_PROC_FILE, 0666, NULL);
		if (gsl_config_proc == NULL)
		{
			print_info("create_proc_entry %s failed\n", GSL_CONFIG_PROC_FILE);
		}
		else
		{
			gsl_config_proc->read_proc = gsl_config_read_proc;
			gsl_config_proc->write_proc = gsl_config_write_proc;
		}
	#else
	    proc_create(GSL_CONFIG_PROC_FILE,0666,NULL,&gsl_seq_fops);
	#endif	
	gsl_proc_flag = 0;
#endif
#ifdef TPD_PROXIMITY

       gsl_obj = kzalloc(sizeof(*gsl_obj), GFP_KERNEL);

	//wake_lock_init(&ps_lock, WAKE_LOCK_SUSPEND, "ps wakelock");  
	ps_ctl.is_use_common_factory = false;
	ps_ctl.open_report_data = ps_open_report_data;
	ps_ctl.enable_nodata = ps_enable_nodata;
	ps_ctl.set_delay = ps_set_delay;
	ps_ctl.batch = ps_batch;
	ps_ctl.flush = ps_flush;
	ps_ctl.is_report_input_direct = false;
	ps_ctl.is_support_batch = false;
	err = ps_register_control_path(&ps_ctl);
	if (err) {
		printk("register fail = %d\n", err);
		//goto exit_sensor_obj_attach_fail;
	}

	ps_data.get_data = ps_get_data;
	ps_data.vender_div = 100;
	err = ps_register_data_path(&ps_data);
	if (err) {
		printk("tregister fail = %d\n", err);
		//goto exit_sensor_obj_attach_fail;
	}
	//tpd_enable_ps(1);
#endif
#ifdef GSL_GESTURE
    gsl_gesture_init();
#endif
    gsl_sysfs_init();
	printk("==tpd_i2c_probe end==\n");
	enable_irq(gsl_touch_irq);
	
	gls_version[0]=0x3;
	gls_version[1]=0x0;
	gls_version[2]=0x0;
	gls_version[3]=0x0;

	ret=i2c_smbus_write_i2c_block_data(i2c_client, 0xf0, 4, gls_version);
	

	ret=i2c_smbus_read_i2c_block_data(i2c_client, 0x04, 4, gls_version);
	if(ret< 0)
		printk("Unable to read gsl 0x34 reg\n");


	/************check diffrent lcd vendor******************/
		if(lcd_product == 2)
		sprintf(tp_string_version,"XINYUAN,GSL3670,V%02x%02x%02x%02x",gls_version[3],gls_version[2],gls_version[1],gls_version[0]);
		if(lcd_product == 1)
		sprintf(tp_string_version,"TONGXINGDA,GSL3670,V%02x%02x%02x%02x",gls_version[3],gls_version[2],gls_version[1],gls_version[0]);
		if(lcd_product == 0)
		sprintf(tp_string_version,"NONE,GSL3670,V%02x%02x%02x%02x",gls_version[3],gls_version[2],gls_version[1],gls_version[0]);
	
    hardwareinfo_set_prop(HARDWARE_TP, tp_string_version);
	tp_is_resume=1;
	return 0;
	out:
	printk("goto out for release source\n");	
	ret = regulator_disable(tpd->reg);
	if (ret != 0)
		TPD_DMESG("Failed to disable reg-vgp6: %d\n", ret);
    regulator_put(tpd->reg);
	
    gpio_free(tpd_int_gpio_number);
    gpio_free(tpd_rst_gpio_number);
	return -1;
}

static int tpd_i2c_remove(struct i2c_client *client)
{
	printk("==tpd_i2c_remove==\n");
	gpio_free(tpd_rst_gpio_number);
	gpio_free(tpd_int_gpio_number);
	return 0;
}


static const struct i2c_device_id tpd_i2c_id[] = {{GSL3670_NAME,0},{}};
#ifdef ADD_I2C_DEVICE_ANDROID_4_0
static struct i2c_board_info __initdata gslX680_i2c_tpd={ I2C_BOARD_INFO(GSL3670_NAME, (GSLX680_ADDR))};
#else
static unsigned short force[] = {0, (GSLX680_ADDR << 1), I2C_CLIENT_END,I2C_CLIENT_END};
static const unsigned short * const forces[] = { force, NULL };
//static struct i2c_client_address_data addr_data = { .forces = forces,};
#endif
#if defined(CONFIG_KST_ANDROID_SDK_6_TP_COMPATIBLE_GSL)
static const struct of_device_id tpd_of_match[] = {
	{.compatible = "mediatek,cap_touch_gsl"},
	{},
};
#else
static const struct of_device_id gsl3670_dt_match[] = {
	{.compatible = "GSL3670"},
	{},
};
#endif

MODULE_DEVICE_TABLE(of, gsl3670_dt_match);

struct i2c_driver tpd_i2c_driver = {
	.driver = {
		.name =  GSL3670_NAME,
		.of_match_table = gsl3670_dt_match,
	#ifndef ADD_I2C_DEVICE_ANDROID_4_0	 
		.owner = THIS_MODULE,
	#endif
	},
	.probe = tpd_i2c_probe,
	.remove = tpd_i2c_remove,
	.id_table = tpd_i2c_id,
	//#ifndef ADD_I2C_DEVICE_ANDROID_4_0
	.address_list = (const unsigned short *)forces,
	//#endif
};

#ifdef CONFIG_OF
static int of_get_gsl6730_platform_data(struct device *dev)
{
	/*int ret, num;*/

	if (dev->of_node) {
		const struct of_device_id *match;

		match = of_match_device(of_match_ptr(gsl3670_dt_match), dev);
		if (!match) {
			printk("Error: No device match found\n");
			return -ENODEV;
		}
	}

	tpd_rst_gpio_number = of_get_named_gpio(dev->of_node, "reset-gpio", 0);
	tpd_int_gpio_number = of_get_named_gpio(dev->of_node, "irq-gpio", 0);
	/*ret = of_property_read_u32(dev->of_node, "rst-gpio", &num);
	if (!ret)
		tpd_rst_gpio_number = num;
	ret = of_property_read_u32(dev->of_node, "int-gpio", &num);
	if (!ret)
		tpd_int_gpio_number = num;
 	 */
	printk("%s get tpd_rst_gpio_number %d\n", __FUNCTION__,tpd_rst_gpio_number);
	printk("%s get tpd_int_gpio_number  %d\n", __FUNCTION__,tpd_int_gpio_number);
	return 0;
}
#else
static int of_get_gsl6730_platform_data(struct device *dev)
{
	return 0;
}
#endif



int tpd_local_init(void)
{
	int retval;
	
	printk("==tpd_local_init==\n");
	//boot_mode = get_boot_mode();
	
	tpd->reg = regulator_get(tpd->tpd_dev, "vtouch");
	retval = regulator_set_voltage(tpd->reg, 2800000, 2800000);

	if (retval != 0) {
		TPD_DMESG("Failed to set reg-vgp6 voltage: %d\n", retval);
		return -1;
	}


	retval = regulator_enable(tpd->reg);
	if (retval != 0)
		TPD_DMESG("Failed to enable reg-vgp6: %d\n", retval);

	if(i2c_add_driver(&tpd_i2c_driver)!=0) {
		TPD_DMESG("unable to add i2c driver.\n");
		return -1;
	}
	/*
	if(tpd_load_status == 0)
	{
		TPD_DMESG("add error touch panel driver.\n");
		i2c_del_driver(&tpd_i2c_driver);
		return -1;
	}
	*/
//#ifdef TPD_HAVE_BUTTON
//	tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
//#endif
	if (tpd_dts_data.use_tpd_button) {
		tpd_button_setting(tpd_dts_data.tpd_key_num, tpd_dts_data.tpd_key_local,
		tpd_dts_data.tpd_key_dim_local);
	}

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
	TPD_DO_WARP = 1;
	memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT*4);
	memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT*4);
#endif

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
	memcpy(tpd_calmat, tpd_calmat_local, 8*4);
	memcpy(tpd_def_calmat, tpd_def_calmat_local, 8*4);
#endif
	tpd_type_cap = 1;

	printk("==tpd_local_init end==\n");
	return 0;
}

/* Function to manage low power suspend */
void tpd_suspend(struct device *h)
{
	u8 read_buf[4] = {0};
	
	printk("==tpd_suspend==\n");
	//printk("==TPD_PROXIMITY== %d \n",tpd_proximity_flag);
	
	
		read_buf[3] = 0x00;
		read_buf[2] = 0x00;
		read_buf[1] = 0x00;
		read_buf[0] = 0x4;
		i2c_smbus_write_i2c_block_data(i2c_client, 0xf0, 4, read_buf);
		i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 4, read_buf);
	printk("======read 0x444440: %x %x %x %x ======\n",read_buf[3], read_buf[2], read_buf[1], read_buf[0]);		
#ifdef TPD_PROXIMITY
	if (tpd_proximity_flag == 1)
	{
		return;
	}
#endif
	tpd_halt = 1;
#ifdef GSL_GESTURE
    if(gsl_gesture_flag == 1){
        u8 buf[4];
        //unsigned int temp;
        gsl_lcd_flag = 0;
        msleep(10);
        buf[0] = 0xa;
        buf[1] = 0;
        buf[2] = 0;
        buf[3] = 0;
        i2c_smbus_write_i2c_block_data(i2c_client,0xf0,4,buf);
        buf[0] = 0;
        buf[1] = 0;
        buf[2] = 0x1;
        buf[3] = 0x5a;
        i2c_smbus_write_i2c_block_data(i2c_client,0x8,4,buf);
        msleep(10);
        return;
    }
#endif

	disable_irq(gsl_touch_irq);
#ifdef GSL_MONITOR
	printk( "gsl_ts_suspend () : cancel gsl_monitor_work\n");
	cancel_delayed_work_sync(&gsl_monitor_work);
#endif
	gpio_direction_output(tpd_rst_gpio_number,0);//GSL_TP_GPIO_OUTPUT(GTP_RST_PORT, 0);
	tp_is_resume=0;
}

/* Function to manage power-on resume */
void tpd_resume(struct device *h)
{
	u8 read_buf[4] = {0};
		
	printk("==tpd_resume==\n");
		//printk("==TPD_PROXIMITY== %d \n",tpd_proximity_flag);
#ifdef GSL_GESTURE
    if(gsl_gesture_flag == 1){
        u8 buf[4];
        //unsigned int temp;
        gpio_direction_output(tpd_rst_gpio_number,0);//GSL_TP_GPIO_OUTPUT(GTP_RST_PORT, 0);
        msleep(20);
        gpio_direction_output(tpd_rst_gpio_number,1);//GSL_TP_GPIO_OUTPUT(GTP_RST_PORT, 1);
        msleep(5);
        buf[0] = 0xa;
        buf[1] = 0;
        buf[2] = 0;
        buf[3] = 0;
        i2c_smbus_write_i2c_block_data(i2c_client,0xf0,4,buf);
        buf[0] = 0;
        buf[1] = 0;
        buf[2] = 0;
        buf[3] = 0x5a;
        i2c_smbus_write_i2c_block_data(i2c_client,0x8,4,buf);
        msleep(5);		
    }
#endif

	gpio_direction_output(tpd_rst_gpio_number,1);//GSL_TP_GPIO_OUTPUT(GTP_RST_PORT, 1);
	msleep(20);	
	reset_chip(i2c_client);
	startup_chip(i2c_client);
	check_mem_data(i2c_client);	
#ifdef GSL_MONITOR
	printk( "gsl_ts_resume () : queue gsl_monitor_work\n");
	queue_delayed_work(gsl_monitor_workqueue, &gsl_monitor_work, 300);
#endif	
	enable_irq(gsl_touch_irq);

#ifdef TPD_PROXIMITY
		if (tpd_proximity_flag == 1)
		{
			tpd_enable_ps(1);
		}
#endif
	tpd_halt = 0;	
		read_buf[3] = 0x00;
		read_buf[2] = 0x00;
		read_buf[1] = 0x00;
		read_buf[0] = 0x4;
		i2c_smbus_write_i2c_block_data(i2c_client, 0xf0, 4, read_buf);
		i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 4, read_buf);
	printk("======read 0x444440: %x %x %x %x ======\n",read_buf[3], read_buf[2], read_buf[1], read_buf[0]);	

	tp_is_resume=1;
	if(enable_tp_ps==1)
	{
		ps_enable_nodata(1);
		enable_tp_ps=0;
		
	}

}

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = GSLX680_NAME,
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
//#ifdef TPD_HAVE_BUTTON
//	.tpd_have_button = 1,
//#else
//	.tpd_have_button = 0,
//#endif
};

/* called when loaded into kernel */
static int __init tpd_driver_init(void) {
	int rc;
	printk("Sileadinc gslX680 touch panel driver init\n");
	tpd_get_dts_info();
	rc = of_scan_flat_dt(dt_get_SKU, NULL);
	if(rc)
	    printk("gsl3670 scan %d\n",rc);
	if(tpd_driver_add(&tpd_device_driver) < 0)
		printk("add gslX680 driver failed\n");
	return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void) {
	printk("Sileadinc gslX680 touch panel driver exit\n");
	//input_unregister_device(tpd->dev);
	tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);


