/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
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
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
//#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <linux/input/mt.h>
#include <linux/wakelock.h>
//Begin Neostra huangxiaohui add  20160720
#include <linux/kobject.h>
#include <linux/sysfs.h>
//End Neostra huangxiaohui  20160720
#include "include/tpd_ft5x0x_common.h"

#include "focaltech_core.h"
/* #include "ft5x06_ex_fun.h" */

#include "tpd.h"
#include "base.h"
/* #define TIMER_DEBUG */
#include "mt_boot_common.h"

#ifdef TIMER_DEBUG
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#endif

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
#include <mach/md32_ipi.h>
#include <mach/md32_helper.h>
#endif

#include <asm/uaccess.h> 
#include <linux/proc_fs.h> 
#define ACER_GESTURE_WAKEUP   1
#if ACER_GESTURE_WAKEUP
//mtk add begin
/*Neostra liudongke add for optimize TP power consumption start 20170228*/
struct wake_lock acer_suspend_lock; //Neostra liudongke add for TP wake up from deep sleep
//int tpd_i2c_halt = 0;
//int suspend_gesture = 0;
//static DECLARE_WAIT_QUEUE_HEAD(waiter_resume);
/*Neostra liudongke add for optimize TP power consumption end 20170228*/
//mtk add end
#define GESTURE_PROC_NAME  "acer_EnableGesture"
#define MAX_TRACKINGID  255
static char mProcData[10];
static bool mIsEnableGestureWakeUp = true;
static bool mIsEnabletwofinger = true;
static bool mIsEnablefivefinger = true;
static bool mIsEnableDoubleTab = true;
static bool mIsEnableVirtualHomeKey = true;
static bool mIsEnableSliderPoweronoff = true;
static bool mIsEnableSmartCover = true;
static struct proc_dir_entry *mProc_dir_entry;
#endif



#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
enum DOZE_T {
	DOZE_DISABLED = 0,
	DOZE_ENABLED = 1,
	DOZE_WAKEUP = 2,
};
static DOZE_T doze_status = DOZE_DISABLED;
#endif

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
static s8 ftp_enter_doze(struct i2c_client *client);

enum TOUCH_IPI_CMD_T {
	/* SCP->AP */
	IPI_COMMAND_SA_GESTURE_TYPE,
	/* AP->SCP */
	IPI_COMMAND_AS_CUST_PARAMETER,
	IPI_COMMAND_AS_ENTER_DOZEMODE,
	IPI_COMMAND_AS_ENABLE_GESTURE,
	IPI_COMMAND_AS_GESTURE_SWITCH,
};

struct Touch_Cust_Setting {
	u32 i2c_num;
	u32 int_num;
	u32 io_int;
	u32 io_rst;
};

struct Touch_IPI_Packet {
	u32 cmd;
	union {
		u32 data;
		Touch_Cust_Setting tcs;
	} param;
};

/* static bool tpd_scp_doze_en = FALSE; */
static bool tpd_scp_doze_en = TRUE;
DEFINE_MUTEX(i2c_access);
#endif

#define TPD_SUPPORT_POINTS	10


struct i2c_client *i2c_client = NULL;
struct task_struct *thread_tpd = NULL;
/*******************************************************************************
* 4.Static variables
*******************************************************************************/
struct i2c_client *fts_i2c_client 				= NULL;
struct input_dev *fts_input_dev				=NULL;
#ifdef TPD_AUTO_UPGRADE
static bool is_update = false;
#endif
#ifdef CONFIG_FT_AUTO_UPGRADE_SUPPORT
u8 *tpd_i2c_dma_va = NULL;
dma_addr_t tpd_i2c_dma_pa = 0;
#endif


static struct kobject *touchscreen_dir=NULL;
static struct kobject *virtual_dir=NULL;
static struct kobject *touchscreen_dev_dir=NULL;
static char *vendor_name=NULL;
static u8 ctp_fw_version;
//Begin Neostra huangxiaohui add  20160720
static u8 tp_vendor_id;
static struct i2c_client *vendor_id_client 				= NULL;
//End Neostra huangxiaohui  20160720
//static int tpd_keys[TPD_VIRTUAL_KEY_MAX] = { 0 };
//static int tpd_keys_dim[TPD_VIRTUAL_KEY_MAX][4]={0};

#define WRITE_BUF_SIZE  1016
#define PROC_UPGRADE							0
#define PROC_READ_REGISTER						1
#define PROC_WRITE_REGISTER					    2
#define PROC_AUTOCLB							4
#define PROC_UPGRADE_INFO						5
#define PROC_WRITE_DATA						    6
#define PROC_READ_DATA							7
#define PROC_SET_TEST_FLAG						8
static unsigned char proc_operate_mode 			= PROC_UPGRADE;



static DECLARE_WAIT_QUEUE_HEAD(waiter);

static irqreturn_t tpd_eint_interrupt_handler(int irq, void *dev_id);


static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);
static void tpd_resume(struct device *h);
static void tpd_suspend(struct device *h);
static int tpd_flag;
/*static int point_num = 0;
static int p_point_num = 0;*/

unsigned int tpd_rst_gpio_number = 0;
unsigned int tpd_int_gpio_number = 1;
unsigned int touch_irq = 0;
#define TPD_OK 0


/* Register define */
#define DEVICE_MODE	0x00
#define GEST_ID		0x01
#define TD_STATUS	0x02

#define TOUCH1_XH	0x03
#define TOUCH1_XL	0x04
#define TOUCH1_YH	0x05
#define TOUCH1_YL	0x06

#define TOUCH2_XH	0x09
#define TOUCH2_XL	0x0A
#define TOUCH2_YH	0x0B
#define TOUCH2_YL	0x0C

#define TOUCH3_XH	0x0F
#define TOUCH3_XL	0x10
#define TOUCH3_YH	0x11
#define TOUCH3_YL	0x12

#define TPD_RESET_ISSUE_WORKAROUND
#define TPD_MAX_RESET_COUNT	3

#define AC_CHARGE_DETECT 1
#if AC_CHARGE_DETECT
extern bool upmu_is_chr_det(void);
static void tpd_charger_check(int resume)
{
    static int ft5x46_charger_state = 0;

    int ret = 0;
	int charger_state = upmu_is_chr_det();
    //printk("[FTS] charger_state: %d, cur_state: %d\n",charger_state, ft5x46_charger_state);

    if (resume || (ft5x46_charger_state != charger_state))
    {
        ft5x46_charger_state = charger_state;

        if (ft5x46_charger_state != 0)  // charger plugged in
        {
            ret = fts_write_reg(i2c_client, 0x8b, 0x01);
        }
        else
        {
            ret = fts_write_reg(i2c_client, 0x8b, 0x00);
        }

        if (ret < 0)
        {
            TPD_DMESG("fts i2c write 0x8b error\n");
        }
    }
}
#endif

#if ACER_GESTURE_WAKEUP
static ssize_t read_proc(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	int len = 0;
	//touch_debug(DEBUG_INFO," ---hxh read_proc ---\n");
	len = sprintf(buf, "%s", mProcData);
	return len;
}

static ssize_t write_proc(struct file *file, const char *buf, size_t count, loff_t *data)
{
	//printk("---hxh write_proc count = %lu\n", count);
	// mProcData length is 7, ex:1111011
	// from left to right, each number is on behalf of as below.
	// 1: two finger on/off. 2: five finger on/off. 3: Double tab on/off
	// 4: Virtual Home Key on/off. 5: To Reserve. 6: Slider to Power on/off. 7:Smart Cover on/off.
	
	if (count == 8) {
		memset(mProcData, 0, sizeof(mProcData));
		memcpy(mProcData, buf, count);

		if (simple_strtoul(mProcData, NULL, 10) > 0) {
			mIsEnableGestureWakeUp = true;
		} else {
			mIsEnableGestureWakeUp = false; 
		}
		
		if(mProcData[0] == '0'){
			mIsEnabletwofinger = false;
		}else{
			mIsEnabletwofinger = true;
		}
		
		if(mProcData[1] == '0'){
			mIsEnablefivefinger = false;
		}else{
			mIsEnablefivefinger = true;
		}
		
		if(mProcData[2] == '0'){
			mIsEnableDoubleTab = false;
		}else{
			mIsEnableDoubleTab = true;
		}
		
		if(mProcData[3] == '0'){
			mIsEnableVirtualHomeKey = false;
		}else{
			mIsEnableVirtualHomeKey = true;
		}
		
		if(mProcData[5] == '0'){
			mIsEnableSliderPoweronoff = false;
		}else{
			mIsEnableSliderPoweronoff = true;
		}
		
		if(mProcData[6] == '0'){
			mIsEnableSmartCover = false;
		}else{
			mIsEnableSmartCover = true;
		}

	}
	return 1;
}


static struct file_operations wakeup_ops = {
	.owner = THIS_MODULE,
	.write = write_proc,
	.read = read_proc,
};


void create_new_proc_entry(void)
{
	//mProc_dir_entry = create_proc_entry(GESTURE_PROC_NAME, 0666, NULL);
      mProc_dir_entry = proc_create(GESTURE_PROC_NAME, 0777, NULL, &wakeup_ops);
        if (mProc_dir_entry == NULL)
        {
            //printk("Couldn't create proc entry!");
        }
        else
        {
            //printk(" Create proc entry success!");

        }

	memset(mProcData, 0, sizeof(mProcData));
	sprintf(mProcData, "%s", "00000");
}

int proc_init(void)
{
	create_new_proc_entry();
	return 0;
}

void proc_cleanup(void)
{
	remove_proc_entry(GESTURE_PROC_NAME, NULL);
}
#endif


//Begin Neostra huangxiaohui add to read TP version's interface 20160720
/*Neostra liudongke add for optimize TP power consumption start 20170228*/
/*
extern u32 get_devinfo_with_index(u32 index);

static ssize_t cpu_version_show(struct device *dev,
               struct device_attribute *attr, char *buf)
{
       
  int ret; 
  unsigned int devinfo;
  devinfo = (get_devinfo_with_index(4) >> 20)&0x01;
  if(devinfo ==1)
  {
	  ret=sprintf(buf, "MT8163B\n");
  }
  else
  {
	  ret=sprintf(buf, "MT8163A\n");  
  }

  return ret; 



}
*/
/*Neostra liudongke add for optimize TP power consumption end 20170228*/

static ssize_t version_show(struct device *dev,
               struct device_attribute *attr, char *buf)
{
       
  int ret; 

  fts_read_reg(vendor_id_client, 0xA6, &ctp_fw_version);
   
  ret=sprintf(buf, "ID:0x%02x VER:%02x\n",tp_vendor_id,ctp_fw_version);			   
			     
  return ret; 

}

static DEVICE_ATTR(version, S_IRUGO, version_show, NULL);
//static DEVICE_ATTR(cpuversion, S_IRUGO, cpu_version_show, NULL); //Neostra liudongke add for optimize TP power consumption 20170228

static struct kobject *android_touch_kobj;

static int touch_sysfs_init(void)
{
       int ret ;

       android_touch_kobj = kobject_create_and_add("android_touch", NULL) ;
       if (android_touch_kobj == NULL) {
               TPD_DMESG("error :%s: subsystem_register failed\n", __func__);
               ret = -ENOMEM;
               return ret;
       }

       ret = sysfs_create_file(android_touch_kobj, &dev_attr_version.attr);
       if (ret < 0) {
               TPD_DMESG("error: %s: sysfs_create_file failed\n", __func__);
               ret = -EINVAL;
               return ret;
       }
/*Neostra liudongke add for optimize TP power consumption start 20170228*/
      // ret = sysfs_create_file(android_touch_kobj, &dev_attr_cpuversion.attr);
      // if (ret < 0) {
      //         printk("error: %s: sysfs_create_file failed\n", __func__);
       //        ret = -EINVAL;
        //       return ret;
     //  }	   
/*Neostra liudongke add for optimize TP power consumption end 20170228*/  
       return 0 ;
}

static void touch_sysfs_deinit(void)
{
       sysfs_remove_file(android_touch_kobj, &dev_attr_version.attr);
	   //sysfs_remove_file(android_touch_kobj, &dev_attr_cpuversion.attr); //Neostra liudongke add for optimize TP power consumption 20170228
       kobject_del(android_touch_kobj);
}
//End Neostra huangxiaohui  20160720



#ifdef TIMER_DEBUG

static struct timer_list test_timer;

static void timer_func(unsigned long data)
{
	tpd_flag = 1;
	wake_up_interruptible(&waiter);

	mod_timer(&test_timer, jiffies + 100*(1000/HZ));
}

static int init_test_timer(void)
{
	memset((void *)&test_timer, 0, sizeof(test_timer));
	test_timer.expires  = jiffies + 100*(1000/HZ);
	test_timer.function = timer_func;
	test_timer.data     = 0;
	init_timer(&test_timer);
	add_timer(&test_timer);
	return 0;
}
#endif


#if defined(CONFIG_TPD_ROTATE_90) || defined(CONFIG_TPD_ROTATE_270) || defined(CONFIG_TPD_ROTATE_180)
/*
static void tpd_swap_xy(int *x, int *y)
{
	int temp = 0;

	temp = *x;
	*x = *y;
	*y = temp;
}
*/
/*
static void tpd_rotate_90(int *x, int *y)
{
//	int temp;

	*x = TPD_RES_X + 1 - *x;

	*x = (*x * TPD_RES_Y) / TPD_RES_X;
	*y = (*y * TPD_RES_X) / TPD_RES_Y;

	tpd_swap_xy(x, y);
}
*/
static void tpd_rotate_180(int *x, int *y)
{
	*y = TPD_RES_Y + 1 - *y;
	*x = TPD_RES_X + 1 - *x;
}
/*
static void tpd_rotate_270(int *x, int *y)
{
//	int temp;

	*y = TPD_RES_Y + 1 - *y;

	*x = (*x * TPD_RES_Y) / TPD_RES_X;
	*y = (*y * TPD_RES_X) / TPD_RES_Y;

	tpd_swap_xy(x, y);
}
*/
#endif
struct touch_info {
	int y[TPD_SUPPORT_POINTS];
	int x[TPD_SUPPORT_POINTS];
	int p[TPD_SUPPORT_POINTS];
	int id[TPD_SUPPORT_POINTS];
	int count;
};

/*dma declare, allocate and release*/
//#define __MSG_DMA_MODE__
#ifdef __MSG_DMA_MODE__
	u8 *g_dma_buff_va = NULL;
	dma_addr_t g_dma_buff_pa = 0;
#endif

#ifdef __MSG_DMA_MODE__

	static void msg_dma_alloct(void)
	{
	    if (NULL == g_dma_buff_va)
    		{
       		 tpd->dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
       		 g_dma_buff_va = (u8 *)dma_alloc_coherent(&tpd->dev->dev, 128, &g_dma_buff_pa, GFP_KERNEL);
    		}

	    	if(!g_dma_buff_va)
		{
	        	TPD_DMESG("[DMA][Error] Allocate DMA I2C Buffer failed!\n");
	    	}
	}
	static void msg_dma_release(void){
		if(g_dma_buff_va)
		{
	     		dma_free_coherent(NULL, 128, g_dma_buff_va, g_dma_buff_pa);
	        	g_dma_buff_va = NULL;
	        	g_dma_buff_pa = 0;
			TPD_DMESG("[DMA][release] Allocate DMA I2C Buffer release!\n");
	    	}
	}
#endif

static DEFINE_MUTEX(i2c_access);
static DEFINE_MUTEX(i2c_rw_access);

#if (defined(CONFIG_TPD_HAVE_CALIBRATION) && !defined(CONFIG_TPD_CUSTOM_CALIBRATION))
/* static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX; */
/* static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX; */
static int tpd_def_calmat_local_normal[8]  = TPD_CALIBRATION_MATRIX_ROTATION_NORMAL;
static int tpd_def_calmat_local_factory[8] = TPD_CALIBRATION_MATRIX_ROTATION_FACTORY;
#endif

static const struct i2c_device_id ft5x0x_tpd_id[] = {{"ft5x0x", 0}, {} };
static const struct of_device_id ft5x0x_dt_match[] = {
	{.compatible = "mediatek,cap_touch"},
	{},
};
MODULE_DEVICE_TABLE(of, ft5x0x_dt_match);

#if 0
//mtk add begin
static int ft5x0x_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{
		suspend_gesture = true;
		tpd_i2c_halt = 1;
		enable_irq_wake(touch_irq);
    return 0;
}

static int ft5x0x_i2c_resume(struct i2c_client *client)
{
		suspend_gesture = false;
		tpd_i2c_halt = 0;
		wake_up_interruptible(&waiter_resume);
    return 0;
}
//mtk add end
#endif

static struct i2c_driver tpd_i2c_driver = {
	.driver = {
		.of_match_table = of_match_ptr(ft5x0x_dt_match),
		.name = "ft5x0x",
	},
	.probe = tpd_probe,
	.remove = tpd_remove,
//mtk add begin
//  .suspend	= ft5x0x_i2c_suspend,
//  .resume		= ft5x0x_i2c_resume,
//mtk add end
        
	.id_table = ft5x0x_tpd_id,
	.detect = tpd_i2c_detect,
};

static int of_get_ft5x0x_platform_data(struct device *dev)
{
	/*int ret, num;*/

	if (dev->of_node) {
		const struct of_device_id *match;

		match = of_match_device(of_match_ptr(ft5x0x_dt_match), dev);
		if (!match) {
			TPD_DMESG("Error: No device match found\n");
			return -ENODEV;
		}
	}
	tpd_rst_gpio_number = of_get_named_gpio(dev->of_node, "rst-gpio", 0);
	tpd_int_gpio_number = of_get_named_gpio(dev->of_node, "int-gpio", 0);
	/*ret = of_property_read_u32(dev->of_node, "rst-gpio", &num);
	if (!ret)
		tpd_rst_gpio_number = num;
	ret = of_property_read_u32(dev->of_node, "int-gpio", &num);
	if (!ret)
		tpd_int_gpio_number = num;
  */
	//printk("g_vproc_en_gpio_number %d\n", tpd_rst_gpio_number);
	//printk("g_vproc_vsel_gpio_number %d\n", tpd_int_gpio_number);
	return 0;
}

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
static ssize_t show_scp_ctrl(struct device *dev, struct device_attribute *attr, char *buf)
{
	return 0;
}
static ssize_t store_scp_ctrl(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	u32 cmd;
	Touch_IPI_Packet ipi_pkt;

	if (kstrtoul(buf, 10, &cmd)) {
		TPD_DEBUG("[SCP_CTRL]: Invalid values\n");
		return -EINVAL;
	}

	TPD_DEBUG("SCP_CTRL: Command=%d", cmd);
	switch (cmd) {
	case 1:
	    /* make touch in doze mode */
	    tpd_scp_wakeup_enable(TRUE);
	    tpd_suspend(NULL);
	    break;
	case 2:
	    tpd_resume(NULL);
	    break;
		/*case 3:
	    // emulate in-pocket on
	    ipi_pkt.cmd = IPI_COMMAND_AS_GESTURE_SWITCH,
	    ipi_pkt.param.data = 1;
		md32_ipi_send(IPI_TOUCH, &ipi_pkt, sizeof(ipi_pkt), 0);
	    break;
	case 4:
	    // emulate in-pocket off
	    ipi_pkt.cmd = IPI_COMMAND_AS_GESTURE_SWITCH,
	    ipi_pkt.param.data = 0;
		md32_ipi_send(IPI_TOUCH, &ipi_pkt, sizeof(ipi_pkt), 0);
	    break;*/
	case 5:
		{
				Touch_IPI_Packet ipi_pkt;

				ipi_pkt.cmd = IPI_COMMAND_AS_CUST_PARAMETER;
			    ipi_pkt.param.tcs.i2c_num = TPD_I2C_NUMBER;
			ipi_pkt.param.tcs.int_num = CUST_EINT_TOUCH_PANEL_NUM;
				ipi_pkt.param.tcs.io_int = tpd_int_gpio_number;
			ipi_pkt.param.tcs.io_rst = tpd_rst_gpio_number;
			if (md32_ipi_send(IPI_TOUCH, &ipi_pkt, sizeof(ipi_pkt), 0) < 0)
				TPD_DEBUG("[TOUCH] IPI cmd failed (%d)\n", ipi_pkt.cmd);

			break;
		}
	default:
	    TPD_DEBUG("[SCP_CTRL] Unknown command");
	    break;
	}

	return size;
}
static DEVICE_ATTR(tpd_scp_ctrl, 0664, show_scp_ctrl, store_scp_ctrl);
#endif

static struct device_attribute *ft5x0x_attrs[] = {
#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
	&dev_attr_tpd_scp_ctrl,
#endif
};


static ssize_t mtk_ctp_firmware_vertion_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{

  int ret; 
//Begin Neostra huangxiaohui mod  20160726
  fts_read_reg(vendor_id_client, 0xA6, &ctp_fw_version);
   
  ret=sprintf(buf, "ID:0x%02x VER:%02x\n",tp_vendor_id,ctp_fw_version);			   
//End Neostra huangxiaohui  20160726
			     
  return ret;
 
}
static struct kobj_attribute ctp_firmware_vertion_attr = {
	.attr = {
		 .name = "firmware_version",                    
		 .mode = S_IRUGO,
		 },
	.show = &mtk_ctp_firmware_vertion_show,
};

static ssize_t mtk_ctp_firmware_update_store(struct kobject *kobj, struct kobj_attribute *attr,const char *buf, size_t count)
{
    unsigned char writebuf[WRITE_BUF_SIZE];
	int buflen = count;
	int writelen = 0;
	int ret = 0;
	
#if FT_ESD_PROTECT
    esd_switch(0);
	apk_debug_flag = 1;
//printk("\n  zax v= %d \n",apk_debug_flag);
#endif
	if (copy_from_user(&writebuf, buf, buflen)) {
		dev_err(&fts_i2c_client->dev, "%s:copy from user error\n", __func__);
#if FT_ESD_PROTECT
	esd_switch(1);
    apk_debug_flag = 0;
#endif
		return -EFAULT;
	}
	proc_operate_mode = writebuf[0];

	switch (proc_operate_mode) {
	
	case PROC_UPGRADE:
		{
			char upgrade_file_path[128];
			memset(upgrade_file_path, 0, sizeof(upgrade_file_path));
			sprintf(upgrade_file_path, "%s", writebuf + 1);
			upgrade_file_path[buflen-1] = '\0';
			TPD_DEBUG("%s\n", upgrade_file_path);
			//#if FT_ESD_PROTECT
			//	esd_switch(0);apk_debug_flag = 1;
			//#endif
			disable_irq(fts_i2c_client->irq);
			ret = fts_ctpm_fw_upgrade_with_app_file(fts_i2c_client, upgrade_file_path);
			enable_irq(fts_i2c_client->irq);
			if (ret < 0) {
				dev_err(&fts_i2c_client->dev, "%s:upgrade failed.\n", __func__);
				#if FT_ESD_PROTECT
					esd_switch(1);apk_debug_flag = 0;
				#endif
				return ret;
			}
			//#if FT_ESD_PROTECT
			//	esd_switch(1);apk_debug_flag = 0;
			//#endif
		}
		break;
	//case PROC_SET_TEST_FLAG:
	
	//	break;
	case PROC_SET_TEST_FLAG:
		#if FT_ESD_PROTECT
		apk_debug_flag=writebuf[1];
		if(1==apk_debug_flag)
			esd_switch(0);
		else if(0==apk_debug_flag)
			esd_switch(1);
		printk("\n zax flag=%d \n",apk_debug_flag);
		#endif
		break;
	case PROC_READ_REGISTER:
		writelen = 1;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
#if FT_ESD_PROTECT
					esd_switch(1);apk_debug_flag = 0;
				#endif
			dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_WRITE_REGISTER:
		writelen = 2;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
#if FT_ESD_PROTECT
					esd_switch(1);apk_debug_flag = 0;
				#endif
			dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_AUTOCLB:
		TPD_DEBUG("%s: autoclb\n", __func__);
		fts_ctpm_auto_clb(fts_i2c_client);
		break;
	case PROC_READ_DATA:
	case PROC_WRITE_DATA:
		writelen = count - 1;
		if(writelen>0)
		{
			ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
			if (ret < 0) {
#if FT_ESD_PROTECT
					esd_switch(1);apk_debug_flag = 0;
				#endif
				dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
				return ret;
			}
		}
		break;
	default:
		break;
	}
	
	#if FT_ESD_PROTECT
		//printk("\n  zax proc w 1 \n");		
esd_switch(1);apk_debug_flag = 0;
//printk("\n  zax v= %d \n",apk_debug_flag);
			#endif
	return count; 
 
}


static struct kobj_attribute ctp_firmware_update_attr = {
	.attr = {
		 .name = "firmware_update",
		 .mode = S_IWUGO,
		 },
	.store = &mtk_ctp_firmware_update_store,

};
static ssize_t mtk_ctp_vendor_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
 int ret;
 ret=sprintf(buf,"%s\n",vendor_name); 
 return ret;
}

static struct kobj_attribute ctp_vendor_attr = {
	.attr = {
		 .name = "vendor",
		 .mode = S_IRUGO,
		 },
	.show = &mtk_ctp_vendor_show,
};

static struct attribute *mtk_properties_attrs[] = {
	&ctp_firmware_vertion_attr.attr,
	&ctp_vendor_attr.attr,
	&ctp_firmware_update_attr.attr,
	NULL
};

static struct attribute_group mtk_ctp_attr_group = {
	.attrs = mtk_properties_attrs,
};

#if 1
static int create_ctp_node(void)
{
  int ret;
  virtual_dir = virtual_device_parent(NULL);
  if(!virtual_dir)
  {
   printk("Get virtual dir failed\n");
   return -ENOMEM;
  }
  touchscreen_dir=kobject_create_and_add("touchscreen",virtual_dir);
  if(!touchscreen_dir)
  {
   printk("Create touchscreen dir failed\n");
   return -ENOMEM;
  }
  touchscreen_dev_dir=kobject_create_and_add("touchscreen_dev",touchscreen_dir);
  if(!touchscreen_dev_dir)
  {
   printk("Create touchscreen_dev dir failed\n");
   return -ENOMEM;
  }
  ret=sysfs_create_group(touchscreen_dev_dir, &mtk_ctp_attr_group);
  if(ret)
  {
    printk("create mtk_ctp_firmware_vertion_attr_group error\n");
  } 
 
  return 0;
}


#endif
static void tpd_down(int x, int y, int p, int id)
{
	/*Neostra modify for TP Coordinate adjustment 20161220 start*/
	int warp;
#if defined(CONFIG_TPD_ROTATE_90)
	tpd_rotate_90(&x, &y);
#elif defined(CONFIG_TPD_ROTATE_270)
	tpd_rotate_270(&x, &y);
#elif defined(CONFIG_TPD_ROTATE_180)
	tpd_rotate_180(&x, &y);
#endif

warp = x;
x=y;
y=warp;

//x=800-x;
y=1920-y; //Neostra liudongke modify TP resolution 20170227
/*Neostra modify for TP Coordinate adjustment 20161220 end*/
#ifdef TPD_SOLVE_CHARGING_ISSUE
	if (0 != x) {
#else
	{
#endif
        
		input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id);
		//TPD_DMESG("===hxh=== %s x:%d y:%d p:%d\n", __func__, x, y, p);
		input_report_key(tpd->dev, BTN_TOUCH, 1);
		input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);
		input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
		input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
		input_mt_sync(tpd->dev);
	
	}
}

static void tpd_up(int x, int y,int id)
{
#if defined(CONFIG_TPD_ROTATE_90)
	tpd_rotate_90(&x, &y);
#elif defined(CONFIG_TPD_ROTATE_270)
	tpd_rotate_270(&x, &y);
#elif defined(CONFIG_TPD_ROTATE_180)
	tpd_rotate_180(&x, &y);
#endif

#ifdef TPD_SOLVE_CHARGING_ISSUE
	if (0 != x) {
#else
	{
#endif
		//TPD_DMESG("===hxh=== %s x:%d y:%d\n", __func__, x, y);
		input_report_key(tpd->dev, BTN_TOUCH, 0);
		input_mt_sync(tpd->dev);


	}
}

/*Coordination mapping*/
/*
static void tpd_calibrate_driver(int *x, int *y)
{
	int tx;

	tx = ((tpd_def_calmat[0] * (*x)) + (tpd_def_calmat[1] * (*y)) + (tpd_def_calmat[2])) >> 12;
	*y = ((tpd_def_calmat[3] * (*x)) + (tpd_def_calmat[4] * (*y)) + (tpd_def_calmat[5])) >> 12;
	*x = tx;
}
*/
static int tpd_touchinfo(struct touch_info *cinfo, struct touch_info *pinfo)
{
	int i = 0;
	char data[63] = {0};
	//u8 report_rate = 0;
	u16 high_byte, low_byte;
	char writebuf[10]={0};
	//u8 fwversion = 0;

	writebuf[0]=0x00;
	fts_i2c_read(i2c_client, writebuf,  1, data, 62);

	//fts_read_reg(i2c_client, 0xa6, &fwversion);
	//fts_read_reg(i2c_client, 0x88, &report_rate);

	//TPD_DEBUG("FW version=%x]\n", fwversion);

#if 0
	TPD_DEBUG("received raw data from touch panel as following:\n");
	for (i = 0; i < 8; i++)
		TPD_DEBUG("data[%d] = 0x%02X ", i, data[i]);
	TPD_DEBUG("\n");
	for (i = 8; i < 16; i++)
		TPD_DEBUG("data[%d] = 0x%02X ", i, data[i]);
	TPD_DEBUG("\n");
	for (i = 16; i < 24; i++)
		TPD_DEBUG("data[%d] = 0x%02X ", i, data[i]);
	TPD_DEBUG("\n");
	for (i = 24; i < 32; i++)
		TPD_DEBUG("data[%d] = 0x%02X ", i, data[i]);
	TPD_DEBUG("\n");
#endif
//By Gujh_Eric, 2016/5/26, 10:26:47 // 	if (report_rate < 8) {
//By Gujh_Eric, 2016/5/26, 10:26:47 // 		report_rate = 0x8;
//By Gujh_Eric, 2016/5/26, 10:26:47 // 		if ((fts_write_reg(i2c_client, 0x88, report_rate)) < 0)
//By Gujh_Eric, 2016/5/26, 10:26:47 // 			printk("I2C write report rate error, line: %d\n", __LINE__);
//By Gujh_Eric, 2016/5/26, 10:26:47 // 	}

	/* Device Mode[2:0] == 0 :Normal operating Mode*/
	if ((data[0] & 0x70) != 0)
		return false;

	memcpy(pinfo, cinfo, sizeof(struct touch_info));
	memset(cinfo, 0, sizeof(struct touch_info));
	//for (i = 0; i < TPD_SUPPORT_POINTS; i++)
		//cinfo->p[i] = 1;	/* Put up */

	/*get the number of the touch points*/
	cinfo->count = data[2] & 0x0f;

	TPD_DEBUG("Number of touch points = %d\n", cinfo->count);

	TPD_DEBUG("Procss raw data...\n");

	for (i = 0; i < cinfo->count; i++) {
		cinfo->p[i] = (data[3 + 6 * i] >> 6) & 0x0003; /* event flag */
		cinfo->id[i] = data[3+6*i+2]>>4; 						// touch id

		/*get the X coordinate, 2 bytes*/
		high_byte = data[3 + 6 * i];
		high_byte <<= 8;
		high_byte &= 0x0F00;

		low_byte = data[3 + 6 * i + 1];
		low_byte &= 0x00FF;
		cinfo->x[i] = high_byte | low_byte;

		/*get the Y coordinate, 2 bytes*/
		high_byte = data[3 + 6 * i + 2];
		high_byte <<= 8;
		high_byte &= 0x0F00;

		low_byte = data[3 + 6 * i + 3];
		low_byte &= 0x00FF;
		cinfo->y[i] = high_byte | low_byte;
        //printk("cinfo->x[%d] = %d, cinfo->y[%d] = %d, cinfo->p[%d] = %d\n", i,cinfo->x[i], i, cinfo->y[i], i, cinfo->p[i]);
		//TPD_DEBUG(" cinfo->x[%d] = %d, cinfo->y[%d] = %d, cinfo->p[%d] = %d\n", i,
		//cinfo->x[i], i, cinfo->y[i], i, cinfo->p[i]);
	}




#ifdef CONFIG_TPD_HAVE_CALIBRATION
	for (i = 0; i < cinfo->count; i++) {
		tpd_calibrate_driver(&(cinfo->x[i]), &(cinfo->y[i]));
		TPD_DEBUG(" cinfo->x[%d] = %d, cinfo->y[%d] = %d, cinfo->p[%d] = %d\n", i,
		cinfo->x[i], i, cinfo->y[i], i, cinfo->p[i]);
	}
#endif

	return true;

};



/************************************************************************
* Name: fts_i2c_read
* Brief: i2c read
* Input: i2c info, write buf, write len, read buf, read len
* Output: get data in the 3rd buf
* Return: fail <0
***********************************************************************/
int fts_i2c_read(struct i2c_client *client, char *writebuf,int writelen, char *readbuf, int readlen)
{
  int ret;
	mutex_lock(&i2c_rw_access);

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			pr_err("f%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			pr_err("%s:i2c read error.\n", __func__);
	}
	mutex_unlock(&i2c_rw_access);	
	return ret;
}


/************************************************************************
* Name: fts_i2c_write
* Brief: i2c write
* Input: i2c info, write buf, write len
* Output: no
* Return: fail <0
***********************************************************************/
int fts_i2c_write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = writelen,
		 .buf = writebuf,
		 },
	};

	mutex_lock(&i2c_rw_access);
	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error.\n", __func__);

	mutex_unlock(&i2c_rw_access);	
	return ret;
}

/************************************************************************
* Name: fts_write_reg
* Brief: write register
* Input: i2c info, reg address, reg value
* Output: no
* Return: fail <0
***********************************************************************/
int fts_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue)
{
	unsigned char buf[2] = {0};

	buf[0] = regaddr;
	buf[1] = regvalue;

	return fts_i2c_write(client, buf, sizeof(buf));
}
/************************************************************************
* Name: fts_read_reg
* Brief: read register
* Input: i2c info, reg address, reg value
* Output: get reg value
* Return: fail <0
***********************************************************************/
int fts_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue)
{

	return fts_i2c_read(client, &regaddr, 1, regvalue, 1);

}

#if USB_CHARGE_DETECT
extern int FG_charging_status ;
int close_to_ps_flag_value = 1;	// 1: close ; 0: far away
int charging_flag = 0;
#endif
static int touch_event_handler(void *unused)
{
	int i = 0;
	#if FTS_GESTRUE_EN
	int ret = 0;
	u8 state = 0;
	#endif
	#if USB_CHARGE_DETECT
	u8 data;
	#endif
	struct touch_info cinfo, pinfo, finfo;
	struct sched_param param = { .sched_priority = 4 };

	if (tpd_dts_data.use_tpd_button) {
		memset(&finfo, 0, sizeof(struct touch_info));
		for (i = 0; i < TPD_SUPPORT_POINTS; i++)
			finfo.p[i] = 1;
	}

	sched_setscheduler(current, SCHED_RR, &param);

	do {
		/*enable_irq(touch_irq);*/
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, tpd_flag != 0);

		tpd_flag = 0;

		set_current_state(TASK_RUNNING);

    //mtk add begin
	/*Neostra liudongke add for optimize TP power start consumption 20170228*/
		//if (suspend_gesture == true) {
		//	suspend_gesture = false;
		//	wake_lock_timeout(&acer_suspend_lock, 2*HZ);
		//	    printk("wake lock on end \n");
			
		//	set_current_state(TASK_INTERRUPTIBLE);
		//	wait_event_interruptible(waiter_resume, tpd_i2c_halt == 0);
		//	set_current_state(TASK_RUNNING);			
		//}
	/*Neostra liudongke add for optimize TP power end consumption 20170228*/
  	//mtk add end
 
#if AC_CHARGE_DETECT
        tpd_charger_check(0);
#endif

#if USB_CHARGE_DETECT
		if((FG_charging_status != 0) && (charging_flag == 0))
		{
			data = 0x1;
			charging_flag = 1;
			fts_write_reg(i2c_client, 0x8B, 0x01);  
		}
		else
		{
			if((FG_charging_status == 0) && (charging_flag == 1))
			{
				charging_flag = 0;
				data = 0x0;
				fts_write_reg(i2c_client, 0x8B, 0x00);  					
			}
		}
#endif

		#if FTS_GESTRUE_EN
		if(mIsEnableGestureWakeUp)/*Neostra huangxiaohui modify 20160923*/
		{
			ret = fts_read_reg(fts_i2c_client, 0xd0,&state);
			if (ret<0)
			{
				printk("[Focal][Touch] read value fail");
				//return ret;
			}
			//printk("tpd fts_read_Gestruedata state=%d\n",state);
		     	if(state ==1)
		     	{
			        fts_read_Gestruedata();
			        continue;
		    	}
		}
		 #endif

		TPD_DEBUG("touch_event_handler start\n");

		if (tpd_touchinfo(&cinfo, &pinfo)) {
			if (tpd_dts_data.use_tpd_button) {
				if (cinfo.p[0] == 0)
					memcpy(&finfo, &cinfo, sizeof(struct touch_info));
			}

			if ((cinfo.y[0] >= TPD_RES_Y) && (pinfo.y[0] < TPD_RES_Y)
			&& ((pinfo.p[0] == 0) || (pinfo.p[0] == 2))) {
				TPD_DEBUG("Dummy release --->\n");
				tpd_up(pinfo.x[0], pinfo.y[0],pinfo.id[0]);
				input_sync(tpd->dev);
				continue;
			}
            #if 0 
			if (tpd_dts_data.use_tpd_button) {
				if ((cinfo.y[0] <= TPD_RES_Y && cinfo.y[0] != 0) && (pinfo.y[0] > TPD_RES_Y)
				&& ((pinfo.p[0] == 0) || (pinfo.p[0] == 2))) {
					TPD_DEBUG("Dummy key release --->\n");
					//tpd_button(pinfo.x[0], pinfo.y[0], 0);
					tpd_down(pinfo.x[0], pinfo.y[0], 0)
					input_sync(tpd->dev);
					continue;
				}

			if ((cinfo.y[0] > TPD_RES_Y) || (pinfo.y[0] > TPD_RES_Y)) {
				if (finfo.y[0] > TPD_RES_Y) {
					if ((cinfo.p[0] == 0) || (cinfo.p[0] == 2)) {
							TPD_DEBUG("Key press --->\n");
							tpd_button(pinfo.x[0], pinfo.y[0], 1);
					} else if ((cinfo.p[0] == 1) &&
						((pinfo.p[0] == 0) || (pinfo.p[0] == 2))) {
							TPD_DEBUG("Key release --->\n");
							tpd_down(pinfo.x[0], pinfo.y[0], 0);
					}
					input_sync(tpd->dev);
				}
				continue;
			}
			
			}
             #endif

			if (cinfo.count > 0) {
				for (i = 0; i < cinfo.count; i++)
					tpd_down(cinfo.x[i], cinfo.y[i], i + 1, cinfo.id[i]);
					//tpd_down(cinfo.x[i], cinfo.y[i], , i);
			} else {
#ifdef TPD_SOLVE_CHARGING_ISSUE
				tpd_up(1, 48);
#else
				tpd_up(cinfo.x[0], cinfo.y[0],cinfo.id[0]);
#endif

			}
			input_sync(tpd->dev);

		}
	} while (!kthread_should_stop());

	TPD_DEBUG("touch_event_handler exit\n");

	return 0;
}

static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, TPD_DEVICE);

	return 0;
}

static irqreturn_t tpd_eint_interrupt_handler(int irq, void *dev_id)
{
	//TPD_DEBUG("TPD interrupt has been triggered\n");
	tpd_flag = 1;
	wake_up_interruptible(&waiter);
	return IRQ_HANDLED;
}
static int tpd_irq_registration(void)
{  
    #if 0
 	struct device_node *node = NULL;
	int ret = 0;
	u32 ints[2] = {0,0};

	node = of_find_matching_node(node, touch_of_match);
	if (node) {
		/*touch_irq = gpio_to_irq(tpd_int_gpio_number);*/
		of_property_read_u32_array(node,"debounce", ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);

		touch_irq = irq_of_parse_and_map(node, 0);
		ret = request_irq(touch_irq, tpd_eint_interrupt_handler,
					IRQF_TRIGGER_FALLING, "TOUCH_PANEL-eint", NULL);
			if (ret > 0)
				TPD_DMESG("tpd request_irq IRQ LINE NOT AVAILABLE!.");
	} else {
		TPD_DMESG("[%s] tpd request_irq can not find touch eint device node!.", __func__);
	}
	
	return 0;
	#endif
	
	struct device_node *node = NULL;
		int ret = 0;

		node = of_find_compatible_node(NULL, NULL, "mediatek,cap_touch");
		//node = of_find_matching_node(node, touch_of_match);
		if (node) {
			/*touch_irq = gpio_to_irq(tpd_int_gpio_number);*/
			touch_irq = irq_of_parse_and_map(node, 0);
			ret = request_irq(touch_irq, tpd_eint_interrupt_handler,
						IRQF_TRIGGER_FALLING/*IRQF_TRIGGER_NONE*/, TPD_DEVICE, NULL);
				if (ret > 0)
					TPD_DMESG("tpd request_irq IRQ LINE NOT AVAILABLE!.");
		} else {
			TPD_DMESG("[%s] tpd request_irq can not find touch eint device node!.", __func__);
		}
		return 0;
	
}
#if 0
int hidi2c_to_stdi2c(struct i2c_client * client)
{
	u8 auc_i2c_write_buf[5] = {0};
	int bRet = 0;

	auc_i2c_write_buf[0] = 0xeb;
	auc_i2c_write_buf[1] = 0xaa;
	auc_i2c_write_buf[2] = 0x09;

	fts_i2c_write(client, auc_i2c_write_buf, 3);

	msleep(10);

	auc_i2c_write_buf[0] = auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = 0;

	fts_i2c_read(client, auc_i2c_write_buf, 0, auc_i2c_write_buf, 3);

	if(0xeb==auc_i2c_write_buf[0] && 0xaa==auc_i2c_write_buf[1] && 0x08==auc_i2c_write_buf[2])
	{
		bRet = 1;
	}
	else
		bRet = 0;

	return bRet;

}
#endif
#if MTK_CTP_NODE 
#define CTP_PROC_FILE "tp_info"

static int ctp_proc_read_show (struct seq_file* m, void* data)
{
	char temp[40] = {0};
	//char vendor_name[20] = {0};

	/*if(temp_pid == 0x00 || temp_pid == 0x01){	//add by liuzhen
		sprintf(vendor_name,"%s","O-Film");
	}else if(temp_pid == 0x02){
		sprintf(vendor_name,"%s","Mudong");
	}else{
		sprintf(vendor_name,"%s","Reserve");
	}*/
	//sprintf(temp, "[Vendor]O-Film,[Fw]%s,[IC]GT915\n",temp_ver); //changed by cao
	sprintf(temp, "[Vendor]%s,[Fw]%x,[IC]FT5446\n","O-Film",4); 
	seq_printf(m, "%s\n", temp);
	//printk("vid:%s,firmware:0x%04x\n",temp_ver, temp_pid);
	return 0;
}

static int ctp_proc_open (struct inode* inode, struct file* file) 
{
    return single_open(file, ctp_proc_read_show, inode->i_private);
}

static const struct file_operations g_ctp_proc = 
{
    .open = ctp_proc_open,
    .read = seq_read,
};
#endif

static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int retval = TPD_OK;
//By Gujh_Eric, 2016/5/26, 10:27:10 // 	u8 report_rate = 0;
	int reset_count = 0;
	char data;
	int err = 0;

	i2c_client = client;
	fts_i2c_client = client;
	vendor_id_client= client;//Neostra huangxiaohui add  20160726
	fts_input_dev=tpd->dev;
	if(i2c_client->addr != 0x38)
	{
		i2c_client->addr = 0x38;
		printk("frank_zhonghua:i2c_client_FT->addr=%d\n",i2c_client->addr);
	}
	/*Neostra liudongke add for optimize TP power consumption start 20170228*/
	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT || get_boot_mode() == LOW_POWER_OFF_CHARGING_BOOT)//for power off charging current too large
		return -1;
    /*Neostra liudongke add for optimize TP power consumption end 20170228*/
	of_get_ft5x0x_platform_data(&client->dev);
	/* configure the gpio pins */

	retval = gpio_request_one(tpd_rst_gpio_number, GPIOF_OUT_INIT_LOW,
				 "touchp_reset");
	if (retval < 0) {
		printk("Unable to request gpio reset_pin\n");
		return -1;
	}
	retval = gpio_request_one(tpd_int_gpio_number, GPIOF_IN,
				 "tpd_int");
	if (retval < 0) {
		printk("Unable to request gpio int_pin\n");
		gpio_free(tpd_rst_gpio_number);
		return -1;
	}
	
	/*Neostra liudongke modify TP reset power on time sequence according to datasheet begain 20170221*/
	gpio_direction_output(tpd_rst_gpio_number, 0);
	msleep(10);
	//gpio_direction_output(tpd_rst_gpio_number, 1);
	//msleep(50);

	TPD_DMESG("mtk_tpd: tpd_probe ft5x0x\n");

	retval = regulator_enable(tpd->reg);
	if (retval != 0)
		printk("Failed to enable reg-vgp6: %d\n", retval);
	
   msleep(10);
   gpio_direction_output(tpd_rst_gpio_number, 1);
   msleep(20);
   /*Neostra liudongke modify TP reset power on time sequence according to datasheet end 20170221*/
   
	/* set INT mode */

	//tpd_gpio_as_int(tpd_int_gpio_number);
	gpio_direction_input(tpd_int_gpio_number);

//mtk add begin
#if ACER_GESTURE_WAKEUP
	wake_lock_init(&acer_suspend_lock, WAKE_LOCK_SUSPEND, "acer wakelock"); //Neostra liudongke add for TP wake up from deep sleep
#endif
//mtk add end

	tpd_irq_registration();
	msleep(100);
//	msg_dma_alloct();

#ifdef CONFIG_FT_AUTO_UPGRADE_SUPPORT

    if (NULL == tpd_i2c_dma_va)
    {
        tpd->dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
        tpd_i2c_dma_va = (u8 *)dma_alloc_coherent(&tpd->dev->dev, 250, &tpd_i2c_dma_pa, GFP_KERNEL);
    }
    if (!tpd_i2c_dma_va)
		TPD_DMESG("TPD dma_alloc_coherent error!\n");
	else
		TPD_DMESG("TPD dma_alloc_coherent success!\n");
#endif

#if FTS_GESTRUE_EN
	fts_Gesture_init(tpd->dev);
#endif

reset_proc:
	/* Reset CTP */
	/*Neostra liudongke modify TP reset power on time sequence according to datasheet begain 20170221*/
	//tpd_gpio_output(tpd_rst_gpio_number, 0);
	//msleep(20);
	//tpd_gpio_output(tpd_rst_gpio_number, 1);
	//msleep(400);
	/*Neostra liudongke modify TP reset power on time sequence according to datasheet end 20170221*/
	err = fts_read_reg(i2c_client, 0x00, &data);

	//printk("fts_i2c:err %d,data:%d\n", err,data);
	if(err< 0 || data!=0)// reg0 data running state is 0; other state is not 0
	{
		TPD_DMESG("I2C transfer error, line: %d\n", __LINE__);
#ifdef TPD_RESET_ISSUE_WORKAROUND
		if ( ++reset_count < TPD_MAX_RESET_COUNT )
		{
			goto reset_proc;
		}
#endif
		retval	= regulator_disable(tpd->reg); //disable regulator
		if(retval)
		{
			printk("focaltech tpd_probe regulator_disable() failed!\n");
		}

		regulator_put(tpd->reg);
//		msg_dma_release();
		gpio_free(tpd_rst_gpio_number);
		gpio_free(tpd_int_gpio_number);
		return -1;
	}
	tpd_load_status = 1;
/*#ifdef CONFIG_CUST_FTS_APK_DEBUG
	//ft_rw_iic_drv_init(client);

	//ft5x0x_create_sysfs(client);

	ft5x0x_create_apk_debug_channel(client);
#endif
*/   
    //device_create_file(tpd->tpd_dev, &tp_attr_foo); 
	#ifdef  MTK_CTP_NODE
    if((proc_create(CTP_PROC_FILE,0444,NULL,&g_ctp_proc))==NULL)
    {
      printk("proc_create tp vertion node error\n");
	}
	#endif
	#ifdef  MTK_CTP_NODE
	create_ctp_node();
    #endif


    touch_sysfs_init();	//Neostra huangxiaohui add  20160726
	
	//touch_class = class_create(THIS_MODULE,"FT5446");
	#ifdef SYSFS_DEBUG
                fts_create_sysfs(fts_i2c_client);
	#endif
	//hidi2c_to_stdi2c(fts_i2c_client);
	fts_get_upgrade_array();
	#ifdef FTS_CTL_IIC
		 if (fts_rw_iic_drv_init(fts_i2c_client) < 0)
			 dev_err(&client->dev, "%s:[FTS] create fts control iic driver failed\n", __func__);
	#endif

	#ifdef FTS_APK_DEBUG
		fts_create_apk_debug_channel(fts_i2c_client);
	#endif

    /*Neostra liudongke modify TP reset power on time sequence according to datasheet begain 20170221 */
	#if 0
	tpd_gpio_output(tpd_rst_gpio_number, 0);
	msleep(20);
	tpd_gpio_output(tpd_rst_gpio_number, 1);
	msleep(400);
	#endif
	/*Neostra liudongke modify TP reset power on time sequence according to datasheet end 20170221 */

    //printk("********************hxh don't Enter CTP Auto Upgrade********************\n");
	#ifdef TPD_AUTO_UPGRADE
		printk("********************Enter CTP Auto Upgrade********************\n");
		is_update = true;
		fts_ctpm_auto_upgrade(fts_i2c_client);
		is_update = false;
	#endif

	#if 0
	/* Reset CTP */

	tpd_gpio_output(tpd_rst_gpio_number, 0);
	msleep(20);
	tpd_gpio_output(tpd_rst_gpio_number, 1);
	msleep(400);
	#endif
/*#ifdef CONFIG_FT_AUTO_UPGRADE_SUPPORT
	tpd_auto_upgrade(client);
#endif*/
//printk("********************hxh rm report rate********************\n");
	/* Set report rate 80Hz */
//By Gujh_Eric, 2016/5/26, 10:26:53 // 	report_rate = 0x8;
//By Gujh_Eric, 2016/5/26, 10:26:53 // 	if ((fts_write_reg(i2c_client, 0x88, report_rate)) < 0) {
//By Gujh_Eric, 2016/5/26, 10:26:53 // 		if ((fts_write_reg(i2c_client, 0x88, report_rate)) < 0)
//By Gujh_Eric, 2016/5/26, 10:26:53 // 			printk("I2C write report rate error, line: %d\n", __LINE__);
//By Gujh_Eric, 2016/5/26, 10:26:53 // 	}

	/* tpd_load_status = 1; */

	thread_tpd = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(thread_tpd)) {
		retval = PTR_ERR(thread_tpd);
		TPD_DMESG(TPD_DEVICE " failed to create kernel thread_tpd: %d\n", retval);
	}

	
#ifdef TIMER_DEBUG
	init_test_timer();
#endif

	{
		u8 ver;
		u8 uc_tp_vendor_id;//Neostra huangxiaohui add  20160726

		fts_read_reg(client, 0xA6, &ver);
		ctp_fw_version = ver;

		fts_read_reg(client, FTS_REG_VENDOR_ID, &uc_tp_vendor_id);//Neostra huangxiaohui add  20160726
		tp_vendor_id = uc_tp_vendor_id;//Neostra huangxiaohui add  20160726
		
		//printk(TPD_DEVICE " fts_read_reg version : %d\n", ver);
        fts_read_reg(client, 0xA8, &ver);
        //Begin Neostra huangxiaohui modify vendor_name 20170512
        if(tp_vendor_id==0x3e)
        {
            vendor_name="taiguan";
        }
        else if(tp_vendor_id==0x7b)
        {
            vendor_name="pingbo";
        }
        else
        {
            vendor_name="taiguan";//default
        }
        //End Neostra huangxiaohui modify vendor_name 20170512
		//printk("vendor_name=%s fwvertion=%x\n",vendor_name,fwvertion);
	}
   
    //Begin Neostar huangjianlong 20170320
#ifdef CONFIG_NEOSTRA_HW_INFO_NODE		
    neostra_add_hw_info("tp-ic", "FT5826S", 0);	
	neostra_add_hw_info("tp-fw", NULL, ctp_fw_version);
	neostra_add_hw_info("tp-vendor", NULL, tp_vendor_id);
#endif	
	//End Neostar huangjianlong 20170320
	
	
#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
	int ret;

	ret = get_md32_semaphore(SEMAPHORE_TOUCH);
	if (ret < 0)
		pr_err("[TOUCH] HW semaphore reqiure timeout\n");
#endif

	return 0;
}

static int tpd_remove(struct i2c_client *client)
{
	TPD_DEBUG("TPD removed\n");
#ifdef CONFIG_CUST_FTS_APK_DEBUG
	//ft_rw_iic_drv_exit();
#endif

#ifdef CONFIG_FT_AUTO_UPGRADE_SUPPORT
	if (tpd_i2c_dma_va) {
		dma_free_coherent(NULL, 4096, tpd_i2c_dma_va, tpd_i2c_dma_pa);
		tpd_i2c_dma_va = NULL;
		tpd_i2c_dma_pa = 0;
	}
#endif
    touch_sysfs_deinit();//Neostra huangxiaohui add  20160726
	gpio_free(tpd_rst_gpio_number);
	gpio_free(tpd_int_gpio_number);

	return 0;
}

static int tpd_local_init(void)
{
	int retval;

	//printk("Focaltech FT5x0x I2C Touchscreen Driver...\n");
	tpd->reg = regulator_get(tpd->tpd_dev, "vtouch");
	retval = regulator_set_voltage(tpd->reg, 3300000, 3300000);//Neostra huangxiaohui add  for bug 131357 20160803
	if (retval != 0) {
		TPD_DMESG("Failed to set reg-vgp1 voltage: %d\n", retval);
		return -1;
	}
	if (i2c_add_driver(&tpd_i2c_driver) != 0) {
		TPD_DMESG("unable to add i2c driver.\n");
		return -1;
	}
     /* tpd_load_status = 1; */
	if (tpd_dts_data.use_tpd_button) {
		tpd_button_setting(tpd_dts_data.tpd_key_num, tpd_dts_data.tpd_key_local,
		tpd_dts_data.tpd_key_dim_local);
	}

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
	TPD_DO_WARP = 1;
	memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT * 4);
	memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT * 4);
#endif

#if (defined(CONFIG_TPD_HAVE_CALIBRATION) && !defined(CONFIG_TPD_CUSTOM_CALIBRATION))

	memcpy(tpd_calmat, tpd_def_calmat_local_factory, 8 * 4);
	memcpy(tpd_def_calmat, tpd_def_calmat_local_factory, 8 * 4);

	memcpy(tpd_calmat, tpd_def_calmat_local_normal, 8 * 4);
	memcpy(tpd_def_calmat, tpd_def_calmat_local_normal, 8 * 4);

#endif

	//printk("end %s, %d\n", __func__, __LINE__);
	tpd_type_cap = 1;

	return 0;
}

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
static s8 ftp_enter_doze(struct i2c_client *client)
{
	s8 ret = -1;
	s8 retry = 0;
	char gestrue_on = 0x01;
	char gestrue_data;
	int i;

	/* TPD_DEBUG("Entering doze mode..."); */
	pr_alert("Entering doze mode...");

	/* Enter gestrue recognition mode */
	ret = fts_write_reg(i2c_client, FT_GESTRUE_MODE_SWITCH_REG, gestrue_on);
	if (ret < 0) {
		/* TPD_DEBUG("Failed to enter Doze %d", retry); */
		pr_alert("Failed to enter Doze %d", retry);
		return ret;
	}
	msleep(30);

	for (i = 0; i < 10; i++) {
		fts_read_reg(i2c_client, FT_GESTRUE_MODE_SWITCH_REG, &gestrue_data);
		if (gestrue_data == 0x01) {
			doze_status = DOZE_ENABLED;
			/* TPD_DEBUG("FTP has been working in doze mode!"); */
			pr_alert("FTP has been working in doze mode!");
			break;
		}
		msleep(20);
		fts_write_reg(i2c_client, FT_GESTRUE_MODE_SWITCH_REG, gestrue_on);

	}

	return ret;
}
#endif

static void tpd_resume(struct device *h)
{
	int retval = TPD_OK;

	//TPD_DEBUG("hxh TPD wake up\n");
#if AC_CHARGE_DETECT	
	tpd_charger_check(1);
#endif

/* Neostra huangxiaohui delete for open TP gestrue 20160923
#if ACER_GESTURE_WAKEUP
	if (mIsEnableGestureWakeUp)
	{
		// Gesture WakeUp on
		TPD_DMESG("hxh tpd_resume acer Gesture WakeUp on.");
		return;
	}
	else
	{
		// Gesture WakeUp off
		TPD_DMESG("hxh tpd_resume acer Gesture WakeUp off.");
	}
#endif
*/

	retval = regulator_enable(tpd->reg);
	if (retval != 0)
		TPD_DMESG("Failed to enable reg-vgp1: %d\n", retval);

	tpd_gpio_output(tpd_rst_gpio_number, 0);
	msleep(20);
	tpd_gpio_output(tpd_rst_gpio_number, 1);
	msleep(200);

	#if FTS_GESTRUE_EN
	    if (mIsEnableGestureWakeUp)/*Begin Neostra huangxiaohui modify 20160923*/
		{
		   wake_unlock(&acer_suspend_lock); //Neostra liudongke add for TP wake up from deep sleep 
			fts_write_reg(fts_i2c_client,0xD0,0x00);
		}
	#endif


#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
	doze_status = DOZE_DISABLED;
	/* tpd_halt = 0; */
	int data;

	data = 0x00;

	fts_write_reg(i2c_client, FT_GESTRUE_MODE_SWITCH_REG, data);
/*Neostra liudongke add for optimize TP power consumption start 20170228*/
//#else
//	if(!mIsEnableGestureWakeUp)
//	{
	//	printk("HXH !mIsEnableGestureWakeUp\n");
//		enable_irq(touch_irq);
//	}
/*Neostra liudongke add for optimize TP power consumption end 20170228*/
#endif
#if USB_CHARGE_DETECT
	if(FG_charging_status != 0)
	{
		charging_flag = 0;
	}
	else
	{
		charging_flag = 1;
	}
#endif
}

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
void tpd_scp_wakeup_enable(bool en)
{
	tpd_scp_doze_en = en;
}

void tpd_enter_doze(void)
{

}
#endif

static void tpd_suspend(struct device *h)
{
	int retval = TPD_OK;
	static char data = 0x3;
	#if FTS_GESTRUE_EN
	int i = 0;
	u8 state = 0;
	u8 xxx = 0; /*Neostra huangxiaohui modify 20160923*/
	#endif
	TPD_DMESG("hxh TPD enter sleep\n");
/*
#if ACER_GESTURE_WAKEUP
	if(mIsEnableGestureWakeUp)
	{	
        TPD_DMESG("-->>hxh tpd_suspend acer Gesture  WakeUp on.");
		return;
	}
#endif 	
*/

	#if FTS_GESTRUE_EN
	/*Begin Neostra huangxiaohui modify 20160923*/
	if(mIsEnableGestureWakeUp)
	{
	  /*Neostra liudongke add for TP wake up from deep sleep start*/
		wake_lock_timeout(&acer_suspend_lock, 2*HZ);
		enable_irq_wake(touch_irq);
	  /*Neostra liudongke add for TP wake up from deep sleep end*/ 
        	if(1){
			//memset(coordinate_x,0,255);
			//memset(coordinate_y,0,255);

			fts_write_reg(i2c_client, 0xd0, 0x01);
		  	fts_write_reg(i2c_client, 0xd1, 0XFF/*0x1f*/);
//By Gujh_Eric, 2016/6/15, 14:46:02 // 			fts_write_reg(i2c_client, 0xd2, 0xff);
//By Gujh_Eric, 2016/6/15, 14:46:02 // 			fts_write_reg(i2c_client, 0xd5, 0xff);
//By Gujh_Eric, 2016/6/15, 14:46:02 // 			fts_write_reg(i2c_client, 0xd6, 0xff);
//By Gujh_Eric, 2016/6/15, 14:46:02 // 			fts_write_reg(i2c_client, 0xd7, 0xff);
//By Gujh_Eric, 2016/6/15, 14:46:02 // 			fts_write_reg(i2c_client, 0xd8, 0xff);

			msleep(10);

			for(i = 0; i < 10; i++)
			{
				printk("tpd_suspend4 %d",i);
			  	fts_read_reg(i2c_client, 0xd0, &state);

				fts_read_reg(i2c_client, 0xd1, &xxx);
				if(state == 1)
				{
					TPD_DMESG("TPD gesture write 0x01\n");
	        			return;
				}
				else
				{
					fts_write_reg(i2c_client, 0xd0, 0x01);
					fts_write_reg(i2c_client, 0xd1, 0xff);
//By Gujh_Eric, 2016/6/15, 14:47:29 // 		 			fts_write_reg(i2c_client, 0xd2, 0xff);
//By Gujh_Eric, 2016/6/15, 14:47:29 // 				    fts_write_reg(i2c_client, 0xd5, 0xff);
//By Gujh_Eric, 2016/6/15, 14:47:29 // 					fts_write_reg(i2c_client, 0xd6, 0xff);
//By Gujh_Eric, 2016/6/15, 14:47:29 // 					fts_write_reg(i2c_client, 0xd7, 0xff);
//By Gujh_Eric, 2016/6/15, 14:47:29 // 				  	fts_write_reg(i2c_client, 0xd8, 0xff);
					msleep(10);
			}
		}

		if(i >= 9)
		{
			TPD_DMESG("TPD gesture write 0x01 to d0 fail \n");
			return;
		}
	}
	}
	/*End Neostra huangxiaohui modify 20160923*/
	#endif

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
	int sem_ret;

	tpd_enter_doze();

	int ret;
	char gestrue_data;
	char gestrue_cmd = 0x03;
	static int scp_init_flag;

	/* TPD_DEBUG("[tpd_scp_doze]:init=%d en=%d", scp_init_flag, tpd_scp_doze_en); */

	mutex_lock(&i2c_access);

	sem_ret = release_md32_semaphore(SEMAPHORE_TOUCH);

	if (scp_init_flag == 0) {
		Touch_IPI_Packet ipi_pkt;

		ipi_pkt.cmd = IPI_COMMAND_AS_CUST_PARAMETER;
		ipi_pkt.param.tcs.i2c_num = TPD_I2C_NUMBER;
		ipi_pkt.param.tcs.int_num = CUST_EINT_TOUCH_PANEL_NUM;
		ipi_pkt.param.tcs.io_int = tpd_int_gpio_number;
		ipi_pkt.param.tcs.io_rst = tpd_rst_gpio_number;

		TPD_DEBUG("[TOUCH]SEND CUST command :%d ", IPI_COMMAND_AS_CUST_PARAMETER);

		ret = md32_ipi_send(IPI_TOUCH, &ipi_pkt, sizeof(ipi_pkt), 0);
		if (ret < 0)
			TPD_DEBUG(" IPI cmd failed (%d)\n", ipi_pkt.cmd);

		msleep(20); /* delay added between continuous command */
		/* Workaround if suffer MD32 reset */
		/* scp_init_flag = 1; */
	}

	if (tpd_scp_doze_en) {
		TPD_DEBUG("[TOUCH]SEND ENABLE GES command :%d ", IPI_COMMAND_AS_ENABLE_GESTURE);
		ret = ftp_enter_doze(i2c_client);
		if (ret < 0) {
			TPD_DEBUG("FTP Enter Doze mode failed\n");
	  } else {
			int retry = 5;
	    {
				/* check doze mode */
				fts_read_reg(i2c_client, FT_GESTRUE_MODE_SWITCH_REG, &gestrue_data);
				TPD_DEBUG("========================>0x%x", gestrue_data);
	    }

	    msleep(20);
			Touch_IPI_Packet ipi_pkt = {.cmd = IPI_COMMAND_AS_ENABLE_GESTURE, .param.data = 1};

			do {
				if (md32_ipi_send(IPI_TOUCH, &ipi_pkt, sizeof(ipi_pkt), 1) == DONE)
					break;
				msleep(20);
				TPD_DEBUG("==>retry=%d", retry);
			} while (retry--);

	    if (retry <= 0)
				TPD_DEBUG("############# md32_ipi_send failed retry=%d", retry);

			/*while(release_md32_semaphore(SEMAPHORE_TOUCH) <= 0) {
				//TPD_DEBUG("GTP release md32 sem failed\n");
				pr_alert("GTP release md32 sem failed\n");
			}*/

		}
		/* disable_irq(touch_irq); */
	}

	mutex_unlock(&i2c_access);
#else
	disable_irq(touch_irq);
	fts_write_reg(i2c_client, 0xA5, data);  /* TP enter sleep mode */

	retval = regulator_disable(tpd->reg);
	if (retval != 0)
		TPD_DMESG("Failed to disable reg-vgp1: %d\n", retval);

#endif

}

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = "FT5x0x",
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
	.attrs = {
		.attr = ft5x0x_attrs,
		.num  = ARRAY_SIZE(ft5x0x_attrs),
	},
};

/* called when loaded into kernel */
static int __init tpd_driver_init(void)
{
#if ACER_GESTURE_WAKEUP
	proc_init(); 
#endif
	TPD_DMESG("MediaTek FT5x0x touch panel driver init\n");
	tpd_get_dts_info();
	if (tpd_driver_add(&tpd_device_driver) < 0)
		TPD_DMESG("add FT5x0x driver failed\n");

	return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void)
{
	TPD_DMESG("MediaTek FT5x0x touch panel driver exit\n");
	tpd_driver_remove(&tpd_device_driver);
#if ACER_GESTURE_WAKEUP
    proc_cleanup();
#endif	    
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);

