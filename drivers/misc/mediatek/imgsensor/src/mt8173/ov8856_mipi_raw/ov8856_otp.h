/*file discription : otp*/
#ifndef _OV8856_OTP_H_
#define _OV8856_OTP_H_

#include <linux/slab.h>

#define OTP_DRV_LSC_SIZE 240

struct otp_struct {
int flag;//bit[7]:info,bit[6]:wb,bit[5]:vcm,bit[4]:lenc
int module_id;
int lens_id;
int production_year;
int production_month;
int production_day;
int rg_ratio;
int bg_ratio;
int lenc[OTP_DRV_LSC_SIZE];
int checksum;
int VCM_start;
int VCM_end;
int VCM_dir;
};

#define RG_Ratio_Typical 0x14d
#define BG_Ratio_Typical 0x14d 

static int read_otp_info(struct otp_struct *otp_ptr);
static int apply_otp(struct otp_struct *otp_ptr);

#endif