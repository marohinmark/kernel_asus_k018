/*
 * Copyright (c)  2012 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicensen
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 */

#include "displays/nt35521_vid.h"
#include "mdfld_dsi_dpi.h"
#include "mdfld_dsi_pkg_sender.h"
#include <linux/gpio.h>
#include "psb_drv.h"
#include <linux/lnw_gpio.h>
#include <asm/intel_scu_pmic.h>
#include <linux/HWVersion.h>
//#include "asus_panel_id.h"

extern int Read_PROJ_ID(void);
extern int Read_HW_ID(void);
static int proj_id=0;
static int board_hw_id=0;

#ifdef ASUS_PROJECT_ME372CL_DISPLAY
#define LCD_BL_EN_R 109          //CPU:GP_CORE_013 +96 = 109
#endif
#define NT35521_PANEL_NAME	"NT35521"

#define NT35521_DEBUG 1

/*
 * GPIO pin definition
 */

#ifdef CONFIG_FE170CG //Pin defenition for FE170CG
#define ISET_SEL 76 //GP_AON_076 , INNOLUX: L
#define RESET_INNO 116 //96+20 GP_CORE_020
#define LCD_ANALONG_PWR_EN 108//96+12 GP_CORE_012
#define PANEL_ID 114 //96+18 GP_CORE_018
#define LCD_BL_EN 0x7E           //PMIC:BACKLIGHT_EN
#define LCD_LOGIC_PWR_EN 0x7F    //PMIC:PANEL_EN
#define PWM_ENABLE_GPIO 49 //LED_BL_PWM GP_AON_049
#define PWMCTRL_REG 0xffae9000
#define PWMCTRL_SIZE 0x80
static void __iomem *pwmctrl_mmio;
#define PWM_ENABLE_GPIO 49
#define PWM_BASE_UNIT 0x444 //5,000Hz

#else
#define LCD_BL_EN 0x7E           //PMIC:BACKLIGHT_EN
#define LCD_LOGIC_PWR_EN 0x7F    //PMIC:PANEL_EN
#define PANEL_ID 115             //CPU:GP_SDIO_2_PWR=core 19=96+16=115
#define LCD_ANALONG_PWR_EN 108   //CPU:GP_CORE_012
#define RESET_INNO 84            //CPU:GP_AON_084

//change PANEL_ID , RESET_INNO for SR
#define PANEL_ID_SR 112          //GP_CORE_016 =16 + 96 =112
#define RESET_INNO_SR 116        //GP_CORE_020 = 20 + 96 =116
#define PANEL_ID_ME175CG_SR 54             //CPU:GP_AON_054_PWR=54

#define PWMCTRL_REG 0xffae9000
#define PWMCTRL_SIZE 0x80
static void __iomem *pwmctrl_mmio;
#define PWM_ENABLE_GPIO 49
#define PWM_BASE_UNIT 0x444 //5,000Hz
#endif





union sst_pwmctrl_reg {
	struct {
		u32 pwmtd:8;
		u32 pwmbu:22;
		u32 pwmswupdate:1;
		u32 pwmenable:1;
	} part;
	u32 full;
};

static struct mdfld_dsi_config *nt35521_dsi_config;

#define CMD_SIZE(x) (sizeof((x)) / sizeof((x)[0]))

static u8 cm1[]={0xFF,0xAA,0x55,0xA5,0x80};
//========== Internal setting ==========
static u8 cm2[]={ 0x6F,0x11,0x00 };
static u8 cm3[]={ 0xF7,0x20,0x00 };
static u8 cm4[]={ 0x6F,0x06 };
static u8 cm5[]={ 0xF7,0xA0 };

static u8 data_cm5[]={ 0xA0 };


static u8 cm6[]={ 0x6F,0x19 };
static u8 cm7[]={ 0xF7,0x12 };

static u8 cm8[]={ 0x6F,0x08 };
static u8 cm9[]={ 0xFA,0x40 };
static u8 cm10[]={ 0x6F,0x11 };
static u8 cm11[]={ 0xF3,0x01 };

static u8 ths_prepare1[]={0x6F,0x01};
static u8 ths_prepare2[]={0xf7,0x03};

static u8 cm1_off[]={0xFF,0xAA,0x55,0xA5,0x00};

//========== page0 relative ==========
static u8 cm12[]={ 0xF0,0x55,0xAA,0x52,0x08,0x00 };
static u8 cm13[]={  0xC8, 0x80 };

static u8 cm14[]={ 0xB1,0x68,0x01 };

static u8 cm15[]={ 0xB6,0x08 };

static u8 cm16[]={ 0x6F,0x02 };
static u8 cm17[]={ 0xB8,0x08 };

static u8 cm18[]={ 0xBB,0x54,0x54 };

static u8 cm19[]={ 0xBC,0x05,0x05 };
static u8 cm20[]={ 0xC7,0x01 };

static u8 cm21[]={ 0xBD, 0x02,0xB0,0x0C,0x0A,0x00 };

//========== page1 relative ==========
static u8 cm22[]={ 0xF0,0x55,0xAA,0x52,0x08,0x01 };

static u8 cm23[]={ 0xB0,0x05,0x05 };
static u8 cm24[]={ 0xB1,0x05,0x05 };

static u8 cm25[]={ 0xBC,0x3A,0x01 };
static u8 cm26[]={ 0xBD,0x3E,0x01 };

static u8 cm27[]={ 0xCA,0x00 };

static u8 cm28[]={ 0xC0,0x04 };

static u8 cm29[]={ 0xBE,0x80 };

//static u8 cm30[]={ 0xB3,0x28,0x28 };
static u8 cm30[]={ 0xB3,0x19,0x19 };

static u8 cm31[]={ 0xB4,0x12,0x12 };

//static u8 cm32[]={ 0xB9,0x34,0x34 };
static u8 cm32[]={ 0xB9,0x24,0x24 };

static u8 cm33[]={ 0xBA,0x14,0x14 };

//========== page2 relative ==========
static u8 cm34[]={ 0xF0,0x55,0xAA,0x52,0x08,0x02 };
static u8 cm35[]={ 0xEE,0x02 };
static u8 cm36[]={ 0xEF,0x09,0x06,0x15,0x18 };

static u8 data_cm36[]={ 0x09,0x06,0x15,0x18 };

static u8 cm37[]={ 0xB0,0x00,0x00,0x00,0x08,0x00,0x17 };
static u8 cm38[]={ 0x6F,0x06 };
static u8 cm39[]={ 0xB0,0x00,0x25,0x00,0x30,0x00,0x45 };
static u8 cm40[]={ 0x6F,0x0C };
static u8 cm41[]={ 0xB0,0x00,0x56,0x00,0x7A };

static u8 data_cm37_cm41[]={0x00,0x00,0x00,0x08,0x00,0x17,
					   0x00,0x25,0x00,0x30,0x00,0x45,
					   0x00,0x56,0x00,0x7A};

static u8 cm42[]={ 0xB1,0x00,0xA3,0x00,0xE7,0x01,0x20 };
static u8 cm43[]={ 0x6F,0x06 };
static u8 cm44[]={ 0xB1,0x01,0x7A,0x01,0xC2,0x01,0xC5 };
static u8 cm45[]={ 0x6F,0x0C };
static u8 cm46[]={ 0xB1,0x02,0x06,0x02,0x5F };

static u8 data_cm42_cm46[]={0x00,0xA3,0x00,0xE7,0x01,0x20,
					   0x01,0x7A,0x01,0xC2,0x01,0xC5,
					   0x02,0x06,0x02,0x5F};

static u8 cm47[]={ 0xB2,0x02,0x92,0x02,0xD0,0x02,0xFC };
static u8 cm48[]={ 0x6F,0x06 };
static u8 cm49[]={ 0xB2,0x03,0x35,0x03,0x5D,0x03,0x8B };
static u8 cm50[]={ 0x6F,0x0C };
static u8 cm51[]={ 0xB2,0x03,0xA2,0x03,0xBF };

static u8 data_cm47_cm51[]={0x02,0x92,0x02,0xD0,0x02,0xFC,
					   0x03,0x35,0x03,0x5D,0x03,0x8B,
					   0x03,0xA2,0x03,0xBF};

static u8 cm52[]={ 0xB3,0x03,0xE8,0x03,0xFF };

static u8 data_cm52[]={ 0x03,0xE8,0x03,0xFF };


static u8 cm53[]={ 0xBC,0x00,0x00,0x00,0x08,0x00,0x18 };
static u8 cm54[]={ 0x6F,0x06 };
static u8 cm55[]={ 0xBC,0x00,0x27,0x00,0x32,0x00,0x49 };
static u8 cm56[]={ 0x6F,0x0C };
static u8 cm57[]={ 0xBC,0x00,0x5C,0x00,0x83 };

static u8 data_cm53_cm57[]={0x00,0x00,0x00,0x08,0x00,0x18,
					   0x00,0x27,0x00,0x32,0x00,0x49,
					   0x00,0x5C,0x00,0x83};

static u8 cm58[]={ 0xBD,0x00,0xAF,0x00,0xF3,0x01,0x2A };
static u8 cm59[]={ 0x6F,0x06 };
static u8 cm60[]={ 0xBD,0x01,0x84,0x01,0xCA,0x01,0xCD };
static u8 cm61[]={ 0x6F,0x0C };
static u8 cm62[]={ 0xBD,0x02,0x0E,0x02,0x65 };

static u8 data_cm58_cm62[]={0x00,0xAF,0x00,0xF3,0x01,0x2A,
					   0x01,0x84,0x01,0xCA,0x01,0xCD,
					   0x02,0x0E,0x02,0x65};

static u8 cm63[]={ 0xBE,0x02,0x98,0x02,0xD4,0x03,0x00 };
static u8 cm64[]={ 0x6F,0x06 };
static u8 cm65[]={ 0xBE,0x03,0x37,0x03,0x5F,0x03,0x8D };
static u8 cm66[]={ 0x6F,0x0C };
static u8 cm67[]={ 0xBE,0x03,0xA4,0x03,0xBF };

static u8 data_cm63_cm67[]={0x02,0x98,0x02,0xD4,0x03,0x00,
					   0x03,0x37,0x03,0x5F,0x03,0x8D,
					   0x03,0xA4,0x03,0xBF};

static u8 cm68[]={ 0xBF,0x03,0xE8,0x03,0xFF };

static u8 data_cm68[]={ 0x03,0xE8,0x03,0xFF };


// PAGE6 : GOUT Mapping, VGLO select
static u8 cm69[]={  0xF0, 0x55,0xAA,0x52,0x08,0x06 };
static u8 cm70[]={  0xB0, 0x00,0x17 };
static u8 cm71[]={  0xB1, 0x16,0x15 };
static u8 cm72[]={  0xB2, 0x14,0x13 };
static u8 cm73[]={  0xB3, 0x12,0x11 };
static u8 cm74[]={  0xB4, 0x10,0x2D };
static u8 cm75[]={  0xB5, 0x01,0x08 };
static u8 cm76[]={  0xB6, 0x09,0x31 };
static u8 cm77[]={  0xB7, 0x31,0x31 };
static u8 cm78[]={  0xB8, 0x31,0x31 };
static u8 cm79[]={  0xB9, 0x31,0x31 };
static u8 cm80[]={  0xBA, 0x31,0x31 };
static u8 cm81[]={  0xBB, 0x31,0x31 };
static u8 cm82[]={  0xBC, 0x31,0x31 };
static u8 cm83[]={  0xBD, 0x31,0x09 };
static u8 cm84[]={  0xBE, 0x08,0x01 };
static u8 cm85[]={  0xBF, 0x2D,0x10 };
static u8 cm86[]={  0xC0, 0x11,0x12 };
static u8 cm87[]={  0xC1, 0x13,0x14 };
static u8 cm88[]={  0xC2, 0x15,0x16 };
static u8 cm89[]={  0xC3, 0x17,0x00 };
static u8 cm90[]={  0xE5, 0x31,0x31 };
static u8 cm91[]={  0xC4, 0x00,0x17 };
static u8 cm92[]={  0xC5, 0x16,0x15 };
static u8 cm93[]={  0xC6, 0x14,0x13 };
static u8 cm94[]={  0xC7, 0x12,0x11 };
static u8 cm95[]={  0xC8, 0x10,0x2D };
static u8 cm96[]={  0xC9, 0x01,0x08 };
static u8 cm97[]={  0xCA, 0x09,0x31 };
static u8 cm98[]={  0xCB, 0x31,0x31 };
static u8 cm99[]={  0xCC, 0x31,0x31 };
static u8 cm100[]={  0xCD, 0x31,0x31 };
static u8 cm101[]={  0xCE, 0x31,0x31 };
static u8 cm102[]={  0xCF, 0x31,0x31 };
static u8 cm103[]={  0xD0, 0x31,0x31 };
static u8 cm104[]={  0xD1, 0x31,0x09 };
static u8 cm105[]={  0xD2, 0x08,0x01 };
static u8 cm106[]={  0xD3, 0x2D,0x10 };
static u8 cm107[]={  0xD4, 0x11,0x12 };
static u8 cm108[]={  0xD5, 0x13,0x14 };
static u8 cm109[]={  0xD6, 0x15,0x16 };
static u8 cm110[]={  0xD7, 0x17,0x00 };
static u8 cm111[]={  0xE6, 0x31,0x31 };
static u8 cm112[]={  0xD8, 0x00,0x00,0x00,0x00,0x00 };
static u8 cm113[]={  0xD9, 0x00,0x00,0x00,0x00,0x00 };
static u8 cm114[]={  0xE7, 0x00 };

// PAGE3 :
static u8 cm115[]={  0xF0, 0x55,0xAA,0x52,0x08,0x03 };
static u8 cm116[]={  0xB0, 0x20,0x00 };
static u8 cm117[]={  0xB1, 0x20,0x00 };
static u8 cm118[]={  0xB2, 0x05,0x00,0x42,0x00,0x00 };
static u8 cm119[]={  0xB6, 0x05,0x00,0x42,0x00,0x00 };
static u8 cm120[]={  0xBA, 0x53,0x00,0x42,0x00,0x00 };
static u8 cm121[]={  0xBB, 0x53,0x00,0x42,0x00,0x00 };
static u8 cm122[]={  0xC4, 0x40 };

// PAGE5 :
static u8 cm123[]={  0xF0, 0x55,0xAA,0x52,0x08,0x05 };
static u8 cm124[]={  0xB0, 0x17,0x06 };
static u8 cm125[]={  0xB8, 0x00 };
static u8 cm126[]={  0xBD, 0x03,0x01,0x01,0x00,0x01 };
static u8 cm127[]={  0xB1, 0x17,0x06 };
static u8 cm128[]={  0xB9, 0x00,0x01 };
static u8 cm129[]={  0xB2, 0x17,0x06 };
static u8 cm130[]={  0xBA, 0x00,0x01 };
static u8 cm131[]={  0xB3, 0x17,0x06 };
static u8 cm132[]={  0xBB, 0x0A,0x00 };
static u8 cm133[]={  0xB4, 0x17,0x06 };
static u8 cm134[]={  0xB5, 0x17,0x06 };
static u8 cm135[]={  0xB6, 0x14,0x03 };
static u8 cm136[]={  0xB7, 0x00,0x00 };
static u8 cm137[]={  0xBC, 0x02,0x01 };
static u8 cm138[]={  0xC0, 0x05 };
static u8 cm139[]={  0xC4, 0xA5 };
static u8 cm140[]={  0xC8, 0x03,0x30 };
static u8 cm141[]={  0xC9, 0x03,0x51 };
static u8 cm142[]={  0xD1, 0x00,0x05,0x03,0x00,0x00 };
static u8 cm143[]={  0xD2, 0x00,0x05,0x09,0x00,0x00 };
static u8 cm144[]={  0xE5, 0x02 };
static u8 cm145[]={  0xE6, 0x02 };
static u8 cm146[]={  0xE7, 0x02 };
static u8 cm147[]={  0xE9, 0x02 };
static u8 cm148[]={  0xED, 0x33 };

static u8 cm149[]={  0x11 };
static u8 cm150[]={  0x29 };
//	static u8 cm151[]={  0x35, 0X00 };

#if 0
static u8 bist_cm1[]={  0xF0, 0x55,0xAA,0x52,0x08,0x00  };
static u8 bist_cm2[]={  0xEF, 0x03,0xFF  };
static u8 bist_cm3[]={  0xEE, 0x87,0x78,0x02,0x40 };

static u8 test_cm1[]={ 0x34,0x00 };
static u8 test_cm2[]={ 0x6F, 0x16 };
static u8 test_cm3[]={ 0xF7, 0x10,0xff,0x71 };
static u8 test_cm4[]={ 0xf0  , 0x55  ,0xaa  ,0x52  ,0x8   ,0x0 };
static u8 test_cm5[]={ 0xB1, 0x28, 0x01 };


static u8 test_short_cm1[]={ 0x6F, 0x00 };
static u8 test_short_cm2[]={  0xFF,0xAA };
static u8 test_short_cm3[]={  0x6F, 0x01 };
static u8 test_short_cm4[]={  0xFF,0x55 };
static u8 test_short_cm5[]={  0x6F, 0x02 };
static u8 test_short_cm6[]={  0xFF,0xA5 };
static u8 test_short_cm7[]={  0x6F, 0x03 };
static u8 test_short_cm8[]={  0xFF,0x80 };

static u8 test_short_cm9[]={  0x34,0x00 };
static u8 test_short_cm10[]={  0x6F, 0x16 };
static u8 test_short_cm11[]={  0xF7, 0x10 };
static u8 test_short_cm12[]={  0x6F, 0x17 };
static u8 test_short_cm13[]={  0xF7,0xff };
static u8 test_short_cm14[]={  0x6F, 0x18 };
static u8 test_short_cm15[]={  0xF7,0x71 };
static u8 test_short_cm16[]={  0x6F, 0x00 };
static u8 test_short_cm17[]={  0xf0  , 0x55  };
static u8 test_short_cm18[]={  0x6F, 0x01};
static u8 test_short_cm19[]={  0xf0  , 0xaa  };
static u8 test_short_cm20[]={  0x6F, 0x02 };
static u8 test_short_cm21[]={  0xf0  , 0x52 };
static u8 test_short_cm22[]={  0x6F, 0x03 };
static u8 test_short_cm23[]={  0xf0  , 0x08};
static u8 test_short_cm24[]={  0x6F, 0x04 };
static u8 test_short_cm25[]={  0xf0  , 0x00 };
static u8 test_short_cm26[]={  0x6F, 0x00};
static u8 test_short_cm27[]={  0xB1  , 0x28 };
static u8 test_short_cm28[]={  0x6F, 0x01 };
static u8 test_short_cm29[]={  0xB1  , 0x01 };
static u8 test_short_cm30[]={  0x11  };
#endif


static int pwm_configure(int duty)
{
	union sst_pwmctrl_reg pwmctrl;

	/*Read the PWM register to make sure there is no pending
	*update.
	*/
	pwmctrl.full = readl(pwmctrl_mmio);

	/*check pwnswupdate bit */
	if (pwmctrl.part.pwmswupdate)
		return -EBUSY;
	pwmctrl.part.pwmswupdate = 0x1;
	pwmctrl.part.pwmbu = PWM_BASE_UNIT;
	pwmctrl.part.pwmtd = duty;
	writel(pwmctrl.full,  pwmctrl_mmio);

	return 0;
}


static void pwm_enable(){
	union sst_pwmctrl_reg pwmctrl;

	lnw_gpio_set_alt(PWM_ENABLE_GPIO, LNW_ALT_2);

	/*Enable the PWM by setting PWM enable bit to 1 */
	pwmctrl.full = readl(pwmctrl_mmio);
	pwmctrl.part.pwmenable = 1;
	writel(pwmctrl.full, pwmctrl_mmio);
}

static void pwm_disable(){
	union sst_pwmctrl_reg pwmctrl;
	/*setting PWM enable bit to 0 */
	pwmctrl.full = readl(pwmctrl_mmio);
	pwmctrl.part.pwmenable = 0;
	writel(pwmctrl.full,  pwmctrl_mmio);

	gpio_set_value(PWM_ENABLE_GPIO, 0);
	lnw_gpio_set_alt(PWM_ENABLE_GPIO, 0);
}


//change long cmd to short cmd
static int send_mipi_cmd(
				struct mdfld_dsi_pkg_sender * sender,
				u8 * data,
				u32 len){

	switch(len){
		case 1:
			mdfld_dsi_send_mcs_short_lp(sender, data[0],0, 0, 0);
			break;
		case 2:
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[1], 1, 0);
			break;
		case 3:
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x00, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[1], 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x01, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[2], 1, 0);
			break;
		case 4:
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x00, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[1], 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x01, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[2], 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x02, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[3], 1, 0);
			break;
		case 5:
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x00, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[1], 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x01, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[2], 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x02, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[3], 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x03, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[4], 1, 0);
			break;
		case 6:
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x00, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[1], 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x01, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[2], 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x02, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[3], 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x03, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[4], 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x04, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[5], 1, 0);
			break;

		case 7:
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x00, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[1], 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x01, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[2], 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x02, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[3], 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x03, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[4], 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x04, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[5], 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x05, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[6], 1, 0);
			break;
		default:
			mdfld_dsi_send_mcs_long_lp(sender, data, len, 0);
			break;
	}
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL){
		printk("%s : MDFLD_DSI_CONTROL_ABNORMAL\n",__func__);
		return -EIO;
	}
	else{
		return 0;
	}
}

//normal
static int send_mipi_cmd2(
				struct mdfld_dsi_pkg_sender * sender,
				u8 * data,
				u32 len){

	switch(len){
		case 1:
			mdfld_dsi_send_mcs_short_lp(sender, data[0],0, 0, 0);
			break;
		case 2:
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[1], 1, 0);;
			break;
		default:
			mdfld_dsi_send_mcs_long_lp(sender, data, len, 0);
			break;
	}
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL){
		printk("%s : MDFLD_DSI_CONTROL_ABNORMAL\n",__func__);
		return -EIO;
	}
	else{
		return 0;
	}

}
#define LP_RETRY 5


//compare
static int compare_mipi_reg(
				struct mdfld_dsi_pkg_sender * sender,
				u8 * data,
				u32 len,
				u8 * compare_data){
	int ret =0;
	int retry_times=LP_RETRY;
	u8 data2[20]={0};
	int i,r=0;

	r = mdfld_dsi_read_mcs_lp(sender,data[0],data2, len);

	ret = memcmp(data2, compare_data, len);
	if (!ret){
		return 0;
	}else{
		printk("%s : %d, %02x,",__func__,r,data[0]);
		for(i=0;i<len;i++){
			printk(" %02x", data2[i]);
		}
		printk(" (compare fail, retry again)\n");
		return -EIO;
	}
}


static int mipi_cmd_setting(struct mdfld_dsi_config *dsi_config){

	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int r = 0;
	u8 data[10]={0};
	u8 rdata;
	int retry_times;
	int reset_inno;
#ifdef CONFIG_FE170CG 
	reset_inno = RESET_INNO;
#elif ASUS_PROJECT_ME372CL_DISPLAY
	reset_inno = RESET_INNO_SR;
#elif ASUS_PROJECT_ME175CG_DISPLAY
	reset_inno = RESET_INNO_SR;
#elif ASUS_PROJECT_ME372CG_DISPLAY
	if(proj_id == PROJ_ID_ME372CG_CD)
		reset_inno = RESET_INNO_SR;
	else{
		if(board_hw_id == HW_ID_SR1){ //ME371MG HW_ID_SR1 = EVB
			reset_inno = RESET_INNO;
		}else{
			reset_inno = RESET_INNO_SR;
		}
	}
#endif

	r = mdfld_dsi_read_mcs_lp(sender,0xa,&rdata, 1);


	//RESX : L > H > L > H

	if (gpio_direction_output(reset_inno, 0))
		gpio_set_value_cansleep(reset_inno, 0);

	usleep_range(40000, 45000);             // > 40ms  L

	if (gpio_direction_output(reset_inno, 1))
		gpio_set_value_cansleep(reset_inno, 1);

	usleep_range(2000, 2200);                 //         H

	if (gpio_direction_output(reset_inno, 0))
		gpio_set_value_cansleep(reset_inno, 0);

	usleep_range(2000, 2200);                   //>10 us  L

	if (gpio_direction_output(reset_inno, 1))
		gpio_set_value_cansleep(reset_inno, 1);

	usleep_range(30000, 35000);      //>20ms  H


	retry_times=LP_RETRY;
	do{
		retry_times--;
		send_mipi_cmd2(sender,cm1,CMD_SIZE(cm1));

		//========== Internal setting ==========
		send_mipi_cmd2(sender,cm2,CMD_SIZE(cm2));
		send_mipi_cmd2(sender,cm3,CMD_SIZE(cm3));
		send_mipi_cmd2(sender,cm4,CMD_SIZE(cm4));
		send_mipi_cmd2(sender,cm5,CMD_SIZE(cm5));
		send_mipi_cmd2(sender,cm4,CMD_SIZE(cm4));

		//r = compare_mipi_reg(sender,cm5,CMD_SIZE(data_cm5),data_cm5);

		if(!r) break;
		//if(!retry_times) return -EIO;

	}while(retry_times);

	send_mipi_cmd2(sender,cm6,CMD_SIZE(cm6));
	send_mipi_cmd2(sender,cm7,CMD_SIZE(cm7));
	send_mipi_cmd2(sender,cm8,CMD_SIZE(cm8));
	send_mipi_cmd2(sender,cm9,CMD_SIZE(cm9));
	send_mipi_cmd2(sender,cm10,CMD_SIZE(cm10));
	send_mipi_cmd2(sender,cm11,CMD_SIZE(cm11));
	send_mipi_cmd2(sender,ths_prepare1,CMD_SIZE(ths_prepare1));
	send_mipi_cmd2(sender,ths_prepare2,CMD_SIZE(ths_prepare2));

	send_mipi_cmd2(sender,cm1_off,CMD_SIZE(cm1_off));





	//========== page0 relative ==========
	send_mipi_cmd(sender,cm12,CMD_SIZE(cm12));
	send_mipi_cmd(sender,cm13,CMD_SIZE(cm13)); //Black Frame Insertion when occur error
	send_mipi_cmd(sender,cm14,CMD_SIZE(cm14));
	send_mipi_cmd(sender,cm15,CMD_SIZE(cm15));
	send_mipi_cmd(sender,cm16,CMD_SIZE(cm16));
	send_mipi_cmd(sender,cm17,CMD_SIZE(cm17));
	send_mipi_cmd(sender,cm18,CMD_SIZE(cm18));
	send_mipi_cmd(sender,cm19,CMD_SIZE(cm19));
	send_mipi_cmd(sender,cm20,CMD_SIZE(cm20));
	send_mipi_cmd(sender,cm21,CMD_SIZE(cm21));


	//========== page1 relative ==========
	send_mipi_cmd(sender,cm22,CMD_SIZE(cm22));
	send_mipi_cmd(sender,cm23,CMD_SIZE(cm23));
	send_mipi_cmd(sender,cm24,CMD_SIZE(cm24));
	send_mipi_cmd(sender,cm25,CMD_SIZE(cm25));
	send_mipi_cmd(sender,cm26,CMD_SIZE(cm26));
	send_mipi_cmd(sender,cm27,CMD_SIZE(cm27));
	send_mipi_cmd(sender,cm28,CMD_SIZE(cm28));
	send_mipi_cmd(sender,cm29,CMD_SIZE(cm29));
	send_mipi_cmd(sender,cm30,CMD_SIZE(cm30));
	send_mipi_cmd(sender,cm31,CMD_SIZE(cm31));
	send_mipi_cmd(sender,cm32,CMD_SIZE(cm32));
	send_mipi_cmd(sender,cm33,CMD_SIZE(cm33));





	//========== page2 relative ==========
	send_mipi_cmd(sender,cm34,CMD_SIZE(cm34));
	send_mipi_cmd(sender,cm35,CMD_SIZE(cm35));

	retry_times=LP_RETRY;
	do{
		retry_times--;
		send_mipi_cmd2(sender,cm36,CMD_SIZE(cm36));
		//r = compare_mipi_reg(sender,cm36,CMD_SIZE(data_cm36),data_cm36);

		if(!r) break;
		//if(!retry_times) return -EIO;

	}while(retry_times);

	retry_times=LP_RETRY;
	do{
		retry_times--;
		send_mipi_cmd2(sender,cm37,CMD_SIZE(cm37));
		send_mipi_cmd2(sender,cm38,CMD_SIZE(cm38));
		send_mipi_cmd2(sender,cm39,CMD_SIZE(cm39));
		send_mipi_cmd2(sender,cm40,CMD_SIZE(cm40));
		send_mipi_cmd2(sender,cm41,CMD_SIZE(cm41));

		//r = compare_mipi_reg(sender,cm37,CMD_SIZE(data_cm37_cm41),data_cm37_cm41);

		if(!r) break;
		//if(!retry_times) return -EIO;


	}while(retry_times);

	retry_times=LP_RETRY;
	do{
		retry_times--;
		send_mipi_cmd2(sender,cm42,CMD_SIZE(cm42));
		send_mipi_cmd2(sender,cm43,CMD_SIZE(cm43));
		send_mipi_cmd2(sender,cm44,CMD_SIZE(cm44));
		send_mipi_cmd2(sender,cm45,CMD_SIZE(cm45));
		send_mipi_cmd2(sender,cm46,CMD_SIZE(cm46));


		//r = compare_mipi_reg(sender,cm42,CMD_SIZE(data_cm42_cm46),data_cm42_cm46);

		if(!r) break;
		//if(!retry_times) return -EIO;


	}while(retry_times);

	retry_times=LP_RETRY;
	do{
		retry_times--;
		send_mipi_cmd2(sender,cm47,CMD_SIZE(cm47));
		send_mipi_cmd2(sender,cm48,CMD_SIZE(cm48));
		send_mipi_cmd2(sender,cm49,CMD_SIZE(cm49));
		send_mipi_cmd2(sender,cm50,CMD_SIZE(cm50));
		send_mipi_cmd2(sender,cm51,CMD_SIZE(cm51));

		//r = compare_mipi_reg(sender,cm47,CMD_SIZE(data_cm47_cm51),data_cm47_cm51);

		if(!r) break;
		//if(!retry_times) return -EIO;


	}while(retry_times);


	retry_times=LP_RETRY;
	do{
		retry_times--;
		send_mipi_cmd2(sender,cm52,CMD_SIZE(cm52));
		//r = compare_mipi_reg(sender,cm52,CMD_SIZE(data_cm52),data_cm52);

		if(!r) break;
		//if(!retry_times) return -EIO;

	}while(retry_times);


	retry_times=LP_RETRY;
	do{
		retry_times--;
		send_mipi_cmd2(sender,cm53,CMD_SIZE(cm53));
		send_mipi_cmd2(sender,cm54,CMD_SIZE(cm54));
		send_mipi_cmd2(sender,cm55,CMD_SIZE(cm55));
		send_mipi_cmd2(sender,cm56,CMD_SIZE(cm56));
		send_mipi_cmd2(sender,cm57,CMD_SIZE(cm57));

		//r = compare_mipi_reg(sender,cm53,CMD_SIZE(data_cm53_cm57),data_cm53_cm57);

		if(!r) break;
		//if(!retry_times) return -EIO;


	}while(retry_times);

	retry_times=LP_RETRY;
	do{
		retry_times--;
		send_mipi_cmd2(sender,cm58,CMD_SIZE(cm58));
		send_mipi_cmd2(sender,cm59,CMD_SIZE(cm59));
		send_mipi_cmd2(sender,cm60,CMD_SIZE(cm60));
		send_mipi_cmd2(sender,cm61,CMD_SIZE(cm61));
		send_mipi_cmd2(sender,cm62,CMD_SIZE(cm62));


		//r = compare_mipi_reg(sender,cm58,CMD_SIZE(data_cm58_cm62),data_cm58_cm62);

		if(!r) break;
		//if(!retry_times) return -EIO;


	}while(retry_times);

	retry_times=LP_RETRY;
	do{
		retry_times--;
		send_mipi_cmd2(sender,cm63,CMD_SIZE(cm63));
		send_mipi_cmd2(sender,cm64,CMD_SIZE(cm64));
		send_mipi_cmd2(sender,cm65,CMD_SIZE(cm65));
		send_mipi_cmd2(sender,cm66,CMD_SIZE(cm66));
		send_mipi_cmd2(sender,cm67,CMD_SIZE(cm67));

		//r = compare_mipi_reg(sender,cm63,CMD_SIZE(data_cm63_cm67),data_cm63_cm67);

		if(!r) break;
		//if(!retry_times) return -EIO;


	}while(retry_times);

	retry_times=LP_RETRY;
	do{
		retry_times--;
		send_mipi_cmd2(sender,cm68,CMD_SIZE(cm68));

		//r = compare_mipi_reg(sender,cm68,CMD_SIZE(data_cm68),data_cm68);

		if(!r) break;
		//if(!retry_times) return -EIO;

	}while(retry_times);


	// PAGE6 : GOUT Mapping, VGLO select
	send_mipi_cmd(sender,cm69,CMD_SIZE(cm69));
	send_mipi_cmd(sender,cm70,CMD_SIZE(cm70));
	send_mipi_cmd(sender,cm71,CMD_SIZE(cm71));
	send_mipi_cmd(sender,cm72,CMD_SIZE(cm72));
	send_mipi_cmd(sender,cm73,CMD_SIZE(cm73));
	send_mipi_cmd(sender,cm74,CMD_SIZE(cm74));
	send_mipi_cmd(sender,cm75,CMD_SIZE(cm75));
	send_mipi_cmd(sender,cm76,CMD_SIZE(cm76));
	send_mipi_cmd(sender,cm77,CMD_SIZE(cm77));
	send_mipi_cmd(sender,cm78,CMD_SIZE(cm78));
	send_mipi_cmd(sender,cm79,CMD_SIZE(cm79));
	send_mipi_cmd(sender,cm80,CMD_SIZE(cm80));
	send_mipi_cmd(sender,cm81,CMD_SIZE(cm81));
	send_mipi_cmd(sender,cm82,CMD_SIZE(cm82));
	send_mipi_cmd(sender,cm83,CMD_SIZE(cm83));
	send_mipi_cmd(sender,cm84,CMD_SIZE(cm84));
	send_mipi_cmd(sender,cm85,CMD_SIZE(cm85));
	send_mipi_cmd(sender,cm86,CMD_SIZE(cm86));
	send_mipi_cmd(sender,cm87,CMD_SIZE(cm87));
	send_mipi_cmd(sender,cm88,CMD_SIZE(cm88));
	send_mipi_cmd(sender,cm89,CMD_SIZE(cm89));
	send_mipi_cmd(sender,cm90,CMD_SIZE(cm90));
	send_mipi_cmd(sender,cm91,CMD_SIZE(cm91));
	send_mipi_cmd(sender,cm92,CMD_SIZE(cm92));
	send_mipi_cmd(sender,cm93,CMD_SIZE(cm93));
	send_mipi_cmd(sender,cm94,CMD_SIZE(cm94));
	send_mipi_cmd(sender,cm95,CMD_SIZE(cm95));
	send_mipi_cmd(sender,cm96,CMD_SIZE(cm96));
	send_mipi_cmd(sender,cm97,CMD_SIZE(cm97));
	send_mipi_cmd(sender,cm98,CMD_SIZE(cm98));
	send_mipi_cmd(sender,cm99,CMD_SIZE(cm99));
	send_mipi_cmd(sender,cm100,CMD_SIZE(cm100));
	send_mipi_cmd(sender,cm101,CMD_SIZE(cm101));
	send_mipi_cmd(sender,cm102,CMD_SIZE(cm102));
	send_mipi_cmd(sender,cm103,CMD_SIZE(cm103));
	send_mipi_cmd(sender,cm104,CMD_SIZE(cm104));
	send_mipi_cmd(sender,cm105,CMD_SIZE(cm105));
	send_mipi_cmd(sender,cm106,CMD_SIZE(cm106));
	send_mipi_cmd(sender,cm107,CMD_SIZE(cm107));
	send_mipi_cmd(sender,cm108,CMD_SIZE(cm108));
	send_mipi_cmd(sender,cm109,CMD_SIZE(cm109));
	send_mipi_cmd(sender,cm110,CMD_SIZE(cm110));
	send_mipi_cmd(sender,cm111,CMD_SIZE(cm111));
	send_mipi_cmd(sender,cm112,CMD_SIZE(cm112));
	send_mipi_cmd(sender,cm113,CMD_SIZE(cm113));
	send_mipi_cmd(sender,cm114,CMD_SIZE(cm114));


	// PAGE3 :
	send_mipi_cmd(sender,cm115,CMD_SIZE(cm115));
	send_mipi_cmd(sender,cm116,CMD_SIZE(cm116));
	send_mipi_cmd(sender,cm117,CMD_SIZE(cm117));
	send_mipi_cmd(sender,cm118,CMD_SIZE(cm118));
	send_mipi_cmd(sender,cm119,CMD_SIZE(cm119));
	send_mipi_cmd(sender,cm120,CMD_SIZE(cm120));
	send_mipi_cmd(sender,cm121,CMD_SIZE(cm121));
	send_mipi_cmd(sender,cm122,CMD_SIZE(cm122));


	// PAGE5 :
	send_mipi_cmd(sender,cm123,CMD_SIZE(cm123));
	send_mipi_cmd(sender,cm124,CMD_SIZE(cm124));
	send_mipi_cmd(sender,cm125,CMD_SIZE(cm125));
	send_mipi_cmd(sender,cm126,CMD_SIZE(cm126));
	send_mipi_cmd(sender,cm127,CMD_SIZE(cm127));
	send_mipi_cmd(sender,cm128,CMD_SIZE(cm128));
	send_mipi_cmd(sender,cm129,CMD_SIZE(cm129));
	send_mipi_cmd(sender,cm130,CMD_SIZE(cm130));
	send_mipi_cmd(sender,cm131,CMD_SIZE(cm131));
	send_mipi_cmd(sender,cm132,CMD_SIZE(cm132));
	send_mipi_cmd(sender,cm133,CMD_SIZE(cm133));
	send_mipi_cmd(sender,cm134,CMD_SIZE(cm134));
	send_mipi_cmd(sender,cm135,CMD_SIZE(cm135));
	send_mipi_cmd(sender,cm136,CMD_SIZE(cm136));
	send_mipi_cmd(sender,cm137,CMD_SIZE(cm137));
	send_mipi_cmd(sender,cm138,CMD_SIZE(cm138));
	send_mipi_cmd(sender,cm139,CMD_SIZE(cm139));
	send_mipi_cmd(sender,cm140,CMD_SIZE(cm140));
	send_mipi_cmd(sender,cm141,CMD_SIZE(cm141));
	send_mipi_cmd(sender,cm142,CMD_SIZE(cm142));
	send_mipi_cmd(sender,cm143,CMD_SIZE(cm143));
	send_mipi_cmd(sender,cm144,CMD_SIZE(cm144));
	send_mipi_cmd(sender,cm145,CMD_SIZE(cm145));
	send_mipi_cmd(sender,cm146,CMD_SIZE(cm146));
	send_mipi_cmd(sender,cm147,CMD_SIZE(cm147));
	send_mipi_cmd(sender,cm148,CMD_SIZE(cm148));

	send_mipi_cmd(sender,cm149,CMD_SIZE(cm149));
	send_mipi_cmd(sender,cm150,CMD_SIZE(cm150));
	//send_mipi_cmd(sender,cm151,CMD_SIZE(cm151)); //show TE signal

#if ASUS_PROJECT_ME372CG_DISPLAY
	if(proj_id == PROJ_ID_ME372CG_CD)
		;
	else {
		if(board_hw_id == HW_ID_SR1){ //ME371MG HW_ID_SR1 = EVB
			mdfld_dsi_send_mcs_short_lp(sender,0x53,0x2c,1,0); //brightness setting
		}
	}
#endif

#if 0
	//BIST
	send_mipi_cmd(sender,bist_cm1,CMD_SIZE(bist_cm1));
	send_mipi_cmd(sender,bist_cm2,CMD_SIZE(bist_cm2));
	send_mipi_cmd(sender,bist_cm3,CMD_SIZE(bist_cm3));
#endif

#if 0
	//read test
	r = mdfld_dsi_read_gen_lp(sender,
          0xf0, 0, 1,
          data, 5);
	pr_info("#### device code: %d %02x %02x %02x %02x %02x\n", r, data[0], data[1], data[2], data[3], data[4]);
#endif

#if 0
	//TE show error code (long cmd version)
	send_mipi_cmd(sender,test_cm1,CMD_SIZE(test_cm1));
	send_mipi_cmd(sender,test_cm2,CMD_SIZE(test_cm2));
	send_mipi_cmd(sender,test_cm3,CMD_SIZE(test_cm3));
	send_mipi_cmd(sender,test_cm4,CMD_SIZE(test_cm4));
	send_mipi_cmd(sender,test_cm5,CMD_SIZE(test_cm5));
#endif

#if 0
	//TE show error code (short cmd version)
	send_mipi_cmd(sender,test_short_cm9,CMD_SIZE(test_short_cm9));
	send_mipi_cmd(sender,test_short_cm10,CMD_SIZE(test_short_cm10));
	send_mipi_cmd(sender,test_short_cm11,CMD_SIZE(test_short_cm11));
	send_mipi_cmd(sender,test_short_cm12,CMD_SIZE(test_short_cm12));
	send_mipi_cmd(sender,test_short_cm13,CMD_SIZE(test_short_cm13));
	send_mipi_cmd(sender,test_short_cm14,CMD_SIZE(test_short_cm14));
	send_mipi_cmd(sender,test_short_cm15,CMD_SIZE(test_short_cm15));
	send_mipi_cmd(sender,test_short_cm16,CMD_SIZE(test_short_cm16));
	send_mipi_cmd(sender,test_short_cm17,CMD_SIZE(test_short_cm17));
	send_mipi_cmd(sender,test_short_cm18,CMD_SIZE(test_short_cm18));
	send_mipi_cmd(sender,test_short_cm19,CMD_SIZE(test_short_cm19));
	send_mipi_cmd(sender,test_short_cm20,CMD_SIZE(test_short_cm20));
	send_mipi_cmd(sender,test_short_cm21,CMD_SIZE(test_short_cm21));
	send_mipi_cmd(sender,test_short_cm22,CMD_SIZE(test_short_cm22));
	send_mipi_cmd(sender,test_short_cm23,CMD_SIZE(test_short_cm23));
	send_mipi_cmd(sender,test_short_cm24,CMD_SIZE(test_short_cm24));
	send_mipi_cmd(sender,test_short_cm25,CMD_SIZE(test_short_cm25));
	send_mipi_cmd(sender,test_short_cm26,CMD_SIZE(test_short_cm26));
	send_mipi_cmd(sender,test_short_cm27,CMD_SIZE(test_short_cm27));
	send_mipi_cmd(sender,test_short_cm28,CMD_SIZE(test_short_cm28));
	send_mipi_cmd(sender,test_short_cm29,CMD_SIZE(test_short_cm29));
	send_mipi_cmd(sender,test_short_cm30,CMD_SIZE(test_short_cm30));

#endif

	return 0;

}

static void
nt35521_vid_dsi_controller_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_context *hw_ctx = &dsi_config->dsi_hw_context;

	struct drm_device *dev = dsi_config->dev;
	struct csc_setting csc = {	.pipe = 0,
								.type = CSC_REG_SETTING,
								.enable_state = true,
								.data_len = CSC_REG_COUNT,
								.data.csc_reg_data = {
									0x400, 0x0, 0x4000000, 0x0, 0x0, 0x400}
							 };
	struct gamma_setting gamma = {	.pipe = 0,
									.type = GAMMA_REG_SETTING,
									.enable_state = true,
									.data_len = GAMMA_10_BIT_TABLE_COUNT,
									.gamma_tableX100 = {
										0x000000, 0x020202, 0x040404, 0x060606,
										0x080808, 0x0A0A0A, 0x0C0C0C, 0x0E0E0E,
										0x101010, 0x121212, 0x141414, 0x161616,
										0x181818, 0x1A1A1A, 0x1C1C1C, 0x1E1E1E,
										0x202020, 0x222222, 0x242424, 0x262626,
										0x282828, 0x2A2A2A, 0x2C2C2C, 0x2E2E2E,
										0x303030, 0x323232, 0x343434, 0x363636,
										0x383838, 0x3A3A3A, 0x3C3C3C, 0x3E3E3E,
										0x404040, 0x424242, 0x444444, 0x464646,
										0x484848, 0x4A4A4A, 0x4C4C4C, 0x4E4E4E,
										0x505050, 0x525252, 0x545454, 0x565656,
										0x585858, 0x5A5A5A, 0x5C5C5C, 0x5E5E5E,
										0x606060, 0x626262, 0x646464, 0x666666,
										0x686868, 0x6A6A6A, 0x6C6C6C, 0x6E6E6E,
										0x707070, 0x727272, 0x747474, 0x767676,
										0x787878, 0x7A7A7A, 0x7C7C7C, 0x7E7E7E,
										0x808080, 0x828282, 0x848484, 0x868686,
										0x888888, 0x8A8A8A, 0x8C8C8C, 0x8E8E8E,
										0x909090, 0x929292, 0x949494, 0x969696,
										0x989898, 0x9A9A9A, 0x9C9C9C, 0x9E9E9E,
										0xA0A0A0, 0xA2A2A2, 0xA4A4A4, 0xA6A6A6,
										0xA8A8A8, 0xAAAAAA, 0xACACAC, 0xAEAEAE,
										0xB0B0B0, 0xB2B2B2, 0xB4B4B4, 0xB6B6B6,
										0xB8B8B8, 0xBABABA, 0xBCBCBC, 0xBEBEBE,
										0xC0C0C0, 0xC2C2C2, 0xC4C4C4, 0xC6C6C6,
										0xC8C8C8, 0xCACACA, 0xCCCCCC, 0xCECECE,
										0xD0D0D0, 0xD2D2D2, 0xD4D4D4, 0xD6D6D6,
										0xD8D8D8, 0xDADADA, 0xDCDCDC, 0xDEDEDE,
										0xE0E0E0, 0xE2E2E2, 0xE4E4E4, 0xE6E6E6,
										0xE8E8E8, 0xEAEAEA, 0xECECEC, 0xEEEEEE,
										0xF0F0F0, 0xF2F2F2, 0xF4F4F4, 0xF6F6F6,
										0xF8F8F8, 0xFAFAFA, 0xFCFCFC, 0xFEFEFE,
										0x010000, 0x010000, 0x010000}
								 };

	/* Reconfig lane configuration */
	dsi_config->lane_count = 4;
	dsi_config->lane_config = MDFLD_DSI_DATA_LANE_4_0;
	dsi_config->enable_gamma_csc = ENABLE_GAMMA | ENABLE_CSC;

	hw_ctx->cck_div = 0;
	hw_ctx->pll_bypass_mode = 1;
	hw_ctx->mipi_control = 0x18;
	hw_ctx->intr_en = 0xffffffff;
	hw_ctx->hs_tx_timeout = 0xdcf50;
	hw_ctx->lp_rx_timeout = 0xffff;
	hw_ctx->turn_around_timeout = 0x3f;
	hw_ctx->device_reset_timer = 0xffff;
	hw_ctx->high_low_switch_count = 0x18;
	hw_ctx->init_count = 0x7d0;
	hw_ctx->eot_disable = 0x3;
	hw_ctx->lp_byteclk = 0x4;
	hw_ctx->clk_lane_switch_time_cnt = 0x18000b;
	hw_ctx->dphy_param = 0x2a0f5f0c;

	/* Setup video mode format */
	hw_ctx->video_mode_format = 0xe;

	/* Set up func_prg, RGB888(0x200) */
	hw_ctx->dsi_func_prg = (0x200 | dsi_config->lane_count);

	/* Setup mipi port configuration */
	hw_ctx->mipi = PASS_FROM_SPHY_TO_AFE | dsi_config->lane_config;

	nt35521_dsi_config = dsi_config;

	if (dsi_config->enable_gamma_csc & ENABLE_CSC) {
		/* setting the tuned csc setting */
		drm_psb_enable_color_conversion = 1;
		mdfld_intel_crtc_set_color_conversion(dev, &csc);
	}

	if (dsi_config->enable_gamma_csc & ENABLE_GAMMA) {
		/* setting the tuned gamma setting */
		drm_psb_enable_gamma = 1;
		mdfld_intel_crtc_set_gamma(dev, &gamma);
	}

	pr_debug("[DISPLAY] %s: Enter\n", __func__);
}

static int nt35521_vid_detect(struct mdfld_dsi_config *dsi_config)
{
	int status;
	struct drm_device *dev = dsi_config->dev;
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	u32 dpll_val, device_ready_val;
	int pipe = dsi_config->pipe;

	if (pipe == 0) {
		/*
		 * FIXME: WA to detect the panel connection status, and need to
		 * implement detection feature with get_power_mode DSI command.
		 */
		if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
			OSPM_UHB_FORCE_POWER_ON)) {
			DRM_ERROR("hw begin failed\n");
			return -EAGAIN;
		}

		dpll_val = REG_READ(regs->dpll_reg);
		device_ready_val = REG_READ(regs->device_ready_reg);
		if ((device_ready_val & DSI_DEVICE_READY) &&
		    (dpll_val & DPLL_VCO_ENABLE)) {
			dsi_config->dsi_hw_context.panel_on = true;
			psb_enable_vblank(dev, pipe);
		} else {
			dsi_config->dsi_hw_context.panel_on = false;
			DRM_INFO("%s: panel is not detected!\n", __func__);
		}

		status = MDFLD_DSI_PANEL_CONNECTED;
		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	} else {
		DRM_INFO("%s: do NOT support dual panel\n", __func__);
		status = MDFLD_DSI_PANEL_DISCONNECTED;
	}

	pr_debug("[DISPLAY] %s: Enter\n", __func__);
	return status;
}

static int nt35521_vid_gpio_control(int on){

	int reset_inno;
#ifdef CONFIG_FE170CG
	reset_inno = RESET_INNO;
#elif ASUS_PROJECT_ME372CL_DISPLAY
	reset_inno = RESET_INNO_SR;
#elif ASUS_PROJECT_ME175CG_DISPLAY
	reset_inno = RESET_INNO_SR;
#elif ASUS_PROJECT_ME372CG_DISPLAY
	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);
	if(proj_id == PROJ_ID_ME372CG_CD)
		reset_inno = RESET_INNO_SR;
	else {
		if(board_hw_id == HW_ID_SR1){ //ME371MG HW_ID_SR1 = EVB
			reset_inno = RESET_INNO;
		}else{
			reset_inno = RESET_INNO_SR;
		}
	}
#endif

	if(on){
		intel_scu_ipc_iowrite8(LCD_LOGIC_PWR_EN,0x1);
		usleep_range(1000, 2000);

		if (gpio_direction_output(LCD_ANALONG_PWR_EN, 1))
			gpio_set_value_cansleep(LCD_ANALONG_PWR_EN, 1);

		usleep_range(1000, 2000);

	}else{

		usleep_range(2000, 3000);

		if (gpio_direction_output(reset_inno, 0))
			gpio_set_value_cansleep(reset_inno, 0);

		usleep_range(1000, 2000);

		if (gpio_direction_output(LCD_ANALONG_PWR_EN, 0))
			gpio_set_value_cansleep(LCD_ANALONG_PWR_EN, 0);

		usleep_range(5000, 6000);

		intel_scu_ipc_iowrite8(LCD_LOGIC_PWR_EN,0x0);

		usleep_range(1000, 2000);
	}

	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);

	return 0;

}

static int nt35521_vid_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;

	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);
	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	usleep_range(1000, 1200);

	/* Send TURN_ON packet */
	err = mdfld_dsi_send_dpi_spk_pkg_lp(sender,
					    MDFLD_DSI_DPI_SPK_TURN_ON);
	if (err) {
		DRM_ERROR("Failed to send turn on packet\n");
		return err;
	}

	usleep_range(135000, 136000);  //8 vs
#ifdef CONFIG_FE170CG
	pwm_enable();
#elif ASUS_PROJECT_ME372CL_DISPLAY
	if(board_hw_id == HW_ID_SR1){
		if (gpio_direction_output(LCD_BL_EN_R, 1))
			gpio_set_value_cansleep(LCD_BL_EN_R, 1);
	}
	else
		pwm_enable();
#elif ASUS_PROJECT_ME175CG_DISPLAY
	pwm_enable();
#elif ASUS_PROJECT_ME372CG_DISPLAY
	if(proj_id == PROJ_ID_ME372CG_CD)
		pwm_enable();
	else {
		if(board_hw_id == HW_ID_SR1){ //ME371MG HW_ID_SR1 = EVB
			;
		}else if(board_hw_id == HW_ID_SR2){ //ME371MG HW_ID_SR2 = SR
			;
		}else{
			pwm_enable();
		}
	}
#endif

	/* LCD_BL_EN*/
	intel_scu_ipc_iowrite8(LCD_BL_EN,0x1);

	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);
	return 0;
}

static int nt35521_vid_power_off(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;
	int reset_inno;

	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);
	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}
#ifdef CONFIG_FE170CG
	pwm_disable();
#elif ASUS_PROJECT_ME372CL_DISPLAY
	if(board_hw_id == HW_ID_SR1){
		if (gpio_direction_output(LCD_BL_EN_R, 0))
			gpio_set_value_cansleep(LCD_BL_EN_R, 0);
	}
	else
		pwm_disable();
#elif ASUS_PROJECT_ME175CG_DISPLAY
	pwm_disable();
#elif ASUS_PROJECT_ME372CG_DISPLAY
	if(proj_id == PROJ_ID_ME372CG_CD)
		pwm_disable();
	else {
		if(board_hw_id == HW_ID_SR1){ //ME371MG HW_ID_SR1 = EVB
			;
		}else if(board_hw_id == HW_ID_SR2){ //ME371MG HW_ID_SR2 = SR
			;
		}else{
			pwm_disable();
		}
	}
#endif

	/* LCD_BL_EN*/
	intel_scu_ipc_iowrite8(LCD_BL_EN,0x0);

	//usleep_range(135000, 136000);  //8 vs

	mdfld_dsi_send_mcs_short_lp(sender,0x28,0x00,0,0); //display off
	mdfld_dsi_send_mcs_short_lp(sender,0x10,0x00,0,0); //sleep in

	/* Send SHUT_DOWN packet */
	err = mdfld_dsi_send_dpi_spk_pkg_lp(sender,
					    MDFLD_DSI_DPI_SPK_SHUT_DOWN);
	if (err) {
		DRM_ERROR("Failed to send turn off packet\n");
		return err;
	}

	usleep_range(100000, 101000);  //>100ms

	nt35521_vid_gpio_control(0);

	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);

	return 0;
}

static int nt35521_vid_reset(struct mdfld_dsi_config *dsi_config)
{

	nt35521_vid_gpio_control(1);

	pr_debug("[DISPLAY] %s: Enter\n", __func__);

	return 0;
}

#define PWM0DUTYCYCLE 0x67
#define DUTY_VALUE_MAX	0x63

static int bl_prev_level = 0;
static int nt35521_vid_set_brightness(struct mdfld_dsi_config *dsi_config,
					 int level)
{
	int duty_val = 0;
	int ret = 0;
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}
#ifdef CONFIG_FE170CG
	duty_val = level==1?0:level;
	pwm_configure(duty_val);
#elif ASUS_PROJECT_ME372CL_DISPLAY
	if(board_hw_id == HW_ID_SR1){
		duty_val = ((DUTY_VALUE_MAX + 1) * level) / 255;
		ret = intel_scu_ipc_iowrite8(PWM0DUTYCYCLE, duty_val);
		if (ret)
			DRM_ERROR("write brightness duty value faild\n");
	}
	else{
		duty_val = level==1?0:level;
		pwm_configure(duty_val);
	}
#elif ASUS_PROJECT_ME175CG_DISPLAY
	duty_val = level==1?0:level;
	pwm_configure(duty_val);
#elif ASUS_PROJECT_ME372CG_DISPLAY
	if(proj_id == PROJ_ID_ME372CG_CD){
		duty_val = level==1?0:level;
		pwm_configure(duty_val);
	}
	else{
		if(board_hw_id == HW_ID_SR1){ //ME371MG HW_ID_SR1 = EVB
			duty_val = level;
			mdfld_dsi_send_mcs_short_lp(sender,0x51,duty_val,1,0);
		}else if(board_hw_id == HW_ID_SR2){ //ME371MG HW_ID_SR2 = SR
			duty_val = ((DUTY_VALUE_MAX + 1) * level) / 255;
			ret = intel_scu_ipc_iowrite8(PWM0DUTYCYCLE, duty_val);
			if (ret)
				DRM_ERROR("write brightness duty value faild\n");
		}else{
			duty_val = level==1?0:level;
			pwm_configure(duty_val);
		}
	}
#endif

	//add for debug ++
	if(!!bl_prev_level^!!duty_val)
		DRM_INFO("[DISPLAY] brightness level : %d > %d\n", bl_prev_level, duty_val);

	bl_prev_level = duty_val;
	//add for debug --

	PSB_DEBUG_ENTRY("level = %d , duty_val = %d\n", level, duty_val);

	return 0;
}

struct drm_display_mode *nt35521_vid_get_config_mode(void)
{
	struct drm_display_mode *mode;

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	mode->hdisplay = 800;
	mode->vdisplay = 1280;

#ifdef CONFIG_FE170CG
	mode->hsync_start = 840;
	mode->hsync_end = 844;
	mode->htotal = 884;
	mode->vsync_start = 1300;
	mode->vsync_end = 1304;
	mode->vtotal = 1324;
#elif ASUS_PROJECT_ME175CG_DISPLAY
	mode->hsync_start = 850;
	mode->hsync_end = 854;
	mode->htotal = 904;
	mode->vsync_start = 1305;
	mode->vsync_end = 1309;
	mode->vtotal = 1334;
#else
	mode->hsync_start = 840;
	mode->hsync_end = 844;
	mode->htotal = 884;
	mode->vsync_start = 1300;
	mode->vsync_end = 1304;
	mode->vtotal = 1324;
#endif
	mode->vrefresh = 60;
	mode->clock = mode->vrefresh * mode->vtotal * mode->htotal / 1000;
	mode->type |= DRM_MODE_TYPE_PREFERRED;
	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	pr_debug("[DISPLAY] %s: Enter\n", __func__);
	return mode;
}

static void nt35521_vid_get_panel_info(int pipe, struct panel_info *pi)
{
	pi->width_mm = 94;
	pi->height_mm = 151;
}

static int nt35521_vid_gpio_init(void)
{
#ifdef CONFIG_FE170CG
	gpio_request(PANEL_ID,"PANEL_ID");
	gpio_request(RESET_INNO,"RESET_INNO");
#elif ASUS_PROJECT_ME372CL_DISPLAY
	gpio_request(PANEL_ID_SR,"PANEL_ID_SR");
	gpio_request(RESET_INNO_SR,"RESET_INNO_SR");
	if(board_hw_id == HW_ID_SR1){
		gpio_request(LCD_BL_EN_R,"LCD_BL_EN_R");
	}
#elif ASUS_PROJECT_ME175CG_DISPLAY
	if(board_hw_id == HW_ID_SR1){
		gpio_request(PANEL_ID_SR,"PANEL_ID_SR");
		gpio_request(RESET_INNO_SR,"RESET_INNO_SR");
	}
	else{
		gpio_request(PANEL_ID_SR,"PANEL_ID_ME175CG_SR");
		gpio_request(RESET_INNO_SR,"RESET_INNO_SR");
	}
#elif ASUS_PROJECT_ME372CG_DISPLAY
	//ME372CG
	if(proj_id == PROJ_ID_ME372CG_CD){
		gpio_request(PANEL_ID_SR,"PANEL_ID_SR");
		gpio_request(RESET_INNO_SR,"RESET_INNO_SR");
	}
	else {
		if(board_hw_id == HW_ID_SR1){  //ME371MG HW_ID_SR1 = EVB
			gpio_request(PANEL_ID,"PANEL_ID");
			gpio_request(RESET_INNO,"RESET_INNO");
		}
		else{
			gpio_request(PANEL_ID_SR,"PANEL_ID_SR");
			gpio_request(RESET_INNO_SR,"RESET_INNO_SR");
		}
	}
#endif

	gpio_request(LCD_ANALONG_PWR_EN,"LCD_ANALONG_PWR_EN");

	pr_debug("[DISPLAY] %s: Enter\n", __func__);
	return 0;
}

#define PWM0CLKDIV1 0x61
#define PWM0CLKDIV0 0x62

static int nt35521_vid_brightness_init(void)
{
	int ret = 0;
#ifdef CONFIG_FE170CG
	pwmctrl_mmio = ioremap_nocache(PWMCTRL_REG,PWMCTRL_SIZE);
	lnw_gpio_set_alt(PWM_ENABLE_GPIO, LNW_ALT_2);
#elif ASUS_PROJECT_ME372CL_DISPLAY
	if(board_hw_id == HW_ID_SR1){
		ret = intel_scu_ipc_iowrite8(PWM0CLKDIV1, 0x00);
		if (!ret)
			ret = intel_scu_ipc_iowrite8(PWM0CLKDIV0, 0x25);

		if (ret)
			pr_err("%s: PWM0CLKDIV set failed\n", __func__);
		else
			PSB_DEBUG_ENTRY("PWM0CLKDIV set to 0x%04x\n", 0x25);
	}
	else{
		pwmctrl_mmio = ioremap_nocache(PWMCTRL_REG,PWMCTRL_SIZE);
		lnw_gpio_set_alt(PWM_ENABLE_GPIO, LNW_ALT_2);
	}
#elif ASUS_PROJECT_ME175CG_DISPLAY
	pwmctrl_mmio = ioremap_nocache(PWMCTRL_REG,PWMCTRL_SIZE);
	lnw_gpio_set_alt(PWM_ENABLE_GPIO, LNW_ALT_2);
#elif ASUS_PROJECT_ME372CG_DISPLAY
	if(proj_id == PROJ_ID_ME372CG_CD){
		pwmctrl_mmio = ioremap_nocache(PWMCTRL_REG,PWMCTRL_SIZE);
		lnw_gpio_set_alt(PWM_ENABLE_GPIO, LNW_ALT_2);
	}
	else {
		if(board_hw_id == HW_ID_SR1){ //ME371MG HW_ID_SR1 = EVB
			;
		}else if(board_hw_id == HW_ID_SR2){ //ME371MG HW_ID_SR2 = SR
			ret = intel_scu_ipc_iowrite8(PWM0CLKDIV1, 0x00);
			if (!ret)
				ret = intel_scu_ipc_iowrite8(PWM0CLKDIV0, 0x25);

			if (ret)
				pr_err("%s: PWM0CLKDIV set failed\n", __func__);
			else
				PSB_DEBUG_ENTRY("PWM0CLKDIV set to 0x%04x\n", 0x25);

		}else{
			pwmctrl_mmio = ioremap_nocache(PWMCTRL_REG,PWMCTRL_SIZE);
			lnw_gpio_set_alt(PWM_ENABLE_GPIO, LNW_ALT_2);
		}
	}
#endif

	pr_debug("[DISPLAY] %s: Enter\n", __func__);
	return ret;
}

#ifdef NT35521_DEBUG

static int send_mipi_ret = -1;
static int read_mipi_ret = -1;
static u8 read_mipi_data = 0;

static ssize_t send_mipi_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    int x0=0, x1=0;
    struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(nt35521_dsi_config);

    sscanf(buf, "%x,%x", &x0, &x1);

    send_mipi_ret = mdfld_dsi_send_mcs_short_lp(sender,x0,x1,1,0);

	DRM_INFO("[DISPLAY] send %x,%x : ret = %d\n",x0,x1,send_mipi_ret);

    return count;
}

static ssize_t send_mipi_show(struct device *dev,
	struct device_attribute *attr, const char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n",send_mipi_ret);
}


static ssize_t read_mipi_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    int x0=0;
    struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(nt35521_dsi_config);

    sscanf(buf, "%x", &x0);

    read_mipi_ret = mdfld_dsi_read_mcs_lp(sender,x0,&read_mipi_data,1);
    if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
        read_mipi_ret = -EIO;

	DRM_INFO("[DISPLAY] read 0x%x :ret=%d data=0x%x\n", x0, read_mipi_ret, read_mipi_data);

    return count;
}

static ssize_t read_mipi_show(struct device *dev,
	struct device_attribute *attr, const char *buf)
{
	return snprintf(buf, PAGE_SIZE, "ret=%d data=0x%x\n",read_mipi_ret,read_mipi_data);
}

DEVICE_ATTR(send_mipi_nt35521,S_IRUGO | S_IWUSR, send_mipi_show,send_mipi_store);
DEVICE_ATTR(read_mipi_nt35521,S_IRUGO | S_IWUSR, read_mipi_show,read_mipi_store);


static struct attribute *nt35521_attrs[] = {
        &dev_attr_send_mipi_nt35521.attr,
        &dev_attr_read_mipi_nt35521.attr,
        NULL
};

static struct attribute_group nt35521_attr_group = {
        .attrs = nt35521_attrs,
        .name = "nt35521",
};

#endif

#if 0
static int init_asus_panel_id(){
	int panel_id_value=0;

#if CONFIG_FE170CG
	panel_id_value = gpio_get_value(PANEL_ID)?0x1:0x0;
#elif ASUS_PROJECT_ME372CG_DISPLAY
	if(proj_id == PROJ_ID_ME372CG_CD){
		panel_id_value = gpio_get_value(PANEL_ID_SR)?0x1:0x0;
	}
	else {
		if(board_hw_id == HW_ID_SR1){  //ME371MG HW_ID_SR1 = EVB
			panel_id_value = gpio_get_value(PANEL_ID)?0x1:0x0;
		}
		else{
			panel_id_value = gpio_get_value(PANEL_ID_SR)?0x1:0x0;
		}
	}
#elif ASUS_PROJECT_ME175CG_DISPLAY
	if(board_hw_id == HW_ID_SR1){  //ME175CG HW_ID_SR1 = EVB
		panel_id_value = gpio_get_value(PANEL_ID_SR)?0x1:0x0;
	}
	else{
		panel_id_value = gpio_get_value(PANEL_ID_ME175CG_SR)?0x1:0x0;
	}
#else
	panel_id_value = gpio_get_value(PANEL_ID_SR)?0x1:0x0;
#endif
	asus_panel_id = ME372CG_PANEL | panel_id_value;

	DRM_INFO("[DISPLAY] %s: asus_panel_id = 0x%x\n", __func__, asus_panel_id);

	return 0;
}
#endif

void nt35521_vid_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	int ret = 0;

	p_funcs->get_config_mode = nt35521_vid_get_config_mode;
	p_funcs->get_panel_info = nt35521_vid_get_panel_info;
	p_funcs->dsi_controller_init = nt35521_vid_dsi_controller_init;
	p_funcs->detect = nt35521_vid_detect;
	p_funcs->power_on = nt35521_vid_power_on;
	p_funcs->drv_ic_init = mipi_cmd_setting;
	p_funcs->power_off = nt35521_vid_power_off;
	p_funcs->reset = nt35521_vid_reset;
	p_funcs->set_brightness = nt35521_vid_set_brightness;

	proj_id = Read_PROJ_ID();
	board_hw_id = Read_HW_ID();

	ret = nt35521_vid_gpio_init();
	if (ret)
		DRM_ERROR("Faild to request GPIO for B101UAN01.7 panel\n");

	ret = nt35521_vid_brightness_init();
	if (ret)
		DRM_ERROR("Faild to initilize PWM of MSCI\n");

	//init_asus_panel_id();

#ifdef NT35521_DEBUG

    sysfs_create_group(&dev->dev->kobj, &nt35521_attr_group);

#endif

	DRM_INFO("[DISPLAY] %s: board_hw_id = %d\n", __func__, board_hw_id);

	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);

}

static int nt35521_vid_lcd_probe(struct platform_device *pdev)
{
	int ret = 0;

	DRM_INFO("%s: NT35521 panel detected\n", __func__);
	intel_mid_panel_register(nt35521_vid_init);


	return 0;
}

struct platform_driver nt35521_vid_lcd_driver = {
	.probe	= nt35521_vid_lcd_probe,
	.driver	= {
		.name	= NT35521_PANEL_NAME,
		.owner	= THIS_MODULE,
	},
};

#ifdef ASUS_PROJECT_ME372CL_DISPLAY
struct platform_driver nt35521_vid_lcd_driver_for_cl_evb = {
	.probe	= nt35521_vid_lcd_probe,
	.driver	= {
		.name	= "B101UAN017 VID",
		.owner	= THIS_MODULE,
	},
};
#endif
