/*
 * Copyright 2013 ASUS Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
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
 * Authors:
 * Kunyang Fan <kunyang_fan@asus.com>
 */

#include "displays/truly_vid.h"
#include "mdfld_dsi_dpi.h"
#include "mdfld_dsi_pkg_sender.h"
#include <linux/gpio.h>
#include <linux/sfi.h>
#include <asm/intel_scu_pmic.h>

/*
 * GPIO pin definition
 */
#define PMIC_GPIO_BACKLIGHT_EN	0x7E
#define PMIC_GPIO_PANEL_EN			0x7F
#define LCD_BL_EN 		110       
#define EN_VDD_BL		109
#define DISP_RST_N		57 
#define STB1_EN_EVB		55
#define STB2_EN_EVB 	56
#define STB1_EN_SR1		161
#define STB2_EN_SR1		162
#define DISP_TE 			102

static struct mdfld_dsi_config *b101uan017_dsi_config;
#define PROJECT_ID_EVB	0x00
#define PROJECT_ID_SR1	0x01
struct b101uan017_vid_data{
	u8 project_id;
	unsigned int gpio_lcd_en;
	unsigned int gpio_bl_en;
	unsigned int gpio_stb1_en;
	unsigned int gpio_stb2_en;
	unsigned int gpio_lcd_rst;
};
static struct b101uan017_vid_data gpio_settings_data; 


struct mipi_dsi_short_cmd{
	int delay;
	int len;
	u8 commands[2];
};

struct mipi_dsi_long_cmd{
	int delay;
	int len;
	u8 *commands;
};

long long leve1_cnt = 0;
long long vsync_timeout_cnt = 0;

static struct mipi_dsi_short_cmd initial_commd_set[] = {    
    {  1, 2, {0xFF, 0x24}}, // delay 1 ms
    
    {  0, 2, {0x9D, 0xB0}}, //SGRB/SMX_REG/CTB/CRT
    {  0, 2, {0x72, 0x00}}, // 1920X1080
    {  0, 2, {0x93, 0x04}}, // BP/FP Setting
    {  0, 2, {0x94, 0x04}},
	//Inversion Type          
    {  0, 2, {0x9B, 0x0F}}, // Set Inversion Type
    {  0, 2, {0x8A, 0x33}},
	//Less Transition
    {  0, 2, {0x86, 0x1B}}, // Less Transition
    {  0, 2, {0x87, 0x39}},
    {  0, 2, {0x88, 0x1B}},
    {  0, 2, {0x89, 0x39}},
    {  0, 2, {0x8B, 0xF4}},
    {  0, 2, {0x8C, 0x01}},
	//RTN setting 
    {  0, 2, {0x90, 0x79}}, // RTN Setting
    {  0, 2, {0x91, 0x44}},
    {  0, 2, {0x92, 0x79}},
	//CGOUT Pin Assignment                
    {  0, 2, {0x00, 0x0F}}, // CGOUT Mapping
    {  0, 2, {0x01, 0x00}},
    {  0, 2, {0x02, 0x00}},
    {  0, 2, {0x03, 0x00}},
    {  0, 2, {0x04, 0x0B}},
    {  0, 2, {0x05, 0x0C}},
    {  0, 2, {0x06, 0x00}},
    {  0, 2, {0x07, 0x00}},
    {  0, 2, {0x08, 0x00}},
    {  0, 2, {0x09, 0x00}},
    {  0, 2, {0x0A, 0x03}},
    {  0, 2, {0x0B, 0x04}},
    {  0, 2, {0x0C, 0x01}},
    {  0, 2, {0x0D, 0x13}},
    {  0, 2, {0x0E, 0x15}},
    {  0, 2, {0x0F, 0x17}},
    {  0, 2, {0x10, 0x0F}},
    {  0, 2, {0x11, 0x00}},
    {  0, 2, {0x12, 0x00}},
    {  0, 2, {0x13, 0x00}},
    {  0, 2, {0x14, 0x0B}},
    {  0, 2, {0x15, 0x0C}},
    {  0, 2, {0x16, 0x00}},
    {  0, 2, {0x17, 0x00}},
    {  0, 2, {0x18, 0x00}},
    {  0, 2, {0x19, 0x00}},
    {  0, 2, {0x1A, 0x03}},
    {  0, 2, {0x1B, 0x04}},
    {  0, 2, {0x1C, 0x01}},
    {  0, 2, {0x1D, 0x13}},
    {  0, 2, {0x1E, 0x15}},
    {  0, 2, {0x1F, 0x17}},    
	//GVST Setting   
    {  0, 2, {0x20, 0x09}}, // GVST Setting
    {  0, 2, {0x21, 0x01}},
    {  0, 2, {0x22, 0x00}},
    {  0, 2, {0x23, 0x00}},
    {  0, 2, {0x24, 0x00}},
    {  0, 2, {0x25, 0x6D}},
    {  0, 2, {0x26, 0x00}},
    {  0, 2, {0x27, 0x00}},    
	//GCLK Setting            
    {  0, 2, {0x2F, 0x02}}, // GCLK Setting
    {  0, 2, {0x30, 0x04}},
    {  0, 2, {0x31, 0x49}},
    {  0, 2, {0x32, 0x23}},
    {  0, 2, {0x33, 0x01}},
    {  0, 2, {0x34, 0x00}},
    {  0, 2, {0x35, 0x69}},
    {  0, 2, {0x36, 0x00}},
    {  0, 2, {0x37, 0x2D}},
    {  0, 2, {0x38, 0x08}},
    {  0, 2, {0x39, 0x00}},
    {  0, 2, {0x3A, 0x69}},    
    // U2D/ D2U
    {  0, 2, {0x29, 0x58}},
    {  0, 2, {0x2A, 0x16}},
    
    {  0, 2, {0x5B, 0x00}}, // CTRL, APO
    {  0, 2, {0x5F, 0x75}},
    {  0, 2, {0x63, 0x00}},
    {  0, 2, {0x67, 0x04}},
    //MUX Setting	            
    {  0, 2, {0x7B, 0x80}}, // MUX
    {  0, 2, {0x7C, 0xD8}},
    {  0, 2, {0x7D, 0x60}},
    {  0, 2, {0x7E, 0x10}},
    {  0, 2, {0x7F, 0x19}},
    {  0, 2, {0x80, 0x00}},
    {  0, 2, {0x81, 0x06}},
    {  0, 2, {0x82, 0x03}},
    {  0, 2, {0x83, 0x00}},
    {  0, 2, {0x84, 0x03}},
    {  0, 2, {0x85, 0x07}},
    {  0, 2, {0x74, 0x10}},
    {  0, 2, {0x75, 0x19}},
    {  0, 2, {0x76, 0x06}},
    {  0, 2, {0x77, 0x03}},       
     // Source Control
    {  0, 2, {0x78, 0x00}},
    {  0, 2, {0x79, 0x00}},
    {  0, 2, {0x99, 0x33}},
    {  0, 2, {0x98, 0x00}},
        
	//CGOUT Abnormal Power-OFF Setting                                    
    {  0, 2, {0xB3, 0x28}}, // APO Control
    {  0, 2, {0xB4, 0x05}},
    {  0, 2, {0xB5, 0x10}},
    {  0, 2, {0xFB, 0x01}},  
    {  1, 2, {0xFF, 0x20}}, // Power Set, delay 1 ms
    {  0, 2, {0x00, 0x01}},
    {  0, 2, {0x01, 0x55}},
    {  0, 2, {0x02, 0x45}},
    {  0, 2, {0x03, 0x55}},
    {  0, 2, {0x05, 0x50}},
    {  0, 2, {0x06, 0x9E}},
    {  0, 2, {0x07, 0xA8}},
    {  0, 2, {0x08, 0x0C}},
    
    
    {  0, 2, {0x0B, 0x87}},
    {  0, 2, {0x0C, 0x87}},
    {  0, 2, {0x0E, 0x00}},
    {  0, 2, {0x0F, 0x00}}, 
    {  0, 2, {0x11, 0x27}},
    {  0, 2, {0x12, 0x27}},
    {  0, 2, {0x13, 0x03}},
    {  0, 2, {0x14, 0x0A}},
    {  0, 2, {0x15, 0x99}},
    {  0, 2, {0x16, 0x99}},
    {  0, 2, {0x30, 0x60}},
    {  0, 2, {0x31, 0x54}},
    {  0, 2, {0x32, 0x4B}},
    {  0, 2, {0x6D, 0x44}},
    {  0, 2, {0x58, 0x05}},
    {  0, 2, {0x59, 0x05}},
    {  0, 2, {0x5A, 0x05}},
    {  0, 2, {0x5B, 0x05}},
    {  0, 2, {0x5C, 0x00}},
    {  0, 2, {0x5D, 0x00}},
    {  0, 2, {0x5E, 0x00}},
    {  0, 2, {0x5F, 0x00}},
    
     // PWM Control
    {  0, 2, {0x1B, 0x39}},
    {  0, 2, {0x1C, 0x39}},
    {  0, 2, {0x1D, 0x47}},   
    {  0, 2, {0xFB, 0x01}}, 
    {  0, 2, {0xFF, 0x20}},
    {  0, 2, {0xFB, 0x01}}, // delay 1ms
    //R(+) MCR cmd
    {  0, 2, {0x75, 0x00}}, // Red Gamma +
    {  0, 2, {0x76, 0x00}},
    {  0, 2, {0x77, 0x00}},
    {  0, 2, {0x78, 0x0D}},
    {  0, 2, {0x79, 0x00}},
    {  0, 2, {0x7A, 0x27}},
    {  0, 2, {0x7B, 0x00}},
    {  0, 2, {0x7C, 0x3F}},
    {  0, 2, {0x7D, 0x00}},
    {  0, 2, {0x7E, 0x51}},
    {  0, 2, {0x7F, 0x00}},
    {  0, 2, {0x80, 0x61}},
    {  0, 2, {0x81, 0x00}},
    {  0, 2, {0x82, 0x71}},
    {  0, 2, {0x83, 0x00}},
    {  0, 2, {0x84, 0x7F}},
    {  0, 2, {0x85, 0x00}},
    {  0, 2, {0x86, 0x8B}},
    {  0, 2, {0x87, 0x00}},
    {  0, 2, {0x88, 0xB6}},
    {  0, 2, {0x89, 0x00}},
    {  0, 2, {0x8A, 0xDA}},
    {  0, 2, {0x8B, 0x01}},
    {  0, 2, {0x8C, 0x15}},
    {  0, 2, {0x8D, 0x01}},
    {  0, 2, {0x8E, 0x46}},
    {  0, 2, {0x8F, 0x01}},
    {  0, 2, {0x90, 0x97}},
    {  0, 2, {0x91, 0x01}},
    {  0, 2, {0x92, 0xDD}},
    {  0, 2, {0x93, 0x01}},
    {  0, 2, {0x94, 0xDF}},
    {  0, 2, {0x95, 0x02}},
    {  0, 2, {0x96, 0x20}},
    {  0, 2, {0x97, 0x02}},
    {  0, 2, {0x98, 0x66}},
    {  0, 2, {0x99, 0x02}},
    {  0, 2, {0x9A, 0x91}},
    {  0, 2, {0x9B, 0x02}},
    {  0, 2, {0x9C, 0xC8}},
    {  0, 2, {0x9D, 0x02}},
    {  0, 2, {0x9E, 0xEF}},
    {  0, 2, {0x9F, 0x03}},
    {  0, 2, {0xA0, 0x22}},
    {  0, 2, {0xA2, 0x03}},
    {  0, 2, {0xA3, 0x31}},
    {  0, 2, {0xA4, 0x03}},
    {  0, 2, {0xA5, 0x43}},
    {  0, 2, {0xA6, 0x03}},
    {  0, 2, {0xA7, 0x57}},
    {  0, 2, {0xA9, 0x03}},
    {  0, 2, {0xAA, 0x70}},
    {  0, 2, {0xAB, 0x03}},
    {  0, 2, {0xAC, 0x8F}},
    {  0, 2, {0xAD, 0x03}},
    {  0, 2, {0xAE, 0xB1}},
    {  0, 2, {0xAF, 0x03}},
    {  0, 2, {0xB0, 0xDF}},
    {  0, 2, {0xB1, 0x03}},
    {  0, 2, {0xB2, 0xFF}},
    //R(-) MCR cmd
    {  0, 2, {0xB3, 0x00}}, // Red Gamma -
    {  0, 2, {0xB4, 0x00}},
    {  0, 2, {0xB5, 0x00}},
    {  0, 2, {0xB6, 0x0D}},
    {  0, 2, {0xB7, 0x00}},
    {  0, 2, {0xB8, 0x27}},
    {  0, 2, {0xB9, 0x00}},
    {  0, 2, {0xBA, 0x3F}},
    {  0, 2, {0xBB, 0x00}},
    {  0, 2, {0xBC, 0x51}},
    {  0, 2, {0xBD, 0x00}},
    {  0, 2, {0xBE, 0x61}},
    {  0, 2, {0xBF, 0x00}},
    {  0, 2, {0xC0, 0x71}},
    {  0, 2, {0xC1, 0x00}},
    {  0, 2, {0xC2, 0x7F}},
    {  0, 2, {0xC3, 0x00}},
    {  0, 2, {0xC4, 0x8B}},
    {  0, 2, {0xC5, 0x00}},
    {  0, 2, {0xC6, 0xB6}},
    {  0, 2, {0xC7, 0x00}},
    {  0, 2, {0xC8, 0xDA}},
    {  0, 2, {0xC9, 0x01}},
    {  0, 2, {0xCA, 0x15}},
    {  0, 2, {0xCB, 0x01}},
    {  0, 2, {0xCC, 0x46}},
    {  0, 2, {0xCD, 0x01}},
    {  0, 2, {0xCE, 0x97}},
    {  0, 2, {0xCF, 0x01}},
    {  0, 2, {0xD0, 0xDD}},
    {  0, 2, {0xD1, 0x01}},
    {  0, 2, {0xD2, 0xDF}},
    {  0, 2, {0xD3, 0x02}},
    {  0, 2, {0xD4, 0x20}},
    {  0, 2, {0xD5, 0x02}},
    {  0, 2, {0xD6, 0x66}},
    {  0, 2, {0xD7, 0x02}},
    {  0, 2, {0xD8, 0x91}},
    {  0, 2, {0xD9, 0x02}},
    {  0, 2, {0xDA, 0xC8}},
    {  0, 2, {0xDB, 0x02}},
    {  0, 2, {0xDC, 0xEF}},
    {  0, 2, {0xDD, 0x03}},
    {  0, 2, {0xDE, 0x22}},
    {  0, 2, {0xDF, 0x03}},
    {  0, 2, {0xE0, 0x31}},
    {  0, 2, {0xE1, 0x03}},
    {  0, 2, {0xE2, 0x43}},
    {  0, 2, {0xE3, 0x03}},
    {  0, 2, {0xE4, 0x57}},
    {  0, 2, {0xE5, 0x03}},
    {  0, 2, {0xE6, 0x70}},
    {  0, 2, {0xE7, 0x03}},
    {  0, 2, {0xE8, 0x8F}},
    {  0, 2, {0xE9, 0x03}},
    {  0, 2, {0xEA, 0xB1}},
    {  0, 2, {0xEB, 0x03}},
    {  0, 2, {0xEC, 0xDF}},
    {  0, 2, {0xED, 0x03}},
    {  0, 2, {0xEE, 0xFF}},
    //G(+) MCR cmd
    {  0, 2, {0xEF, 0x00}},
    {  0, 2, {0xF0, 0x00}},
    {  0, 2, {0xF1, 0x00}},
    {  0, 2, {0xF2, 0x0E}},
    {  0, 2, {0xF3, 0x00}},
    {  0, 2, {0xF4, 0x28}},
    {  0, 2, {0xF5, 0x00}},
    {  0, 2, {0xF6, 0x3E}},
    {  0, 2, {0xF7, 0x00}},
    {  0, 2, {0xF8, 0x50}},
    {  0, 2, {0xF9, 0x00}},
    {  0, 2, {0xFA, 0x60}},   
    {  0, 2, {0xFF, 0x21}},
    {  0, 2, {0xFB, 0x01}},
    {  0, 2, {0x00, 0x00}},
    {  0, 2, {0x01, 0x70}},
    {  0, 2, {0x02, 0x00}},
    {  0, 2, {0x03, 0x7E}},
    {  0, 2, {0x04, 0x00}},
    {  0, 2, {0x05, 0x8B}},
    {  0, 2, {0x06, 0x00}},
    {  0, 2, {0x07, 0xB6}},
    {  0, 2, {0x08, 0x00}},
    {  0, 2, {0x09, 0xD9}},
    {  0, 2, {0x0A, 0x01}},
    {  0, 2, {0x0B, 0x14}},
    {  0, 2, {0x0C, 0x01}},
    {  0, 2, {0x0D, 0x45}},
    {  0, 2, {0x0E, 0x01}},
    {  0, 2, {0x0F, 0x96}},
    {  0, 2, {0x10, 0x01}},
    {  0, 2, {0x11, 0xDC}},
    {  0, 2, {0x12, 0x01}},
    {  0, 2, {0x13, 0xDE}},
    {  0, 2, {0x14, 0x02}},
    {  0, 2, {0x15, 0x1F}},
    {  0, 2, {0x16, 0x02}},
    {  0, 2, {0x17, 0x65}},
    {  0, 2, {0x18, 0x02}},
    {  0, 2, {0x19, 0x8F}},
    {  0, 2, {0x1A, 0x02}},
    {  0, 2, {0x1B, 0xC6}},
    {  0, 2, {0x1C, 0x02}},
    {  0, 2, {0x1D, 0xEC}},
    {  0, 2, {0x1E, 0x03}},
    {  0, 2, {0x1F, 0x1E}},
    {  0, 2, {0x20, 0x03}},
    {  0, 2, {0x21, 0x2E}},
    {  0, 2, {0x22, 0x03}},
    {  0, 2, {0x23, 0x3F}},
    {  0, 2, {0x24, 0x03}},
    {  0, 2, {0x25, 0x52}},
    {  0, 2, {0x26, 0x03}},
    {  0, 2, {0x27, 0x6B}},
    {  0, 2, {0x28, 0x03}},
    {  0, 2, {0x29, 0x84}},
    {  0, 2, {0x2A, 0x03}},
    {  0, 2, {0x2B, 0xA8}},
    {  0, 2, {0x2D, 0x03}},
    {  0, 2, {0x2F, 0xCE}},
    {  0, 2, {0x30, 0x03}},
    {  0, 2, {0x31, 0xFF}},
    //G(-) MCR cmd
    {  0, 2, {0x32, 0x00}}, // Green Gamma -
    {  0, 2, {0x33, 0x00}},
    {  0, 2, {0x34, 0x00}},
    {  0, 2, {0x35, 0x0E}},
    {  0, 2, {0x36, 0x00}},
    {  0, 2, {0x37, 0x28}},
    {  0, 2, {0x38, 0x00}},
    {  0, 2, {0x39, 0x3E}},
    {  0, 2, {0x3A, 0x00}},
    {  0, 2, {0x3B, 0x50}},
    {  0, 2, {0x3D, 0x00}},
    {  0, 2, {0x3F, 0x60}},
    {  0, 2, {0x40, 0x00}},
    {  0, 2, {0x41, 0x70}},
    {  0, 2, {0x42, 0x00}},
    {  0, 2, {0x43, 0x7E}},
    {  0, 2, {0x44, 0x00}},
    {  0, 2, {0x45, 0x8B}},
    {  0, 2, {0x46, 0x00}},
    {  0, 2, {0x47, 0xB6}},
    {  0, 2, {0x48, 0x00}},
    {  0, 2, {0x49, 0xD9}},
    {  0, 2, {0x4A, 0x01}},
    {  0, 2, {0x4B, 0x14}},
    {  0, 2, {0x4C, 0x01}},
    {  0, 2, {0x4D, 0x45}},
    {  0, 2, {0x4E, 0x01}},
    {  0, 2, {0x4F, 0x96}},
    {  0, 2, {0x50, 0x01}},
    {  0, 2, {0x51, 0xDC}},
    {  0, 2, {0x52, 0x01}},
    {  0, 2, {0x53, 0xDE}},
    {  0, 2, {0x54, 0x02}},
    {  0, 2, {0x55, 0x1F}},
    {  0, 2, {0x56, 0x02}},
    {  0, 2, {0x58, 0x65}},
    {  0, 2, {0x59, 0x02}},
    {  0, 2, {0x5A, 0x8F}},
    {  0, 2, {0x5B, 0x02}},
    {  0, 2, {0x5C, 0xC6}},
    {  0, 2, {0x5D, 0x02}},
    {  0, 2, {0x5E, 0xEC}},
    {  0, 2, {0x5F, 0x03}},
    {  0, 2, {0x60, 0x1E}},
    {  0, 2, {0x61, 0x03}},
    {  0, 2, {0x62, 0x2E}},
    {  0, 2, {0x63, 0x03}},
    {  0, 2, {0x64, 0x3F}},
    {  0, 2, {0x65, 0x03}},
    {  0, 2, {0x66, 0x52}},
    {  0, 2, {0x67, 0x03}},
    {  0, 2, {0x68, 0x6B}},
    {  0, 2, {0x69, 0x03}},
    {  0, 2, {0x6A, 0x84}},
    {  0, 2, {0x6B, 0x03}},
    {  0, 2, {0x6C, 0xA8}},
    {  0, 2, {0x6D, 0x03}},
    {  0, 2, {0x6E, 0xCE}},
    {  0, 2, {0x6F, 0x03}},
    {  0, 2, {0x70, 0xFF}},
    //B(+) MCR cmd
    {  0, 2, {0x71, 0x00}}, // Blue Gamma +
    {  0, 2, {0x72, 0x00}},
    {  0, 2, {0x73, 0x00}},
    {  0, 2, {0x74, 0x09}},
    {  0, 2, {0x75, 0x00}},
    {  0, 2, {0x76, 0x21}},
    {  0, 2, {0x77, 0x00}},
    {  0, 2, {0x78, 0x36}},
    {  0, 2, {0x79, 0x00}},
    {  0, 2, {0x7A, 0x49}},
    {  0, 2, {0x7B, 0x00}},
    {  0, 2, {0x7C, 0x59}},
    {  0, 2, {0x7D, 0x00}},
    {  0, 2, {0x7E, 0x68}},
    {  0, 2, {0x7F, 0x00}},
    {  0, 2, {0x80, 0x76}},
    {  0, 2, {0x81, 0x00}},
    {  0, 2, {0x82, 0x82}},
    {  0, 2, {0x83, 0x00}},
    {  0, 2, {0x84, 0xAE}},
    {  0, 2, {0x85, 0x00}},
    {  0, 2, {0x86, 0xD1}},
    {  0, 2, {0x87, 0x01}},
    {  0, 2, {0x88, 0x0C}},
    {  0, 2, {0x89, 0x01}},
    {  0, 2, {0x8A, 0x3E}},
    {  0, 2, {0x8B, 0x01}},
    {  0, 2, {0x8C, 0x90}},
    {  0, 2, {0x8D, 0x01}},
    {  0, 2, {0x8E, 0xD7}},
    {  0, 2, {0x8F, 0x01}},
    {  0, 2, {0x90, 0xD9}},
    {  0, 2, {0x91, 0x02}},
    {  0, 2, {0x92, 0x1B}},
    {  0, 2, {0x93, 0x02}},
    {  0, 2, {0x94, 0x62}},
    {  0, 2, {0x95, 0x02}},
    {  0, 2, {0x96, 0x8D}},
    {  0, 2, {0x97, 0x02}},
    {  0, 2, {0x98, 0xC4}},
    {  0, 2, {0x99, 0x02}},
    {  0, 2, {0x9A, 0xEB}},
    {  0, 2, {0x9B, 0x03}},
    {  0, 2, {0x9C, 0x1E}},
    {  0, 2, {0x9D, 0x03}},
    {  0, 2, {0x9E, 0x30}},
    {  0, 2, {0x9F, 0x03}},
    {  0, 2, {0xA0, 0x3F}},
    {  0, 2, {0xA2, 0x03}},
    {  0, 2, {0xA3, 0x54}},
    {  0, 2, {0xA4, 0x03}},
    {  0, 2, {0xA5, 0x73}},
    {  0, 2, {0xA6, 0x03}},
    {  0, 2, {0xA7, 0x9B}},
    {  0, 2, {0xA9, 0x03}},
    {  0, 2, {0xAA, 0xDB}},
    {  0, 2, {0xAB, 0x03}},
    {  0, 2, {0xAC, 0xF1}},
    {  0, 2, {0xAD, 0x03}},
    {  0, 2, {0xAE, 0xFF}},
    //B(-) MCR cmd
    {  0, 2, {0xAF, 0x00}}, // Blue Gamma -
    {  0, 2, {0xB0, 0x00}},
    {  0, 2, {0xB1, 0x00}},
    {  0, 2, {0xB2, 0x09}},
    {  0, 2, {0xB3, 0x00}},
    {  0, 2, {0xB4, 0x21}},
    {  0, 2, {0xB5, 0x00}},
    {  0, 2, {0xB6, 0x36}},
    {  0, 2, {0xB7, 0x00}},
    {  0, 2, {0xB8, 0x49}},
    {  0, 2, {0xB9, 0x00}},
    {  0, 2, {0xBA, 0x59}},
    {  0, 2, {0xBB, 0x00}},
    {  0, 2, {0xBC, 0x68}},
    {  0, 2, {0xBD, 0x00}},
    {  0, 2, {0xBE, 0x76}},
    {  0, 2, {0xBF, 0x00}},
    {  0, 2, {0xC0, 0x82}},
    {  0, 2, {0xC1, 0x00}},
    {  0, 2, {0xC2, 0xAE}},
    {  0, 2, {0xC3, 0x00}},
    {  0, 2, {0xC4, 0xD1}},
    {  0, 2, {0xC5, 0x01}},
    {  0, 2, {0xC6, 0x0C}},
    {  0, 2, {0xC7, 0x01}},
    {  0, 2, {0xC8, 0x3E}},
    {  0, 2, {0xC9, 0x01}},
    {  0, 2, {0xCA, 0x90}},
    {  0, 2, {0xCB, 0x01}},
    {  0, 2, {0xCC, 0xD7}},
    {  0, 2, {0xCD, 0x01}},
    {  0, 2, {0xCE, 0xD9}},
    {  0, 2, {0xCF, 0x02}}, 
    {  0, 2, {0xD0, 0x1B}},
    {  0, 2, {0xD1, 0x02}},
    {  0, 2, {0xD2, 0x62}},
    {  0, 2, {0xD3, 0x02}},
    {  0, 2, {0xD4, 0x8D}},
    {  0, 2, {0xD5, 0x02}},
    {  0, 2, {0xD6, 0xC4}},
    {  0, 2, {0xD7, 0x02}},
    {  0, 2, {0xD8, 0xEB}},
    {  0, 2, {0xD9, 0x03}},
    {  0, 2, {0xDA, 0x1E}},
    {  0, 2, {0xDB, 0x03}},
    {  0, 2, {0xDC, 0x30}},
    {  0, 2, {0xDD, 0x03}},
    {  0, 2, {0xDE, 0x3F}},
    {  0, 2, {0xDF, 0x03}},
    {  0, 2, {0xE0, 0x54}},
    {  0, 2, {0xE1, 0x03}},
    {  0, 2, {0xE2, 0x73}},
    {  0, 2, {0xE3, 0x03}},
    {  0, 2, {0xE4, 0x9B}},
    {  0, 2, {0xE5, 0x03}},
    {  0, 2, {0xE6, 0xDB}},
    {  0, 2, {0xE7, 0x03}},
    {  0, 2, {0xE8, 0xF1}},
    {  0, 2, {0xE9, 0x03}},
    {  0, 2, {0xEA, 0xFF}},
};
static u8 vid_power_on_set_1[] = {0xFF, 0x10};
static u8 vid_mode_setting[] = {0xBB, 0x03};
static u8 vid_porch_setting[] = {0x3B, 0x03, 0x0A, 0x0A, 0x0A, 0x0A};
static u8 vid_reload_setting[] = {0xFB, 0x01};
static u8 vid_abc_setting_1[] = {0x53, 0x24};
static u8 vid_abc_setting_2[] = {0x55, 0x00};
static u8 vid_abc_setting_3[] = {0x5E, 0x00};
static u8 vid_tearing_en[] = {0x35, 0x01};
static u8 vid_display_on[] = {0x29, 0x00};
static u8 vid_sleep_out[] = {0x11, 0x00};
static u8 vid_backlight_setting[] = {0x51, 0xFF};
static u8 vid_scalling[] ={0x58,0x03};
static u8 vid_2A[] = {0x2A, 0x00, 0x00, 0x02, 0xCF};
static u8 vid_2B[] = {0x2B, 0x00, 0x00, 0x04, 0xFF};


static struct mipi_dsi_long_cmd vid_power_on_set[] = {
    {  1, sizeof(vid_power_on_set_1), vid_power_on_set_1},
    {  0, sizeof(vid_mode_setting), vid_mode_setting}, // MIPI Video Mode Bypass RAM
    {  0, sizeof(vid_porch_setting), vid_porch_setting}, // Video mode Porch Setting (VBP, VFP, HBP, HFP)
    {  120, sizeof(vid_sleep_out), vid_sleep_out}, // Sleep Out sleep 120 ms
   // {  0, sizeof(vid_2A),vid_2A},
    //{  0, sizeof(vid_2B),vid_2B},
    //{  0, sizeof(vid_scalling),vid_scalling},
    	//{1, sizeof(vid_tearing_en), vid_tearing_en},
    {  0, sizeof(vid_display_on), vid_display_on},    // Display on
};


static struct mipi_dsi_short_cmd power_off_set[] = {
    {  1, 2, {0xFF, 0x10}}, // delay 1 ms
    {  0, 2, {0x51, 0x00}}, // Write backlight Brighness
    { 20, 2, {0x28, 0x00}}, // Display off & sleep 20ms
    { 100, 2, {0x10, 0x00}}, // sleep in & sleep 100 ms  
};


static int mipi_dsi_send_short_cmd_hs(struct mdfld_dsi_pkg_sender *sender, struct mipi_dsi_short_cmd *cmd){
    int err = 0;
    //printk("DCS Send command 0x%x, 0x%x\n", cmd->commands[0], (cmd->len > 1 ? cmd->commands[0] : 0));
	
    if(cmd->len == 1)
        err = mdfld_dsi_send_mcs_short_hs(sender, cmd->commands[0], 0, 1,
				    MDFLD_DSI_SEND_PACKAGE);
    else
	 err = mdfld_dsi_send_mcs_short_hs(sender, cmd->commands[0], cmd->commands[1], 1,
				    MDFLD_DSI_SEND_PACKAGE);	
   
    if (err != 0 || sender->status) {
        DRM_ERROR("DCS %s sent failed with status=%d\n", __func__, sender->status);
        return err;
    }
    if(cmd->delay) 
        udelay(1000 * cmd->delay);		
	
    return 0;
}

static int mipi_dsi_send_short_cmd(struct mdfld_dsi_pkg_sender *sender, struct mipi_dsi_short_cmd *cmd){
    int err = 0;
    //printk("DCS Send command 0x%x, 0x%x\n", cmd->commands[0], (cmd->len > 1 ? cmd->commands[0] : 0));
	
    if(cmd->len == 1)
        err = mdfld_dsi_send_mcs_short_lp(sender, cmd->commands[0], 0, 1,
				    MDFLD_DSI_SEND_PACKAGE);
    else
	 err = mdfld_dsi_send_mcs_short_lp(sender, cmd->commands[0], cmd->commands[1], 1,
				    MDFLD_DSI_SEND_PACKAGE);	
   
    if (err != 0 || sender->status) {
        DRM_ERROR("DCS %s sent failed with status=%d\n", __func__, sender->status);
        return err;
    }
    if(cmd->delay) 
        udelay(1000 * cmd->delay);		
	
    return 0;
}

static int mipi_dsi_send_long_cmd(struct mdfld_dsi_pkg_sender *sender, struct mipi_dsi_long_cmd *cmd){
    int err = 0, i;
    u8 data[8] = {0};

    switch(cmd->len){
    case 1:
	  err = mdfld_dsi_send_mcs_short_lp(sender, cmd->commands[0], 0, 1,
				    MDFLD_DSI_SEND_PACKAGE);
	  break;
    case 2:
        err = mdfld_dsi_send_mcs_short_lp(sender, cmd->commands[0], cmd->commands[1], 1,
				    MDFLD_DSI_SEND_PACKAGE);
	  break;
    default:
        err = mdfld_dsi_send_mcs_long_lp(sender,
				   cmd->commands,
				   cmd->len,
				   MDFLD_DSI_SEND_PACKAGE);
        if(err == 0 && cmd->len - 1 < 8){
            for(i = 0; i < 4; i++){
                mdfld_dsi_read_mcs_lp(sender, cmd->commands[0], data, cmd->len - 1);
                if(memcmp(cmd->commands + 1, data, cmd->len - 1) == 0)
                    break;
                if(i<3)
                    err = mdfld_dsi_send_mcs_long_lp(sender,cmd->commands,cmd->len,
                              MDFLD_DSI_SEND_PACKAGE);
		    }

            if(i == 4){
                DRM_ERROR("Error to send long command:");
                for(i = 0; i < cmd->len - 1; i++)
                    DRM_ERROR(" 0x%02X", data[i]);
                DRM_ERROR("\n");
            }
        }
    }
    
	
    if (err != 0 || sender->status) {
        DRM_ERROR("DCS %s sent failed with status=%d\n", __func__, sender->status);
        return err;
    }
    if(cmd->delay) 
        udelay(1000 * cmd->delay);		
	
    return 0;
}


/*Used for truly panel module*/
static int truly_drv_ic_init(struct mdfld_dsi_config *dsi_config)
{
	int r = 0, i;
	u8 data[16] = {0};

	PSB_DEBUG_ENTRY("\n");
    DRM_INFO("[DISPLAY] %s: Enter\n", __func__);
	memset(data, 0, sizeof(data));
	struct mdfld_dsi_pkg_sender *sender
				= mdfld_dsi_get_pkg_sender(dsi_config);
	sender->status = MDFLD_DSI_PKG_SENDER_FREE;
      if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		printk("[Leon] me506cg_macp MDFLD_DSI_CONTROL_ABNORMAL !!\n");

   
      for(i = 0; i < ARRAY_SIZE(initial_commd_set); i++)
          mipi_dsi_send_short_cmd(sender, &initial_commd_set[i]);

        r = mdfld_dsi_read_mcs_lp(sender, 0xDA , data, 1);
      printk("[Display] r=%d 0xDA=0x%X\n", r, data[0]);
      r = mdfld_dsi_read_mcs_lp(sender, 0xDB, data + 1, 1);
      printk("[Display] r=%d 0xDB=0x%X\n", r, data[1]);
      mdfld_dsi_read_mcs_lp(sender, 0xDC, data + 2, 1);
      printk("[Display] r=%d 0xDC=0x%X\n", r, data[2]);

	DRM_INFO("[DISPLAY] %s: Exit\n", __func__);
	return 0;
}

static void truly_vid_dsi_controller_init(struct mdfld_dsi_config *dsi_config)
{
	struct drm_device *dev = dsi_config->dev;
	struct mdfld_dsi_hw_context *hw_ctx = &dsi_config->dsi_hw_context;
	DRM_INFO("[DISPLAY]3 %s: Enter\n", __func__);

	/* Reconfig lane configuration */
	dsi_config->lane_count = 4;
	dsi_config->lane_config = MDFLD_DSI_DATA_LANE_4_0;
	dsi_config->enable_gamma_csc = ENABLE_GAMMA;
	
	/*
	 * Data Rate = 1108 * 1942 * 60 * 24 / 4 = 774.624Mbps,
	 * DSI Clock Frequency = 774.624 / 2 = 387.312MHz,
	 * UI = 1 / 387.312 / 2 = 1.2480ns ~= 1.29 ns
	 */
	hw_ctx->cck_div = 0; /*  Set it to 0 for 800mhz */
	hw_ctx->pll_bypass_mode = 0; // > if > 800 0 
	hw_ctx->mipi_control = 0x18;
	hw_ctx->intr_en = 0xffffffff;
	hw_ctx->hs_tx_timeout = 0xffffff;
	hw_ctx->lp_rx_timeout = 0xffffff;
	hw_ctx->turn_around_timeout = 0x1f; 
	hw_ctx->device_reset_timer = 0xffff; 
	hw_ctx->high_low_switch_count = 0x25; 
	hw_ctx->init_count = 0xf0;
	hw_ctx->eot_disable = 0x03;
	hw_ctx->lp_byteclk = 0x0A;
      hw_ctx->clk_lane_switch_time_cnt = 0x250012;
	hw_ctx->dphy_param =  0x28186411;

	/* Setup video mode format */
	hw_ctx->video_mode_format = 0xf;

	/* Set up func_prg, RGB888(0x200) */
	hw_ctx->dsi_func_prg = (0x200 | dsi_config->lane_count);

	/* Setup mipi port configuration */
	hw_ctx->mipi = PASS_FROM_SPHY_TO_AFE | dsi_config->lane_config;

	b101uan017_dsi_config = dsi_config;

	if (dsi_config->enable_gamma_csc & ENABLE_GAMMA) {
		/* setting the tuned gamma setting */
		drm_psb_enable_gamma = 1;
#if GAMMA_USER_SETTING
		mdfld_intel_crtc_set_gamma(dev, &gamma_settings);
#endif
	}

	DRM_INFO("[DISPLAY]3 %s: End\n", __func__);
}


static
int mdfld_dsi_truly_detect(struct mdfld_dsi_config *dsi_config)
{
	int status;
	struct drm_device *dev = dsi_config->dev;
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	u32 dpll_val, device_ready_val;
	int pipe = dsi_config->pipe;

	PSB_DEBUG_ENTRY("\n");

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

	return status;
}


static int truly_vid_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err, i;
	struct b101uan017_vid_data *pdata = &gpio_settings_data;

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}
	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);

	for(i = 0; i < ARRAY_SIZE(vid_power_on_set); i++)
	    mipi_dsi_send_long_cmd(sender, &vid_power_on_set[i]);

      /*Add or not*/
      usleep_range(10000, 15000);
	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,
					    MDFLD_DSI_DPI_SPK_TURN_ON);
	if (err) {
		DRM_ERROR("Failed to send turn on packet\n");
		return err;
	}
	//msleep(150);


      //gpio_direction_output(pdata->gpio_lcd_en, 1);
      //intel_scu_ipc_iowrite8(PMIC_GPIO_PANEL_EN, 0x01);
      gpio_direction_output(pdata->gpio_bl_en, 1);
      intel_scu_ipc_iowrite8(PMIC_GPIO_BACKLIGHT_EN, 0x01);

	return 0;
}

static int truly_vid_power_off(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err, i;
	struct b101uan017_vid_data *pdata = &gpio_settings_data;

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}
	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);

      /* Turn off the backlight*/
      gpio_direction_output(pdata->gpio_bl_en, 0);
      intel_scu_ipc_iowrite8(PMIC_GPIO_BACKLIGHT_EN, 0);
      usleep_range(1000, 1500);
	  
	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,
					    MDFLD_DSI_DPI_SPK_SHUT_DOWN);
	if (err) {
		DRM_ERROR("Failed to send turn off packet\n");
		return err;
	}
	usleep_range(10000, 15000);

    /* Send power off command*/
      for(i = 0; i < ARRAY_SIZE(power_off_set); i++)
           mipi_dsi_send_short_cmd_hs(sender, &power_off_set[i]);  

    /* Driver IC power off sequence*/
    usleep_range(10000, 15000);
    gpio_direction_output(pdata->gpio_stb1_en, 0);
    usleep_range(5000, 5500);
    gpio_direction_output(pdata->gpio_stb2_en, 0);
    usleep_range(62000, 62500);
    gpio_direction_output(pdata->gpio_lcd_rst, 0);
    usleep_range(5000, 5500);
    gpio_direction_output(pdata->gpio_lcd_en, 0);
    intel_scu_ipc_iowrite8(PMIC_GPIO_PANEL_EN, 0);
    usleep_range(1000, 1500);
    
    return 0;
}

#define PWM0CLKDIV1 0x61
#define PWM0CLKDIV0 0x62

#define PWMCTRL_REG 0xffae9000
#define PWMCTRL_SIZE 0x80

#define PWM0DUTYCYCLE 0x67
#define DUTY_VALUE_MAX 0x63
#define BRIGHTNESS_LEVEL_MAX 100
#define BACKLIGHT_DUTY_FACTOR	0xff

static int mdfld_dsi_truly_set_brightness(struct mdfld_dsi_config *dsi_config,
					 int level)
{
	int duty_val = 0;
	int ret = 0;

      printk("[DISPLAY] brightness level = %d\n", level);
      duty_val = ((DUTY_VALUE_MAX + 1) * level) / BRIGHTNESS_LEVEL_MAX;

	ret = intel_scu_ipc_iowrite8(PWM0DUTYCYCLE, (duty_val > DUTY_VALUE_MAX ? DUTY_VALUE_MAX : duty_val));
	if (ret)
		DRM_ERROR("write brightness duty value faild\n");

	return ret;
}

#ifdef DEBUG
static
int mdfld_dsi_h8c7_panel_reset(struct mdfld_dsi_config *dsi_config)
{
	static int mipi_reset_gpio;
	int ret = 0;

	PSB_DEBUG_ENTRY("\n");

	if (mipi_reset_gpio == 0) {
		ret = get_gpio_by_name("mipi-reset");
		if (ret < 0) {
			DRM_ERROR("Faild to get panel reset gpio, " \
				  "use default reset pin\n");
			ret = 128;
		}

		mipi_reset_gpio = ret;

		ret = gpio_request(mipi_reset_gpio, "mipi_display");
		if (ret) {
			DRM_ERROR("Faild to request panel reset gpio\n");
			return -EINVAL;
		}

		gpio_direction_output(mipi_reset_gpio, 0);
	}

	gpio_set_value_cansleep(mipi_reset_gpio, 0);
	/* HW reset need minmum 3ms */
	mdelay(11);

	gpio_set_value_cansleep(mipi_reset_gpio, 1);
	mdelay(5);

	return 0;
}
#endif

static int truly_vid_reset(struct mdfld_dsi_config *dsi_config)
{
      struct b101uan017_vid_data *pdata = &gpio_settings_data;

	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);

      // start the initial state
      gpio_direction_output(pdata->gpio_lcd_en, 0);
      intel_scu_ipc_iowrite8(PMIC_GPIO_PANEL_EN, 0x00);
	usleep_range(1000, 1500);
      gpio_direction_output(pdata->gpio_bl_en, 0);
      intel_scu_ipc_iowrite8(PMIC_GPIO_BACKLIGHT_EN, 0x00);
      usleep_range(1000, 1500);
      gpio_direction_output(pdata->gpio_lcd_rst, 0);
      usleep_range(1000, 1500);
      gpio_direction_output(pdata->gpio_stb1_en, 0);
      usleep_range(1000, 1500);
      gpio_direction_output(pdata->gpio_stb2_en, 0);
	usleep_range(10000, 15000);

	// start power on sequence
	gpio_direction_output(pdata->gpio_lcd_en, 1);
	intel_scu_ipc_iowrite8(PMIC_GPIO_PANEL_EN, 0x01);
	usleep_range(5000, 5500);
	gpio_direction_output(pdata->gpio_stb2_en, 1);
	usleep_range(5000, 5500);
	gpio_direction_output(pdata->gpio_stb1_en, 1);
	usleep_range(10000, 15000);
	gpio_direction_output(pdata->gpio_lcd_rst, 1);  
	usleep_range(5000, 5500);
	gpio_direction_output(pdata->gpio_lcd_rst, 0);  
	usleep_range(10000, 11000);
	gpio_direction_output(pdata->gpio_lcd_rst, 1);
	usleep_range(10000, 11000);

	return 0;
}

static struct drm_display_mode *truly_vid_get_config_mode(void)
{
	struct drm_display_mode *mode;

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;
	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);

	mode->hdisplay = 1080;
	mode->vdisplay = 1920;
	mode->hsync_start = mode->hdisplay + 10;		/* HFP */
	mode->hsync_end = mode->hsync_start + 8;		/* HSW */
	mode->htotal = mode->hsync_end + 10;			/* HBP */
	mode->vsync_start = mode->vdisplay + 10;		/* VFP */
	mode->vsync_end = mode->vsync_start + 2;		/* VSW */
	mode->vtotal = mode->vsync_end + 10;			/* VBP */
	mode->vrefresh = 60;
	mode->clock = mode->vrefresh * mode->vtotal * mode->htotal / 1000;
	mode->type |= DRM_MODE_TYPE_PREFERRED;
	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	return mode;
}

static void truly_vid_get_panel_info(int pipe, struct panel_info *pi)
{
	pi->width_mm = 75;
	pi->height_mm = 133;
}

static int truly_vid_gpio_init(void)
{
      int ret;
      struct b101uan017_vid_data *pdata = &gpio_settings_data;
      pdata->project_id = 0;
	DRM_INFO("[DISPLAY] %s: Enter project ID:0x%02X\n", __func__, pdata->project_id);
	pdata->gpio_lcd_en = get_gpio_by_name("LCD_VDD_EN");
	pdata->gpio_bl_en = get_gpio_by_name("LCD_BL_EN");
	pdata->gpio_lcd_rst = get_gpio_by_name("DISP_RST_N");
	pdata->gpio_stb1_en = get_gpio_by_name("STB1_EN");
	pdata->gpio_stb2_en = get_gpio_by_name("STB2_EN");
	
	ret = gpio_request(pdata->gpio_stb1_en, "STB1_EN");
	if (ret < 0)
		DRM_ERROR("Faild to get panel GPIO STB1_EN:%d\n", pdata->gpio_stb1_en);
	gpio_direction_output(pdata->gpio_stb1_en, 1);

	ret = gpio_request(pdata->gpio_stb2_en, "STB2_EN");
	if (ret < 0)
		DRM_ERROR("Faild to get panel GPIO STB2_EN:%d\n", pdata->gpio_stb2_en);
	gpio_direction_output(pdata->gpio_stb2_en, 1);

	ret = gpio_request(pdata->gpio_lcd_rst, "DISP_RST_N");
	if (ret < 0)
		DRM_ERROR("Faild to get panel GPIO DISP_RST_N:%d\n", pdata->gpio_lcd_rst);
	gpio_direction_output(pdata->gpio_lcd_rst, 1);

	ret = gpio_request(pdata->gpio_lcd_en, "LCD_VDD_EN");
	if (ret < 0)
		DRM_ERROR("Faild to get panel GPIO LCD_VDD_EN:%d\n", pdata->gpio_lcd_en);
	gpio_direction_output(pdata->gpio_lcd_en, 1);

	ret = gpio_request(pdata->gpio_bl_en, "LCD_BL_EN");
	if (ret < 0)
		DRM_ERROR("Faild to get panel GPIO LCD_BL_EN:%d\n", pdata->gpio_bl_en);
	gpio_direction_output(pdata->gpio_bl_en, 1);

      gpio_request(DISP_TE, "DISP_TE");
      gpio_direction_output(DISP_TE, 0);

      intel_scu_ipc_iowrite8(PMIC_GPIO_PANEL_EN, 0x01);
      intel_scu_ipc_iowrite8(PMIC_GPIO_BACKLIGHT_EN, 0x01);

      return 0;
}



static int truly_vid_brightness_init(void)
{
	int ret = 0;
	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);
	ret = intel_scu_ipc_iowrite8(PWM0CLKDIV1, 0x00);
	if (!ret)
		ret = intel_scu_ipc_iowrite8(PWM0CLKDIV0, 0x25);

	if (ret)
		pr_err("%s: PWM0CLKDIV set failed\n", __func__);
	else
		PSB_DEBUG_ENTRY("PWM0CLKDIV set to 0x%04x\n", 0x25);

	return ret;
}

void truly_vid_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	int ret;
	PSB_DEBUG_ENTRY("\n");

	p_funcs->get_config_mode =  truly_vid_get_config_mode;
	p_funcs->get_panel_info = truly_vid_get_panel_info;
	p_funcs->reset = truly_vid_reset;
	p_funcs->drv_ic_init = truly_drv_ic_init;
	p_funcs->dsi_controller_init = truly_vid_dsi_controller_init;
	p_funcs->detect = mdfld_dsi_truly_detect;
	p_funcs->power_on = truly_vid_power_on;
	p_funcs->power_off = truly_vid_power_off;
	p_funcs->set_brightness = mdfld_dsi_truly_set_brightness;
	ret = truly_vid_gpio_init();
	if (ret)
		DRM_ERROR("Faild to request GPIO for B101UAN01.7 panel\n");

	ret = truly_vid_brightness_init();
	if (ret)
		DRM_ERROR("Faild to initilize PWM of MSCI\n");
}

static int truly_lcd_vid_probe(struct platform_device *pdev)
{
	DRM_INFO("%s: Truly panel detected\n", __func__);
	intel_mid_panel_register(truly_vid_init);

	return 0;
}

struct platform_driver truly_lcd_driver = {
	.probe	= truly_lcd_vid_probe,
	.driver	= {
		.name	= "B101UAN017 VID",
		.owner	= THIS_MODULE,
	},
};


