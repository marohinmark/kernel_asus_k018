
#define HIMAX_X_MAX 	2240 //picasso 2624	2240  asus 2240 3008 2240
#define HIMAX_Y_MAX	1408 //picasso 1728	1408	asus 1280 1856 1408
#define HIMAX_X_MAX_370T  2112
#define HIMAX_Y_MAX_370T  1280
#define HIMAX_X_MAX_202T  2944
#define HIMAX_Y_MAX_202T  1856


#ifndef _LINUX_HX8531_H
#define _LINUX_HX8531_H

#define HIMAX_TS_NAME	"hx8528"

struct himax_i2c_platform_data {
	uint16_t version;
	int abs_x_min;
	int abs_x_max;
	int abs_y_min;
	int abs_y_max;
	int intr_gpio;
	int rst_gpio;
	int tp_pwr_gpio;
};

#endif /* _LINUX_HX8531_H */
