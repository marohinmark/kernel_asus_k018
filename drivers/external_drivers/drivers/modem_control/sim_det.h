/*
 * linux/drivers/modem_control/sim_det.h
 *
 */

#include <linux/kernel.h>
#include <linux/switch.h>
#include <linux/notifier.h>

#ifndef _SIM_DET_H
#define _SIM_DET_H

int sim_detect_init(void *drv, void *data);
int sim_detect_exit(void);
#endif /* _SIM_DET_H */