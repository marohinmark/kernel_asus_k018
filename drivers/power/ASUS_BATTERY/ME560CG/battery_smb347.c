/*
 * Copyright (c) 2012, ASUSTek, Inc. All Rights Reserved.
 */
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/power/smb347-me560cg-charger.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/usb/otg.h>
#include <linux/kernel.h>	/* Needed for KERN_DEBUG */
#include <linux/wakelock.h>
#include <linux/usb/penwell_otg.h>

#include "../smb347_external_include.h"
#include "asus_battery.h"

/*
 *  config function
 */
#define CANCEL_SOFT_HOT_TEMP_LIMIT   0
#define SMB347_INTERRUPT             0
#define SMB347_IRQ_INIT              0
/*
 * Configuration registers. These are mirrored to volatile RAM and can be
 * written once %CMD_A_ALLOW_WRITE is set in %CMD_A register. They will be
 * reloaded from non-volatile registers after POR.
 */
#define CFG_CHARGE_CURRENT			0x00
#define CFG_CHARGE_CURRENT_FCC_MASK		0xe0
#define CFG_CHARGE_CURRENT_FCC_SHIFT		5
#define CFG_CHARGE_CURRENT_PCC_MASK		0x18
#define CFG_CHARGE_CURRENT_PCC_SHIFT		3
#define CFG_CHARGE_CURRENT_TC_MASK		0x07
#define CFG_CURRENT_LIMIT			0x01
#define CFG_CURRENT_LIMIT_DC_MASK		0xf0
#define CFG_CURRENT_LIMIT_DC_SHIFT		4
#define CFG_CURRENT_LIMIT_USB_MASK		0x0f
#define CFG_VARIOUS_FUNCS			0x02
#define CFG_VARIOUS_FUNCS_PRIORITY_USB		BIT(2)
#define CFG_FLOAT_VOLTAGE			0x03
#define CFG_FLOAT_VOLTAGE_THRESHOLD_MASK	0xc0
#define CFG_FLOAT_VOLTAGE_THRESHOLD_SHIFT	6
#define CFG_STAT				0x05
#define CFG_STAT_DISABLED			BIT(5)
#define CFG_STAT_ACTIVE_HIGH			BIT(7)
#define CFG_PIN					0x06
#define CFG_PIN_EN_CTRL_MASK			0x60
#define CFG_PIN_EN_CTRL_ACTIVE_HIGH		0x40
#define CFG_PIN_EN_CTRL_ACTIVE_LOW		0x60
#define CFG_PIN_EN_APSD_IRQ			BIT(1)
#define CFG_PIN_EN_CHARGER_ERROR		BIT(2)
#define CFG_THERM				0x07
#define CFG_THERM_SOFT_HOT_COMPENSATION_MASK	0x03
#define CFG_THERM_SOFT_HOT_COMPENSATION_SHIFT	0
#define CFG_THERM_SOFT_COLD_COMPENSATION_MASK	0x0c
#define CFG_THERM_SOFT_COLD_COMPENSATION_SHIFT	2
#define CFG_THERM_MONITOR_DISABLED		BIT(4)
#define CFG_SYSOK				0x08
#define CFG_SYSOK_SUSPEND_HARD_LIMIT_DISABLED	BIT(2)
#define CFG_OTHER				0x09
#define CFG_OTHER_RID_MASK			0xc0
#define CFG_OTHER_RID_DISABLED_OTG_PIN		0x40
#define CFG_OTHER_RID_ENABLED_OTG_I2C		0x80
#define CFG_OTHER_RID_ENABLED_AUTO_OTG		0xc0
#define CFG_OTHER_OTG_PIN_ACTIVE_LOW		BIT(5)
#define CFG_OTG					0x0a
#define CFG_OTG_TEMP_THRESHOLD_MASK		0x30
#define CFG_OTG_TEMP_THRESHOLD_SHIFT		4
#define CFG_OTG_CC_COMPENSATION_MASK		0xc0
#define CFG_OTG_CC_COMPENSATION_SHIFT		6
#define CFG_OTG_BATTERY_UVLO_THRESHOLD_MASK	0x03
#define CFG_TEMP_LIMIT				0x0b
#define CFG_TEMP_LIMIT_SOFT_HOT_MASK		0x03
#define CFG_TEMP_LIMIT_SOFT_HOT_SHIFT		0
#define CFG_TEMP_LIMIT_SOFT_COLD_MASK		0x0c
#define CFG_TEMP_LIMIT_SOFT_COLD_SHIFT		2
#define CFG_TEMP_LIMIT_HARD_HOT_MASK		0x30
#define CFG_TEMP_LIMIT_HARD_HOT_SHIFT		4
#define CFG_TEMP_LIMIT_HARD_COLD_MASK		0xc0
#define CFG_TEMP_LIMIT_HARD_COLD_SHIFT		6
#define CFG_FAULT_IRQ				0x0c
#define CFG_FAULT_IRQ_DCIN_UV			BIT(2)
#define CFG_FAULT_IRQ_OTG_UV			BIT(5)
#define CFG_STATUS_IRQ				0x0d
#define CFG_STATUS_IRQ_CHARGE_TIMEOUT		BIT(7)
#define CFG_STATUS_IRQ_TERMINATION_OR_TAPER	BIT(4)
#define CFG_ADDRESS				0x0e

/* Command registers */
#define CMD_A					0x30
#define CMD_A_CHG_ENABLED			BIT(1)
#define CMD_A_SUSPEND_ENABLED			BIT(2)
#define CMD_A_OTG_ENABLED			BIT(4)
#define CMD_A_ALLOW_WRITE			BIT(7)
#define CMD_B					0x31
#define CMD_C					0x33

/* Interrupt Status registers */
#define IRQSTAT_A				0x35
#define IRQSTAT_C				0x37
#define IRQSTAT_C_TERMINATION_STAT		BIT(0)
#define IRQSTAT_C_TERMINATION_IRQ		BIT(1)
#define IRQSTAT_C_TAPER_IRQ			BIT(3)
#define IRQSTAT_D				0x38
#define IRQSTAT_D_CHARGE_TIMEOUT_STAT		BIT(2)
#define IRQSTAT_D_CHARGE_TIMEOUT_IRQ		BIT(3)
#define IRQSTAT_E				0x39
#define IRQSTAT_E_USBIN_UV_STAT			BIT(0)
#define IRQSTAT_E_USBIN_UV_IRQ			BIT(1)
#define IRQSTAT_E_DCIN_UV_STAT			BIT(4)
#define IRQSTAT_E_DCIN_UV_IRQ			BIT(5)
#define IRQSTAT_F				0x3a
#define IRQSTAT_F_OTG_UV_IRQ			BIT(5)
#define IRQSTAT_F_OTG_UV_STAT			BIT(4)

/* Status registers */
#define STAT_A					0x3b
#define STAT_A_FLOAT_VOLTAGE_MASK		0x3f
#define STAT_B					0x3c
#define STAT_C					0x3d
#define STAT_C_CHG_ENABLED			BIT(0)
#define STAT_C_HOLDOFF_STAT			BIT(3)
#define STAT_C_CHG_MASK				0x06
#define STAT_C_CHG_SHIFT			1
#define STAT_C_CHG_TERM				BIT(5)
#define STAT_C_CHARGER_ERROR			BIT(6)
#define STAT_E					0x3f

#define STATUS_UPDATE_INTERVAL			(HZ * 60)	/*60 sec */

struct smb347_otg_event {
	struct list_head	node;
	bool			param;
};

/**
 * struct smb347_charger - smb347 charger instance
 * @lock: protects concurrent access to online variables
 * @client: pointer to i2c client
 * @mains: power_supply instance for AC/DC power
 * @usb: power_supply instance for USB power
 * @battery: power_supply instance for battery
 * @mains_online: is AC/DC input connected
 * @usb_online: is USB input connected
 * @charging_enabled: is charging enabled
 * @running: the driver is up and running
 * @dentry: for debugfs
 * @otg: pointer to OTG transceiver if any
 * @otg_nb: notifier for OTG notifications
 * @otg_work: work struct for OTG notifications
 * @otg_queue: queue holding received OTG notifications
 * @otg_queue_lock: protects concurrent access to @otg_queue
 * @otg_last_param: last parameter value from OTG notification
 * @otg_enabled: OTG VBUS is enabled
 * @otg_battery_uv: OTG battery undervoltage condition is on
 * @pdata: pointer to platform data
 */
struct smb347_charger {
	struct mutex		lock;
	struct i2c_client	*client;
	struct power_supply	mains;
	struct power_supply	usb;
	struct power_supply	battery;
	bool			mains_online;
	bool			usb_online;
	bool			charging_enabled;
	bool			running;
	struct dentry		*dentry;
	struct otg_transceiver	*otg;
	struct notifier_block	otg_nb;
	struct work_struct	otg_work;
	struct list_head	otg_queue;
	spinlock_t		otg_queue_lock;
	bool			otg_enabled;
	bool			otg_battery_uv;
	const struct smb347_charger_platform_data	*pdata;
#if 0
	struct delayed_work	smb347_statmon_worker;
#endif
	/* wake lock to prevent S3 during charging */
	struct wake_lock wakelock;
};

static struct smb347_charger *smb347_dev;
static bool ischargerSuspend = false;
static bool isUSBSuspendNotify = false;
#if 0
static char *smb347_power_supplied_to[] = {
			"max170xx_battery",
			"max17042_battery",
			"max17047_battery",
			"max17050_battery",
};

/* Fast charge current in uA */
static const unsigned int fcc_tbl[] = {
	700000,
	900000,
	1200000,
	1500000,
	1800000,
	2000000,
	2200000,
	2500000,
};

/* Pre-charge current in uA */
static const unsigned int pcc_tbl[] = {
	100000,
	150000,
	200000,
	250000,
};

/* Termination current in uA */
static const unsigned int tc_tbl[] = {
	37500,
	50000,
	100000,
	150000,
	200000,
	250000,
	500000,
	600000,
};

/* Input current limit in uA */
static const unsigned int icl_tbl[] = {
	300000,
	500000,
	700000,
	900000,
	1200000,
	1500000,
	1800000,
	2000000,
	2200000,
	2500000,
};

/* Charge current compensation in uA */
static const unsigned int ccc_tbl[] = {
	250000,
	700000,
	900000,
	1200000,
};
#endif

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define EXPORT_CHARGER_OTG

#define DEBUG 1
#define DRIVER_VERSION			"1.1.0"

#define SMB347_MASK(BITS, POS)  ((unsigned char)(((1 << BITS) - 1) << POS))

/* Register definitions */
#define CHG_CURRENT_REG			0x00
#define INPUT_CURRENT_LIMIT_REG	0x01
#define VAR_FUNC_REG			0x02
#define FLOAT_VOLTAGE_REG		0x03
#define CHG_CTRL_REG			0x04
#define STAT_TIMER_REG			0x05
#define PIN_ENABLE_CTRL_REG		0x06
#define THERM_CTRL_A_REG		0x07
#define SYSOK_USB3_SELECT_REG	0x08
#define OTHER_CTRL_A_REG		0x09
#define OTG_TLIM_THERM_CNTRL_REG				0x0A
#define HARD_SOFT_LIMIT_CELL_TEMP_MONITOR_REG	0x0B
#define FAULT_INTERRUPT_REG		0x0C
#define STATUS_INTERRUPT_REG	0x0D
//#define SYSOK_REG		0x0E chris: only smb349 contain this register
#define I2C_BUS_SLAVE_REG		0x0E //chris: add
#define CMD_A_REG		0x30
#define CMD_B_REG		0x31
#define CMD_C_REG		0x33
#define INTERRUPT_A_REG		0x35
#define INTERRUPT_B_REG		0x36
#define INTERRUPT_C_REG		0x37
#define INTERRUPT_D_REG		0x38
#define INTERRUPT_E_REG		0x39
#define INTERRUPT_F_REG		0x3A
#define STATUS_A_REG	0x3B
#define STATUS_B_REG	0x3C
#define STATUS_C_REG	0x3D
#define STATUS_D_REG	0x3E
#define STATUS_E_REG	0x3F

/* Status bits and masks */
#define CHG_STATUS_MASK		SMB347_MASK(2, 1)
#define CHG_ENABLE_STATUS_BIT		BIT(0)

/* Control bits and masks */
#define FAST_CHG_CURRENT_MASK			SMB347_MASK(4, 4)
#define AC_INPUT_CURRENT_LIMIT_MASK		SMB347_MASK(4, 0)
#define PRE_CHG_CURRENT_MASK			SMB347_MASK(3, 5)
#define TERMINATION_CURRENT_MASK		SMB347_MASK(3, 2)
#define PRE_CHG_TO_FAST_CHG_THRESH_MASK	SMB347_MASK(2, 6)
#define FLOAT_VOLTAGE_MASK				SMB347_MASK(6, 0)
#define CHG_ENABLE_BIT			BIT(1)
#define VOLATILE_W_PERM_BIT		BIT(7)
#define USB_SELECTION_BIT		BIT(1)
#define SYSTEM_FET_ENABLE_BIT	BIT(7)
#define AUTOMATIC_INPUT_CURR_LIMIT_BIT			BIT(4)
#define AUTOMATIC_POWER_SOURCE_DETECTION_BIT	BIT(2)
#define BATT_OV_END_CHG_BIT		BIT(1)
#define VCHG_FUNCTION			BIT(0)
#define CURR_TERM_END_CHG_BIT	BIT(6)

#define OTGID_PIN_CONTROL_MASK	SMB347_MASK(2, 6)			// OTHER_CTRL_A_REG
#define OTGID_PIN_CONTROL_BITS	BIT(6)			// OTHER_CTRL_A_REG (RID Disable, OTG Pin Control)

#define OTG_CURRENT_LIMIT_AT_USBIN_MASK	SMB347_MASK(2, 2)	// OTG_TLIM_THERM_CNTRL_REG
#define OTG_CURRENT_LIMIT_750mA	(BIT(2) | BIT(3))
#define OTG_CURRENT_LIMIT_250mA	BIT(2)

#define CFG_411V				BIT(1)|BIT(2)|BIT(3)|BIT(4)
#define CFG_435V				BIT(1)|BIT(3)|BIT(5)
#define CFG_432V				BIT(0)|BIT(3)|BIT(5)
#define CFG_FAST_CHARGE            0x69
#define CFG_SOFT_LIMIT   	   0x0a
#define CFG_SOFT_700mA   	   BIT(6)
#define CFG_AICL        	   BIT(4)
#define CFG_1200mA        	   BIT(2)
#define CFG_1500mA        	   BIT(2) | BIT(0)



#define CREATE_DEBUGFS_INTERRUPT_STATUS_REGISTERS

extern struct battery_info_reply batt_info;
extern unsigned int query_cable_status(void);
struct wake_lock wakelock_cable;

static int smb347_read_reg(struct i2c_client *client, int reg,
				u8 *val, int ifDebug)
{
	s32 ret;
	struct smb347_charger *smb347_chg;

	smb347_chg = i2c_get_clientdata(client);
	ret = i2c_smbus_read_byte_data(smb347_chg->client, reg);
	if (ret < 0) {
		dev_err(&smb347_chg->client->dev,
			"i2c read fail: can't read from Reg%02Xh: %d\n", reg, ret);
		return ret;
	} else {
		*val = ret;
	}
	if (ifDebug) pr_info("Reg%02Xh = " BYTETOBINARYPATTERN "\n", reg, BYTETOBINARY(*val));

	return 0;
}

static int smb347_write_reg(struct i2c_client *client, int reg,
						u8 val)
{
	s32 ret;
	struct smb347_charger *smb347_chg;

	//pr_info("write " BYTETOBINARYPATTERN " to Reg%02Xh\n", BYTETOBINARY(val), reg);
	smb347_chg = i2c_get_clientdata(client);

	ret = i2c_smbus_write_byte_data(smb347_chg->client, reg, val);
	if (ret < 0) {
		dev_err(&smb347_chg->client->dev,
			"i2c write fail: can't write %02X to %02X: %d\n",
			val, reg, ret);
		return ret;
	}
	return 0;
}

static int smb347_masked_write(struct i2c_client *client, int reg,
		u8 mask, u8 val)
{
	s32 rc;
	u8 temp;

	//pr_info("\n");
	rc = smb347_read_reg(client, reg, &temp, 0);
	if (rc) {
		pr_err("smb347_read_reg failed: reg=%03X, rc=%d\n", reg, rc);
		return rc;
	}
	temp &= ~mask;
	temp |= val & mask;
	rc = smb347_write_reg(client, reg, temp);
	if (rc) {
		pr_err("smb347_write failed: reg=%03X, rc=%d\n", reg, rc);
		return rc;
	}
	return 0;
}

static int smb347_read(struct smb347_charger *smb, u8 reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(smb->client, reg);
	if (ret < 0)
		dev_warn(&smb->client->dev, "failed to read reg 0x%x: %d\n",
			 reg, ret);
	return ret;
}

static int smb347_write(struct smb347_charger *smb, u8 reg, u8 val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(smb->client, reg, val);
	if (ret < 0)
		dev_warn(&smb->client->dev, "failed to write reg 0x%x: %d\n",
			 reg, ret);

	return ret;
}

/*
 * smb347_set_writable - enables/disables writing to non-volatile registers
 * @smb: pointer to smb347 charger instance
 *
 * You can enable/disable writing to the non-volatile configuration
 * registers by calling this function.
 *
 * Returns %0 on success and negative errno in case of failure.
 */
static int smb347_set_writable(struct smb347_charger *smb, bool writable)
{
	int ret;

	ret = smb347_read(smb, CMD_A);
	if (ret < 0)
		return ret;

	if (writable)
		ret |= CMD_A_ALLOW_WRITE;
	else
		ret &= ~CMD_A_ALLOW_WRITE;

	return smb347_write(smb, CMD_A, ret);
}

#ifdef EXPORT_CHARGER_OTG

static int otg(int toggle)
{
	int ret;

	if (!smb347_dev) {
		pr_info("Warning: smb347_dev is null due to probe function has error\n");
		return 1;
	}

	ret = smb347_set_writable(smb347_dev, true);
	if (ret < 0)
		return ret;

	if (toggle) {
//ME371MG only
#if 0
		if (HW_ID_EVB == Read_HW_ID()){
        		/* In order to switch D+/- to MSIC, set ACOK with active_high for ME371MG EVB */
        		smb347_masked_write(smb347_dev->client, SYSOK_USB3_SELECT_REG, BIT(0), 1);
		}
#endif
                
                /* 1.Disable OTG */
                ret = smb347_masked_write(smb347_dev->client, CMD_A_REG,
			BIT(4), 0);
		if (ret) {
			pr_err("Failed to set OTG Disable bit ret=%d\n", ret);
			return ret;
		}


                /* 2.Set OTG current limit to 250mA & UVLO 2.7V*/
		// Refer to SMB347 Application Note 72 to solve serious problems
		ret = smb347_masked_write(smb347_dev->client, OTG_TLIM_THERM_CNTRL_REG,
			BIT(0)|BIT(1)|BIT(2)|BIT(3), BIT(2));
		if (ret) {
			pr_err("Failed to set OTG current limit 250mA. ret=%d\n", ret);
			return ret;
		}

                /* 3.Enable OTG Function*/
		ret = smb347_masked_write(smb347_dev->client, CMD_A_REG,
			BIT(4), BIT(4));
		if (ret) {
			pr_err("Failed to set OTG Enable bit ret=%d\n", ret);
			return ret;
		}

                
                /* 4.Set OTG current limit to 750mA & UVLO 2.7V*/
		// Refer to SMB347 Application Note 72 to solve serious problems
		ret = smb347_masked_write(smb347_dev->client, OTG_TLIM_THERM_CNTRL_REG,
			BIT(0)|BIT(1)|BIT(2)|BIT(3), BIT(2)|BIT(3));
		if (ret) {
			pr_err("Failed to set OTG current limit 750mA. ret=%d\n", ret);
			return ret;
		}

		/* OTG 5V function set to PIN control due to HW will reset IC to default
			when external USB device is removed. This function support in SR2. And
			charger IC will be replaced with smb345 */
#if 0
		if (HW_ID_EVB != Read_HW_ID() && HW_ID_SR1 != Read_HW_ID())
		{
#endif
                        /* 5.OTG function change to PIN control */

			ret = smb347_masked_write(smb347_dev->client, OTHER_CTRL_A_REG,
					BIT(6)|BIT(7), BIT(6));
			if (ret) {
				pr_err("Failed to set OTGID_PIN_CONTROL_BITS rc=%d\n", ret);
				return ret;
			}
#if 0
		}
#endif
	} else {

//ME371MG only
#if 0
                
		if (HW_ID_EVB == Read_HW_ID()){
        		/* In order to switch D+/- to MSIC, set ACOK with active_low for ME371MG EVB */
        		smb347_masked_write(smb347_dev->client, SYSOK_USB3_SELECT_REG, BIT(0), 0);
		}

		/* OTG 5V function set to PIN control due to HW will reset IC to default
			when external USB device is removed. This function support in SR2. And
			charger IC will be replaced with smb345 */
		if (HW_ID_EVB == Read_HW_ID() || HW_ID_SR1 == Read_HW_ID())
		{
			// Refer to SMB347 Application Note 72 to solve serious problems
			ret = smb347_masked_write(smb347_dev->client, OTG_TLIM_THERM_CNTRL_REG,
					OTG_CURRENT_LIMIT_AT_USBIN_MASK, OTG_CURRENT_LIMIT_250mA);
			if (ret) {
				pr_err("Failed to set OTG current limit 750mA. ret=%d\n", ret);
				return ret;
			}

			ret = smb347_masked_write(smb347_dev->client, CMD_A_REG,
					BIT(4), 0);
			if (ret) {
				pr_err("Failed to set OTG Enable bit ret=%d\n", ret);
				return ret;
			}
		}
#endif
	}

	smb347_dev->otg_enabled = (toggle > 0 ? true : false);
	return 0;
}

int setSMB347Charger(int usb_state)
{
	int ret = 0;
        batt_info.cable_source = usb_state;
  if (!batt_info.isValidBattID) {
      pr_info("%s  not Valid BattID\n", __func__);
      smb347_charging_toggle(false);
      return 0;
  }

	switch (usb_state)
	{
	case USB_IN:
		pr_info("usb_state: USB_IN\n");
		usb_to_battery_callback(USB_PC);
		break;
	case AC_IN:
		pr_info("usb_state: AC_IN\n");
		usb_to_battery_callback(USB_ADAPTER);
		break;
	case CABLE_OUT:
		pr_info("usb_state: CABLE_OUT\n");
		usb_to_battery_callback(NO_CABLE);
		break;
	case ENABLE_5V:
		pr_info("usb_state: ENABLE_5V\n");
		ret = otg(1);
		break;
	case DISABLE_5V:
		pr_info("usb_state: DISABLE_5V\n");
		ret = otg(0);
		break;
	default:
		pr_info("ERROR: wrong usb state value = %d\n", usb_state);
		ret = 1;
	}

	return ret;
}
EXPORT_SYMBOL(setSMB347Charger);
#endif

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#if 0
/* Convert register value to current using lookup table */
static int hw_to_current(const unsigned int *tbl, size_t size, unsigned int val)
{
	if (val >= size)
		return -EINVAL;
	return tbl[val];
}

/* Convert current to register value using lookup table */
static int current_to_hw(const unsigned int *tbl, size_t size, unsigned int val)
{
	size_t i;

	for (i = 0; i < size; i++)
		if (val < tbl[i])
			break;
	return i > 0 ? i - 1 : -EINVAL;
}
#endif

/**
 * smb347_update_status - updates the charging status
 * @smb: pointer to smb347 charger instance
 *
 * Function checks status of the charging and updates internal state
 * accordingly. Returns %0 if there is no change in status, %1 if the
 * status has changed and negative errno in case of failure.
 */
static int smb347_update_status(struct smb347_charger *smb)
{
	bool usb = false;
	bool dc = false;
	int ret;

	ret = smb347_read(smb, IRQSTAT_E);
	if (ret < 0)
		return ret;

	/*
	 * Dc and usb are set depending on whether they are enabled in
	 * platform data _and_ whether corresponding undervoltage is set.
	 */
	if (!smb->otg_enabled) {
		if (smb->pdata->use_mains)
			dc = !(ret & IRQSTAT_E_DCIN_UV_STAT);
		if (smb->pdata->use_usb)
			usb = !(ret & IRQSTAT_E_USBIN_UV_STAT);
	}

	mutex_lock(&smb->lock);
	ret = smb->mains_online != dc || smb->usb_online != usb;
	smb->mains_online = dc;
	smb->usb_online = usb;
	mutex_unlock(&smb->lock);

	return ret;
}

#if 0
/**
 * smb347_status_monitor - worker function to monitor status
 * @work: delayed work handler structure
 * Context: Can sleep
 *
 * Monitors status of the charger and updates the charging status.
 * Note: This worker is manily added to notify the user space about
 * capacity, health and status chnages.
 */
static void smb347_status_monitor(struct work_struct *work)
{
	struct smb347_charger *smb = container_of(work,
			struct smb347_charger, smb347_statmon_worker.work);
	int ret;

	pm_runtime_get_sync(&smb->client->dev);

	ret = smb347_update_status(smb);
	if (ret < 0)
		dev_err(&smb->client->dev, "error in updating smb347 status\n");

	if (smb->pdata->use_mains)
		power_supply_changed(&smb->mains);
	if (smb->pdata->use_usb)
		power_supply_changed(&smb->usb);
	schedule_delayed_work(&smb->smb347_statmon_worker,
						STATUS_UPDATE_INTERVAL);
	pm_runtime_put_sync(&smb->client->dev);
}
#endif


int smb347_control_JEITA(bool on) {
    int ret = 0;

	if (!smb347_dev) {
		pr_info("Warning: smb347_dev is null due to probe function has error\n");
		return 1;
	}

	pr_warn("%s: *** control_JEITA: %s ***\n", __func__, on ? "on" : "off");

	mutex_lock(&smb347_dev->lock);

	ret = smb347_set_writable(smb347_dev, true);
	if (ret < 0)
		goto out;

	/* Config CFG_TEMP_LIMIT register */
	ret = smb347_read(smb347_dev, CFG_TEMP_LIMIT);
	if (ret < 0)
		goto out;

	/*
	 * Make the control JEITA functionality controllable by a write to the
	 * command register unless pin control is specified in the platform
	 * data.
	 */
	ret &= (BIT(6)|BIT(7));
	if (on) {
		ret |= (BIT(0)|BIT(1)|BIT(4)|BIT(5));
	} else {
	  ret &= 0xDD;
		ret |= (BIT(0)|BIT(4));
	}

  pr_info("write control JEITA = 0x%02x\n", ret);
	ret = smb347_write(smb347_dev, CFG_TEMP_LIMIT, ret);
	if (ret < 0)
		goto out;

  // === Set Soft Hot Limit Behavior ====
  /* Config CFG_THERM register */
	ret = smb347_read(smb347_dev, CFG_THERM);
	if (ret < 0)
		goto out;

  ret &= ~(BIT(0)|BIT(1));
	if (on) {
		// No Response
	} else {
		// Float Voltage Compensation
		ret |= BIT(1);
	}

  pr_info("write control JEITA,Set Soft Hot Limit Behavior = 0x%02x\n", ret);
	ret = smb347_write(smb347_dev,  CFG_THERM, ret);
	if (ret < 0)
		goto out;
out:
	mutex_unlock(&smb347_dev->lock);
	return ret;

}

int smb347_get_aicl_result() {
        int ret = 0;

	if (!smb347_dev) {
		pr_info("Warning: smb347_dev is null due to probe function has error\n");
		return 1;
	}

	pr_info(" ***  %s ***\n", __func__);

	mutex_lock(&smb347_dev->lock);

	ret = smb347_set_writable(smb347_dev, true);
	if (ret < 0)
		goto out;

	/* Config VAR_FUNC_REG register */
	ret = smb347_read(smb347_dev, STAT_E);
	if (ret < 0)
		goto out;

        ret &= 0x0f;
        pr_info("smb347 get aicl result = 0x%02x\n", ret);
out:
	mutex_unlock(&smb347_dev->lock);
	return ret;

}

int smb347_set_battery_0V() {
        int ret = 0;

	if (!smb347_dev) {
		pr_info("Warning: smb347_dev is null due to probe function has error\n");
		return 1;
	}

	pr_info(" ***  %s ***\n", __func__);

	mutex_lock(&smb347_dev->lock);

	ret = smb347_set_writable(smb347_dev, true);
	if (ret < 0)
		goto out;

	/* Config VAR_FUNC_REG register */
	ret = smb347_read(smb347_dev, VAR_FUNC_REG);
	if (ret < 0)
		goto out;

	ret &= ~BIT(1);
        pr_info("write  battery 0V does not charger cycle = 0x%02x\n", ret);
	ret = smb347_write(smb347_dev, VAR_FUNC_REG, ret);
	if (ret < 0)
		goto out;

out:
	mutex_unlock(&smb347_dev->lock);
	return ret;

}

int smb347_set_voltage(int v) {
    int ret = 0;

	if (!smb347_dev) {
		pr_info("Warning: smb347_dev is null due to probe function has error\n");
		return 1;
	}

	pr_warn("%s: *** charging voltage: %d mV ***\n", __func__, v);

	mutex_lock(&smb347_dev->lock);

	ret = smb347_set_writable(smb347_dev, true);
	if (ret < 0)
		goto out;

	/* Config CFG_FLOAT_VOLTAGE register */
	ret = smb347_read(smb347_dev, CFG_FLOAT_VOLTAGE);
	if (ret < 0)
		goto out;

	/*
	 * Make the voltage functionality controllable by a write to the
	 * command register unless pin control is specified in the platform
	 * data.
	 */
	ret &= (BIT(6)|BIT(7));
        switch (v) {
            case 4110:
               /* voltage = 4.11v */
	       ret |= CFG_411V;
               break;
            case 4350:
               /* voltage = 4.35v */
	       ret |= CFG_435V;
               break;
            case 4320:
            default:
               /* voltage = 4.32v */
               ret |= CFG_432V;
               break;
        }

    pr_info("write voltage = 0x%02x\n", ret);
	ret = smb347_write(smb347_dev, CFG_FLOAT_VOLTAGE, ret);
	if (ret < 0)
		goto out;

out:
	mutex_unlock(&smb347_dev->lock);
	return ret;

}

int smb347_set_fast_charge(void) {
    int ret = 0;

	if (!smb347_dev) {
		pr_info("Warning: smb347_dev is null due to probe function has error\n");
		return 1;
	}

	pr_info("== %s ==", __func__);

	mutex_lock(&smb347_dev->lock);

	ret = smb347_set_writable(smb347_dev, true);
	if (ret < 0)
		goto out;

	/* Config CFG_CHARGE_CURRENT register */
	ret = smb347_read(smb347_dev, CFG_CHARGE_CURRENT);
	if (ret < 0)
		goto out;

	/*
	 * Make the fast charge functionality controllable by a write to the
	 * command register unless pin control is specified in the platform
	 * data.
	 */
	ret = CFG_FAST_CHARGE;
        pr_info("set fast charge = 0x%02x\n", ret);
	ret = smb347_write(smb347_dev, CFG_CHARGE_CURRENT, ret);
	if (ret < 0)
		goto out;

        /* Config CFG_SOFT_LIMIT register */
	ret = smb347_read(smb347_dev, CFG_SOFT_LIMIT);
	if (ret < 0)
		goto out;

	/*
	 * Make the fast charge functionality controllable by a write to the
	 * command register unless pin control is specified in the platform
	 * data.
	 */
	ret &= ~(BIT(7));
	ret |= CFG_SOFT_700mA;
    pr_info("set soft limit 700mA = 0x%02x\n", ret);
	ret = smb347_write(smb347_dev, CFG_SOFT_LIMIT, ret);
	if (ret < 0)
		goto out;
out:
	mutex_unlock(&smb347_dev->lock);
	return ret;

}

int smb347_AC_in_current(int c) {
    int ret = 0;

	if (!smb347_dev) {
		pr_info("Warning: smb347_dev is null due to probe function has error\n");
		return 1;
	}

	pr_info("== %s ==", __func__);

	mutex_lock(&smb347_dev->lock);

	ret = smb347_set_writable(smb347_dev, true);
	if (ret < 0)
		goto out;

	/* Config VAR_FUNC_REG register */
	ret = smb347_read(smb347_dev, VAR_FUNC_REG);
	if (ret < 0)
		goto out;

	ret &= ~CFG_AICL;
	ret = smb347_write(smb347_dev, VAR_FUNC_REG, ret);
	if (ret < 0)
		goto out;

    /* Config INPUT_CURRENT_LIMIT_REG register */
	ret = smb347_read(smb347_dev, INPUT_CURRENT_LIMIT_REG);
	if (ret < 0)
		goto out;

	ret &= 0xf0;
        switch (c) {
           case 1500:
              ret |= CFG_1500mA;
              break;
           case 1200:
           default:
	      ret |= CFG_1200mA;
              break;
        }
    pr_info("set AC IN = 0x%02x\n", ret);
	ret = smb347_write(smb347_dev, INPUT_CURRENT_LIMIT_REG, ret);
	if (ret < 0)
		goto out;

	/* Config VAR_FUNC_REG register */
	ret = smb347_read(smb347_dev, VAR_FUNC_REG);
	if (ret < 0)
		goto out;

	ret |=CFG_AICL;
	ret = smb347_write(smb347_dev, VAR_FUNC_REG, ret);
	if (ret < 0)
		goto out;
out:
	mutex_unlock(&smb347_dev->lock);
	return ret;

}

int smb347_charging_toggle(bool on)
{
	int ret = 0;

	if (!smb347_dev) {
		pr_info("Warning: smb347_dev is null due to probe function has error\n");
		return 1;
	}

	if (!on)
	pr_warn("%s: *** charging toggle: %s ***\n", __func__, on ? "ON" : "OFF");

	mutex_lock(&smb347_dev->lock);

	ret = smb347_set_writable(smb347_dev, true);
	if (ret < 0)
		goto out;

	/* Config CFG_PIN register */

	ret = smb347_read(smb347_dev, CFG_PIN);
	if (ret < 0)
		goto out;

	/*
	 * Make the charging functionality controllable by a write to the
	 * command register unless pin control is specified in the platform
	 * data.
	 */
	ret &= ~CFG_PIN_EN_CTRL_MASK;
	if (on) {
		/* set Pin Controls - active low (ME371MG connect EN to GROUND) */
		ret |= CFG_PIN_EN_CTRL_ACTIVE_LOW;
	} else {
		/* Do nothing, 0 means i2c control
			. I2C Control - "0" in Command Register disables charger */
	}

	ret = smb347_write(smb347_dev, CFG_PIN, ret);
	if (ret < 0)
		goto out;
#if 0
	/* Config CMD_A register */

	ret = smb347_read(smb347_dev, CMD_A);
	if (ret < 0)
		goto out;

	//smb347_dev->charging_enabled = on;

	if (on)
		ret |= CMD_A_CHG_ENABLED;
	else
		ret &= ~CMD_A_CHG_ENABLED;

	ret = smb347_write(smb347_dev, CMD_A, ret);
#endif
out:
	mutex_unlock(&smb347_dev->lock);
	return ret;
}

#if CANCEL_SOFT_HOT_TEMP_LIMIT
static int cancel_soft_hot_temp_limit(struct smb347_charger *smb)
{
	int ret;

	/* ME371MG EVB/SR1 meet this problem that smb347 stop charging
		when IC temperature is high up to Soft Hot Limit. But Battery
		is not full charging. We disable this limitation.
	*/

	pr_warn("set soft hot temperature limit behavior to No Response\n");
	ret = smb347_read(smb, CFG_THERM);
	if (ret < 0)
		return ret;

	ret &= ~CFG_THERM_SOFT_HOT_COMPENSATION_MASK;

	ret = smb347_write(smb, CFG_THERM, ret);
	if (ret < 0)
		return ret;

        return ret;
}
#endif

#if 0
/*
 * smb347_is_online - returns whether input power source is connected
 * @smb: pointer to smb347 charger instance
 *
 * Returns %true if input power source is connected. Note that this is
 * dependent on what platform has configured for usable power sources. For
 * example if USB is disabled, this will return %false even if the USB
 * cable is connected.
 */
static bool smb347_is_online(struct smb347_charger *smb)
{
	bool ret;

	mutex_lock(&smb->lock);
	ret = (smb->usb_online || smb->mains_online) && !smb->otg_enabled;
	mutex_unlock(&smb->lock);

	return ret;
}

/**
 * smb347_charging_status - returns status of charging
 * @smb: pointer to smb347 charger instance
 *
 * Function returns charging status. %0 means no charging is in progress,
 * %1 means pre-charging, %2 fast-charging and %3 taper-charging.
 */
static int smb347_charging_status(struct smb347_charger *smb)
{
	int ret;

	if (!smb347_is_online(smb))
		return 0;

	ret = smb347_read(smb, STAT_C);
	if (ret < 0)
		return 0;

	return (ret & STAT_C_CHG_MASK) >> STAT_C_CHG_SHIFT;
}

static int smb347_charging_set(struct smb347_charger *smb, bool enable)
{
	int ret = 0;

	if (smb->pdata->enable_control != SMB347_CHG_ENABLE_SW) {
		dev_dbg(&smb->client->dev,
			"charging enable/disable in SW disabled\n");
		return 0;
	}

	mutex_lock(&smb->lock);
	if (smb->charging_enabled != enable) {
		ret = smb347_read(smb, CMD_A);
		if (ret < 0)
			goto out;

		smb->charging_enabled = enable;

		if (enable)
			ret |= CMD_A_CHG_ENABLED;
		else
			ret &= ~CMD_A_CHG_ENABLED;

		ret = smb347_write(smb, CMD_A, ret);
	}
out:
	mutex_unlock(&smb->lock);
	return ret;
}

static inline int smb347_charging_enable(struct smb347_charger *smb)
{
	/* prevent system from entering s3 while charger is connected */
	if (!wake_lock_active(&smb->wakelock))
		wake_lock(&smb->wakelock);
	return smb347_charging_set(smb, true);
}

static inline int smb347_charging_disable(struct smb347_charger *smb)
{
	int ret;

	ret = smb347_charging_set(smb, false);
	/* release the wake lock when charger is unplugged */
	if (wake_lock_active(&smb->wakelock))
		wake_unlock(&smb->wakelock);

	return ret;
}

static int smb347_update_online(struct smb347_charger *smb)
{
	int ret;

	/*
	 * Depending on whether valid power source is connected or not, we
	 * disable or enable the charging. We do it manually because it
	 * depends on how the platform has configured the valid inputs.
	 */
	if (smb347_is_online(smb)) {
		ret = smb347_charging_enable(smb);
		if (ret < 0)
			dev_err(&smb->client->dev,
				"failed to enable charging\n");
	} else {
		ret = smb347_charging_disable(smb);
		if (ret < 0)
			dev_err(&smb->client->dev,
				"failed to disable charging\n");
	}

	return ret;
}

static int smb347_otg_set(struct smb347_charger *smb, bool enable)
{
	const struct smb347_charger_platform_data *pdata = smb->pdata;
	int ret;

	mutex_lock(&smb->lock);

	if (pdata->otg_control == SMB347_OTG_CONTROL_SW) {
		ret = smb347_read(smb, CMD_A);
		if (ret < 0)
			goto out;

		if (enable)
			ret |= CMD_A_OTG_ENABLED;
		else
			ret &= ~CMD_A_OTG_ENABLED;

		ret = smb347_write(smb, CMD_A, ret);
		if (ret < 0)
			goto out;
	} else {
		/*
		 * Switch to pin control or auto-OTG depending on how
		 * platform has configured.
		 */
		smb347_set_writable(smb, true);

		ret = smb347_read(smb, CFG_OTHER);
		if (ret < 0) {
			smb347_set_writable(smb, false);
			goto out;
		}

		ret &= ~CFG_OTHER_RID_MASK;

		switch (pdata->otg_control) {
		case SMB347_OTG_CONTROL_SW_PIN:
			if (enable) {
				ret |= CFG_OTHER_RID_DISABLED_OTG_PIN;
				ret |= CFG_OTHER_OTG_PIN_ACTIVE_LOW;
			} else {
				ret &= ~CFG_OTHER_OTG_PIN_ACTIVE_LOW;
			}
			break;

		case SMB347_OTG_CONTROL_SW_AUTO:
			if (enable)
				ret |= CFG_OTHER_RID_ENABLED_AUTO_OTG;
			break;

		default:
			dev_err(&smb->client->dev,
				"impossible OTG control configuration: %d\n",
				pdata->otg_control);
			break;
		}

		ret = smb347_write(smb, CFG_OTHER, ret);
		smb347_set_writable(smb, false);
		if (ret < 0)
			goto out;
	}

	smb->otg_enabled = enable;

out:
	mutex_unlock(&smb->lock);
	return ret;
}

static inline int smb347_otg_enable(struct smb347_charger *smb)
{
	return smb347_otg_set(smb, true);
}

static inline int smb347_otg_disable(struct smb347_charger *smb)
{
	return smb347_otg_set(smb, false);
}

static void smb347_otg_drive_vbus(struct smb347_charger *smb, bool enable)
{
	if (enable == smb->otg_enabled)
		return;

	if (enable) {
		if (smb->otg_battery_uv) {
			dev_dbg(&smb->client->dev,
				"battery low voltage, won't enable OTG VBUS\n");
			return;
		}

		/*
		 * Normal charging must be disabled first before we try to
		 * enable OTG VBUS.
		 */
		smb347_charging_disable(smb);
		smb347_otg_enable(smb);
		smb347_update_status(smb);
		dev_dbg(&smb->client->dev, "OTG VBUS on\n");
	} else {
		smb347_otg_disable(smb);
		smb347_update_status(smb);
		/*
		 * Only re-enable charging if we have some power supply
		 * connected.
		 */
		if (smb347_is_online(smb)) {
			smb347_charging_enable(smb);
			smb->otg_battery_uv = false;
			/* Small delay-interval(10-11ms) for
			 * STAT-C to be updated
			 */
			usleep_range(10000, 11000);
		}
		dev_dbg(&smb->client->dev, "OTG VBUS off\n");
	}

	if (smb->pdata->use_mains)
		power_supply_changed(&smb->mains);
	if (smb->pdata->use_usb)
		power_supply_changed(&smb->usb);
}

static void smb347_otg_work(struct work_struct *work)
{
	struct smb347_charger *smb =
		container_of(work, struct smb347_charger, otg_work);
	struct smb347_otg_event *evt, *tmp;
	unsigned long flags;

	/* Process the whole event list in one go. */
	spin_lock_irqsave(&smb->otg_queue_lock, flags);
	list_for_each_entry_safe(evt, tmp, &smb->otg_queue, node) {
		list_del(&evt->node);
		spin_unlock_irqrestore(&smb->otg_queue_lock, flags);

		/* For now we only support set vbus events */
		smb347_otg_drive_vbus(smb, evt->param);
		kfree(evt);

		spin_lock_irqsave(&smb->otg_queue_lock, flags);
	}
	spin_unlock_irqrestore(&smb->otg_queue_lock, flags);
}

static int smb347_otg_notifier(struct notifier_block *nb, unsigned long event,
			       void *param)
{
	struct smb347_charger *smb =
		container_of(nb, struct smb347_charger, otg_nb);
	struct smb347_otg_event *evt;

	dev_dbg(&smb->client->dev, "OTG notification: %lu\n", event);
	if (!param || event != USB_EVENT_DRIVE_VBUS || !smb->running)
		return NOTIFY_DONE;

	evt = kzalloc(sizeof(*evt), GFP_ATOMIC);
	if (!evt) {
		dev_err(&smb->client->dev,
			"failed to allocate memory for OTG event\n");
		return NOTIFY_DONE;
	}

	evt->param = *(bool *)param;
	INIT_LIST_HEAD(&evt->node);

	spin_lock(&smb->otg_queue_lock);
	list_add_tail(&evt->node, &smb->otg_queue);
	spin_unlock(&smb->otg_queue_lock);

	queue_work(system_nrt_wq, &smb->otg_work);
	return NOTIFY_OK;
}

static int smb347_set_charge_current(struct smb347_charger *smb)
{
	int ret, val;

	ret = smb347_read(smb, CFG_CHARGE_CURRENT);
	if (ret < 0)
		return ret;

	if (smb->pdata->max_charge_current) {
		val = current_to_hw(fcc_tbl, ARRAY_SIZE(fcc_tbl),
				    smb->pdata->max_charge_current);
		if (val < 0)
			return val;

		ret &= ~CFG_CHARGE_CURRENT_FCC_MASK;
		ret |= val << CFG_CHARGE_CURRENT_FCC_SHIFT;
	}

	if (smb->pdata->pre_charge_current) {
		val = current_to_hw(pcc_tbl, ARRAY_SIZE(pcc_tbl),
				    smb->pdata->pre_charge_current);
		if (val < 0)
			return val;

		ret &= ~CFG_CHARGE_CURRENT_PCC_MASK;
		ret |= val << CFG_CHARGE_CURRENT_PCC_SHIFT;
	}

	if (smb->pdata->termination_current) {
		val = current_to_hw(tc_tbl, ARRAY_SIZE(tc_tbl),
				    smb->pdata->termination_current);
		if (val < 0)
			return val;

		ret &= ~CFG_CHARGE_CURRENT_TC_MASK;
		ret |= val;
	}

	return smb347_write(smb, CFG_CHARGE_CURRENT, ret);
}

static int smb347_set_current_limits(struct smb347_charger *smb)
{
	int ret, val;

	ret = smb347_read(smb, CFG_CURRENT_LIMIT);
	if (ret < 0)
		return ret;

	if (smb->pdata->mains_current_limit) {
		val = current_to_hw(icl_tbl, ARRAY_SIZE(icl_tbl),
				    smb->pdata->mains_current_limit);
		if (val < 0)
			return val;

		ret &= ~CFG_CURRENT_LIMIT_DC_MASK;
		ret |= val << CFG_CURRENT_LIMIT_DC_SHIFT;
	}

	if (smb->pdata->usb_hc_current_limit) {
		val = current_to_hw(icl_tbl, ARRAY_SIZE(icl_tbl),
				    smb->pdata->usb_hc_current_limit);
		if (val < 0)
			return val;

		ret &= ~CFG_CURRENT_LIMIT_USB_MASK;
		ret |= val;
	}

	return smb347_write(smb, CFG_CURRENT_LIMIT, ret);
}

static int smb347_set_voltage_limits(struct smb347_charger *smb)
{
	int ret, val;

	ret = smb347_read(smb, CFG_FLOAT_VOLTAGE);
	if (ret < 0)
		return ret;

	if (smb->pdata->pre_to_fast_voltage) {
		val = smb->pdata->pre_to_fast_voltage;

		/* uV */
		val = clamp_val(val, 2400000, 3000000) - 2400000;
		val /= 200000;

		ret &= ~CFG_FLOAT_VOLTAGE_THRESHOLD_MASK;
		ret |= val << CFG_FLOAT_VOLTAGE_THRESHOLD_SHIFT;
	}

	if (smb->pdata->max_charge_voltage) {
		val = smb->pdata->max_charge_voltage;

		/* uV */
		val = clamp_val(val, 3500000, 4500000) - 3500000;
		val /= 20000;

		ret |= val;
	}

	ret = smb347_write(smb, CFG_FLOAT_VOLTAGE, ret);
	if (ret < 0)
		return ret;

	if (smb->pdata->otg_uvlo_voltage) {
		val = smb->pdata->otg_uvlo_voltage;

		val = clamp_val(val, 2700000, 3300000) - 2700000;
		val /= 200000;

		ret = smb347_read(smb, CFG_OTG);
		if (ret < 0)
			return ret;

		ret &= ~CFG_OTG_BATTERY_UVLO_THRESHOLD_MASK;
		ret |= val & 0x3;

		ret = smb347_write(smb, CFG_OTG, ret);
		if (ret < 0)
			return ret;
	}

	return ret;
}

static int smb347_set_temp_limits(struct smb347_charger *smb)
{
	bool enable_therm_monitor = false;
	int ret, val;

	if (smb->pdata->chip_temp_threshold) {
		val = smb->pdata->chip_temp_threshold;

		/* degree C */
		val = clamp_val(val, 100, 130) - 100;
		val /= 10;

		ret = smb347_read(smb, CFG_OTG);
		if (ret < 0)
			return ret;

		ret &= ~CFG_OTG_TEMP_THRESHOLD_MASK;
		ret |= val << CFG_OTG_TEMP_THRESHOLD_SHIFT;

		ret = smb347_write(smb, CFG_OTG, ret);
		if (ret < 0)
			return ret;
	}

	ret = smb347_read(smb, CFG_TEMP_LIMIT);
	if (ret < 0)
		return ret;

	if (smb->pdata->soft_cold_temp_limit != SMB347_TEMP_USE_DEFAULT) {
		val = smb->pdata->soft_cold_temp_limit;

		val = clamp_val(val, 0, 15);
		val /= 5;
		/* this goes from higher to lower so invert the value */
		val = ~val & 0x3;

		ret &= ~CFG_TEMP_LIMIT_SOFT_COLD_MASK;
		ret |= val << CFG_TEMP_LIMIT_SOFT_COLD_SHIFT;

		enable_therm_monitor = true;
	}

	if (smb->pdata->soft_hot_temp_limit != SMB347_TEMP_USE_DEFAULT) {
		val = smb->pdata->soft_hot_temp_limit;

		val = clamp_val(val, 40, 55) - 40;
		val /= 5;

		ret &= ~CFG_TEMP_LIMIT_SOFT_HOT_MASK;
		ret |= val << CFG_TEMP_LIMIT_SOFT_HOT_SHIFT;

		enable_therm_monitor = true;
	}

	if (smb->pdata->hard_cold_temp_limit != SMB347_TEMP_USE_DEFAULT) {
		val = smb->pdata->hard_cold_temp_limit;

		val = clamp_val(val, -5, 10) + 5;
		val /= 5;
		/* this goes from higher to lower so invert the value */
		val = ~val & 0x3;

		ret &= ~CFG_TEMP_LIMIT_HARD_COLD_MASK;
		ret |= val << CFG_TEMP_LIMIT_HARD_COLD_SHIFT;

		enable_therm_monitor = true;
	}

	if (smb->pdata->hard_hot_temp_limit != SMB347_TEMP_USE_DEFAULT) {
		val = smb->pdata->hard_hot_temp_limit;

		val = clamp_val(val, 50, 65) - 50;
		val /= 5;

		ret &= ~CFG_TEMP_LIMIT_HARD_HOT_MASK;
		ret |= val << CFG_TEMP_LIMIT_HARD_HOT_SHIFT;

		enable_therm_monitor = true;
	}

	ret = smb347_write(smb, CFG_TEMP_LIMIT, ret);
	if (ret < 0)
		return ret;

	/*
	 * If any of the temperature limits are set, we also enable the
	 * thermistor monitoring.
	 *
	 * When soft limits are hit, the device will start to compensate
	 * current and/or voltage depending on the configuration.
	 *
	 * When hard limit is hit, the device will suspend charging
	 * depending on the configuration.
	 */
	if (enable_therm_monitor) {
		ret = smb347_read(smb, CFG_THERM);
		if (ret < 0)
			return ret;

		ret &= ~CFG_THERM_MONITOR_DISABLED;

		ret = smb347_write(smb, CFG_THERM, ret);
		if (ret < 0)
			return ret;
	}

	if (smb->pdata->suspend_on_hard_temp_limit) {
		ret = smb347_read(smb, CFG_SYSOK);
		if (ret < 0)
			return ret;

		ret &= ~CFG_SYSOK_SUSPEND_HARD_LIMIT_DISABLED;

		ret = smb347_write(smb, CFG_SYSOK, ret);
		if (ret < 0)
			return ret;
	}

	if (smb->pdata->soft_temp_limit_compensation !=
	    SMB347_SOFT_TEMP_COMPENSATE_DEFAULT) {
		val = smb->pdata->soft_temp_limit_compensation & 0x3;

		ret = smb347_read(smb, CFG_THERM);
		if (ret < 0)
			return ret;

		ret &= ~CFG_THERM_SOFT_HOT_COMPENSATION_MASK;
		ret |= val << CFG_THERM_SOFT_HOT_COMPENSATION_SHIFT;

		ret &= ~CFG_THERM_SOFT_COLD_COMPENSATION_MASK;
		ret |= val << CFG_THERM_SOFT_COLD_COMPENSATION_SHIFT;

		ret = smb347_write(smb, CFG_THERM, ret);
		if (ret < 0)
			return ret;
	}

	if (smb->pdata->charge_current_compensation) {
		val = current_to_hw(ccc_tbl, ARRAY_SIZE(ccc_tbl),
				    smb->pdata->charge_current_compensation);
		if (val < 0)
			return val;

		ret = smb347_read(smb, CFG_OTG);
		if (ret < 0)
			return ret;

		ret &= ~CFG_OTG_CC_COMPENSATION_MASK;
		ret |= (val & 0x3) << CFG_OTG_CC_COMPENSATION_SHIFT;

		ret = smb347_write(smb, CFG_OTG, ret);
		if (ret < 0)
			return ret;
	}

	return ret;
}

static int smb347_hw_init(struct smb347_charger *smb)
{
	int ret;

	ret = smb347_set_writable(smb, true);
	if (ret < 0)
		return ret;

	/*
	 * Program the platform specific configuration values to the device
	 * first.
	 */
	ret = smb347_set_charge_current(smb);
	if (ret < 0)
		goto fail;

	ret = smb347_set_current_limits(smb);
	if (ret < 0)
		goto fail;

	ret = smb347_set_voltage_limits(smb);
	if (ret < 0)
		goto fail;

	ret = smb347_set_temp_limits(smb);
	if (ret < 0)
		goto fail;

	/* If USB charging is disabled we put the USB in suspend mode */
	if (!smb->pdata->use_usb) {
		ret = smb347_read(smb, CMD_A);
		if (ret < 0)
			goto fail;

		ret |= CMD_A_SUSPEND_ENABLED;

		ret = smb347_write(smb, CMD_A, ret);
		if (ret < 0)
			goto fail;
	}

	/* Setup OTG VBUS control depending on the platform data. */
	ret = smb347_read(smb, CFG_OTHER);
	if (ret < 0)
		goto fail;

	ret &= ~CFG_OTHER_RID_MASK;

	switch (smb->pdata->otg_control) {
	case SMB347_OTG_CONTROL_DISABLED:
		break;

	case SMB347_OTG_CONTROL_SW:
	case SMB347_OTG_CONTROL_SW_PIN:
	case SMB347_OTG_CONTROL_SW_AUTO:
		smb->otg = otg_get_transceiver();
		if (smb->otg) {
			INIT_WORK(&smb->otg_work, smb347_otg_work);
			INIT_LIST_HEAD(&smb->otg_queue);
			spin_lock_init(&smb->otg_queue_lock);

			smb->otg_nb.notifier_call = smb347_otg_notifier;
			ret = otg_register_notifier(smb->otg, &smb->otg_nb);
			if (ret < 0) {
				otg_put_transceiver(smb->otg);
				smb->otg = NULL;
				goto fail;
			}

			dev_info(&smb->client->dev,
				"registered to OTG notifications\n");
		}
		break;

	case SMB347_OTG_CONTROL_PIN:
		ret |= CFG_OTHER_RID_DISABLED_OTG_PIN;
		ret |= CFG_OTHER_OTG_PIN_ACTIVE_LOW;
		break;

	case SMB347_OTG_CONTROL_AUTO:
		ret |= CFG_OTHER_RID_ENABLED_AUTO_OTG;
		break;
	}

	ret = smb347_write(smb, CFG_OTHER, ret);
	if (ret < 0)
		goto fail;

	ret = smb347_read(smb, CFG_PIN);
	if (ret < 0)
		goto fail;

	/*
	 * Make the charging functionality controllable by a write to the
	 * command register unless pin control is specified in the platform
	 * data.
	 */
	ret &= ~CFG_PIN_EN_CTRL_MASK;

	switch (smb->pdata->enable_control) {
	case SMB347_CHG_ENABLE_SW:
		/* Do nothing, 0 means i2c control */
		break;
	case SMB347_CHG_ENABLE_PIN_ACTIVE_LOW:
		ret |= CFG_PIN_EN_CTRL_ACTIVE_LOW;
		break;
	case SMB347_CHG_ENABLE_PIN_ACTIVE_HIGH:
		ret |= CFG_PIN_EN_CTRL_ACTIVE_HIGH;
		break;
	}

	/* Disable Automatic Power Source Detection (APSD) interrupt. */
	ret &= ~CFG_PIN_EN_APSD_IRQ;

	ret = smb347_write(smb, CFG_PIN, ret);
	if (ret < 0)
		goto fail;

	/*
	 * Summit recommends that register 0x02 (CFG_VARIOUS_FUNCS) is
	 * programmed last. This has something to do with the fact that
	 * current limits are correctly updated. We really don't have
	 * anything else to configure there except that we update the input
	 * source priority.
	 */
	ret = smb347_read(smb, CFG_VARIOUS_FUNCS);
	if (ret < 0)
		goto fail;

	ret &= ~CFG_VARIOUS_FUNCS_PRIORITY_USB;

	ret = smb347_write(smb, CFG_VARIOUS_FUNCS, ret);
	if (ret < 0)
		goto fail;

	ret = smb347_update_status(smb);
	if (ret < 0)
		goto fail;

	ret = smb347_update_online(smb);

	smb347_set_writable(smb, false);
	return ret;

fail:
	if (smb->otg) {
		otg_unregister_notifier(smb->otg, &smb->otg_nb);
		otg_put_transceiver(smb->otg);
		smb->otg = NULL;
	}
	smb347_set_writable(smb, false);
	return ret;
}
#endif

#if SMB347_INTERRUPT
static irqreturn_t smb347_interrupt(int irq, void *data)
{
	struct smb347_charger *smb = data;
	int stat_c, irqstat_c, irqstat_d, irqstat_e, irqstat_f;
	irqreturn_t ret = IRQ_NONE;

	pm_runtime_get_sync(&smb->client->dev);
	dev_warn(&smb->client->dev, "%s\n", __func__);

	stat_c = smb347_read(smb, STAT_C);
	if (stat_c < 0) {
		dev_warn(&smb->client->dev, "reading STAT_C failed\n");
		return IRQ_NONE;
	}

	irqstat_c = smb347_read(smb, IRQSTAT_C);
	if (irqstat_c < 0) {
		dev_warn(&smb->client->dev, "reading IRQSTAT_C failed\n");
		return IRQ_NONE;
	}

	irqstat_d = smb347_read(smb, IRQSTAT_D);
	if (irqstat_d < 0) {
		dev_warn(&smb->client->dev, "reading IRQSTAT_D failed\n");
		return IRQ_NONE;
	}

	irqstat_e = smb347_read(smb, IRQSTAT_E);
	if (irqstat_e < 0) {
		dev_warn(&smb->client->dev, "reading IRQSTAT_E failed\n");
		return IRQ_NONE;
	}

	irqstat_f = smb347_read(smb, IRQSTAT_F);
	if (irqstat_f < 0) {
		dev_warn(&smb->client->dev, "reading IRQSTAT_F failed\n");
		return IRQ_NONE;
	}

	/*
	 * If we get charger error we report the error back to user.
	 * If the error is recovered charging will resume again.
	 */
	if (stat_c & STAT_C_CHARGER_ERROR) {
		dev_err(&smb->client->dev,
			"****** charging stopped due to charger error ******\n");

#if 0
		if (smb->pdata->show_battery)
			power_supply_changed(&smb->battery);
#endif

		ret = IRQ_HANDLED;
	}

	/*
	 * If we reached the termination current the battery is charged and
	 * we can update the status now. Charging is automatically
	 * disabled by the hardware.
	 */
	if (irqstat_c & (IRQSTAT_C_TERMINATION_IRQ | IRQSTAT_C_TAPER_IRQ)) {
		if ((irqstat_c & IRQSTAT_C_TERMINATION_STAT) &&
						smb->pdata->show_battery)
#if 0
			power_supply_changed(&smb->battery);
#endif
		dev_info(&smb->client->dev,
			"[Charge Terminated] Going to HW Maintenance mode\n");
		ret = IRQ_HANDLED;
	}

	/*
	 * If we got a complete charger timeout int that means the charge
	 * full is not detected with in charge timeout value.
	 */
	if (irqstat_d & IRQSTAT_D_CHARGE_TIMEOUT_IRQ) {
		dev_info(&smb->client->dev,
			"[Charge Timeout]:Total Charge Timeout INT recieved\n");
		if (irqstat_d & IRQSTAT_D_CHARGE_TIMEOUT_STAT)
			dev_info(&smb->client->dev,
				"[Charge Timeout]:charging stopped\n");
#if 0
		if (smb->pdata->show_battery)
			power_supply_changed(&smb->battery);
#endif
		ret = IRQ_HANDLED;
	}

	/*
	 * If we got an under voltage interrupt it means that AC/USB input
	 * was connected or disconnected.
	 */
	if (irqstat_e & (IRQSTAT_E_USBIN_UV_IRQ | IRQSTAT_E_DCIN_UV_IRQ)) {
		if (smb347_update_status(smb) > 0) {
#if 0
			smb347_update_online(smb);
			if (smb->pdata->use_mains)
				power_supply_changed(&smb->mains);
			if (smb->pdata->use_usb)
				power_supply_changed(&smb->usb);
#endif
		}

		if (smb->mains_online || smb->usb_online)
			dev_info(&smb->client->dev, "Charger connected\n");
		else
			dev_info(&smb->client->dev, "Charger disconnected\n");

		ret = IRQ_HANDLED;
	}

	/*
	 * If the battery voltage falls below OTG UVLO the VBUS is
	 * automatically turned off but we must not enable it again unless
	 * UVLO is cleared. It will be cleared when external power supply
	 * is connected and the battery voltage goes over the UVLO
	 * threshold.
	 */
	if (irqstat_f & IRQSTAT_F_OTG_UV_IRQ) {
		smb->otg_battery_uv = !!(irqstat_f & IRQSTAT_F_OTG_UV_STAT);
		dev_info(&smb->client->dev, "Vbatt is below OTG UVLO\n");
		ret = IRQ_HANDLED;
	}

	pm_runtime_put_sync(&smb->client->dev);
	return ret;
}
#endif

static int smb347_irq_set(struct smb347_charger *smb, bool enable)
{
	int ret;

#if 0
	ret = smb347_set_writable(smb, true);
	if (ret < 0)
		return ret;
#endif

	/*
	 * Enable/disable interrupts for:
	 *	- under voltage
	 *	- termination current reached
	 *	- charger error
	 */
	if (enable) {
#if 0
		int val = CFG_FAULT_IRQ_DCIN_UV;

		if (smb->otg)
			val |= CFG_FAULT_IRQ_OTG_UV;

		ret = smb347_write(smb, CFG_FAULT_IRQ, val);
		if (ret < 0)
			goto fail;

		val = CFG_STATUS_IRQ_CHARGE_TIMEOUT |
			CFG_STATUS_IRQ_TERMINATION_OR_TAPER;
		ret = smb347_write(smb, CFG_STATUS_IRQ, val);
		if (ret < 0)
			goto fail;
#endif
// Need charger error IRQ
		ret = smb347_read(smb, CFG_PIN);
		if (ret < 0)
			goto fail;

		ret |= CFG_PIN_EN_CHARGER_ERROR;

		ret = smb347_write(smb, CFG_PIN, ret);
	} else {
#if 0
		ret = smb347_write(smb, CFG_FAULT_IRQ, 0);
		if (ret < 0)
			goto fail;

		ret = smb347_write(smb, CFG_STATUS_IRQ, 0);
		if (ret < 0)
			goto fail;
#endif
// cancel charger error IRQ
		ret = smb347_read(smb, CFG_PIN);
		if (ret < 0)
			goto fail;

		ret &= ~CFG_PIN_EN_CHARGER_ERROR;

		ret = smb347_write(smb, CFG_PIN, ret);
	}

fail:
#if 0
	smb347_set_writable(smb, false);
#endif
	return ret;
}

static irqreturn_t smb347_inok_interrupt(int irq, void *data)
{
	struct smb347_charger *smb = data;

	irqreturn_t ret = IRQ_NONE;

	pm_runtime_get_sync(&smb->client->dev);

	dev_warn(&smb->client->dev, "%s\n", __func__);
	if (gpio_get_value(smb->pdata->inok_gpio)) {
		dev_warn(&smb->client->dev, "%s: >>> INOK pin (HIGH) <<<\n", __func__);
	}
	else {
		dev_warn(&smb->client->dev, "%s: >>> INOK pin (LOW) <<<\n", __func__);
	}

	pm_runtime_put_sync(&smb->client->dev);
	return ret;
}

static int smb347_inok_gpio_init(struct smb347_charger *smb)
{
	const struct smb347_charger_platform_data *pdata = smb->pdata;
	int ret, irq = gpio_to_irq(pdata->inok_gpio);

	ret = gpio_request_one(pdata->inok_gpio, GPIOF_IN, smb->client->name);
	if (ret < 0) {
		printk(KERN_DEBUG "smb347: request INOK gpio fail!\n");
		goto fail;
	}

	ret = request_threaded_irq(irq, NULL, smb347_inok_interrupt,
					IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					smb->client->name,
					smb);
	if (ret < 0) {
		printk(KERN_DEBUG "smb347: config INOK gpio as IRQ fail!\n");
		goto fail_gpio;
	}

	return 0;
fail_gpio:
	gpio_free(pdata->inok_gpio);
fail:
	smb->client->irq = 0;
	return ret;
}

static inline int smb347_irq_enable(struct smb347_charger *smb)
{
	return smb347_irq_set(smb, true);
}

static inline int smb347_irq_disable(struct smb347_charger *smb)
{
	return smb347_irq_set(smb, false);
}

#if SMB347_IRQ_INIT
static int smb347_irq_init(struct smb347_charger *smb)
{
    int ret;
#if 0
        const struct smb347_charger_platform_data *pdata = smb->pdata;
	int ret, irq = gpio_to_irq(pdata->irq_gpio);

	ret = gpio_request_one(pdata->irq_gpio, GPIOF_IN, smb->client->name);
	if (ret < 0)
		goto fail;

	ret = request_threaded_irq(irq, NULL, smb347_interrupt,
				   IRQF_TRIGGER_FALLING, smb->client->name,
				   smb);
	ret = request_threaded_irq(irq, NULL, smb347_interrupt,
				   IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				   smb->client->name,
				   smb);
	if (ret < 0)
		goto fail_gpio;
#endif

	ret = smb347_set_writable(smb, true);
	if (ret < 0)
		goto fail;

	/*
	 * Configure the STAT output to be suitable for interrupts: disable
	 * all other output (except interrupts) and make it active low.
	 */
	ret = smb347_read(smb, CFG_STAT);
	if (ret < 0)
		goto fail_readonly;

	ret &= ~CFG_STAT_ACTIVE_HIGH;
	ret |= CFG_STAT_DISABLED;

	ret = smb347_write(smb, CFG_STAT, ret);
	if (ret < 0)
		goto fail_readonly;

	ret = smb347_irq_enable(smb);
	if (ret < 0)
		goto fail_readonly;

#if 0
	smb347_set_writable(smb, false);
	smb->client->irq = irq;
	enable_irq_wake(smb->client->irq);
#endif
	return 0;

fail_readonly:
#if 0
	smb347_set_writable(smb, false);
fail_irq:
	free_irq(irq, smb);
fail_gpio:
	gpio_free(pdata->irq_gpio);
#endif
fail:
	smb->client->irq = 0;
	return ret;
}
#endif

#if 0
static int smb347_mains_get_property(struct power_supply *psy,
				     enum power_supply_property prop,
				     union power_supply_propval *val)
{
	struct smb347_charger *smb =
		container_of(psy, struct smb347_charger, mains);

	if (prop == POWER_SUPPLY_PROP_ONLINE) {
		val->intval = smb->mains_online;
		return 0;
	}
	return -EINVAL;
}

static enum power_supply_property smb347_mains_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int smb347_usb_get_property(struct power_supply *psy,
				   enum power_supply_property prop,
				   union power_supply_propval *val)
{
	struct smb347_charger *smb =
		container_of(psy, struct smb347_charger, usb);

	if (prop == POWER_SUPPLY_PROP_ONLINE) {
		val->intval = smb->usb_online;
		return 0;
	}
	return -EINVAL;
}
#endif

bool smb347_has_charger_error(void)
{
	int ret;

	if (!smb347_dev)
		return -EINVAL;

	ret = smb347_read(smb347_dev, STAT_C);
	if (ret < 0)
		return true;

	if (ret & STAT_C_CHARGER_ERROR)
		return true;

	return false;
}

int smb347_get_charging_status(void)
{
	int ret, status;

	if (!smb347_dev)
		return -EINVAL;

#if 0
	if (!smb347_is_online(smb347_dev))
		return POWER_SUPPLY_STATUS_DISCHARGING;
#endif

	ret = smb347_read(smb347_dev, STAT_C);
	if (ret < 0)
		return ret;

#if 0
	dev_info(&smb347_dev->client->dev,
			"Charging Status: STAT_C:0x%x\n", ret);
#endif

	if ((ret & STAT_C_CHARGER_ERROR) ||
		(ret & STAT_C_HOLDOFF_STAT)) {
		/* set to NOT CHARGING upon charger error
		 * or charging has stopped.
		 */
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	} else {
		if ((ret & STAT_C_CHG_MASK) >> STAT_C_CHG_SHIFT) {
			/* set to charging if battery is in pre-charge,
			 * fast charge or taper charging mode.
			 */
			status = POWER_SUPPLY_STATUS_CHARGING;
		} else if (ret & STAT_C_CHG_TERM) {
			/* set the status to FULL if battery is not in pre
			 * charge, fast charge or taper charging mode AND
			 * charging is terminated at least once.
			 */
			status = POWER_SUPPLY_STATUS_FULL;
		} else {
			/* in this case no charger error or termination
			 * occured but charging is not in progress!!!
			 */
			status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
	}

	return status;
}
//EXPORT_SYMBOL_GPL(smb347_get_charging_status);

#if 0
static enum power_supply_property smb347_usb_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int smb347_battery_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct smb347_charger *smb =
			container_of(psy, struct smb347_charger, battery);
	const struct smb347_charger_platform_data *pdata = smb->pdata;
	int ret;

	ret = smb347_update_status(smb);
	if (ret < 0)
		return ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = smb347_get_charging_status();
		if (ret < 0)
			return ret;
		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		if (!smb347_is_online(smb))
			return -ENODATA;

		/*
		 * We handle trickle and pre-charging the same, and taper
		 * and none the same.
		 */
		switch (smb347_charging_status(smb)) {
		case 1:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
			break;
		case 2:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
			break;
		default:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
			break;
		}
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = pdata->battery_info.technology;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = pdata->battery_info.voltage_min_design;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = pdata->battery_info.voltage_max_design;
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = pdata->battery_info.charge_full_design;
		break;

	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = pdata->battery_info.name;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property smb347_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_MODEL_NAME,
};
#endif

static int smb347_debugfs_show(struct seq_file *s, void *data)
{
	struct smb347_charger *smb = s->private;
	int ret;
	u8 reg;

	seq_printf(s, "Control registers:\n");
	seq_printf(s, "==================\n");
	seq_printf(s, "#Addr\t#Value\n");
	for (reg = CFG_CHARGE_CURRENT; reg <= CFG_ADDRESS; reg++) {
		ret = smb347_read(smb, reg);
		seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN "\n", reg, BYTETOBINARY(ret));
	}
	seq_printf(s, "\n");

	seq_printf(s, "Command registers:\n");
	seq_printf(s, "==================\n");
	seq_printf(s, "#Addr\t#Value\n");
	ret = smb347_read(smb, CMD_A);
	seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN "\n", CMD_A, BYTETOBINARY(ret));
	ret = smb347_read(smb, CMD_B);
	seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN "\n", CMD_B, BYTETOBINARY(ret));
	ret = smb347_read(smb, CMD_C);
	seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN "\n", CMD_C, BYTETOBINARY(ret));
	seq_printf(s, "\n");

	seq_printf(s, "Interrupt status registers:\n");
	seq_printf(s, "===========================\n");
	seq_printf(s, "#Addr\t#Value\n");
	for (reg = IRQSTAT_A; reg <= IRQSTAT_F; reg++) {
		ret = smb347_read(smb, reg);
		seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN "\n", reg, BYTETOBINARY(ret));
	}
	seq_printf(s, "\n");

	seq_printf(s, "Status registers:\n");
	seq_printf(s, "=================\n");
	seq_printf(s, "#Addr\t#Value\n");
	for (reg = STAT_A; reg <= STAT_E; reg++) {
		ret = smb347_read(smb, reg);
		seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN "\n", reg, BYTETOBINARY(ret));
	}

	return 0;
}

static int smb347_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, smb347_debugfs_show, inode->i_private);
}

static const struct file_operations smb347_debugfs_fops = {
	.open		= smb347_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


static int cable_status_notify2(struct notifier_block *self, unsigned long action, void *dev)
{
   if (ischargerSuspend) {
       printk(KERN_INFO "chager is suspend but USB still notify !!!\n", __func__);
       wake_lock(&wakelock_cable);
       isUSBSuspendNotify = true;
       return NOTIFY_OK;
   }

   switch (action) {
      case CHRG_SDP:
         printk(KERN_INFO "%s CHRG_SDP !!!\n", __func__);
         setSMB347Charger(USB_IN);
         break;

      case CHRG_CDP:
         printk(KERN_INFO "%s CHRG_CDP !!!\n", __func__);
         setSMB347Charger(AC_IN);
         break;

      case CHRG_DCP:
          printk(KERN_INFO "%s CHRG_DCP !!!\n", __func__);
          setSMB347Charger(AC_IN);
          break;

      case CHRG_ACA:
          printk(KERN_INFO "%s CHRG_ACA !!!\n", __func__);
          setSMB347Charger(AC_IN);
          break;

      case CHRG_SE1:
          printk(KERN_INFO "%s CHRG_SE1 !!!\n", __func__);
          setSMB347Charger(AC_IN);
          break;

      case CHRG_MHL:
          printk(KERN_INFO "%s CHRG_MHL !!!\n", __func__);
          setSMB347Charger(AC_IN);
          break;

      case CHRG_UNKNOWN:
      default:
          printk(KERN_INFO "%s CHRG_UNKNOWN !!!\n", __func__);
          setSMB347Charger(CABLE_OUT);
	  break;
   }
   return NOTIFY_OK;
}

static struct notifier_block cable_status_notifier2 = {
	.notifier_call = cable_status_notify2,
};

extern int cable_status_register_client(struct notifier_block *nb);
extern int cable_status_unregister_client(struct notifier_block *nb);


static int smb347_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	const struct smb347_charger_platform_data *pdata;
	struct device *dev = &client->dev;
	struct smb347_charger *smb;
	int ret;

	printk(KERN_DEBUG "==== smb347_probe ====\n");

	pdata = dev->platform_data;
	if (!pdata)
		return -EINVAL;

	if (!pdata->use_mains && !pdata->use_usb)
		return -EINVAL;

	smb = devm_kzalloc(dev, sizeof(*smb), GFP_KERNEL);
	if (!smb)
		return -ENOMEM;

	i2c_set_clientdata(client, smb);

	mutex_init(&smb->lock);
	smb->client = client;
	smb->pdata = pdata;

#if 0
	/* init wake lock */
	wake_lock_init(&smb->wakelock,
		WAKE_LOCK_SUSPEND, "smb347_wakelock");
#endif

	/* enable register writing - chris */
	ret = smb347_set_writable(smb, true);
	if (ret < 0)
		return ret;
	// Refer to SMB347 Application Note 72 to solve serious problems - chris
	ret = smb347_masked_write(smb->client, OTG_TLIM_THERM_CNTRL_REG,
			OTG_CURRENT_LIMIT_AT_USBIN_MASK, OTG_CURRENT_LIMIT_250mA);
	if (ret < 0)
		return ret;

#if 0
	ret = smb347_hw_init(smb);
	if (ret < 0)
		return ret;

	if (smb->pdata->use_mains) {
		smb->mains.name = "smb347-mains";
		smb->mains.type = POWER_SUPPLY_TYPE_MAINS;
		smb->mains.get_property = smb347_mains_get_property;
		smb->mains.properties = smb347_mains_properties;
		smb->mains.num_properties = ARRAY_SIZE(smb347_mains_properties);
		smb->mains.supplied_to = smb347_power_supplied_to;
		smb->mains.num_supplicants =
				ARRAY_SIZE(smb347_power_supplied_to);
		ret = power_supply_register(dev, &smb->mains);
		if (ret < 0)
			return ret;
	}

	if (smb->pdata->use_usb) {
		smb->usb.name = "smb347-usb";
		smb->usb.type = POWER_SUPPLY_TYPE_USB;
		smb->usb.get_property = smb347_usb_get_property;
		smb->usb.properties = smb347_usb_properties;
		smb->usb.num_properties = ARRAY_SIZE(smb347_usb_properties);
		smb->usb.supplied_to = smb347_power_supplied_to;
		smb->usb.num_supplicants = ARRAY_SIZE(smb347_power_supplied_to);
		ret = power_supply_register(dev, &smb->usb);
		if (ret < 0) {
			if (smb->pdata->use_mains)
				power_supply_unregister(&smb->mains);
			return ret;
		}
	}

	if (smb->pdata->show_battery) {
		smb->battery.name = "smb347-battery";
		smb->battery.type = POWER_SUPPLY_TYPE_BATTERY;
		smb->battery.get_property = smb347_battery_get_property;
		smb->battery.properties = smb347_battery_properties;
		smb->battery.num_properties =
				ARRAY_SIZE(smb347_battery_properties);
		ret = power_supply_register(dev, &smb->battery);
		if (ret < 0) {
			if (smb->pdata->use_usb)
				power_supply_unregister(&smb->usb);
			if (smb->pdata->use_mains)
				power_supply_unregister(&smb->mains);
			return ret;
		}
	}
#endif

	/* Init Runtime PM State */
	pm_runtime_put_noidle(&smb->client->dev);
	pm_schedule_suspend(&smb->client->dev, MSEC_PER_SEC);

//ME371MG only
#if 0
	/*
	 * Interrupt pin is optional. If it is connected, we setup the
	 * interrupt support here.
	 */
	if (pdata->irq_gpio >= 0) {
		ret = smb347_irq_init(smb);
		if (ret < 0) {
			dev_warn(dev, "failed to initialize IRQ: %d\n", ret);
			dev_warn(dev, "disabling IRQ support\n");
		}
	}
#endif

	/* INOK pin configuration */
	if (pdata->inok_gpio >= 0) {
		ret = smb347_inok_gpio_init(smb);
		if (ret < 0) {
			dev_warn(dev, "failed to initialize INOK gpio: %d\n", ret);
		}
	}

#if 0
	INIT_DELAYED_WORK_DEFERRABLE(&smb->smb347_statmon_worker,
						smb347_status_monitor);
#endif

	smb->running = true;
	smb->dentry = debugfs_create_file("smb347-regs", S_IRUSR, NULL, smb,
					  &smb347_debugfs_fops);

#if 0
	/* Start the status monitoring worker */
	schedule_delayed_work(&smb->smb347_statmon_worker, 0);
#endif

	cable_status_register_client(&cable_status_notifier2);
        wake_lock_init(&wakelock_cable, WAKE_LOCK_SUSPEND, "cable_wakelock");

	printk(KERN_DEBUG "==== smb347_probe done ====\n");
	smb347_dev = smb;
  cable_status_notify2( NULL, query_cable_status(), dev);
	return 0;
}

static int smb347_remove(struct i2c_client *client)
{
	struct smb347_charger *smb = i2c_get_clientdata(client);

	cable_status_unregister_client(&cable_status_notifier2);

	if (!IS_ERR_OR_NULL(smb->dentry))
		debugfs_remove(smb->dentry);

	smb->running = false;

#if 0
	if (client->irq) {
		smb347_irq_disable(smb);
		free_irq(client->irq, smb);
		gpio_free(smb->pdata->irq_gpio);
	}

	if (smb->otg) {
		struct smb347_otg_event *evt, *tmp;

		otg_unregister_notifier(smb->otg, &smb->otg_nb);
		smb347_otg_disable(smb);
		otg_put_transceiver(smb->otg);

		/* Clear all the queued events. */
		flush_work_sync(&smb->otg_work);
		list_for_each_entry_safe(evt, tmp, &smb->otg_queue, node) {
			list_del(&evt->node);
			kfree(evt);
		}
	}
#endif

#if 0
	wake_lock_destroy(&smb->wakelock);
#endif

#if 0
	if (smb->pdata->show_battery)
		power_supply_unregister(&smb->battery);
	if (smb->pdata->use_usb)
		power_supply_unregister(&smb->usb);
	if (smb->pdata->use_mains)
		power_supply_unregister(&smb->mains);
#endif

	pm_runtime_get_noresume(&smb->client->dev);

	return 0;
}

void smb347_shutdown(struct i2c_client *client)
{
#if 0
	struct smb347_charger *smb = i2c_get_clientdata(client);
#endif

	dev_info(&client->dev, "%s\n", __func__);

        if (batt_info.cable_status == USB_ADAPTER || batt_info.cable_status == USB_PC) {
             /*ME560CG SOC control JEIAT*/
             smb347_control_JEITA(false);
        }

	/* Disable OTG during shutdown */
	otg(0);
#if 0
	if (smb)
		smb347_otg_drive_vbus(smb, false);
#endif
//ME371MG only
#if 0
	if (HW_ID_EVB == Read_HW_ID() || HW_ID_SR1 == Read_HW_ID() || HW_ID_SR1_with_final_LCD == Read_HW_ID())
	{
		/* Restore default charging behavior */
		smb347_charging_toggle(true);
	}
#endif

	return;
}

#ifdef CONFIG_PM
static int smb347_prepare(struct device *dev)
{
	struct smb347_charger *smb = dev_get_drvdata(dev);

#if 0
	if (smb347_is_online(smb))
		return -EBUSY;
#endif

	/*
	 * disable irq here doesn't mean smb347 interrupt
	 * can't wake up system. smb347 interrupt is triggered
	 * by GPIO pin, which is always active.
	 * When resume callback calls enable_irq, kernel
	 * would deliver the buffered interrupt (if it has) to
	 * driver.
	 */
	if (smb->client->irq > 0)
		disable_irq(smb->client->irq);

#if 0
	cancel_delayed_work_sync(&smb->smb347_statmon_worker);
#endif

	dev_info(&smb->client->dev, "smb347 suspend\n");

	return 0;
}

static void smb347_complete(struct device *dev)
{
	struct smb347_charger *smb = dev_get_drvdata(dev);

	if (smb->client->irq > 0)
		enable_irq(smb->client->irq);

	/* check if the wakeup is due to charger connect */
	if (smb347_update_status(smb) > 0) {
#if 0
		smb347_update_online(smb);
#endif
		if (smb->mains_online || smb->usb_online)
			dev_info(&smb->client->dev,
				"wakeup due to charger connect\n");
		else	/* most unlkely to happen */
			dev_info(&smb->client->dev,
				"wake up due to charger disconnect\n");
	}

#if 0
	/* Start the status monitoring worker */
	schedule_delayed_work(&smb->smb347_statmon_worker,
					msecs_to_jiffies(500));
#endif

	dev_info(&smb->client->dev, "smb347 resume\n");

}
#else
#define smb347_prepare NULL
#define smb347_complete NULL
#endif

#ifdef CONFIG_PM_RUNTIME
static int smb347_runtime_suspend(struct device *dev)
{
	dev_info(dev, "%s called\n", __func__);
	return 0;
}

static int smb347_runtime_resume(struct device *dev)
{
	dev_info(dev, "%s called\n", __func__);
	return 0;
}

static int smb347_runtime_idle(struct device *dev)
{

	dev_info(dev, "%s called\n", __func__);
	return 0;
}
#else
#define smb347_runtime_suspend	NULL
#define smb347_runtime_resume	NULL
#define smb347_runtime_idle	NULL
#endif

static int smb347_suspend(struct device *dev)
{
    printk("%s called\n", __func__);
    if (batt_info.cable_status == USB_ADAPTER || batt_info.cable_status == USB_PC) {
        /*ME560CG SOC control JEIAT*/
        smb347_control_JEITA(false);
    }
    ischargerSuspend = true;
    return 0;
}

static int smb347_resume(struct device *dev)
{
    printk("%s called\n", __func__);
    if (batt_info.cable_status == USB_ADAPTER || batt_info.cable_status == USB_PC) {
        /*ME560CG SOC control JEIAT*/
        smb347_control_JEITA(true);
    }
    ischargerSuspend = false;
    if (isUSBSuspendNotify) {
       isUSBSuspendNotify = false;
       cable_status_notify2( NULL, query_cable_status(), dev);
       wake_unlock(&wakelock_cable);
    }
    return 0;
}


static const struct i2c_device_id smb347_id[] = {
	{ SMB345_DEV_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, smb347_id);

static const struct dev_pm_ops smb347_pm_ops = {
	.prepare		= smb347_prepare,
	.complete		= smb347_complete,
	.suspend                = smb347_suspend,
	.resume	                = smb347_resume,
	.runtime_suspend	= smb347_runtime_suspend,
	.runtime_resume		= smb347_runtime_resume,
	.runtime_idle		= smb347_runtime_idle,
};

static struct i2c_driver smb347_driver = {
	.driver = {
		.name	= SMB345_DEV_NAME,
		.owner	= THIS_MODULE,
		.pm	= &smb347_pm_ops,
	},
	.probe		= smb347_probe,
	.remove		= smb347_remove,
	.shutdown	= smb347_shutdown,
	.id_table	= smb347_id,
};

static int __init smb347_init(void)
{
	printk(KERN_DEBUG "++++ Inside smb347_init ++++\n");
	return i2c_add_driver(&smb347_driver);
}
module_init(smb347_init);

static void __exit smb347_exit(void)
{
	i2c_del_driver(&smb347_driver);
}
module_exit(smb347_exit);

MODULE_AUTHOR("Bruce E. Robertson <bruce.e.robertson@intel.com>");
MODULE_AUTHOR("Mika Westerberg <mika.westerberg@linux.intel.com>");
MODULE_AUTHOR("Chris Chang <chris1_chang@asus.com>");
MODULE_DESCRIPTION("SMB347 battery charger driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("i2c:smb347");
