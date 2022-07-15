#
#  Copyright (c) 2013, ASUSTek, Inc. All Rights Reserved.
#

ifeq ($(TARGET_BUILD_VARIANT),eng)
    KBUILD_CFLAGS += -DFE170CG_ENG_BUILD
else ifeq ($(TARGET_BUILD_VARIANT),user)
    KBUILD_CFLAGS += -DFE170CG_USER_BUILD
else ifeq ($(TARGET_BUILD_VARIANT),userdebug)
    KBUILD_CFLAGS += -DFE170CG_USERDEBUG_BUILD
else
    KBUILD_CFLAGS += -DFE170CG_OTHER_BUILD
endif

ifeq ($(ASUS_PROJECT),FE170CG)
    KBUILD_CFLAGS += -DASUS_PROJECT_FE170CG_TOUCH
endif

obj-$(CONFIG_UPI_BATTERY)				+= ug31xx/
obj-$(CONFIG_FE170CG_BATTERY_SMB345)	                += battery_smb345.o
obj-$(CONFIG_ASUS_ENGINEER_MODE)                        += asus_battery_proc_fs_asustag.o \
                                                           asus_battery_power.o
obj-$(CONFIG_ASUS_ENGINEER_MODE)                        += asus_battery_proc_fs.o

obj-$(CONFIG_FE170CG_BATTERY_BQ27520) += bq27520_battery_core.o    \
                                         bq27520_factory_data.o    \
                                         bq27520_proc_fs.o         \
                                         bq27520_proc_fs_asustag.o \
                                         bq27520_battery_upt_i2c.o

obj-$(CONFIG_EC_POWER)               += asus_ec_power.o util.o
obj-$(CONFIG_LEDS_INTEL_KPD)         += led.o
obj-$(CONFIG_FE170CG_PMIC_SNB5072C1) += snb5072c1_pmic.o
obj-$(CONFIG_SN280X_CHARGER)         += sn280x.o
obj-$(CONFIG_FE170CG)                += battery_bq24156.o
obj-$(CONFIG_UPI_BATTERY)            += uPILogger.o
