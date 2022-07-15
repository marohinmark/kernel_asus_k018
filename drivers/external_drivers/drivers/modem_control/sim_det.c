#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/mdm_ctrl_board.h>
#include <linux/hsi/hsi.h>
#include "mdm_util.h"
#include "sim_det.h"

#define SIM_DELAY HZ/10
#define NAME_SIM_PLUG "ril_sim_plug"

#define SIM1_PRESENT (1 << 0)
#define SIM2_PRESENT (1 << 1)

static struct workqueue_struct *sim_det_wq;
static struct delayed_work sim_det_work;
static struct delayed_work cdump_work;
static struct switch_dev sim_switch;

static int sim_plug_state;
static bool is_switch;
static struct mdm_ctrl *drv_data;
static struct mdm_ctrl_cpu_data *mdm_cpu;

/**
 * mdm_ctrl_configure_gpio - Configure GPIOs
 * @gpio: GPIO to configure
 * @direction: GPIO direction - 0: IN | 1: OUT
 *
 */
static inline int mdm_ctrl_configure_gpio(int gpio,
					  int direction,
					  int value, const char *desc)
{
	int ret;

	ret = gpio_request(gpio, "ModemControl");

	if (direction)
		ret += gpio_direction_output(gpio, value);
	else
		ret += gpio_direction_input(gpio);

	if (ret) {
		pr_err(DRVNAME ": Unable to configure GPIO%d (%s)", gpio, desc);
		ret = -ENODEV;
	}

	return ret;
}

/* callbacks for switch device */
static ssize_t print_sim_plug_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", NAME_SIM_PLUG);
}

static ssize_t print_sim_plug_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%d\n", sim_plug_state);
}

void sim_detect_work(struct work_struct *work)
{
	int value = 0;

	sim_plug_state = 0;

	if (mdm_cpu->gpio_sim1_cd > 0) {
		value = gpio_get_value(mdm_cpu->gpio_sim1_cd);
		pr_info(DRVNAME": %s GPIO_SIM1_CD = 0x%x\n", __func__, value);
		if (value)
			sim_plug_state |= SIM1_PRESENT;
	}

	if (mdm_cpu->gpio_sim2_cd > 0) {
		value = gpio_get_value(mdm_cpu->gpio_sim2_cd);
		pr_info(DRVNAME": %s GPIO_SIM2_CD = 0x%x\n", __func__, value);
		if (value)
			sim_plug_state |= SIM2_PRESENT;
	}

	if (is_switch)
		switch_set_state(&sim_switch, sim_plug_state);
}

static irqreturn_t sim1_cd_isr(int irq, void *data)
{
	queue_delayed_work(sim_det_wq, &sim_det_work, SIM_DELAY);
	return IRQ_HANDLED;
}

static irqreturn_t sim2_cd_isr(int irq, void *data)
{
	queue_delayed_work(sim_det_wq, &sim_det_work, SIM_DELAY);
	return IRQ_HANDLED;
}

static ssize_t gpio_dump_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char temp[80] = {0};
	ssize_t size = 0;

	if (mdm_cpu->gpio_sim1_cd > 0) {
		size += sprintf(temp, "sim1_cd 0x%x ",
					gpio_get_value(mdm_cpu->gpio_sim1_cd));
	}

	if (mdm_cpu->gpio_sim2_cd > 0) {
		size += sprintf(temp + size, "sim2_cd 0x%x ",
					gpio_get_value(mdm_cpu->gpio_sim2_cd));
	}

	if (mdm_cpu->gpio_req_cdump > 0) {
		size += sprintf(temp + size, "req_cdump 0x%x ",
					gpio_get_value(mdm_cpu->gpio_req_cdump));
	}

	size--;
	size += sprintf(temp + size, "\n");

	size = sprintf(buf, temp);
	pr_info(DRVNAME": %s\n", buf);

	return size;
}
static DEVICE_ATTR(gpio_dump, 0664, gpio_dump_show, NULL);

void trigger_cdump_work(struct work_struct *work)
{
	pr_info(DRVNAME": %s\n", __func__);
	gpio_set_value(mdm_cpu->gpio_req_cdump, 1);
	mdelay(1);
	gpio_set_value(mdm_cpu->gpio_req_cdump, 0);
}

static ssize_t req_cdump_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t status;
	long value;

	status = strict_strtol(buf, 0, &value);
	pr_info(DRVNAME": %s value = %d\n", __func__, (int)value);

	if (value)
		queue_delayed_work(sim_det_wq, &cdump_work, 0);

	return size;
}
static DEVICE_ATTR(req_cdump, 0664, NULL, req_cdump_store);

static int request_cdump_notify(struct notifier_block *self,
			       unsigned long action, void *dev)
{
	queue_delayed_work(sim_det_wq, &cdump_work, 0);

	return NOTIFY_OK;
}

static struct notifier_block request_cdump_nb = {
	.notifier_call = request_cdump_notify,
};

int sim_detect_init(void *drv, void *data)
{
	int ret = 0;

	drv_data = drv;
	mdm_cpu = data;

	sim_det_wq = create_singlethread_workqueue("sim_det_wq");
	INIT_DELAYED_WORK(&sim_det_work, sim_detect_work);
	INIT_DELAYED_WORK(&cdump_work, trigger_cdump_work);

	/* register switch class */
	sim_switch.name = NAME_SIM_PLUG;
	sim_switch.print_name = print_sim_plug_name;
	sim_switch.print_state = print_sim_plug_state;
	ret = switch_dev_register(&sim_switch);
	if (ret < 0) {
		pr_info (DRVNAME": Could not register switch device, ret = %d\n", ret);
		is_switch = 0;
	} else
		is_switch = 1;

	/* Configure the SIM1_AP_CD gpio */
	if (mdm_cpu->gpio_sim1_cd > 0) {
		ret = mdm_ctrl_configure_gpio(mdm_cpu->gpio_sim1_cd,
				0, 0, "SIM1_AP_CD");
		if (ret) {
			pr_err(DRVNAME": config failed for GPIO%d (SIM1_AP_CD)",
					mdm_cpu->gpio_sim1_cd);
			gpio_free(mdm_cpu->gpio_sim1_cd);
			mdm_cpu->gpio_sim1_cd = 0;
		} else {
			mdm_cpu->irq_sim1_cd = gpio_to_irq(mdm_cpu->gpio_sim1_cd);
			ret = request_irq(mdm_cpu->irq_sim1_cd,
					sim1_cd_isr,
					IRQF_TRIGGER_RISING |
					IRQF_TRIGGER_FALLING |
					IRQF_NO_SUSPEND,
					"sim1_cd",
					mdm_cpu);
			if (ret) {
				pr_err(DRVNAME": IRQ request failed for GPIO%d (SIM1_AP_CD)",
					mdm_cpu->gpio_sim1_cd);
				mdm_cpu->irq_sim1_cd = -1;
			}
		}
	}

	/* Configure the SIM2_AP_CD gpio */
	if (mdm_cpu->gpio_sim2_cd > 0) {
		ret = mdm_ctrl_configure_gpio(mdm_cpu->gpio_sim2_cd,
				0, 0, "SIM2_AP_CD");

		if (ret) {
			pr_err(DRVNAME": config failed for GPIO%d (SIM2_AP_CD)",
					mdm_cpu->gpio_sim2_cd);
			gpio_free(mdm_cpu->gpio_sim2_cd);
			mdm_cpu->gpio_sim2_cd = 0;
		} else {
			mdm_cpu->irq_sim2_cd = gpio_to_irq(mdm_cpu->gpio_sim2_cd);
			ret = request_irq(mdm_cpu->irq_sim2_cd,
					sim2_cd_isr,
					IRQF_TRIGGER_RISING |
					IRQF_TRIGGER_FALLING |
					IRQF_NO_SUSPEND,
					"sim2_cd",
					mdm_cpu);
			if (ret) {
				pr_err(DRVNAME": IRQ request failed for GPIO%d (SIM2_AP_CD)",
					mdm_cpu->gpio_sim2_cd);
				mdm_cpu->irq_sim2_cd = -1;
			}
		}
	}

	if (mdm_cpu->gpio_sim1_cd > 0 || mdm_cpu->gpio_sim2_cd > 0)
		queue_delayed_work(sim_det_wq, &sim_det_work, 0);

	/* Configure the AP_REQ_CDUMP gpio */
	if (mdm_cpu->gpio_req_cdump > 0) {
		ret = mdm_ctrl_configure_gpio(mdm_cpu->gpio_req_cdump,
				1, 0, "AP_REQ_CDUMP");
		if (ret) {
			pr_err(DRVNAME": config failed for GPIO%d (AP_REQ_CDUMP)",
					mdm_cpu->gpio_req_cdump);
			gpio_free(mdm_cpu->gpio_req_cdump);
			mdm_cpu->gpio_req_cdump = 0;
		} else {
			ret = device_create_file(drv_data->dev, &dev_attr_req_cdump);
			if (ret)
				pr_info(DRVNAME": fail to create file (req_cdump)\n");
		}
	}

	ret = device_create_file(drv_data->dev, &dev_attr_gpio_dump);
	if (ret)
		pr_info(DRVNAME": fail to create file (gpio_dump)\n");

	hsi_register_notify(&request_cdump_nb);

	return 0;
}

int sim_detect_exit(void)
{
	pr_info(DRVNAME": %s unregister resources\n", __func__);

	hsi_unregister_notify(&request_cdump_nb);

	if (mdm_cpu->irq_sim1_cd > 0)
		free_irq(mdm_cpu->irq_sim1_cd, NULL);
	if (mdm_cpu->irq_sim2_cd > 0)
		free_irq(mdm_cpu->irq_sim2_cd, NULL);

	device_remove_file(drv_data->dev, &dev_attr_gpio_dump);
	device_remove_file(drv_data->dev, &dev_attr_req_cdump);

	cancel_delayed_work_sync(&sim_det_work);
	cancel_delayed_work_sync(&cdump_work);
	destroy_workqueue(sim_det_wq);

	if (is_switch) {
		switch_dev_unregister(&sim_switch);
		is_switch = 0;
	}

	if (mdm_cpu->gpio_sim1_cd > 0)
		gpio_free(mdm_cpu->gpio_sim1_cd);
	if (mdm_cpu->gpio_sim2_cd > 0)
		gpio_free(mdm_cpu->gpio_sim2_cd);
	if (mdm_cpu->gpio_req_cdump > 0)
		gpio_free(mdm_cpu->gpio_req_cdump);

	return 0;
}

