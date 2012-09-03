#ifndef MSP430_UPDATER_H
#define MSP430_UPDATER_H

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/version.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/vmalloc.h>

typedef struct {
    uint32_t length;
    uint16_t data[8192/2];
} msp430_update_data_t;

typedef struct {
	struct device *dev;
    struct work_struct update_worker;
    struct class* class;
    struct cdev ctrl_cdev;
    msp430_update_data_t update_data;
} msp430_updater_t;

#endif //MSP430_UPDATER_H
