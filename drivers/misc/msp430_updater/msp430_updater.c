//Yes I know this is horrible
#include "Devices430.c"
#include "LowLevelFunc430.c"
#include "JTAGfunc430.c"
#include "Replicator430.c"


#include "msp430_updater.h"
#include "Replicator430.h"

#define sdump_buffer(buffer, length, name) mod_sdump_buffer( "msp430_updater", __FUNCTION__, __LINE__, (buffer), (length), (name))
void mod_sdump_buffer( const char * module_name, const char * function, int line, const uint8_t* buf, uint32_t length, const char* name)
{
    int i, j, i_valid;
    u32 remainder = (length % 16);
    u32 top_len = length + (remainder ? 16 - remainder : 0);
    u32 char_index;
    u8 c;

    if (name) {
        printk("%s:%i DUMP: \"%s\" (%p) length %d\n", function, line, name, buf, length);
    } else {
        printk("%s:%i DUMP: (%p) length %d\n", function, line, buf, length);
    }

    printk("%08d  ", 0);

    for(i=0; i<top_len; i++) {

        i_valid = (i < length);

        if (i_valid) {
            printk("%02x ", buf[i]);
        } else {
            printk("   ");
        }

        if(i % 16 == 15 && i != 0) {
            printk("  |");
            j = 0;
            for(j=0; j<16; j++) {
                char_index = i - 15 + j;
                if (char_index < length) {
                    c = buf[char_index];
                    printk("%c", (c >= '0' && c <= 'z') ? c : '.');
                } else {
                    printk("%c", ' ');
                }
            }
            if (i == top_len - 1) {
                printk("|");
            } else {
                printk("|\n%08d  ", i+1);
            }

        } else if (i % 8 == 7) {
            printk(" ");
        }


    }
    printk( "\n");
}

#define MAJOR_NUM 100
#define DEVICE_NAME "msp430_updater"

#define SIOCTL(n) (100+n)

/* user space ioctl's */
#define MSP430_UPDATER_IOCTL_START_UPDATE           SIOCTL(0)
#define MSP430_UPDATER_IOCTL_GET_UPADTE_STATUS      SIOCTL(1)

msp430_updater_t* mu = NULL;

static int device_open = 0;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
void msp430_updater_update_worker(void* work)
#else
void msp430_updater_update_worker(struct work_struct* work)
#endif
{
    msp430_updater_t* dev = mu;
    do_msp430_update(&dev->update_data);
}

static long msp430_updater_ioctl(struct file *filp, uint cmd, ulong arg)
{
    msp430_updater_t* dev = mu;
    uint32_t status = 0;
    int ret = 0;

    switch(cmd) {
        case MSP430_UPDATER_IOCTL_START_UPDATE:
            printk("msp430_updater start update\n");

            //get the update length
            if( copy_from_user( &dev->update_data.length,
                        &((msp430_update_data_t*)arg)->length,
                        sizeof(uint32_t) ) ) {

                ret = -EINVAL;
                goto end;
            }

            //make sure its not bigger than we support
            if(dev->update_data.length*2 > 8192) {
                ret = -EINVAL;
                goto end;
            }

            //copy the actual update data
            if( copy_from_user( dev->update_data.data,
                        ((msp430_update_data_t*)arg)->data,
                        dev->update_data.length*2 ) ) { //length is in 16 bit words

                ret = -EINVAL;
                goto end;
            }

            printk("Doing update of length %d words\n", dev->update_data.length);
            //sdump_buffer(dev->update_data.data, dev->update_data.length*2, "update_data");

            //start the update worker
            schedule_work(&dev->update_worker);
            break;
        case MSP430_UPDATER_IOCTL_GET_UPADTE_STATUS:
            status = GetUpdateStatus();
            printk("msp430_updater get status: %d\n", status);
            break;
        default:
            ret = -EINVAL;
            printk("Unknown ioctl on msp430_updater device (%d)\n", cmd);
            break;
    }

end:
    return ret;
}

static int msp430_updater_open(struct inode * inode, struct file * file)
{
    if(device_open) {
        return -EBUSY;
    }

    device_open++;

    return 0;
}

static int msp430_updater_release(struct inode * inode, struct file * file)
{
    device_open--;
    return 0;
}

static struct file_operations msp430_updater_ctl_fops = {
    .owner          = THIS_MODULE,
    .unlocked_ioctl = msp430_updater_ioctl,
    .compat_ioctl   = msp430_updater_ioctl,
    .open           = msp430_updater_open,
    .release        = msp430_updater_release,
};

static struct class* msp430_updater_class;

int msp430_updater_register_ctrl_device( msp430_updater_t* dev )
{
    cdev_init( &dev->ctrl_cdev, &msp430_updater_ctl_fops );
    dev->ctrl_cdev.owner = THIS_MODULE;
    cdev_add( &dev->ctrl_cdev, MKDEV(MAJOR_NUM, 0), 1 );

    dev->class = msp430_updater_class;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
    device_create( dev->class, NULL, MKDEV(MAJOR_NUM, 0), "msp430_updater_ctl" );
#else
    device_create( dev->class, NULL, MKDEV(MAJOR_NUM, 0), dev, "msp430_updater_ctl" );
#endif
    return 0;
}

void msp430_updater_unregister_ctrl_device( msp430_updater_t* dev )
{
    device_destroy( dev->class, MKDEV(MAJOR_NUM, 0) );
    cdev_del( &dev->ctrl_cdev );
}

static int __init msp430_updater_probe(struct platform_device *pdev)
{
    int ret = 0;

    printk("msp430_updater probe\n");

	mu = kzalloc(sizeof(msp430_updater_t), GFP_KERNEL);
	if(!mu) {
		return -ENOMEM;
    }

	mu->dev = &pdev->dev;

    ret = msp430_updater_register_ctrl_device(mu);
    if(ret != 0) {
        printk("ERROR: Failed to register ctl device");
        kfree(mu);
        mu = NULL;
        return ret;
    }

	platform_set_drvdata(pdev, mu);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
    INIT_WORK( &mu->update_worker, msp430_updater_update_worker, &mu->update_worker);
#else
    INIT_WORK( &mu->update_worker, msp430_updater_update_worker );
#endif

	dev_info(&pdev->dev, "initialized msp430_updater\n");

	return ret;
}

static int __exit msp430_updater_remove(struct platform_device *pdev)
{
    printk("msp430_updater remove\n");

    platform_set_drvdata(pdev, NULL);

    msp430_updater_unregister_ctrl_device(mu);

    if(mu) {
		kfree(mu);
        mu = NULL;
    }

	dev_info(&pdev->dev, "removed msp430_updater\n");

	return 0;
}

static struct platform_driver msp430_updater_driver = {
    .probe = msp430_updater_probe,
	.driver = {
		.name = "msp430_updater",
		.owner = THIS_MODULE,
	},
	.remove = __exit_p(msp430_updater_remove),
};

static int __init msp430_updater_init(void)
{
    int ret = 0;
    printk("msp430_updater init\n");

    ret = register_chrdev_region( MKDEV( MAJOR_NUM, 0 ), 255 , DEVICE_NAME );

    if(ret != 0) {
        printk("Failed to register control device");
        return ret;
    }

    msp430_updater_class = class_create( THIS_MODULE, DEVICE_NAME );

    if( IS_ERR( msp430_updater_class ) ) {
        printk("failed to create class\n");
        unregister_chrdev_region( MKDEV( MAJOR_NUM, 0 ), 255 );
        return PTR_ERR( msp430_updater_class );
    }

	return platform_driver_probe(&msp430_updater_driver, msp430_updater_probe);
}

static void __exit msp430_updater_exit(void)
{
    printk("msp430_updater exit\n");
	platform_driver_unregister(&msp430_updater_driver);

    class_destroy(msp430_updater_class);
    unregister_chrdev_region( MKDEV( MAJOR_NUM, 0 ), 255 );
}

module_init(msp430_updater_init);
module_exit(msp430_updater_exit);

MODULE_AUTHOR("Jeremy Hammer <jeremyh@cetoncorp.com>");
MODULE_DESCRIPTION("Ceton MSP430 Updater");
MODULE_LICENSE("GPL v2");
