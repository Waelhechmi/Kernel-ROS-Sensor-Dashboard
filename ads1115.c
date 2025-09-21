/* -------------------- ads1115.c -------------------- */

/*
 Virtual ADS1115 driver: simulates a 16-bit ADC conversion register and config
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/moduleparam.h>

#define ADS_DEVICE_NAME    "ads1115"
#define ADS_I2C_ADDRESS    0x49

static int ads_value = 0; /* 16-bit signed simulated ADC */
module_param(ads_value, int, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(ads_value, "Initial ADC raw value (-32768..32767)");

struct ads1115_registers {
    int16_t conversion; /* 16-bit ADC result */
    uint16_t config;
};

static struct timer_list ads_timer;
static int ads_dir = 1;
static dev_t ads_dev = 0;
static struct class *ads_class;
static struct i2c_client *ads_virtual_client;
static struct cdev ads_cdev;
static struct mutex ads_mutex;
static struct ads1115_registers *ads_regs;

static int ads_open(struct inode *inode, struct file *file)
{
    if (!mutex_trylock(&ads_mutex))
        return -EBUSY;
    pr_info("%s: opened\n", ADS_DEVICE_NAME);
    return 0;
}
static int ads_release(struct inode *inode, struct file *file)
{
    mutex_unlock(&ads_mutex);
    pr_info("%s: closed\n", ADS_DEVICE_NAME);
    return 0;
}

static ssize_t ads_read(struct file *filp, char __user *buf, size_t len, loff_t *off)
{
    if (len > sizeof(ads_regs->conversion))
        len = sizeof(ads_regs->conversion);
    if (copy_to_user(buf, &ads_regs->conversion, len))
        return -EFAULT;
    pr_debug("%s: read conversion 0x%04x (%d)\n", ADS_DEVICE_NAME, (uint16_t)ads_regs->conversion, ads_regs->conversion);
    return len;
}

static ssize_t ads_write(struct file *filp, const char __user *buf, size_t len, loff_t *off)
{
    if (len > sizeof(ads_regs->config))
        len = sizeof(ads_regs->config);
    if (copy_from_user(&ads_regs->config, buf, len))
        return -EFAULT;
    pr_info("%s: config set 0x%04x\n", ADS_DEVICE_NAME, ads_regs->config);
    return len;
}

static const struct file_operations ads_fops = {
    .owner = THIS_MODULE,
    .open = ads_open,
    .release = ads_release,
    .read = ads_read,
    .write = ads_write,
};

static void ads_timer_callback(struct timer_list *t)
{
    if (ads_dir == 1) {
        ads_value += 512;
        if (ads_value > 30000)
            ads_dir = -1;
    } else {
        ads_value -= 512;
        if (ads_value < -30000)
            ads_dir = 1;
    }
    ads_regs->conversion = (int16_t)ads_value;
    mod_timer(&ads_timer, jiffies + msecs_to_jiffies(200));
}

static int __init ads_init(void)
{
    struct i2c_adapter *adapter;
    struct i2c_board_info info = { I2C_BOARD_INFO("ads1115", ADS_I2C_ADDRESS) };

    adapter = i2c_get_adapter(1);
    if (!adapter) {
        pr_err("%s: failed to get i2c adapter 1\n", ADS_DEVICE_NAME);
        return -ENODEV;
    }
    ads_virtual_client = i2c_new_client_device(adapter, &info);
    i2c_put_adapter(adapter);
    if (!ads_virtual_client) {
        pr_err("%s: failed to create virtual i2c client\n", ADS_DEVICE_NAME);
        return -ENODEV;
    }

    if (alloc_chrdev_region(&ads_dev, 0, 1, ADS_DEVICE_NAME) < 0) {
        pr_err("%s: alloc_chrdev_region failed\n", ADS_DEVICE_NAME);
        goto err_unregister_i2c;
    }

    cdev_init(&ads_cdev, &ads_fops);
    if (cdev_add(&ads_cdev, ads_dev, 1) < 0) {
        pr_err("%s: cdev_add failed\n", ADS_DEVICE_NAME);
        goto err_unregister_chr;
    }

    ads_class = class_create( ADS_DEVICE_NAME);
    if (IS_ERR(ads_class)) {
        pr_err("%s: class_create failed\n", ADS_DEVICE_NAME);
        goto err_cdev_del;
    }

    if (IS_ERR(device_create(ads_class, NULL, ads_dev, NULL, ADS_DEVICE_NAME))) {
        pr_err("%s: device_create failed: %ld\n", ADS_DEVICE_NAME, PTR_ERR(device_create(ads_class, NULL, ads_dev, NULL, ADS_DEVICE_NAME)));
        goto err_class;
    }

    ads_regs = kzalloc(sizeof(*ads_regs), GFP_KERNEL);
    if (!ads_regs) {
        pr_err("%s: kzalloc failed\n", ADS_DEVICE_NAME);
        goto err_device;
    }
    ads_regs->conversion = (int16_t)ads_value;
    mutex_init(&ads_mutex);
    timer_setup(&ads_timer, ads_timer_callback, 0);
    mod_timer(&ads_timer, jiffies + msecs_to_jiffies(200));

    pr_info("%s: inserted\n", ADS_DEVICE_NAME);
    return 0;

err_device:
    device_destroy(ads_class, ads_dev);
err_class:
    class_destroy(ads_class);
err_cdev_del:
    cdev_del(&ads_cdev);
err_unregister_chr:
    unregister_chrdev_region(ads_dev, 1);
err_unregister_i2c:
    i2c_unregister_device(ads_virtual_client);
    return -ENODEV;
}

static void __exit ads_exit(void)
{
    del_timer_sync(&ads_timer);
    mutex_destroy(&ads_mutex);
    kfree(ads_regs);
    device_destroy(ads_class, ads_dev);
    class_destroy(ads_class);
    cdev_del(&ads_cdev);
    unregister_chrdev_region(ads_dev, 1);
    i2c_unregister_device(ads_virtual_client);
    pr_info("%s: removed\n", ADS_DEVICE_NAME);
}

module_init(ads_init);
module_exit(ads_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Hechmi Wael");
MODULE_DESCRIPTION("Virtual ADS1115 ADC driver");
MODULE_VERSION("1.0");
