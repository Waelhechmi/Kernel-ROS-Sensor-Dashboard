/* -------------------- tmp102.c -------------------- */

/*
 Virtual TMP102 driver
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

#define TMP_DEVICE_NAME    "tmp102"
#define TMP_I2C_ADDRESS    0x48
#define TMP_MEM_SIZE       4

#define TMP102_TEMP_REGISTER   0x00
#define TMP102_CONFIG_REGISTER 0x01
#define CONFIG_12 0x60A0
#define CONFIG_13 0x64A0
#define MAX_TEMP 0x7FF
#define MIN_TEMP 0x000
#define STEP 0xC

static short temp = 0x500; /* default initial tmp raw value */
module_param(temp, short, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(temp, "Initial raw temperature value (12-bit)");

struct tmp102_registers {
    int16_t temperature;
    uint16_t configuration;
};

static struct timer_list tmp_timer;
static int tmp_direction = 1;
static dev_t tmp_dev = 0;
static struct class *tmp_class;
static struct i2c_client *tmp_virtual_client;
static struct cdev tmp_cdev;
static struct mutex tmp_mutex;
static struct tmp102_registers *tmp_regs;

static int tmp_open(struct inode *inode, struct file *file)
{
    if (!mutex_trylock(&tmp_mutex))
        return -EBUSY;
    pr_info("%s: opened\n", TMP_DEVICE_NAME);
    return 0;
}

static int tmp_release(struct inode *inode, struct file *file)
{
    mutex_unlock(&tmp_mutex);
    pr_info("%s: closed\n", TMP_DEVICE_NAME);
    return 0;
}

static ssize_t tmp_read(struct file *filp, char __user *buf, size_t len, loff_t *off)
{
    if (len > sizeof(tmp_regs->temperature))
        len = sizeof(tmp_regs->temperature);
    if (copy_to_user(buf, &tmp_regs->temperature, len))
        return -EFAULT;
    pr_debug("%s: read temp 0x%04x (%d)\n", TMP_DEVICE_NAME, tmp_regs->temperature, tmp_regs->temperature);
    return len;
}

static ssize_t tmp_write(struct file *filp, const char __user *buf, size_t len, loff_t *off)
{
    if (len > sizeof(tmp_regs->configuration))
        len = sizeof(tmp_regs->configuration);
    if (copy_from_user(&tmp_regs->configuration, buf, len))
        return -EFAULT;
    pr_info("%s: configuration written: 0x%04x\n", TMP_DEVICE_NAME, tmp_regs->configuration);
    return len;
}

static const struct file_operations tmp_fops = {
    .owner = THIS_MODULE,
    .open = tmp_open,
    .release = tmp_release,
    .read = tmp_read,
    .write = tmp_write,
};

static void tmp_timer_callback(struct timer_list *t)
{
    if (tmp_direction == 1) {
        if (temp <= MAX_TEMP)
            temp += STEP;
        else
            tmp_direction = -1;
    } else {
        if (temp > MIN_TEMP)
            temp -= STEP;
        else
            tmp_direction = 1;
    }
    tmp_regs->temperature = temp & 0x0FFF; /* 12 bits */
    mod_timer(&tmp_timer, jiffies + msecs_to_jiffies(300));
}

static int __init tmp102_init(void)
{
    struct i2c_adapter *adapter;
    struct i2c_board_info info = { I2C_BOARD_INFO("i2c_virtual_tmp102", TMP_I2C_ADDRESS) };

    /* create virtual i2c client */
    adapter = i2c_get_adapter(1);
    if (!adapter) {
        pr_err("%s: failed to get I2C adapter 1\n", TMP_DEVICE_NAME);
        return -ENODEV;
    }
    tmp_virtual_client = i2c_new_client_device(adapter, &info);
    i2c_put_adapter(adapter);
    if (!tmp_virtual_client) {
        pr_err("%s: failed to create virtual i2c client\n", TMP_DEVICE_NAME);
        return -ENODEV;
    }

    /* allocate major/minor */
    if (alloc_chrdev_region(&tmp_dev, 0, 1, TMP_DEVICE_NAME) < 0) {
        pr_err("%s: alloc_chrdev_region failed\n", TMP_DEVICE_NAME);
        goto err_unregister_i2c;
    }

    /* initialize cdev */
    cdev_init(&tmp_cdev, &tmp_fops);
    if (cdev_add(&tmp_cdev, tmp_dev, 1) < 0) {
        pr_err("%s: cdev_add failed\n", TMP_DEVICE_NAME);
        goto err_unregister_chr;
    }

    /* create class and device for /dev/tmp102 */
    tmp_class = class_create(TMP_DEVICE_NAME);
    if (IS_ERR(tmp_class)) {
        pr_err("%s: class_create failed\n", TMP_DEVICE_NAME);
        goto err_cdev_del;
    }

    if (IS_ERR(device_create(tmp_class, NULL, tmp_dev, NULL, TMP_DEVICE_NAME))) {
        pr_err("%s: device_create failed\n", TMP_DEVICE_NAME);
        goto err_class;
    }

    /* allocate registers memory */
    tmp_regs = kzalloc(sizeof(*tmp_regs), GFP_KERNEL);
    if (!tmp_regs) {
        pr_err("%s: kzalloc failed\n", TMP_DEVICE_NAME);
        goto err_device;
    }
    tmp_regs->configuration = CONFIG_12;

    mutex_init(&tmp_mutex);
    timer_setup(&tmp_timer, tmp_timer_callback, 0);
    mod_timer(&tmp_timer, jiffies + msecs_to_jiffies(1000));

    pr_info("%s: inserted\n", TMP_DEVICE_NAME);
    return 0;

err_device:
    device_destroy(tmp_class, tmp_dev);
err_class:
    class_destroy(tmp_class);
err_cdev_del:
    cdev_del(&tmp_cdev);
err_unregister_chr:
    unregister_chrdev_region(tmp_dev, 1);
err_unregister_i2c:
    i2c_unregister_device(tmp_virtual_client);
    return -ENODEV;
}

static void __exit tmp102_exit(void)
{
    del_timer_sync(&tmp_timer);
    mutex_destroy(&tmp_mutex);
    kfree(tmp_regs);
    device_destroy(tmp_class, tmp_dev);
    class_destroy(tmp_class);
    cdev_del(&tmp_cdev);
    unregister_chrdev_region(tmp_dev, 1);
    i2c_unregister_device(tmp_virtual_client);
    pr_info("%s: removed\n", TMP_DEVICE_NAME);
}

module_init(tmp102_init);
module_exit(tmp102_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Hechmi Wael");
MODULE_DESCRIPTION("Virtual TMP102 sensor driver");
MODULE_VERSION("2.1");
