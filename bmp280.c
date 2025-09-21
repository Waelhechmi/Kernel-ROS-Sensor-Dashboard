/* -------------------- bmp280.c -------------------- */

/*
 Virtual BMP280 driver: simulates temperature and pressure registers
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

#define BMP_DEVICE_NAME    "bmp280"
#define BMP_I2C_ADDRESS    0x76

static int bmp_temp = 2500; /* centi-degrees (25.00 C) */
module_param(bmp_temp, int, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(bmp_temp, "Initial temperature in centi-degrees");

static int bmp_press = 101325; /* Pa */
module_param(bmp_press, int, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(bmp_press, "Initial pressure in Pa");

struct bmp280_registers {
    int32_t temperature_centi;
    int32_t pressure_pa;
    uint16_t configuration;
};

static struct timer_list bmp_timer;
static int bmp_temp_dir = 1;
static int bmp_press_dir = 1;
static dev_t bmp_dev = 0;
static struct class *bmp_class;
static struct i2c_client *bmp_virtual_client;
static struct cdev bmp_cdev;
static struct mutex bmp_mutex;
static struct bmp280_registers *bmp_regs;

static int bmp_open(struct inode *inode, struct file *file)
{
    if (!mutex_trylock(&bmp_mutex))
        return -EBUSY;
    pr_info("%s: opened\n", BMP_DEVICE_NAME);
    return 0;
}

static int bmp_release(struct inode *inode, struct file *file)
{
    mutex_unlock(&bmp_mutex);
    pr_info("%s: closed\n", BMP_DEVICE_NAME);
    return 0;
}

struct bmp_user_out {
    int32_t temperature_centi;
    int32_t pressure_pa;
};

static ssize_t bmp_read(struct file *filp, char __user *buf, size_t len, loff_t *off)
{
    size_t want = sizeof(struct bmp_user_out);
    struct bmp_user_out out;
    if (len < want)
        len = want;
    out.temperature_centi = bmp_regs->temperature_centi;
    out.pressure_pa = bmp_regs->pressure_pa;
    if (copy_to_user(buf, &out, want))
        return -EFAULT;
    pr_debug("%s: read T=%d (centi) P=%d (Pa)\n", BMP_DEVICE_NAME, out.temperature_centi, out.pressure_pa);
    return want;
}

static ssize_t bmp_write(struct file *filp, const char __user *buf, size_t len, loff_t *off)
{
    if (len >= sizeof(bmp_regs->configuration)) {
        if (copy_from_user(&bmp_regs->configuration, buf, sizeof(bmp_regs->configuration)))
            return -EFAULT;
        pr_info("%s: configuration set 0x%04x\n", BMP_DEVICE_NAME, bmp_regs->configuration);
        return sizeof(bmp_regs->configuration);
    }
    return -EINVAL;
}

static const struct file_operations bmp_fops = {
    .owner = THIS_MODULE,
    .open = bmp_open,
    .release = bmp_release,
    .read = bmp_read,
    .write = bmp_write,
};

static void bmp_timer_callback(struct timer_list *t)
{
    /* small oscillation for temperature */
    if (bmp_temp_dir == 1) {
        bmp_temp += 5;
        if (bmp_temp > 3000)
            bmp_temp_dir = -1;
    } else {
        bmp_temp -= 5;
        if (bmp_temp < 2000)
            bmp_temp_dir = 1;
    }

    /* pressure oscillation */
    if (bmp_press_dir == 1) {
        bmp_press += 10;
        if (bmp_press > 102500)
            bmp_press_dir = -1;
    } else {
        bmp_press -= 10;
        if (bmp_press < 100500)
            bmp_press_dir = 1;
    }

    bmp_regs->temperature_centi = bmp_temp;
    bmp_regs->pressure_pa = bmp_press;

    mod_timer(&bmp_timer, jiffies + msecs_to_jiffies(500));
}

static int __init bmp_init(void)
{
    struct i2c_adapter *adapter;
    struct i2c_board_info info = { I2C_BOARD_INFO("i2c_virtual_bmp280", BMP_I2C_ADDRESS) };

    /* create virtual i2c client */
    adapter = i2c_get_adapter(1);
    if (!adapter) {
        pr_err("%s: failed to get i2c adapter 1\n", BMP_DEVICE_NAME);
        return -ENODEV;
    }

    bmp_virtual_client = i2c_new_client_device(adapter, &info);
    i2c_put_adapter(adapter);
    if (!bmp_virtual_client) {
        pr_err("%s: failed to create virtual i2c client\n", BMP_DEVICE_NAME);
        return -ENODEV;
    }

    /* allocate major/minor */
    if (alloc_chrdev_region(&bmp_dev, 0, 1, BMP_DEVICE_NAME) < 0) {
        pr_err("%s: alloc_chrdev_region failed\n", BMP_DEVICE_NAME);
        goto err_unregister_i2c;
    }

    /* initialize cdev */
    cdev_init(&bmp_cdev, &bmp_fops);
    if (cdev_add(&bmp_cdev, bmp_dev, 1) < 0) {
        pr_err("%s: cdev_add failed\n", BMP_DEVICE_NAME);
        goto err_unregister_chr;
    }

    /* create class and device */
    bmp_class = class_create( BMP_DEVICE_NAME);
    if (IS_ERR(bmp_class)) {
        pr_err("%s: class_create failed\n", BMP_DEVICE_NAME);
        goto err_cdev_del;
    }

    if (IS_ERR(device_create(bmp_class, NULL, bmp_dev, NULL, BMP_DEVICE_NAME))) {
        pr_err("%s: device_create failed\n", BMP_DEVICE_NAME);
        goto err_class;
    }

    /* allocate registers memory */
    bmp_regs = kzalloc(sizeof(*bmp_regs), GFP_KERNEL);
    if (!bmp_regs) {
        pr_err("%s: kzalloc failed\n", BMP_DEVICE_NAME);
        goto err_device;
    }
    bmp_regs->temperature_centi = bmp_temp;
    bmp_regs->pressure_pa = bmp_press;

    mutex_init(&bmp_mutex);
    timer_setup(&bmp_timer, bmp_timer_callback, 0);
    mod_timer(&bmp_timer, jiffies + msecs_to_jiffies(500));

    pr_info("%s: inserted\n", BMP_DEVICE_NAME);
    return 0;

err_device:
    device_destroy(bmp_class, bmp_dev);
err_class:
    class_destroy(bmp_class);
err_cdev_del:
    cdev_del(&bmp_cdev);
err_unregister_chr:
    unregister_chrdev_region(bmp_dev, 1);
err_unregister_i2c:
    i2c_unregister_device(bmp_virtual_client);
    return -ENODEV;
}

static void __exit bmp_exit(void)
{
    del_timer_sync(&bmp_timer);
    mutex_destroy(&bmp_mutex);
    kfree(bmp_regs);
    device_destroy(bmp_class, bmp_dev);
    class_destroy(bmp_class);
    cdev_del(&bmp_cdev);
    unregister_chrdev_region(bmp_dev, 1);
    i2c_unregister_device(bmp_virtual_client);
    pr_info("%s: removed\n", BMP_DEVICE_NAME);
}

module_init(bmp_init);
module_exit(bmp_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Hechmi Wael");
MODULE_DESCRIPTION("Virtual BMP280 sensor driver (temperature+pressure)");
MODULE_VERSION("1.1");
