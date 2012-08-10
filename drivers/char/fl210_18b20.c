#include    <linux/module.h>
#include    <linux/fs.h>
#include    <linux/kernel.h>
#include    <linux/init.h>
#include    <linux/delay.h>
#include    <linux/cdev.h>
#include    <linux/device.h>
#include    <linux/gpio.h>
#include    <plat/gpio-cfg.h>

#define    DEVICE_NAME "TEM0"
static struct cdev cdev;
struct class *tem_class;
static dev_t devno;
static int major = 243;

void tem_reset(void)
{
    s3c_gpio_cfgpin(S5PV210_GPH3(1), S3C_GPIO_SFN(1));
    gpio_set_value(S5PV210_GPH3(1), 1);
    udelay(100);
    gpio_set_value(S5PV210_GPH3(1), 0);
    udelay(600);
    gpio_set_value(S5PV210_GPH3(1), 1);
    udelay(100);
    s3c_gpio_cfgpin(S5PV210_GPH3(1), S3C_GPIO_SFN(0));
}

void tem_wbyte(unsigned char data)
{
    int i;

    s3c_gpio_cfgpin(S5PV210_GPH3(1), S3C_GPIO_SFN(1));
    for (i = 0; i < 8; ++i)
    {
        gpio_set_value(S5PV210_GPH3(1), 0);
        udelay(1);

        if (data & 0x01)
        {
            gpio_set_value(S5PV210_GPH3(1), 1);
        }
        udelay(60);
        gpio_set_value(S5PV210_GPH3(1), 1);
        udelay(15);
        data >>= 1;
    }
    gpio_set_value(S5PV210_GPH3(1), 1);
}

unsigned char tem_rbyte(void)
{
    int i;
    unsigned char ret = 0;

    for (i = 0; i < 8; ++i)
    {
        s3c_gpio_cfgpin(S5PV210_GPH3(1), S3C_GPIO_SFN(1));
        gpio_set_value(S5PV210_GPH3(1), 0);
        udelay(1);
        gpio_set_value(S5PV210_GPH3(1), 1);

        s3c_gpio_cfgpin(S5PV210_GPH3(1), S3C_GPIO_SFN(0));
        ret >>= 1;
        if (gpio_get_value(S5PV210_GPH3(1)))
        {
            ret |= 0x80;    
        }
        udelay(60);
    }
    s3c_gpio_cfgpin(S5PV210_GPH3(1), S3C_GPIO_SFN(1));


    return ret;
}

static ssize_t tem_read(struct file *filp, char *buf, size_t len, loff_t *offset)
{
    unsigned char low, high;

    tem_reset();
    udelay(420);
    tem_wbyte(0xcc);
    tem_wbyte(0x44);

    mdelay(750);
    tem_reset();
    udelay(400);
    tem_wbyte(0xcc);
    tem_wbyte(0xbe);

    low = tem_rbyte();
    high = tem_rbyte();

    *buf = low / 16 + high * 16;

    *(buf + 1) = (low & 0x0f) * 10 / 16 + (high & 0x0f) * 100 / 16 % 10;
    return 0;
}

static struct file_operations tem_fops = 
{
    .owner    = THIS_MODULE,
    .read    = tem_read,
};

static int __init tem_init(void)
{
    int result;
    devno = MKDEV(major, 0);

    result = register_chrdev_region(devno, 1, DEVICE_NAME);
    if (result)
    {
        printk("register failed\n");    
        return result;
    }

    cdev_init(&cdev, &tem_fops);
    cdev.owner = THIS_MODULE;
    cdev.ops = &tem_fops;
    result = cdev_add(&cdev, devno, 1);
    if (result)
    {
        printk("cdev add failed\n");    
        goto fail1;
    }

    tem_class = class_create(THIS_MODULE, "tmp_class");
    if (IS_ERR(tem_class))
    {
        printk("class create failed\n");    
        goto fail2;
    }

    device_create(tem_class, NULL, devno, DEVICE_NAME, DEVICE_NAME);
    return 0;
fail2:
    cdev_del(&cdev);
fail1:
    unregister_chrdev_region(devno, 1);
    return result;
}

static void __exit tem_exit(void)
{
    device_destroy(tem_class, devno);
    class_destroy(tem_class);
    cdev_del(&cdev);
    unregister_chrdev_region(devno, 1);
}

module_init(tem_init);
module_exit(tem_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("jhk");
