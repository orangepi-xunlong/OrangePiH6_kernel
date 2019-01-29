#include <linux/init.h>
#include <linux/module.h>
#include <linux/pwm.h>
#include <linux/delay.h>

struct pwm_device *pwm;
static int __init test_init(void)
{
    int ret = -ENODEV;
    pwm = pwm_request(0, "mypwmdev");
    if (NULL == pwm)
        goto err0;

    pwm_config(pwm, 500000, 20000000); // 0.5ms
    pwm_set_polarity(pwm, PWM_POLARITY_NORMAL);
    pwm_enable(pwm);

    msleep(3000);
    printk("change ...\n");
    pwm_disable(pwm);
    pwm_config(pwm, 1000000, 2000000); // 1ms
    pwm_enable(pwm);


    return 0;

err0:
    return ret;
}

static void __exit test_exit(void)
{
    pwm_disable(pwm);
    pwm_free(pwm);
}

module_init(test_init);
module_exit(test_exit);

MODULE_LICENSE("GPL");

