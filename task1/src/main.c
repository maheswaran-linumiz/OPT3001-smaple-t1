		// opt3001 sensor value get after user button control without main,loop 

#include <errno.h>
#include <stdio.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <device.h>
#include <drivers/sensor.h>
#include <drivers/i2c.h>
#include <init.h>
#include <kernel.h>
#include <drivers/gpio.h>


#define device_id  0x7F
#define OPT3001_I2C_ADDR   0x44
static struct k_timer my_timer;

#define SW0_NODE        DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,
		                                                              {0});
static struct gpio_callback button_cb_data;




static void lux_(void)
{
	int ret;
	uint16_t buf[1];
	uint16_t raw;
	uint16_t result, exponent;
        float lux;
	double	light;
        struct sensor_value lux_value;


	const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));


	const struct device *dev = device_get_binding("OPT3001");
	__ASSERT(dev != NULL, "Failed to get device binding");
	__ASSERT(device_is_ready(dev), "Device %s is not ready", dev->name);
	printk("device is %p, name is %s\n", dev, dev->name);

      		printk("\nbuf0:0x%x\n",buf[0]);
                 printk("\nbuf1:0x%x\n",buf[1]);

		ret = sensor_sample_fetch(dev); 
	if (ret) {
		printk("sensor_sample_fetch failed ret %d\n", ret);
	        return;
	         }
         	ret = sensor_channel_get(dev, SENSOR_CHAN_LIGHT, &lux_value);

        if (ret) {
                printk("sensor_channel_get failed lux %d\n", ret);     
                return;
		 }		
        if (!device_is_ready(i2c_dev))
                {
		printk("I2C: Device is not ready.\n");
		return;
		 }
	        ret = i2c_burst_read(i2c_dev,0x44,0x7F,&buf[0],2);
       if(ret < 0)
                {
                printk("failed to read the address\n");
                return;
               }
       light = sensor_value_to_double (&lux_value);
		printf("light:%f\n",light);
		k_msleep(2000);
}

K_WORK_DEFINE(my_work,lux_);

static void my_timer_handler(struct k_timer *dummy)
		{
		k_work_submit(&my_work);
		}

void button_pressed(const struct device *dev, struct gpio_callback *cb,
		                    uint32_t pins)
{
	printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
		k_timer_init(&my_timer,my_timer_handler,NULL);
                k_timer_start(&my_timer,K_SECONDS(2),K_MSEC(1000));
}		

void fun(void)
{
	       int ret;
     if (!device_is_ready(button.port))
                {
		printk("Error: button device %s is not ready\n",button.port->name);
		return;
		}
	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
        if (ret != 0)
                {
                printk("Error %d: failed to configure %s pin %d\n",ret, button.port->name, button.pin);
                return;
                }
        ret = gpio_pin_interrupt_configure_dt(&button,GPIO_INT_EDGE_TO_ACTIVE);
        if (ret != 0)
                {
                printk("Error %d: failed to configure interrupt on %s pin %d\n",ret, button.port->name, button.pin);
                return;
                }

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
        gpio_add_callback(button.port, &button_cb_data);
	        printk("Set up button at %s pin %d\n", button.port->name, button.pin);
		
		
					
}


 SYS_INIT(fun,APPLICATION,2);


