			// sample for opt3001 sensor 

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <sys/printk.h>
#include <sys/__assert.h>

static void do_main(const struct device *dev)
{
	int ret;
	struct sensor_value lux_value;
while (1) {
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
	 	printk("lux val1: %d \n",lux_value.val1);
	 	printk("lux val2: %d \n",lux_value.val2);
	 	k_sleep(K_MSEC(1000));
	  }



}

void main(void)
{
	//const struct device *dev = DEVICE_DT_GET_ANY(ti_opt3001);
	const struct device *dev = device_get_binding("OPT3001");
	__ASSERT(dev != NULL, "Failed to get device binding");
	__ASSERT(device_is_ready(dev), "Device %s is not ready", dev->name);
	printk("device is %p, name is %s\n", dev, dev->name);
	do_main(dev);
}

