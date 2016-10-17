/*
 * Copyright (c) 2015  Borislav Sapundzhiev
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/i2c.h>

/* module for explicit detect i2c device by bus id */

#define	I2C_BUS_MAX	4

/* device probing addreses */
static const unsigned short probe_i2c_addrs[] = I2C_ADDRS(0x6A, 0x6B);

static struct i2c_board_info l3gd20_i2c_info[] = {

	{ I2C_BOARD_INFO("l3gd20_gyr", 0x6B), }
};

static int __init l3gd20_probe_init(void)
{
	int nr;

	printk(KERN_ALERT "%s probing bus max %d\n",__FUNCTION__, I2C_BUS_MAX);

	for(nr =0; nr < I2C_BUS_MAX; nr++) {

		struct i2c_adapter * adap = i2c_get_adapter(nr);
		if(adap) {
			i2c_new_probed_device(adap, l3gd20_i2c_info, probe_i2c_addrs, NULL);
		}
	}

	return 0;
}

static void __exit l3gd20_probe_exit(void)
{
	printk(KERN_ALERT "%s\n", __FUNCTION__);
}

module_init(l3gd20_probe_init);
module_exit(l3gd20_probe_exit);

MODULE_AUTHOR("Borislav Sapundzhiev <BSapundzhiev@gmail.com>");
MODULE_DESCRIPTION("l3gd20 init");
MODULE_LICENSE("GPL");
