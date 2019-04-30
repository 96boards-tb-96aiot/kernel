/*
 * PERICOM 31213A driver 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/extcon.h>
#include <linux/regulator/consumer.h>

#include "pi5usb31213a.h"

#define DRIVER_NAME "pericom_31213a"

/*
   pericom chip struct
  */
struct pericom_31213a_chip {
	struct i2c_client *i2c_client;
	struct device *dev;
	struct mutex i2c_rw_mutex;
    struct regmap *regmap;
    struct work_struct work;
    struct workqueue_struct *pericom_31213a_wq;
    struct extcon_dev *extcon;
    struct gpio_desc *gpio_vbus_5v;
    struct gpio_desc *enable_gpio;
    struct gpio_desc *gpio_int;
    spinlock_t irq_lock;
    int gpio_int_irq;
    int enable_irq;
	u8 cc_state;
    int cc1;
    int cc2;
	int irq;	
};

static const unsigned int pericom_31213a_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_NONE,
};

 /**
 * pericom_31213a_i2c_read()
 *
 * Called by various functions in this driver,
 * This function reads data of an arbitrary length from the sensor,
 * starting from an assigned register address of the sensor, via I2C
 * with a retry mechanism.
 */
static int pericom_31213a_i2c_read(struct pericom_31213a_chip *p31213a_chip,
		unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char retry;
	struct i2c_msg msg[] = {
		{
			.addr = p31213a_chip->i2c_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		},
	};

	mutex_lock(&(p31213a_chip->i2c_rw_mutex));

	for (retry = 0; retry < PERICOM_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(p31213a_chip->i2c_client->adapter, msg, 1) > 0) {
			retval = length;
			break;
		}
		dev_err(&p31213a_chip->i2c_client->dev,
				"%s: I2C retry %d\n",__func__, retry + 1);
		msleep(20);
	}

	if (retry == PERICOM_I2C_RETRY_TIMES) {
		dev_err(&p31213a_chip->i2c_client->dev,
				"%s: I2C read over retry limit\n",	__func__);
		retval = -EIO;
	}

	mutex_unlock(&(p31213a_chip->i2c_rw_mutex));

	return retval;
}

 /**
 * pericom_31213a_i2c_write()
 *
 * Called by various functions in this driver
 *
 * This function writes data of an arbitrary length to the sensor,
 * starting from an assigned register address of the sensor, via I2C with
 * a retry mechanism.
 */
static int pericom_31213a_i2c_write(struct pericom_31213a_chip *p31213a_chip,
		unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char retry;
	struct i2c_msg msg[] = {
		{
			.addr = p31213a_chip->i2c_client->addr,
			.flags = 0,
			.len = length ,
			.buf = data,
		}
	};

	mutex_lock(&(p31213a_chip->i2c_rw_mutex));

	for (retry = 0; retry < PERICOM_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(p31213a_chip->i2c_client->adapter, msg, 1) == 1) {
			retval = length;
			break;
		}
		dev_err(&p31213a_chip->i2c_client->dev,
				"%s: I2C retry %d\n",
				__func__, retry + 1);
		msleep(20);
	}

	if (retry == PERICOM_I2C_RETRY_TIMES) {
		dev_err(&p31213a_chip->i2c_client->dev,
				"%s: I2C write over retry limit\n",
				__func__);
		retval = -EIO;
	}

	mutex_unlock(&(p31213a_chip->i2c_rw_mutex));

	return retval;
}

/*set mode*/
static int pericom_31213a_set_power_mode(struct pericom_31213a_chip *p31213a_chip,enum pericom_power_mode mode)
{
	int ret;
	char buf[2] = {0x20,0};

	//read reg1 value
	pericom_31213a_i2c_read(p31213a_chip,buf,2);
	//construct reg1 value
	buf[1] =(buf[1] & ~PERICOM_POWER_SAVING_MASK)| ((mode << PERICOM_POWER_SAVING_OFFSET )& PERICOM_POWER_SAVING_MASK);
	buf[1] &= ~PERICOM_INTERRUPT_MASK;

	//write reg1 value
	ret = pericom_31213a_i2c_write(p31213a_chip, buf, 2);
	return ret;
}

void pericom_31213a_irq_disable(struct pericom_31213a_chip *p31213a_chip)
{
	unsigned long irqflags = 0;

	spin_lock_irqsave(&p31213a_chip->irq_lock, irqflags);
	if (p31213a_chip->enable_irq) {
		disable_irq_nosync(p31213a_chip->gpio_int_irq);
		p31213a_chip->enable_irq = 0;
	} else {
		dev_warn(p31213a_chip->dev, "irq have already disabled\n");
	}
	spin_unlock_irqrestore(&p31213a_chip->irq_lock, irqflags);
}

void pericom_31213a_irq_enable(struct pericom_31213a_chip *p31213a_chip)
{
	unsigned long irqflags = 0;

	spin_lock_irqsave(&p31213a_chip->irq_lock, irqflags);
	if (!p31213a_chip->enable_irq) {
		enable_irq(p31213a_chip->gpio_int_irq);
		p31213a_chip->enable_irq = 1;
	}
	spin_unlock_irqrestore(&p31213a_chip->irq_lock, irqflags);
}

/* set power saving mode
  * success if return positive value ,or return negative
  */
static int pericom_31213a_set_powersaving_mode(struct pericom_31213a_chip *p31213a_chip)
{
	return pericom_31213a_set_power_mode(p31213a_chip,POWERSAVING_MODE);
}

static int pericom_31213a_attached_state_detect(struct pericom_31213a_chip *p31213a_chip)
{
	unsigned int attached_state = 0x0;
	int ret;
	char reg[4] = {0,0,0,0};
	char port_status = 0x0;
	
	ret = pericom_31213a_i2c_read(p31213a_chip,reg,4);
	if(ret < 0)
	{
	   dev_err(&p31213a_chip->i2c_client->dev,"%s: I2C read error ret = %d\n",__func__, ret);
	}
	
	port_status = (reg[3]>>2)&0x07;
	switch(port_status)
	{
		case 1:
			dev_info(&p31213a_chip->i2c_client->dev, "Device plug in.\n");
			attached_state = EXTCON_USB_HOST;
			// switch on vbus power
			break;
		case 2:
			dev_info(&p31213a_chip->i2c_client->dev, "Host plug in.\n");
			attached_state = EXTCON_USB;
			break;
		default:
			attached_state = EXTCON_NONE;
			break;
	}
	
	return attached_state;
}
/*
static void pericom_31213a_set_extern_state(struct pericom_31213a_chip *p31213a_chip)
{
	unsigned int attached_state = pericom_31213a_attached_state_detect(p31213a_chip);
	if(attached_state == EXTCON_USB_HOST){
		extcon_set_state_sync(p31213a_chip->extcon, EXTCON_USB, false);
		extcon_set_state_sync(p31213a_chip->extcon, EXTCON_USB_HOST, true);
	}else if(attached_state == EXTCON_USB){
		extcon_set_state_sync(p31213a_chip->extcon, EXTCON_USB, true);
		extcon_set_state_sync(p31213a_chip->extcon, EXTCON_USB_HOST, false);
	}else{
		extcon_set_state_sync(p31213a_chip->extcon, EXTCON_USB, true);
		extcon_set_state_sync(p31213a_chip->extcon, EXTCON_USB_HOST, false);
	}
}
*/

static bool ic_is_present(struct pericom_31213a_chip *p31213a_chip)
{
	int ret;
	char reg ;

	//read reg0 value
	ret = pericom_31213a_i2c_read(p31213a_chip,&reg,1);
	
	printk("Pericom 31213A Type C Connector ChipID: %02x \n", reg);

	return (reg == 0x08)? true:false;    //31213A ChipID is 0x08
}

static int pericom_31213a_initgpio(struct pericom_31213a_chip *p31213a_chip)
{
  	 // init gpio
	p31213a_chip->gpio_int = devm_gpiod_get_optional(p31213a_chip->dev, "int-n", GPIOD_IN);
	if (IS_ERR(p31213a_chip->gpio_int))
	{
	   dev_warn(p31213a_chip->dev, "Could not get named GPIO for INT!\n");
	   return PTR_ERR(p31213a_chip->gpio_int);
	}
	
	p31213a_chip->enable_gpio = devm_gpiod_get_optional(p31213a_chip->dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(p31213a_chip->enable_gpio))
	   dev_warn(p31213a_chip->dev, "Could not get named GPIO for ENABLE chip!\n");
	else
	   gpiod_set_value(p31213a_chip->enable_gpio, 1);
	
	// below gpio complict with otg_vbus_drive in usb otg controller driver
	/*
	p31213a_chip->gpio_vbus_5v = devm_gpiod_get_optional(p31213a_chip->dev, "vbus-5v", GPIOD_OUT_LOW);
	printk("p31213a_chip->gpio_vbus_5v =0x%p\n",p31213a_chip->gpio_vbus_5v);
	if (IS_ERR(p31213a_chip->gpio_vbus_5v))
		dev_warn(p31213a_chip->dev, "Could not get named GPIO for VBus5V!\n");
	else
		gpiod_set_value(p31213a_chip->gpio_vbus_5v, 0);
	*/
	
	return 0;	
}

static int pericom_31213a_initialize(struct pericom_31213a_chip *p31213a_chip)
{	
	int ret;
	char reg[4] = {0,0,0,0};
	
	// init register
	ret = pericom_31213a_i2c_read(p31213a_chip,reg,4);
	if(ret < 0)
	{
	   dev_err(&p31213a_chip->i2c_client->dev,"%s: I2C read error ret = %d\n",__func__, ret);
	}
	
	//reg[1] = (reg[1]&~PERICOM_INTERRUPT_MASK) |PERICOM_DRP_MODE;
	reg[1] = 0x66;
	//write reg1 value
	ret = pericom_31213a_i2c_write(p31213a_chip, reg, 2);
	if(ret < 0)
	{
	   dev_err(&p31213a_chip->i2c_client->dev,"%s: I2C write error ret = %d\n",__func__, ret);
	}
	return ret;
	
}

static irqreturn_t pericom_31213a_irq_handler(int irq, void *dev_id)
{
	struct pericom_31213a_chip *p31213a_chip = (struct pericom_31213a_chip *) dev_id;
	char reg[4] = {0,0,0,0};
	char curr_mode = 0x0;
	char int_status = 0x0;
	char cc_status = 0x0;
	char port_status = 0x0;
	char control_status = 0x0;

	// 0.Mask interrupt
	pericom_31213a_i2c_read(p31213a_chip,  reg, 2);
	dev_info(&p31213a_chip->i2c_client->dev, "interrupt .reg=%x,%x,%x,%x\n",reg[0],reg[1],reg[2],reg[3]);
	curr_mode = reg[1] & (PERICOM_ROLE_MODE_MASK|PERICOM_DRP2_TRY_SNK);   //support trysnk mode
	reg[1] = reg[1] | PERICOM_INTERRUPT_MASK;
	pericom_31213a_i2c_write(p31213a_chip, reg, 2);

	// 1.delay 30ms
	// msleep(30);
	// 2.Read reg
	pericom_31213a_i2c_read(p31213a_chip,  reg, 4);
	dev_info(&p31213a_chip->i2c_client->dev, "2.reg=%x,%x,%x,%x\n",reg[0],reg[1],reg[2],reg[3]);

	// 3. Processing
	control_status = reg[1];
	int_status = reg[2];
    if(int_status&0x02)
		dev_info(&p31213a_chip->i2c_client->dev, "TypeC Unpluged.\n");
	if(int_status&0x01)
		dev_info(&p31213a_chip->i2c_client->dev, "TypeC Plugin.\n");
	cc_status = reg[3];
	if(cc_status&0x01)
		dev_info(&p31213a_chip->i2c_client->dev, "CC1 connected.\n");
	if(cc_status&0x02)
		dev_info(&p31213a_chip->i2c_client->dev, "CC2 connected.\n");
	port_status = (reg[3]>>2)&0x07;
	if((cc_status&0x01)|(cc_status&0x02))
	{
		switch(port_status)
		{
			case 1:
				dev_info(&p31213a_chip->i2c_client->dev, "Device plug in.\n");
				// switch on vbus power in usb otg controller driver
				extcon_set_state_sync(p31213a_chip->extcon, EXTCON_USB, false);
				extcon_set_state_sync(p31213a_chip->extcon, EXTCON_USB_HOST, true);
				break;
			case 2:
				dev_info(&p31213a_chip->i2c_client->dev, "Host plug in.\n");
				extcon_set_state_sync(p31213a_chip->extcon, EXTCON_USB, true);
				extcon_set_state_sync(p31213a_chip->extcon, EXTCON_USB_HOST, false);
				break;
			case 3:
				dev_info(&p31213a_chip->i2c_client->dev, "Audio Adapter Accessory plug in.\n");
				break;
			case 4:
				dev_info(&p31213a_chip->i2c_client->dev, "Debug Accessory plug in.\n");
				break;
			default:
				break;
		}
		
	}
	if(reg[3]==0x04) // special process for PI5USB31213A
	{
		reg[1] = 0x01;
		pericom_31213a_i2c_write(p31213a_chip, reg, 2);
		msleep(30);
		reg[1]= control_status;
		pericom_31213a_i2c_write(p31213a_chip, reg, 2);
		msleep(10);
	}

	// 4. Unmask interrupt
	reg[1] = curr_mode;
	pericom_31213a_i2c_write(p31213a_chip, reg, 2);
	

	return IRQ_HANDLED;
}

static int pericom_31213a_probe(struct i2c_client *client,
		const struct i2c_device_id *dev_id)
{
	int retval = 0;
	struct pericom_31213a_chip *p31213a_chip;
	
	printk("*****pericom_31213a_probe*****\n");

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev,
				"%s: SMBus byte data not supported\n",
				__func__);
		return -EIO;
	}

	p31213a_chip = devm_kzalloc(&client->dev,sizeof(*p31213a_chip),GFP_KERNEL);
	if (!p31213a_chip) {
		dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
	}

    p31213a_chip->dev = &client->dev;
	
	pericom_31213a_initgpio(p31213a_chip);
	mutex_init(&(p31213a_chip->i2c_rw_mutex));
	p31213a_chip->i2c_client = client;
	p31213a_chip->irq = gpiod_to_irq(p31213a_chip->gpio_int);
	if (p31213a_chip->irq < 0) {
		dev_err(&client->dev,"Unable to request IRQ for INT_N GPIO! %d\n",p31213a_chip->irq);
		retval = -ENXIO;
	}
	i2c_set_clientdata(client, p31213a_chip);
	
	//check ic present, if not present ,exit, or go on
	if (!ic_is_present(p31213a_chip)) {
		dev_err(&client->dev, "The device is absent\n");
		retval = -ENXIO;
	}

	p31213a_chip->extcon = devm_extcon_dev_allocate(&client->dev, pericom_31213a_cable);
	if (IS_ERR(p31213a_chip->extcon)) {
           dev_err(&client->dev, "failed to allocate extcon device\n");
           retval = -ENXIO;;
   }
   
    retval = devm_extcon_dev_register(&client->dev, p31213a_chip->extcon);
    if (retval < 0) {
          dev_err(&client->dev, "failed to register extcon device\n");
          return retval;
    }
	
	pericom_31213a_initialize(p31213a_chip);
	
	pericom_31213a_attached_state_detect(p31213a_chip);

	/* interrupt */
	retval = request_threaded_irq(p31213a_chip->irq, NULL,
		pericom_31213a_irq_handler, IRQF_TRIGGER_LOW| IRQF_ONESHOT,
		DRIVER_NAME, p31213a_chip);

	if (retval < 0) {
		dev_err(&client->dev,
				"%s: Failed to create irq thread\n",
				__func__);
	}

	if (retval < 0)
		goto err_fs;
	
	pericom_31213a_irq_enable(p31213a_chip);

	return retval;

err_fs:
    free_irq(p31213a_chip->irq,p31213a_chip);

	return retval;
}

static int pericom_31213a_remove(struct i2c_client *client)
{
	//enter power saving mode
	struct pericom_31213a_chip *p31213a_chip = i2c_get_clientdata(client);

	pericom_31213a_set_powersaving_mode(p31213a_chip);
	return 0;
}

static void pericom_31213a_shutdown(struct i2c_client *client)
{
	//enter power saving mode
	struct pericom_31213a_chip *p31213a_chip = i2c_get_clientdata(client);

	pericom_31213a_set_powersaving_mode(p31213a_chip);
}

static const struct i2c_device_id pericom_31213a_id_table[] = {
	{DRIVER_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, pericom_31213a_id_table);

static struct of_device_id pericom_match_table[] = {
	{ .compatible = "pericom,31213a",},
	{ },
};

static struct i2c_driver pericom_31213a_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = pericom_match_table,

	},
	.probe = pericom_31213a_probe,
	.remove = pericom_31213a_remove,
	.shutdown = pericom_31213a_shutdown,
	.id_table = pericom_31213a_id_table,
};

static int __init pericom_31213a_init(void)
{
	return i2c_add_driver(&pericom_31213a_driver);
}

static void __exit pericom_31213a_exit(void)
{
	i2c_del_driver(&pericom_31213a_driver);
}

module_init(pericom_31213a_init);
module_exit(pericom_31213a_exit);

MODULE_AUTHOR("Pericom, Inc.");
MODULE_DESCRIPTION("Pericom 31213A I2C  Driver");
MODULE_LICENSE("GPL v2");
