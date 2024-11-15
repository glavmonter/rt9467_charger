#include <linux/module.h>
#include <linux/i2c.h>

#include "rt9467_charger.h"

static int rt9467_i2c_charger_read(struct rt9467_device_info *di, u8 reg)
{
	struct i2c_client *client = to_i2c_client(di->dev);
	struct i2c_msg msg[2];
	u8 data[2];
	int ret;
	if (!client->adapter)
		return -ENODEV;
	
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;
	msg[1].len = 1;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0)
		return ret;
	
	ret = data[0];
	return ret;
}

static int rt9467_i2c_charger_write(struct rt9467_device_info *di, u8 reg, int value)
{
	return 0;
}

static int rt9467_i2c_charger_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	struct rt9467_device_info *di;
	char *name;

	dev_info(&client->dev, "Init rt9467_i2c_charger_probe");
	dev_info(&client->dev, "id->name: %s", id->name);

	di = devm_kzalloc(&client->dev, sizeof(*di), GFP_KERNEL);
	if (!di) 
		goto err_mem;
	
	name = devm_kasprintf(&client->dev, GFP_KERNEL, "%s", id->name);
	if (!name)
		goto err_mem;
	
	di->dev = &client->dev;
	di->name = name;
	di->bus.read = rt9467_i2c_charger_read;
	di->bus.write = rt9467_i2c_charger_write;
	
	ret = rt9467_i2c_charger_setup(di);
	if (ret)
		goto err_failed;

	i2c_set_clientdata(client, di);
	return 0;

err_mem:
	ret = -ENOMEM;

err_failed:
    return ret;
}

static int rt9467_i2c_charger_remove(struct i2c_client *client)
{
	struct rt9467_device_info *di = i2c_get_clientdata(client);
	rt9467_i2c_charger_teardown(di);
	return 0;
}


static const struct i2c_device_id rt9467_i2c_id_table[] = {
	{ "rt9467-i2c", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rt9467_i2c_id_table);

static const struct of_device_id rt9467_i2c_charger_of_match_table[] = {
    { .compatible = "richtek,rt9467-i2c", },
    {}
};
MODULE_DEVICE_TABLE(of, rt9467_i2c_charger_of_match_table);

static struct i2c_driver rt9467_i2c_charger_driver = {
	.driver = {
        .name = "rt9467-charger-i2c",
        .of_match_table = rt9467_i2c_charger_of_match_table
    },
	.id_table = rt9467_i2c_id_table,
    .probe = rt9467_i2c_charger_probe,
	.remove = rt9467_i2c_charger_remove,
};

module_i2c_driver(rt9467_i2c_charger_driver);

MODULE_LICENSE("GPL");       // License type
MODULE_AUTHOR("Vladimir Meshkov <glavmonter@gmail.com>");  // Author of the module
MODULE_DESCRIPTION("Richteck RT9467 charger only I2C bus");
MODULE_VERSION("1.0");
