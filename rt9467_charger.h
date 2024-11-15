#ifndef __LINUX_RT9467_CHARGER_H__
#define __LINUX_RT9467_CHARGER_H__

struct rt9467_device_info;
struct rt9467_access_methods {
	int (*read)(struct rt9467_device_info *di, u8 reg);
	int (*write)(struct rt9467_device_info *di, u8 reg, int value);
};

struct rt9467_reg_cache {
	int charge_status;
	int charge_current;
	int online;
};

struct rt9467_device_info {
	struct device *dev;
	const char *name;
	struct rt9467_access_methods bus;
	struct rt9467_reg_cache cache;

	unsigned long last_update;
	struct delayed_work work;
	struct mutex lock;
	struct power_supply *psy;
};

int rt9467_i2c_charger_setup(struct rt9467_device_info *di);
void rt9467_i2c_charger_teardown(struct rt9467_device_info *di);

#endif
