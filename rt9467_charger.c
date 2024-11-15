#include <linux/kernel.h>    // kernel header
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/power_supply.h>
#include <linux/linear_range.h>
#include <linux/of_device.h>
#include <linux/regmap.h>

#include "rt9467_charger.h"


#define RT9467_REG_CORE_CTRL0		0x00
#define RT9467_REG_CHG_CTRL1		0x01
#define RT9467_REG_CHG_CTRL2		0x02
#define RT9467_REG_CHG_CTRL3		0x03
#define RT9467_REG_CHG_CTRL4		0x04
#define RT9467_REG_CHG_CTRL5		0x05
#define RT9467_REG_CHG_CTRL6		0x06
#define RT9467_REG_CHG_CTRL7		0x07
#define RT9467_REG_CHG_CTRL8		0x08
#define RT9467_REG_CHG_CTRL9		0x09
#define RT9467_REG_CHG_CTRL10		0x0A
#define RT9467_REG_CHG_CTRL12		0x0C
#define RT9467_REG_CHG_CTRL13		0x0D
#define RT9467_REG_CHG_CTRL14		0x0E
#define RT9467_REG_CHG_ADC			0x11
#define RT9467_REG_CHG_DPDM1		0x12
#define RT9467_REG_CHG_DPDM2		0x13
#define RT9467_REG_DEVICE_ID		0x40
#define RT9467_REG_CHG_STAT			0x42
#define RT9467_REG_ADC_DATA_H		0x44
#define RT9467_REG_CHG_STATC		0x50
#define RT9467_REG_CHG_IRQ1			0x53
#define RT9467_REG_CHG_STATC_CTRL	0x60
#define RT9467_REG_CHG_IRQ1_CTRL	0x63

#define LINEAR_RANGE(_min, _min_sel, _max_sel, _step)		\
	{							\
		.min = _min,					\
		.min_sel = _min_sel,				\
		.max_sel = _max_sel,				\
		.step = _step,					\
	}

#define LINEAR_RANGE_IDX(_idx, _min, _min_sel, _max_sel, _step)	\
	[_idx] = LINEAR_RANGE(_min, _min_sel, _max_sel, _step)

enum {
	RT9467_STAT_READY = 0,
	RT9467_STAT_PROGRESS,
	RT9467_STAT_CHARGE_DONE,
	RT9467_STAT_FAULT
};

enum rt9467_ranges {
	RT9467_RANGE_ICHG = 0,
	RT9467_RANGES_MAX
};


static const struct linear_range rt9467_ranges[RT9467_RANGES_MAX] = {
	LINEAR_RANGE_IDX(RT9467_RANGE_ICHG, 900000, 0x08, 0x31, 100000),
};


static const enum power_supply_property rt9467_charger_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	// POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	// POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	// POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT,
	// POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	// POWER_SUPPLY_PROP_PRECHARGE_CURRENT
};

static inline int rt9467_read(struct rt9467_device_info *di, u8 reg)
{
	int ret;
	if (!di)
		return -EINVAL;
	ret = di->bus.read(di, reg);
	if (ret < 0)
		dev_dbg(di->dev, "failed to read register 0x%02x\n", reg);
	return ret;
}






void rt9467_charger_update(struct rt9467_device_info *di) {
	struct rt9467_reg_cache cache = {0, };

	cache.charge_status = rt9467_read(di, RT9467_REG_CHG_STAT);
	if (!cache.charge_status)
		dev_warn(di->dev, "Read register RT9467_REG_CHG_STAT failed\n");

	cache.charge_current = rt9467_read(di, RT9467_REG_CHG_CTRL7);
	if (!cache.charge_current)
		dev_warn(di->dev, "Read register RT9467_REG_CHG_CTRL7 failed\n");

	cache.online = rt9467_read(di, RT9467_REG_CHG_STATC);
	if (!cache.online)
		dev_warn(di->dev, "Read register RT9467_REG_CHG_CTRL7 failed\n");

	if (di->cache.charge_status != cache.charge_status   ||
		di->cache.charge_current != cache.charge_current ||
		di->cache.online != cache.online) 
		power_supply_changed(di->psy);
	
	if (memcmp(&di->cache, &cache, sizeof(cache)) != 0)
		di->cache = cache;
	
	di->last_update = jiffies;
}

static void rt9467_i2c_charger_poll(struct work_struct *work) {
	struct rt9467_device_info *di =
		container_of(work, struct rt9467_device_info, work.work);

	rt9467_charger_update(di);
	schedule_delayed_work(&di->work, 20 * HZ); // 20 seconds
}

static int rt9467_get_value_from_ranges(struct rt9467_device_info *di,
						int reg_value,
						enum rt9467_ranges rsel,
						int *value)
{
	const struct linear_range *range = rt9467_ranges + rsel;
	return linear_range_get_value(range, reg_value, value);
}

// static int rt9467_set_value_from_ranges(struct rt9467_device_info *di,
// 						enum rt9467_ranges rsel,
// 						int value)
// {
// 	return 0;
// }

static int rt9467_psy_get_status(struct rt9467_device_info *di, int *state)
{
	unsigned int status = (di->cache.charge_status & (3 << 6)) >> 6;
	switch (status) {
	case RT9467_STAT_READY:
		*state = POWER_SUPPLY_STATUS_NOT_CHARGING;
		return 0;
	case RT9467_STAT_PROGRESS:
		*state = POWER_SUPPLY_STATUS_CHARGING;
		return 0;
	case RT9467_STAT_CHARGE_DONE:
		*state = POWER_SUPPLY_STATUS_FULL;
		return 0;
	default:
		*state = POWER_SUPPLY_STATUS_UNKNOWN;
		return 0;
	}
}

static int rt9467_psy_get_online(struct rt9467_device_info *di, int *state)
{
	if (di->cache.online & (1 << 7))
		*state = 1;
	else
		*state = 0;
	return 0;
}

static int rt9467_psy_get_charge_current(struct rt9467_device_info *di, int *microamps)
{
	int mapped_value;
	unsigned int reg = di->cache.charge_current >> 2;
	rt9467_get_value_from_ranges(di, reg, RT9467_RANGE_ICHG, &mapped_value);
	dev_info(di->dev, "charge reg: %d -> %d\n", reg, mapped_value);
	*microamps = mapped_value;
	return 0;
}

static int rt9467_psy_set_charge_current(struct rt9467_device_info *di, int microamps)
{
	dev_info(di->dev, "set_current to %d\n", microamps);
	return 0;
}

static int rt9467_psy_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	struct rt9467_device_info *di = power_supply_get_drvdata(psy);

	mutex_lock(&di->lock);
	if (time_is_before_jiffies(di->last_update + 5 * HZ)) { // За 5 секунд до окончания jiffi
		cancel_delayed_work_sync(&di->work);
		rt9467_i2c_charger_poll(&di->work.work);
	}
	mutex_unlock(&di->lock);

	switch (psp) {
		case POWER_SUPPLY_PROP_STATUS:
			return rt9467_psy_get_status(di, &val->intval);

		case POWER_SUPPLY_PROP_ONLINE:
			return rt9467_psy_get_online(di, &val->intval);

		case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
			return rt9467_psy_get_charge_current(di, &val->intval);

		default:
			return -ENODATA;
	}
	return ret;
}

static int rt9467_psy_set_property(struct power_supply *psy,
					enum power_supply_property psp,
					const union power_supply_propval *val)
{
	int ret = 0;
	struct rt9467_device_info *di = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		return rt9467_psy_set_charge_current(di, val->intval);
	default:
		return -EINVAL;
	}

	return ret;
}

static int rt9467_psy_prop_is_writable(struct power_supply *psy,
					enum power_supply_property psp)
{
	switch (psp) {
		case POWER_SUPPLY_PROP_STATUS:
		case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
			return 1;

		default:
			return 0;
	}
}


int rt9467_i2c_charger_setup(struct rt9467_device_info *di)
{
	struct power_supply_desc *psy_desc;
	struct power_supply_config psy_cfg = {
		.of_node = di->dev->of_node,
		.drv_data = di,
	};

	mutex_init(&di->lock);
	dev_info(di->dev, "Init delayed work");
	INIT_DELAYED_WORK(&di->work, rt9467_i2c_charger_poll);

	psy_desc = devm_kzalloc(di->dev, sizeof(*psy_desc), GFP_KERNEL);
	if (!psy_desc)
		return -ENOMEM;
	
	psy_desc->name = di->name;
	psy_desc->type = POWER_SUPPLY_TYPE_MAINS;
	psy_desc->properties = rt9467_charger_properties;
	psy_desc->num_properties = ARRAY_SIZE(rt9467_charger_properties);
	psy_desc->property_is_writeable = rt9467_psy_prop_is_writable;
	psy_desc->get_property = rt9467_psy_get_property;
	psy_desc->set_property = rt9467_psy_set_property;
	
	di->psy = power_supply_register_no_ws(di->dev, psy_desc, &psy_cfg);
	if (IS_ERR(di->psy))
		return dev_err_probe(di->dev, PTR_ERR(di->psy), "failed to register psy\n");

	schedule_delayed_work(&di->work, 1 * HZ);
	return 0;
}
EXPORT_SYMBOL_GPL(rt9467_i2c_charger_setup);

void rt9467_i2c_charger_teardown(struct rt9467_device_info *di)
{
	cancel_delayed_work_sync(&di->work);

	power_supply_unregister(di->psy);
	mutex_destroy(&di->lock);
}
EXPORT_SYMBOL_GPL(rt9467_i2c_charger_teardown);

MODULE_LICENSE("GPL");       // License type
MODULE_AUTHOR("Vladimir Meshkov <glavmonter@gmail.com>");  // Author of the module
MODULE_DESCRIPTION("Richteck RT9467 charger only I2C bus");
MODULE_VERSION("1.0");
