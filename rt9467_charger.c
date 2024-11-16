#include <linux/kernel.h>    // kernel header
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
#define RT9467_REG_HIGH_ADDRESS		RT9467_REG_CHG_IRQ1_CTRL

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
	LINEAR_RANGE_IDX(RT9467_RANGE_ICHG, 100000, 0x00, 0x31, 100000),
};


enum rt9467_fields {
	F_RST,
	F_CHG_STAT, 		///< Состояние зарядки, только чтение
	F_CHG_EN,			///< Включение и выключение зарядки, только запись. 0 - отключено, 1 - включено (по умолчанию)
	F_CFO_EN,			///< Включение и выключение DCDC
	F_DCDC_EN,			///< Включение и выключение зарядки вообще
	F_SHIP_MODE,
	F_PWR_RDY,  		///< Состояние подключенного адаптера, только чтение
	F_ICHG,				///< Ток заряда, чтение - запись
	F_MAX_FIELDS,
};

static const struct reg_field rt9467_rm_fields[] = {
	[F_RST] 			= REG_FIELD(RT9467_REG_CORE_CTRL0, 7, 7),
	[F_CHG_STAT] 		= REG_FIELD(RT9467_REG_CHG_STAT, 6, 7),
	[F_CHG_EN]			= REG_FIELD(RT9467_REG_CHG_CTRL2, 0, 0),
	[F_SHIP_MODE]		= REG_FIELD(RT9467_REG_CHG_CTRL2, 7, 7),
	[F_CFO_EN]			= REG_FIELD(RT9467_REG_CHG_CTRL2, 1, 1),
	[F_DCDC_EN]     	= REG_FIELD(RT9467_REG_CHG_CTRL2, 0, 1),
	[F_PWR_RDY]			= REG_FIELD(RT9467_REG_CHG_STATC, 7, 7),
	[F_ICHG]			= REG_FIELD(RT9467_REG_CHG_CTRL7, 2, 7),
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
		dev_dbg_ratelimited(di->dev, "failed to read register 0x%02X\n", reg);
	return ret;
}

static inline int rt9467_write(struct rt9467_device_info *di, u8 reg, int data)
{
	int ret;
	if (!di)
		return -EINVAL;
	ret = di->bus.write(di, reg, data);
	if (ret < 0)
		dev_dbg(di->dev, "failed to write register 0x%02X\n", reg);
	return ret;
}

static bool update_if_changed(struct rt9467_device_info *di, u8 reg) {	
	int r = rt9467_read(di, reg);
	if (!r)
		dev_dbg(di->dev, "Read register 0x%02X failed\n", reg);

	if (di->cached_registers[reg] != r) {
		di->cached_registers[reg] = r;
		return true;
	} else {
		return false;
	}
}

void rt9467_charger_update(struct rt9467_device_info *di) {
	bool is_changed = false;
	dev_dbg_ratelimited(di->dev, "Update cached registers\n");

	is_changed |= update_if_changed(di, RT9467_REG_CHG_CTRL2);
	is_changed |= update_if_changed(di, RT9467_REG_CHG_STAT);
	is_changed |= update_if_changed(di, RT9467_REG_CHG_CTRL7);
	is_changed |= update_if_changed(di, RT9467_REG_CHG_STATC);

	if (is_changed)
		power_supply_changed(di->psy);
		
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

static unsigned int rt9467_set_value_from_ranges(enum rt9467_ranges rsel, int value)
{
	const struct linear_range *range = rt9467_ranges + rsel;
	unsigned int sel;
	bool found;
	int ret;
	ret = linear_range_get_selector_high(range, value, &sel, &found);
	if (ret)
		sel = range->max_sel;
	
	return sel;
}

static int rt9467_psy_get_status(struct rt9467_device_info *di, int *state)
{
	unsigned int status;
	int ret;

	ret = regmap_field_read(di->rm_fields[F_CHG_STAT], &status);
	if (ret)
		return ret;

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

static int rt9467_psy_get_charge_current(struct rt9467_device_info *di, int *microamps)
{
	int mapped_value;
	int ret;
	unsigned int reg;
	ret = regmap_field_read(di->rm_fields[F_ICHG], &reg);
	if (ret)
		return ret;

	rt9467_get_value_from_ranges(di, reg, RT9467_RANGE_ICHG, &mapped_value);
	dev_dbg(di->dev, "charge reg: 0x%02X -> %d\n", reg, mapped_value);
	*microamps = mapped_value;
	return 0;
}

static int rt9467_psy_set_charge_current(struct rt9467_device_info *di, int microamps)
{
	unsigned int sel;

	if (microamps < 100000) {
		dev_warn(di->dev, "Minimum value of charge current must be 100000 uA\n");
		microamps = 100000;
	}

	sel = rt9467_set_value_from_ranges(RT9467_RANGE_ICHG, microamps);
	dev_info(di->dev, "set_current to %d, sel: 0x%02X\n", microamps, sel);
	
	di->need_update = 1;
	return regmap_field_write(di->rm_fields[F_ICHG], sel);
}

static int rt9467_psy_set_status(struct rt9467_device_info *di, int status)
{
	if (status > 0)
		status = 1;
	else
		status = 0;
	dev_info(di->dev, "Set status to %d\n", status);
	
	di->need_update = 1;
	return regmap_field_write(di->rm_fields[F_CHG_EN], status);
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
			regmap_field_read(di->rm_fields[F_PWR_RDY], &val->intval);
			return 0; 

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
	case POWER_SUPPLY_PROP_STATUS:
		return rt9467_psy_set_status(di, val->intval);

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

static int rt9467_regmap_read(void *context, unsigned int reg, unsigned int *val)
{
	struct rt9467_device_info *di = context;
	if (!di->need_update) {
		*val = di->cached_registers[reg];
		return 0;
	}

	update_if_changed(di, reg);
	dev_info(di->dev, "Read register 0x%02X before write. val: 0x%02X\n", reg, di->cached_registers[reg]);
	di->need_update = false;

	*val = di->cached_registers[reg];
	return 0;
}

static int rt9467_regmap_write(void *context, unsigned int reg, unsigned int val)
{
	struct rt9467_device_info *di = context;
	dev_info(di->dev, "Regmap write register %d => %d\n", reg, val);
	return rt9467_write(di, reg, val);
}

static const struct regmap_config rt9467_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = RT9467_REG_HIGH_ADDRESS + 1,
	.reg_read = rt9467_regmap_read,
	.reg_write = rt9467_regmap_write,
	.cache_type = REGCACHE_NONE,
};

static ssize_t sysoff_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct rt9467_device_info *di = power_supply_get_drvdata(to_power_supply(dev));
	unsigned int sysoff_enable;
	int ret = regmap_field_read(di->rm_fields[F_SHIP_MODE], &sysoff_enable);
	if (ret)
		return ret;
	dev_info(di->dev, "sysoff: %d\n", sysoff_enable);
	return sysfs_emit(buf, "%d\n", sysoff_enable);
}

static ssize_t sysoff_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct rt9467_device_info *di = power_supply_get_drvdata(to_power_supply(dev));
	unsigned int tmp;
	int ret;

	ret = kstrtouint(buf, 10, &tmp);
	if (ret)
		return ret;

	dev_info(di->dev, "sysoff_enable: %d\n", !!tmp);

	mutex_lock(&di->lock);
	di->need_update = 1;
	ret = regmap_field_write(di->rm_fields[F_SHIP_MODE], !!tmp);
	mutex_unlock(&di->lock);

	if (ret)
		return ret;

	return count;
}

static DEVICE_ATTR_RW(sysoff_enable);

static struct attribute *rt9467_sysfs_attrs[] = {
	&dev_attr_sysoff_enable.attr,
	NULL
};
ATTRIBUTE_GROUPS(rt9467_sysfs);


int rt9467_i2c_charger_setup(struct rt9467_device_info *di)
{
	int i;
	struct device *dev = di->dev;
	struct power_supply_desc *psy_desc;
	struct power_supply_config psy_cfg = {
		.of_node = di->dev->of_node,
		.drv_data = di,
		.attr_grp = rt9467_sysfs_groups,
	};

	mutex_init(&di->lock);
	dev_info(dev, "Init delayed work");
	INIT_DELAYED_WORK(&di->work, rt9467_i2c_charger_poll);

	di->cached_registers = devm_kcalloc(dev, RT9467_REG_HIGH_ADDRESS + 1, sizeof(*di->cached_registers), GFP_KERNEL);
	if (IS_ERR(di->cached_registers))
		return dev_err_probe(dev, PTR_ERR(di->cached_registers), "Cannot allocate cache\n");

	psy_desc = devm_kzalloc(dev, sizeof(*psy_desc), GFP_KERNEL);
	if (IS_ERR(psy_desc))
		return dev_err_probe(dev, PTR_ERR(psy_desc), "Cannot allocate psy_desc\n");
	
	di->regmap = devm_regmap_init(dev, NULL, di, &rt9467_regmap_config);
	if (IS_ERR(di->regmap))
		return dev_err_probe(dev, PTR_ERR(di->regmap), "Cannot allocate regmap structure\n");
	
	di->rm_fields = devm_kcalloc(dev, ARRAY_SIZE(rt9467_rm_fields), sizeof(*di->rm_fields), GFP_KERNEL);
	if (IS_ERR(di->rm_fields))
		return dev_err_probe(dev, PTR_ERR(di->rm_fields), "Cannot allocate regmap fields container\n");
	for (i = 0; i < ARRAY_SIZE(rt9467_rm_fields); i++) {
		di->rm_fields[i] = devm_regmap_field_alloc(dev, di->regmap, rt9467_rm_fields[i]);
		if (IS_ERR(di->rm_fields))
			return dev_err_probe(dev, PTR_ERR(di->rm_fields[i]), "Cannot allocate regmap fiend %d\n", i);
	}

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

	schedule_delayed_work(&di->work, 3 * HZ);
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
