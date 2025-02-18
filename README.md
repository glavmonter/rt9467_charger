# RT9467 Charger DCDC

Экспортирует в `/sys/class/power_supply/rt9467-i2c/`:

* `online` - флаг подключения зарядного устройства
* `constant_charge_current` - значение зарядного тока, мкА
* `status` - состояние Зарядка, Нет зарядки, Неизвестно
* `sysoff_enable_delayed` - запись 1 отключает питание через 10 секунд
* `sysoff_enable` - запись 1 отключает питание немедленно
