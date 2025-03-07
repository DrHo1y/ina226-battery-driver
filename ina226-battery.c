#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/util_macros.h>
#include <linux/regmap.h>

#include <linux/platform_data/ina2xx.h>

/* общие определения регистров */
#define INA2XX_CONFIG			0x00
#define INA2XX_SHUNT_VOLTAGE		0x01 /* readonly */
#define INA2XX_BUS_VOLTAGE		0x02 /* readonly */
#define INA2XX_POWER			0x03 /* readonly */
#define INA2XX_CURRENT			0x04 /* readonly */
#define INA2XX_CALIBRATION		0x05
#define INA226_MASK_ENABLE      	0x06
#define INA226_DIE_ID			0xFF

/* количество регистров */
#define INA226_REGISTERS		8
#define INA2XX_MAX_REGISTERS		8

/* настройки - зависят от варианта использования */
#define INA226_CONFIG_DEFAULT		0x4527	/* среднее=16 */

#define INA2XX_CONVERSION_RATE		15
#define INA2XX_MAX_DELAY		69 /* задержка в мс */

#define INA2XX_RSHUNT_DEFAULT		10000

/* битовая маска для чтения настройки усреднения в регистре конфигурации */
#define INA226_AVG_RD_MASK		0x0E00

#define INA226_READ_AVG(reg)	(((reg) & INA226_AVG_RD_MASK) >> 9)
#define INA226_SHIFT_AVG(val)	((val) << 9)

/*
* Время преобразования напряжения шины и напряжения шунта для ina226 установлено
* на 0b0100 на POR, что в сумме составляет 2200 микросекунд.
*/
#define INA226_TOTAL_CONV_TIME_DEFAULT	2200

/* Добавлено перечисление для идентификаторов устройств */
enum ina2xx_ids { ina226 };

static const struct regmap_config ina226_regmap_config = {
    .reg_bits = 8,
    .val_bits = 16,
};

struct ina2xx_config {
	u16 config_default;
	int calibration_value;
	int registers;
	int shunt_div;
	int bus_voltage_shift;
	int bus_voltage_lsb;	/* uV */
	int power_lsb_factor;
};

struct ina226_data {
    const struct ina2xx_config *config;
    struct regmap *regmap;
    struct power_supply *psy;
    int current_ua;
    int voltage_uv;
    int capacity;
    unsigned long last_update;
    u32 rshunt;
    long current_lsb_uA;
    long power_lsb_uW;
    struct mutex config_lock;
    struct delayed_work work;
    unsigned int poll_interval;
    atomic_t shutdown;
};

static const struct ina2xx_config ina2xx_config1 = {
	.config_default = INA226_CONFIG_DEFAULT,
	.calibration_value = 2048,
	.registers = INA226_REGISTERS,
	.shunt_div = 400,
	.bus_voltage_shift = 0,
	.bus_voltage_lsb = 1250,
	.power_lsb_factor = 25,
};

/* Добавлен прототип функции */
static void ina226_update_values(struct ina226_data *data);


static const int ina226_avg_tab[] = { 1, 4, 16, 64, 128, 256, 512, 1024 };

static irqreturn_t ina226_irq_handler(int irq, void *dev_id)
{
    struct ina226_data *data = dev_id;

    ina226_update_values(data);
    return IRQ_HANDLED;
}

static int ina226_reg_to_interval(u16 config)
{
	int avg = ina226_avg_tab[INA226_READ_AVG(config)];

	/*
    * Умножьте общее время преобразования на количество средних значений.
    * Верните результат в миллисекундах.
    */
	return DIV_ROUND_CLOSEST(avg * INA226_TOTAL_CONV_TIME_DEFAULT, 1000);
}

/*
* Возвращает новое, смещенное значение поля AVG регистра CONFIG,
* для использования с regmap_update_bits
*/
static u16 ina226_interval_to_reg(int interval)
{
	int avg, avg_bits;

	avg = DIV_ROUND_CLOSEST(interval * 1000,
				INA226_TOTAL_CONV_TIME_DEFAULT);
	avg_bits = find_closest(avg, ina226_avg_tab,
				ARRAY_SIZE(ina226_avg_tab));

	return INA226_SHIFT_AVG(avg_bits);
}


/*
* Регистр калибровки установлен на наилучшее значение, что исключает
* ошибки усечения при вычислении текущего регистра в оборудовании.
* Согласно техническому описанию (уравнение 3) наилучшие значения — 2048 для
* ina226 и 4096 для ina219. Они жестко закодированы как значение_калибровки.
*/
static int ina2xx_calibrate(struct ina226_data *data)
{
	return regmap_write(data->regmap, INA2XX_CALIBRATION,
			    data->config->calibration_value);
}

/*
* Инициализация регистров конфигурации и калибровки.
*/
static int ina2xx_init(struct ina226_data *data)
{
	int ret = regmap_write(data->regmap, INA2XX_CONFIG,
			       data->config->config_default);
	if (ret < 0)
		return ret;

	return ina2xx_calibrate(data);
}

static int ina2xx_read_reg(struct ina226_data *data, int reg, unsigned int *regval)
{
	int ret, retry;

	for (retry = 5; retry; retry--) {

		ret = regmap_read(data->regmap, reg, regval);
		if (ret < 0)
			return ret;

		/*
        * Если текущее значение в регистре калибровки равно 0, то
        * регистры мощности и тока также останутся равными 0. В случае
        * сброса чипа давайте проверим регистр калибровки
        * и при необходимости выполним повторную инициализацию.
        * Мы делаем это дополнительное чтение регистра калибровки, если есть
        * намек на сброс чипа.
        */
		if (*regval == 0) {
			unsigned int cal;

			ret = regmap_read(data->regmap, INA2XX_CALIBRATION,
					  &cal);
			if (ret < 0)
				return ret;

			if (cal == 0) {

				ret = ina2xx_init(data);
				if (ret < 0)
					return ret;
				/*
                * Давайте убедимся, что регистры мощности и тока
                * обновлены, прежде чем пытаться
                * снова.
                */
				msleep(INA2XX_MAX_DELAY);
				continue;
			}
		}
		return 0;
	}

	/*
    * Если мы здесь, то, хотя все операции записи прошли успешно, чип
    * все еще возвращает 0 в регистре калибровки. Больше мы
    * ничего не можем сделать здесь.
    */
	return -ENODEV;
}

static void ina226_update_values(struct ina226_data *data)
{
    unsigned int voltage_raw, current_raw;
    int ret;

    /* Считать напряжение шины */
    ret = ina2xx_read_reg(data, INA2XX_BUS_VOLTAGE, &voltage_raw);
    if (!ret) {
        /* 1.25 mV/LSB */
        data->voltage_uv = voltage_raw * 1250;
    }

    /* Read current */
    ret = ina2xx_read_reg(data, INA2XX_CURRENT, &current_raw);
    if (!ret) {
        /* Current LSB = 250 μA (при calibration = 2048 и shunt 0.01 Ohm) */
        data->current_ua = (s16)current_raw * 250;
    }

    /* Упрощенный расчет емкости */
    if (data->voltage_uv > 8400000) {
        data->capacity = 100;
    } else if (data->voltage_uv < 6000000) {
        data->capacity = 0;
    } else {
        data->capacity = (data->voltage_uv - 6000000) * 100 / (8400000 - 6000000);
    }

    data->last_update = jiffies;
}

/*
* Чтобы сохранить фиксированное значение регистра калибровки, произведение
* current_lsb и shunt_resistor также должно быть фиксированным и равным
* shunt_voltage_lsb = 1 / shunt_div, умноженному на 10^9, чтобы
* сохранить масштаб.
*/
static int ina2xx_set_shunt(struct ina226_data *data, long val)
{
	unsigned int dividend = DIV_ROUND_CLOSEST(1000000000,
						  data->config->shunt_div);
	if (val <= 0 || val > dividend)
		return -EINVAL;

	mutex_lock(&data->config_lock);
	data->rshunt = val;
	data->current_lsb_uA = DIV_ROUND_CLOSEST(dividend, val);
	data->power_lsb_uW = data->config->power_lsb_factor *
			     data->current_lsb_uA;
	mutex_unlock(&data->config_lock);

	return 0;
}

static void ina226_work_handler(struct work_struct *work)
{
    struct ina226_data *data = container_of(to_delayed_work(work),
                                  struct ina226_data, work);

    if (atomic_read(&data->shutdown) || !data->psy || !data)
        return;

    mutex_lock(&data->config_lock);
    ina226_update_values(data);
    mutex_unlock(&data->config_lock);

    power_supply_changed(data->psy);

    if (!atomic_read(&data->shutdown))
        schedule_delayed_work(&data->work, msecs_to_jiffies(data->poll_interval));
}

static enum power_supply_property ina226_props[] = {
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_CURRENT_NOW,
    POWER_SUPPLY_PROP_CAPACITY,
};

static int ina226_get_property(struct power_supply *psy,
                               enum power_supply_property psp,
                               union power_supply_propval *val)
{
    struct ina226_data *data = power_supply_get_drvdata(psy);

    switch (psp) {
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        val->intval = data->voltage_uv;
        break;
    case POWER_SUPPLY_PROP_CURRENT_NOW:
        val->intval = data->current_ua;
        break;
    case POWER_SUPPLY_PROP_CAPACITY:
        val->intval = data->capacity;
        break;
    default:
        return -EINVAL;
    }
    return 0;
}

static const struct power_supply_desc ina226_psy_desc = {
    .name           = "ina226-battery",
    .type           = POWER_SUPPLY_TYPE_BATTERY,
    .properties     = ina226_props,
    .num_properties = ARRAY_SIZE(ina226_props),
    .get_property   = ina226_get_property,
};

static int ina226_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct device *dev = &client->dev;
    struct ina226_data *data;
    struct power_supply_config psy_cfg = {};
    u32 val;
    int ret;

    data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;

	data->config = &ina2xx_config1;
    mutex_init(&data->config_lock);

    atomic_set(&data->shutdown, 0);

    if (of_property_read_u32(dev->of_node, "shunt-resistor", &val) < 0) {
		val = INA2XX_RSHUNT_DEFAULT;
	}

    ina2xx_set_shunt(data, val);

    /* Инициализация regmap */
    data->regmap = devm_regmap_init_i2c(client, &ina226_regmap_config);
    if (IS_ERR(data->regmap)) {
		dev_err(dev, "failed to allocate register map\n");
		return PTR_ERR(data->regmap);
    }

    ret = ina2xx_init(data);
	if (ret < 0) {
		dev_err(dev, "error configuring the device: %d\n", ret);
		return -ENODEV;
	}

    /* Инициализация workqueue */
    data->poll_interval= 2000;
    INIT_DELAYED_WORK(&data->work, ina226_work_handler);

    schedule_delayed_work(&data->work, msecs_to_jiffies(data->poll_interval));

    /* Настройка прерывания */
    if (client->irq > 0) {
        ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
                        ina226_irq_handler,
                        IRQF_ONESHOT,
                        "ina226-battery", data);
        if (ret)
            return dev_err_probe(&client->dev, ret, "IRQ failed\n");
    }

    /* Регистрация power_supply */
    psy_cfg.drv_data = data;
    psy_cfg.of_node = client->dev.of_node;
    data->psy = devm_power_supply_register(&client->dev, &ina226_psy_desc, &psy_cfg);
    if (IS_ERR(data->psy))
        return dev_err_probe(&client->dev, PTR_ERR(data->psy),
            "Power supply reg failed\n");

    /* Инициализация значений */
    ina226_update_values(data);

    dev_info(&client->dev, "INA226 initialized (shunt: %u μΩ)\n",
            data->rshunt);
    // Сохраняем данные устройства
    i2c_set_clientdata(client, data);
    return 0;
}

static void ina226_remove(struct i2c_client *client)
{
    struct ina226_data *data = i2c_get_clientdata(client);

    if (!data)
        return;

    data->poll_interval= 0;

    // Устанавливаем флаг завершения
    atomic_set(&data->shutdown, 1);

    cancel_delayed_work_sync(&data->work);

    /* Уничтожение мьютекса */
    mutex_destroy(&data->config_lock);
}

static const struct i2c_device_id ina2xx_id[] = {
	{ "ina226", ina226 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ina2xx_id);

static const struct of_device_id ina226_of_match[] = {
    {
        .compatible = "ti,ina226",
		.data = (void *)ina226
    },
    { }
};

MODULE_DEVICE_TABLE(of, ina226_of_match);

static struct i2c_driver ina226_driver = {
    .driver = {
        .name = "ina226-battery",
        .of_match_table = ina226_of_match,
    },
    .probe = ina226_probe,
    .remove = ina226_remove,
    .id_table	= ina2xx_id,
};
module_i2c_driver(ina226_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("DrHo1y");
MODULE_DESCRIPTION("INA226 Fuel Gauge Driver");
