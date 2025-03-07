Первая рабочая версия драйвера заряда батареи на основе ina226

```
mkdir driver
cd driver
nano Makefile // paste code
nano ina226-battery.c // paste code
make
cp ina226-battery.ko /lib/modules/6.1.43-rockchip-rk3588/
nano /etc/rc.local 
// paste in end 
// insmod /lib/modules/6.1.43-rockchip-rk3588/ina226-battery
```
Для включения драйвера в автозагрузку выполните следующие шаги:
```
echo ina226_battery >> /etc/modules-load.d/modules.conf
cp ina226-battery.ko /lib/modules/6.1.43-rockchip-rk3588/extra/
depmod -a
```