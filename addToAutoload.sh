#!/bin/bash

echo ina226_battery >> /etc/modules-load.d/modules.conf
mkdir -p /lib/modules/6.1.43-rockchip-rk3588/extra/
cp ina226-battery.ko /lib/modules/6.1.43-rockchip-rk3588/extra/
depmod -a
