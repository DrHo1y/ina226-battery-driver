# Указываем имя модуля
obj-m += ina226-battery.o
# Путь к исходным файлам ядра
KDIR := /lib/modules/6.1.43-rockchip-rk3588/build

# Текущая директория
PWD := $(shell pwd)

# Цель по умолчанию
all:
	make -C $(KDIR) M=$(PWD) modules

# Очистка
clean:
	make -C $(KDIR) M=$(PWD) clean

# Установка модуля
install:
	sudo insmod ina226-battery.ko

# Удаление модуля
uninstall:
	sudo rmmod ina226-battery

# Перезагрузка модуля
reload:
	uninstall install

# Пересборка и перезагрузка
rebuild:
	clean all reload

.PHONY: all clean install uninstall reload rebuild
