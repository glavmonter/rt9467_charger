include Kbuild

OBJ_FILE := $(obj-m)
SRC_FILE := $(OBJ_FILE:.o=.c)
CMD_FILE := .$(OBJ_FILE).cmd
MODNAME := $(OBJ_FILE:.o=)

dt:
	dtc -@ -q -I dts -O dtb -o rt9467_charger_i2c.dtbo rt9467_charger_i2c.dts

all:
	make -C ${KDIR} M=$(PWD) ARCH=arm64 CROSS_COMPILE=aarch64-none-linux-gnu- modules

clean:
	make -C ${KDIR} M=$(PWD) clean

