#include $(srctree)/drivers/misc/mediatek/imgsensor/src/Makefile.custom

# Linux driver folder
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/mach/
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/

#ccflags-y += -I$(srctree)/arch/arm/mach-$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/mtk_gslX680/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/gsl3670/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
#ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/unified_driver_4/inc
#ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/power/mt6580
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/alsps/inc
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/hwmon/include/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/alsps/

obj-y	+=  mtk_gsl3670.o
obj-y	+=  gsl_ReadArrayData.o
#chy add 20150406
#obj-y   +=  gsl_point_id.o 

obj-y:=gsl_point_id.o
$(obj)/gsl_point_id.o: $(srctree)/$(obj)/gsl_point_id
	cp $(srctree)/$(obj)/gsl_point_id $(obj)/gsl_point_id.o

OBJS = $(patsubst %.c,%.o,$(shell ls $(srctree)/$(obj)/*.c))
obj-y += $(subst $(srctree)/$(obj)/, ,$(OBJS))

