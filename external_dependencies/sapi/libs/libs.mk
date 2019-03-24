# from https://stackoverflow.com/a/18258352
rwildcard=$(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2) $(filter $(subst *,%,$2),$d))

# from https://stackoverflow.com/a/16151140
uniq = $(if $1,$(firstword $1) $(call uniq,$(filter-out $(firstword $1),$1)))

# from  https://stackoverflow.com/a/18137056
mkfile_path := $(abspath $(lastword $(MAKEFILE_LIST)))
SELF := $(notdir $(patsubst %/,%,$(dir $(mkfile_path))))

LIBS_MOD += CMSIS
LIBS_MOD += Device
LIBS_MOD += emlib
ifeq ($(USE_USB),y)
$(info using EFM32 usb gecko libraries)
LIBS_MOD += middleware/usb_gecko
LIBS_MOD += middleware/usbxpress
endif

LIBS_INC := $(foreach m, $(LIBS_MOD), \
    $(call uniq, $(dir $(call rwildcard, $(SELF)/$(m), *.h))))
LIBS_C_SRC := $(foreach m, $(LIBS_MOD), \
    $(call rwildcard, $(SELF)/$(m), *.c))

C_SRC += $(LIBS_C_SRC)

LD_SCRIPT := $(SELF)/Device/SiliconLabs/EFM32HG/Source/GCC/efm32hg.ld
OOCD_CFG  := $(SELF)/openocd.cfg

ARCH=-mcpu=cortex-m0 -mthumb

LIBS_CFLAGS := $(foreach i, $(LIBS_INC), -I$(i))
CFLAGS += $(LIBS_CFLAGS) -DEFM32HG322F64=1
