# archivo meson para sAPI

# variables globales
sapi_srcs          =  []
sapi_include_dirs  =  []

sapi_include_dirs += include_directories('sapi/libs/CMSIS/Include')
sapi_include_dirs += include_directories('sapi/libs/Device/SiliconLabs/EFM32HG/Include')
sapi_include_dirs += include_directories('sapi/libs/emlib/inc')
sapi_include_dirs += include_directories('sapi/libs/middleware/usb_gecko/inc')
sapi_include_dirs += include_directories('sapi/libs/middleware/usbxpress/inc')
sapi_include_dirs += include_directories('sapi')

# NOTE(elsuizo:2019-03-24): esto lo hacemos a pedal con el script de python `list_all_c_files.py` pero me parece medio trucho
sapi_srcs += ['sapi/src/InitDevice.c']
sapi_srcs += ['sapi/src/emodes.c']
sapi_srcs += ['sapi/src/moveIntVectorToRAM.c']
sapi_srcs += ['sapi/src/sapi_adc.c']
sapi_srcs += ['sapi/src/sapi_board.c']
sapi_srcs += ['sapi/src/sapi_circularBuffer.c']
sapi_srcs += ['sapi/src/sapi_clock.c']
sapi_srcs += ['sapi/src/sapi_core.c']
sapi_srcs += ['sapi/src/sapi_datatypes.c']
sapi_srcs += ['sapi/src/sapi_delay.c']
sapi_srcs += ['sapi/src/sapi_gpio.c']
sapi_srcs += ['sapi/src/sapi_i2c.c']
sapi_srcs += ['sapi/src/sapi_print.c']
sapi_srcs += ['sapi/src/sapi_rtc.c']
sapi_srcs += ['sapi/src/sapi_tick.c']
sapi_srcs += ['sapi/src/sapi_uart.c']

# NOTE(elsuizo:2019-03-24): me faltaba esto para que ande
sapi_srcs = files(sapi_srcs)
