# make
# make flash
# make test
# make test ESPPORT=/dev/ttyUSB0
PROGRAM=Artery
EXTRA_COMPONENTS = extras/i2s_dma extras/ws2812_i2s
include $(espopenrtos)
