APPLICATION = test-soniclib
BOARD ?= lora3a-h10
RIOTBASE ?= $(CURDIR)/../RIOT
LORA3ABASE ?= $(CURDIR)/../lora3a-boards
EXTERNAL_BOARD_DIRS=$(LORA3ABASE)/boards
EXTERNAL_MODULE_DIRS=$(LORA3ABASE)/modules
EXTERNAL_PKG_DIRS=$(LORA3ABASE)/pkg
DEVELHELP ?= 1
QUIET ?= 1
PORT ?= /dev/ttyUSB0

USEMODULE += saml21_cpu_debug
USEMODULE += printf_float
USEMODULE += saul_default
USEPKG += soniclib

#CFLAGS += -DCHDRV_DEBUG=1

include $(RIOTBASE)/Makefile.include
