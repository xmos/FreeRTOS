
TARGET = XCORE-AI-EXPLORER

APP_NAME = RTOSDemo

BASE_DIR = $(abspath .)

DEMO_ROOT = src
KERNEL_ROOT = .././.././.././Source
COMMON_DEMO_ROOT = .././.././Common
PORTABLE_ROOT = $(KERNEL_ROOT)/portable
MEMMANG_ROOT = $(PORTABLE_ROOT)/MemMang
XCC_PORT_ROOT = $(PORTABLE_ROOT)/XCC/XCOREAI

SOURCE_DIRS = $(DEMO_ROOT) $(DEMO_ROOT)/IntQueueTimer $(DEMO_ROOT)/partest $(DEMO_ROOT)/regtest $(COMMON_DEMO_ROOT)/Minimal $(KERNEL_ROOT) $(XCC_PORT_ROOT) $(MEMMANG_ROOT)
INCLUDE_DIRS = $(DEMO_ROOT) $(DEMO_ROOT)/IntQueueTimer $(DEMO_ROOT)/regtest $(KERNEL_ROOT)/include $(XCC_PORT_ROOT) $(COMMON_DEMO_ROOT)/include

EXCLUDE_FILES += ../../../Source/portable/MemMang/heap_1.c  \
                 ../../../Source/portable/MemMang/heap_2.c  \
                 ../../../Source/portable/MemMang/heap_3.c  \
                 ../../../Source/portable/MemMang/heap_5.c  \
                 ../../Common/Minimal/comtest.c             \
                 ../../Common/Minimal/comtest_strings.c     \
                 ../../Common/Minimal/crflash.c             \
                 ../../Common/Minimal/crhook.c              \
                 ../../Common/Minimal/flash.c               \
                 ../../Common/Minimal/flash_timer.c         \
                 ../../Common/Minimal/MessageBufferAMP.c    \
                 ../../Common/Minimal/sp_flop.c             \
                 ../../Common/Minimal/StaticAllocation.c
                 
                 

XCC_FLAGS = -O2 -g -report -fxscope

USED_MODULES = lib_rtos_support

VERBOSE = 0

XMOS_MAKE_PATH ?= ../..
-include $(XMOS_MAKE_PATH)/xcommon/module_xcommon/build/Makefile.common

