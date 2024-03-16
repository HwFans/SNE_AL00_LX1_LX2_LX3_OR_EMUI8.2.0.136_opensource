ifeq ($(strip $(llt_gcov)),y)
GCOV_PROFILE := y
endif

-include $(srctree)/drivers/hisi/modem/config/product/$(OBB_PRODUCT_NAME)/config/balong_product_config.mk
obj-y += s_memory.o

ifeq ($(strip $(CFG_CONFIG_MALLOC_UNIFIED)),YES)
obj-y += malloc_interface.o
endif
obj-$(CONFIG_S_MEMORY_TEST) += s_memory_test.o
