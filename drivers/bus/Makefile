#
# Makefile for the bus drivers.
#

obj-$(CONFIG_MVEBU_MBUS) += mvebu-mbus.o
obj-$(CONFIG_OMAP_OCP2SCP)	+= omap-ocp2scp.o

# Interconnect bus driver for OMAP SoCs.
obj-$(CONFIG_OMAP_INTERCONNECT)	+= omap_l3_smx.o omap_l3_noc.o
# CCI cache coherent interconnect for ARM platforms
obj-$(CONFIG_ARM_CCI)		+= arm-cci.o
obj-$(CONFIG_SUNXI_MBUS)	+= sunxi_mbus.o
