/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale i.MX6Q SabreSD board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __NAD_N110_N180_H
#define __NAD_N110_N180_H

#include "mx6_common.h"
#include <linux/sizes.h>

#define CONFIG_MACH_TYPE	3980
#define CONFIG_MXC_UART_BASE	UART1_BASE

#define CONFIG_MXC_PIC_UART_BASE	UART3_BASE

/* FLASH and environment organization */
#define CONFIG_ENV_SIZE			(8 * 1024)
#define CONFIG_SYS_FSL_USDHC_NUM	1
#ifdef CONFIG_ENV_IS_IN_MMC
#define CONFIG_SYS_MMC_ENV_DEV		0	/* SDHC1 */
#define CONFIG_ENV_OFFSET		(768 * 1024)
#endif

#define CONFIG_SUPPORT_EMMC_BOOT /* eMMC specific */

#ifdef CONFIG_CMD_PCI
#define CONFIG_PCI_SCAN_SHOW
#define CONFIG_PCIE_IMX
#define CONFIG_PCIE_IMX_PERST_GPIO	IMX_GPIO_NR(4, 5)
#define CONFIG_PCIE_IMX_POWER_GPIO	IMX_GPIO_NR(4, 15)
#endif


/* I2C Configs */
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1		/* enable I2C bus 1 */
#define CONFIG_SYS_I2C_MXC_I2C2		/* enable I2C bus 2 */
#define CONFIG_SYS_I2C_MXC_I2C3		/* enable I2C bus 3 */
#define CONFIG_SYS_I2C_SPEED		  100000

/* PMIC */
#define CONFIG_POWER
#define CONFIG_POWER_I2C
#define CONFIG_POWER_PFUZE100
#define CONFIG_POWER_PFUZE100_I2C_ADDR	0x08

/* USB Configs */
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET        /* For OTG port */
#define CONFIG_MXC_USB_PORTSC   (PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS    0

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(10 * SZ_1M)

#define CONFIG_BOARD_LATE_INIT

#define CONFIG_MXC_UART

/* MMC Configs */
#define CONFIG_SYS_FSL_ESDHC_ADDR      0

#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_ETHPRIME			"FEC"
#define CONFIG_FEC_MXC_PHYADDR		4

#if defined(CONFIG_MFG)

#define CONFIG_EXTRA_ENV_SETTINGS \
        "bootargs=" \
	"console=ttymxc0,115200 " \
	"rdinit=/linuxrc " \
	"g_mass_storage.file=/fatfs.bin " \
	"g_mass_storage.removable=1 " \
	"g_mass_storage.stall=0 " \
	"g_mass_storage.idVendor=0x066F " \
	"g_mass_storage.idProduct=0x37FF " \
	"g_mass_storage.iSerialNumber=\"\""

#define CONFIG_BOOTCOMMAND \
	"bootm 0x12000000 0x12C00000 0x18000000"

#else /* not CONFIG_MFG  */

#define CONFIG_EXTRA_ENV_SETTINGS \
        "bootargs=" \
	"console=ttymxc0,115200 " \
	"root=/dev/mmcblk0p1 " \
	"rootwait ro rootfstype=ext4"

#define CONFIG_BOOTCOMMAND \
	"mmc read 0x12000000 0x2000 0x2F00;" \
	"mmc read 0x18000000 0x4F00 0x100;" \
	"bootm 0x12000000 - 0x18000000"

#endif

#define CONFIG_SYS_MEMTEST_START       0x10000000
#define CONFIG_SYS_MEMTEST_END         0x10010000
#define CONFIG_SYS_MEMTEST_SCRATCH     0x10800000

/* Physical Memory Map */
#ifdef NAD_DRAM_HALF_GIG
#define PHYS_SDRAM_SIZE		(1u * 512 * 1024 * 1024)
#else
#define PHYS_SDRAM_SIZE		(1u * 256 * 1024 * 1024)
#endif

#define CONFIG_NR_DRAM_BANKS           1
#define PHYS_SDRAM                     MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE          PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR       IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE       IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

#endif                         /* __NAD_N110_N180_H */
