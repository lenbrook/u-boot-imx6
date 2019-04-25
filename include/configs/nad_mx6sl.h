/*
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale i.MX6SL EVK board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "mx6_common.h"
#include <linux/sizes.h>

#define CONFIG_MACH_TYPE		4307
#define CONFIG_MXC_UART_BASE		UART1_IPS_BASE_ADDR

#define CONFIG_MXC_PIC_UART_BASE	MX6SL_UART3_BASE_ADDR

/* FLASH and environment organization */
#define CONFIG_ENV_SIZE			SZ_8K

#ifdef CONFIG_ENV_IS_IN_MMC
#define CONFIG_SYS_MMC_ENV_DEV		0
#define CONFIG_ENV_OFFSET		(768 * SZ_1K)
#endif

#define CONFIG_SUPPORT_EMMC_BOOT /* eMMC specific */

/* I2C Configs */
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1		/* enable I2C bus 1 */
#define CONFIG_SYS_I2C_MXC_I2C2		/* enable I2C bus 2 */
#define CONFIG_SYS_I2C_MXC_I2C3		/* enable I2C bus 3 */
#define CONFIG_SYS_I2C_SPEED		  100000

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(3 * SZ_1M)

#define CONFIG_BOARD_LATE_INIT

#define CONFIG_MXC_UART

/* MMC Configs */
#define CONFIG_SYS_FSL_ESDHC_ADDR	USDHC1_BASE_ADDR

#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RMII
#define CONFIG_ETHPRIME			"FEC"
#define CONFIG_FEC_MXC_PHYADDR		0

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
        "bootm 0x82000000 0x82C00000 0x88000000"

#else /* not CONFIG_MFG  */

#define CONFIG_EXTRA_ENV_SETTINGS \
        "bootargs=" \
	"console=ttymxc0,115200 " \
	"root=/dev/mmcblk0p1 " \
	"rootwait ro rootfstype=ext4"

#define CONFIG_BOOTCOMMAND \
	"mmc read 0x82000000 0x2000 0x2F00;" \
	"mmc read 0x88000000 0x4F00 0x100;" \
	"bootm 0x82000000 - 0x88000000"

#endif

#define CONFIG_SYS_MEMTEST_START	0x80000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + SZ_512M)

/* Physical Memory Map */
#define PHYS_SDRAM_SIZE			SZ_256M
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

#define CONFIG_MXC_USB_PORTSC              (PORT_PTS_UTMI | PORT_PTS_PTW)


#endif				/* __CONFIG_H */
