/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <linux/errno.h>
#include <asm/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/video.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/crm_regs.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <i2c.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
DECLARE_GLOBAL_DATA_PTR;


#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED | \
		      PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define I2C_PMIC	1

#define I2C_PAD MUX_PAD_CTRL(I2C_PAD_CTRL)

int dram_init(void)
{
	gd->ram_size = get_ram_size((void *)PHYS_SDRAM, PHYS_SDRAM_SIZE);

	return 0;
}

iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_SD3_DAT7__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),//console
	MX6_PAD_SD3_DAT6__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),

	MX6_PAD_SD4_CLK__UART3_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),//pic micro
	MX6_PAD_SD4_CMD__UART3_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_EIM_D23__UART3_CTS_B | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_EIM_EB3__UART3_RTS_B | MUX_PAD_CTRL(UART_PAD_CTRL),
#ifdef CONFIG_NAD_BOOT_GPIO_VM300
	MX6_PAD_EIM_D19__GPIO3_IO19 | MUX_PAD_CTRL(NO_PAD_CTRL), // GPIO from PIC32
#endif
#ifdef CONFIG_NAD_BOOT_GPIO_C658
	MX6_PAD_EIM_D28__GPIO3_IO28 | MUX_PAD_CTRL(NO_PAD_CTRL), // GPIO from PIC32
#endif
};

iomux_v3_cfg_t const enet_pads[] = {
	MX6_PAD_ENET_MDIO__ENET_MDIO		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_MDC__ENET_MDC		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TXC__RGMII_TXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD0__RGMII_TD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD1__RGMII_TD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD2__RGMII_TD2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD3__RGMII_TD3	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TX_CTL__RGMII_TX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_REF_CLK__ENET_TX_CLK	| MUX_PAD_CTRL(ENET_PAD_CTRL),

	MX6_PAD_RGMII_RXC__RGMII_RXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD0__RGMII_RD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD1__RGMII_RD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD2__RGMII_RD2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD3__RGMII_RD3	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RX_CTL__RGMII_RX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* PHY Reset */
	MX6_PAD_ENET_CRS_DV__GPIO1_IO25		| MUX_PAD_CTRL(NO_PAD_CTRL),

	/* ADC POWER DOWN */
	MX6_PAD_NANDF_D4__GPIO2_IO04		| MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_iomux_enet(void)
{

	imx_iomux_v3_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));

	/* Reset PHY */
	gpio_direction_output(IMX_GPIO_NR(1, 25) , 0);
	udelay(500);
	gpio_set_value(IMX_GPIO_NR(1, 25), 1);

}

iomux_v3_cfg_t const usdhc1_pads[] = {
	MX6_PAD_SD1_CLK__SD1_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_CMD__SD1_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT0__SD1_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT1__SD1_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT2__SD1_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT3__SD1_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};


static struct i2c_pads_info i2c_pad_info0 = {
	.scl = {
		.i2c_mode = MX6_PAD_CSI0_DAT9__I2C1_SCL | MUX_PAD_CTRL(I2C_PAD),
		.gpio_mode = MX6_PAD_CSI0_DAT9__GPIO5_IO27 | MUX_PAD_CTRL(I2C_PAD),
		.gp = IMX_GPIO_NR(5, 27)
	},
	.sda = {
		.i2c_mode = MX6_PAD_CSI0_DAT8__I2C1_SDA | MUX_PAD_CTRL(I2C_PAD),
		.gpio_mode = MX6_PAD_CSI0_DAT8__GPIO5_IO26 | MUX_PAD_CTRL(I2C_PAD),
		.gp = IMX_GPIO_NR(5, 26)
	}
};

static struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX6_PAD_KEY_COL3__I2C2_SCL | I2C_PAD,
		.gpio_mode = MX6_PAD_KEY_COL3__GPIO4_IO12 | I2C_PAD,
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		.i2c_mode = MX6_PAD_KEY_ROW3__I2C2_SDA | I2C_PAD,
		.gpio_mode = MX6_PAD_KEY_ROW3__GPIO4_IO13 | I2C_PAD,
		.gp = IMX_GPIO_NR(4, 13)
	}
};

#if 0
static struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode = MX6_PAD_EIM_D17__EIM_DATA17 | I2C_PAD,
		.gpio_mode = MX6_PAD_EIM_D17__GPIO3_IO17 | I2C_PAD,
		.gp = IMX_GPIO_NR(3, 17)
	},
	.sda = {
		.i2c_mode = MX6_PAD_EIM_D18__EIM_DATA18 | I2C_PAD,
		.gpio_mode = MX6_PAD_EIM_D18__GPIO3_IO18 | I2C_PAD,
		.gp = IMX_GPIO_NR(3, 18)
	}
};
#endif


iomux_v3_cfg_t const pcie_pads[] = {
	MX6_PAD_KEY_ROW4__GPIO4_IO15 | MUX_PAD_CTRL(0x178b0),/* POWER */
	MX6_PAD_GPIO_19__GPIO4_IO05 | MUX_PAD_CTRL(NO_PAD_CTRL),/* RESET */
};

static void setup_pcie(void)
{
	imx_iomux_v3_setup_multiple_pads(pcie_pads, ARRAY_SIZE(pcie_pads));
}

#if defined(CONFIG_CMD_PCI)
int imx6_pcie_toggle_reset(void)
{
        gpio_direction_output(IMX_GPIO_NR(4, 5), 0);
        mdelay(50);
        gpio_direction_output(IMX_GPIO_NR(4, 5), 1);
        return 0;
}

int imx6_pcie_toggle_power(void)
{
        gpio_direction_output(IMX_GPIO_NR(4, 15), 0);
	mdelay(20);
        gpio_direction_output(IMX_GPIO_NR(4, 15), 1);
	mdelay(20);
	return 0;
}
#endif /* CONFIG_CMD_PCI */


static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

iomux_v3_cfg_t const usb_pads[] = {
	MX6_PAD_SD4_DAT0__GPIO2_IO08 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_WP_B__GPIO6_IO09 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

#ifdef CONFIG_USB_EHCI_MX6
int board_ehci_hcd_init(int port)
{
        imx_iomux_v3_setup_multiple_pads(usb_pads, ARRAY_SIZE(usb_pads));

	/* Power */
        gpio_direction_output(IMX_GPIO_NR(6, 9), 0);
        mdelay(2);
        gpio_set_value(IMX_GPIO_NR(6, 9), 1);

        /* Reset USB hub */
        gpio_direction_output(IMX_GPIO_NR(2, 8), 0);
        mdelay(2);
        gpio_set_value(IMX_GPIO_NR(2, 8), 1);

        return 0;
}


#endif


#ifdef CONFIG_FSL_ESDHC
struct fsl_esdhc_cfg usdhc_cfg[1] = {
	{USDHC1_BASE_ADDR},
};

#define USDHC2_CD_GPIO	IMX_GPIO_NR(2, 2)
#define USDHC3_CD_GPIO	IMX_GPIO_NR(2, 0)

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC1_BASE_ADDR:
		ret = 1; /* eMMC/uSDHC1 is always present */
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
        int ret;
	int i;

	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-boot device node)    (Physical Port)
	 * mmc0                    SD2
	 * mmc1                    SD3
	 * mmc2                    eMMC
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			imx_iomux_v3_setup_multiple_pads(
				usdhc1_pads, ARRAY_SIZE(usdhc1_pads));
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
			       "(%d) then supported by the board (%d)\n",
			       i + 1, CONFIG_SYS_FSL_USDHC_NUM);
			return -EINVAL;
		}

		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
		if (ret)
		    return ret;
	}

	return 0;
}
#endif

#if 0
int mx6_rgmii_rework(struct phy_device *phydev)
{

	/* if necessary, place rx/tx delays here */
	return 0;
}
#endif

int board_phy_config(struct phy_device *phydev)
{

	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}


/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

int board_eth_init(bd_t *bis)
{

	setup_iomux_enet();
	setup_pcie();

	return cpu_eth_init(bis);
}

int board_early_init_f(void)
{
	setup_iomux_uart();

	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info0);
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);

	return 0;
}

static int pfuze_init(void)
{
	struct pmic *p;
	int ret;
	unsigned int reg;

	ret = power_pfuze100_init(I2C_PMIC);
	if (ret)
		return ret;

	p = pmic_get("PFUZE100");
	ret = pmic_probe(p);
	if (ret)
		return ret;

	pmic_reg_read(p, PFUZE100_DEVICEID, &reg);
	printf("PMIC:  PFUZE100 ID=0x%02x\n", reg);

	/*
	 * In the later versions of the board when
	 * a correctly programmed PMIC is used place
	 * all the initial modifications PMIC settings
	 * that uboot has to provide.
	 */

	return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 8 bit bus width */
	{"emmc", MAKE_CFGVAL(0x40, 0x38, 0x00, 0x00)},
	{NULL,	 0},
};
#endif

void get_mac_from_fuses(void)
{
    unsigned char mac[6];

    imx_get_mac_from_fuse(0, mac);
    eth_env_set_enetaddr("ethaddr", mac);
}

int board_late_init(void)
{
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif
	pfuze_init();
	get_mac_from_fuses();

	return 0;
}

int checkboard(void)
{
	puts("Board: NAD NMB\n");
	return 0;
}
