/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef _BOARD_MX6Q_SABRESD_H
#define _BOARD_MX6Q_SABRESD_H
#include <mach/iomux-mx6q.h>

static iomux_v3_cfg_t mx6q_sabresd_pads[] = {
	/* AUDMUX */
	MX6Q_PAD_CSI0_DAT4__AUDMUX_AUD3_TXC,
	MX6Q_PAD_CSI0_DAT5__AUDMUX_AUD3_TXD,
	MX6Q_PAD_CSI0_DAT6__AUDMUX_AUD3_TXFS,
	MX6Q_PAD_CSI0_DAT7__AUDMUX_AUD3_RXD,

	/* CAN1  */
	MX6Q_PAD_KEY_ROW2__CAN1_RXCAN,
	/* MX6Q_PAD_KEY_COL2__CAN1_TXCAN, */
	MX6Q_PAD_GPIO_2__GPIO_1_2,		/* STNDBY */
	MX6Q_PAD_GPIO_7__GPIO_1_7,		/* NERR */

	/* CCM  */
	MX6Q_PAD_GPIO_0__CCM_CLKO,		/* SGTL500 sys_mclk */
	MX6Q_PAD_GPIO_3__CCM_CLKO2,		/* J5 - Camera MCLK */

	/* ECSPI1 */
	MX6Q_PAD_KEY_COL0__ECSPI1_SCLK,
	MX6Q_PAD_KEY_ROW0__ECSPI1_MOSI,
	MX6Q_PAD_KEY_COL1__ECSPI1_MISO,
	MX6Q_PAD_KEY_ROW1__GPIO_4_9,
	/* ENET */
	MX6Q_PAD_ENET_MDIO__ENET_MDIO,
	MX6Q_PAD_ENET_MDC__ENET_MDC,
	MX6Q_PAD_RGMII_TXC__ENET_RGMII_TXC,
	MX6Q_PAD_RGMII_TD0__ENET_RGMII_TD0,
	MX6Q_PAD_RGMII_TD1__ENET_RGMII_TD1,
	MX6Q_PAD_RGMII_TD2__ENET_RGMII_TD2,
	MX6Q_PAD_RGMII_TD3__ENET_RGMII_TD3,
	MX6Q_PAD_RGMII_TX_CTL__ENET_RGMII_TX_CTL,
	MX6Q_PAD_ENET_REF_CLK__ENET_TX_CLK,
	MX6Q_PAD_RGMII_RXC__ENET_RGMII_RXC,
	MX6Q_PAD_RGMII_RD0__ENET_RGMII_RD0,
	MX6Q_PAD_RGMII_RD1__ENET_RGMII_RD1,
	MX6Q_PAD_RGMII_RD2__ENET_RGMII_RD2,
	MX6Q_PAD_RGMII_RD3__ENET_RGMII_RD3,
	MX6Q_PAD_RGMII_RX_CTL__ENET_RGMII_RX_CTL,
	MX6Q_PAD_ENET_TX_EN__GPIO_1_28,		/* Micrel RGMII Phy Interrupt */
	MX6Q_PAD_EIM_D23__GPIO_3_23,		/* RGMII reset */
	MX6Q_PAD_GPIO_16__ENET_ANATOP_ETHERNET_REF_OUT, /* Internal connect for 1588 TS Clock */

	/* GPIO1 */
	MX6Q_PAD_ENET_RX_ER__GPIO_1_24,		/* J9 - Microphone Detect */

	/* GPIO2 */
	/* MX6Q_PAD_NANDF_D1__GPIO_2_1,*/	/* J14 - Menu Button */
	/* MX6Q_PAD_NANDF_D2__GPIO_2_2,*/	/* J14 - Back Button */
	/* MX6Q_PAD_NANDF_D3__GPIO_2_3,*/	/* J14 - Search Button */
	/* MX6Q_PAD_NANDF_D4__GPIO_2_4,*/	/* J14 - Home Button */
	MX6Q_PAD_EIM_A22__GPIO_2_16,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A21__GPIO_2_17,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A20__GPIO_2_18,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A19__GPIO_2_19,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A18__GPIO_2_20,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A17__GPIO_2_21,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A16__GPIO_2_22,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_RW__GPIO_2_26,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_LBA__GPIO_2_27,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_EB0__GPIO_2_28,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_EB1__GPIO_2_29,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_EB3__GPIO_2_31,	/* J12 - Boot Mode Select */

	/* GPIO3 */
	MX6Q_PAD_EIM_DA0__GPIO_3_0,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA1__GPIO_3_1,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA2__GPIO_3_2,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA3__GPIO_3_3,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA4__GPIO_3_4,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA5__GPIO_3_5,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA6__GPIO_3_6,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA7__GPIO_3_7,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA8__GPIO_3_8,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA9__GPIO_3_9,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA10__GPIO_3_10,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA11__GPIO_3_11,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA12__GPIO_3_12,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA13__GPIO_3_13,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA14__GPIO_3_14,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA15__GPIO_3_15,	/* J12 - Boot Mode Select */

	/* SW4 , SW5 & SW1 */
	MX6Q_PAD_GPIO_4__GPIO_1_4,	/* Volume Up */
	MX6Q_PAD_GPIO_5__GPIO_1_5,	/* Volume Down */
	MX6Q_PAD_EIM_D29__GPIO_3_29,	/* power off */

	/* CAP_TCH_INT1 */
	MX6Q_PAD_NANDF_CLE__GPIO_6_7,

	/* CAP_TCH_INT0 */
	MX6Q_PAD_NANDF_ALE__GPIO_6_8,

	/* eCompass int */
	MX6Q_PAD_EIM_D16__GPIO_3_16,

	/* GPIO5 */
	MX6Q_PAD_EIM_WAIT__GPIO_5_0,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A24__GPIO_5_4,	/* J12 - Boot Mode Select */

	/* GPIO6 */
	MX6Q_PAD_EIM_A23__GPIO_6_6,	/* J12 - Boot Mode Select */
	MX6Q_PAD_NANDF_RB0__GPIO_6_10, /* AUX_5V Enable */

	/* I2C1, WM8958 */
	MX6Q_PAD_CSI0_DAT8__I2C1_SDA,
	MX6Q_PAD_CSI0_DAT9__I2C1_SCL,

	/* I2C2 Camera, MIPI */
	MX6Q_PAD_KEY_COL3__I2C2_SCL,	/* GPIO4[12] */
	MX6Q_PAD_KEY_ROW3__I2C2_SDA,	/* GPIO4[13] */

	/* I2C3 */
	MX6Q_PAD_GPIO_3__I2C3_SCL,	/* GPIO1[3] */
	MX6Q_PAD_GPIO_6__I2C3_SDA,

	/* DISPLAY */
	MX6Q_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK,
	MX6Q_PAD_DI0_PIN15__IPU1_DI0_PIN15,		/* DE */
	MX6Q_PAD_DI0_PIN2__IPU1_DI0_PIN2,		/* HSync */
	MX6Q_PAD_DI0_PIN3__IPU1_DI0_PIN3,		/* VSync */
	MX6Q_PAD_DI0_PIN4__IPU1_DI0_PIN4,		/* Contrast */
	MX6Q_PAD_DISP0_DAT0__IPU1_DISP0_DAT_0,
	MX6Q_PAD_DISP0_DAT1__IPU1_DISP0_DAT_1,
	MX6Q_PAD_DISP0_DAT2__IPU1_DISP0_DAT_2,
	MX6Q_PAD_DISP0_DAT3__IPU1_DISP0_DAT_3,
	MX6Q_PAD_DISP0_DAT4__IPU1_DISP0_DAT_4,
	MX6Q_PAD_DISP0_DAT5__IPU1_DISP0_DAT_5,
	MX6Q_PAD_DISP0_DAT6__IPU1_DISP0_DAT_6,
	MX6Q_PAD_DISP0_DAT7__IPU1_DISP0_DAT_7,
	MX6Q_PAD_DISP0_DAT8__IPU1_DISP0_DAT_8,
	MX6Q_PAD_DISP0_DAT9__IPU1_DISP0_DAT_9,
	MX6Q_PAD_DISP0_DAT10__IPU1_DISP0_DAT_10,
	MX6Q_PAD_DISP0_DAT11__IPU1_DISP0_DAT_11,
	MX6Q_PAD_DISP0_DAT12__IPU1_DISP0_DAT_12,
	MX6Q_PAD_DISP0_DAT13__IPU1_DISP0_DAT_13,
	MX6Q_PAD_DISP0_DAT14__IPU1_DISP0_DAT_14,
	MX6Q_PAD_DISP0_DAT15__IPU1_DISP0_DAT_15,
	MX6Q_PAD_DISP0_DAT16__IPU1_DISP0_DAT_16,
	MX6Q_PAD_DISP0_DAT17__IPU1_DISP0_DAT_17,
	MX6Q_PAD_DISP0_DAT18__IPU1_DISP0_DAT_18,
	MX6Q_PAD_DISP0_DAT19__IPU1_DISP0_DAT_19,
	MX6Q_PAD_DISP0_DAT20__IPU1_DISP0_DAT_20,
	MX6Q_PAD_DISP0_DAT21__IPU1_DISP0_DAT_21,
	MX6Q_PAD_DISP0_DAT22__IPU1_DISP0_DAT_22,
	MX6Q_PAD_DISP0_DAT23__IPU1_DISP0_DAT_23,
	MX6Q_PAD_GPIO_7__GPIO_1_7,		/* J7 - Display Connector GP */
	MX6Q_PAD_GPIO_9__GPIO_1_9,		/* J7 - Display Connector GP */
	/* MX6Q_PAD_NANDF_D0__GPIO_2_0,*/	/* J6 - LVDS Display contrast */

	/* DISP_PWM */
	MX6Q_PAD_SD1_DAT3__PWM1_PWMO,		/* GPIO1[21] */

	/* UART1 for debug */
	MX6Q_PAD_CSI0_DAT10__UART1_TXD,
	MX6Q_PAD_CSI0_DAT11__UART1_RXD,

	/* UART3 for gps */
	MX6Q_PAD_EIM_D24__UART3_TXD,
	MX6Q_PAD_EIM_D25__UART3_RXD,

	/* USBOTG ID pin */
	MX6Q_PAD_ENET_RX_ER__ANATOP_USBOTG_ID,

	/* USB power pin */
	MX6Q_PAD_EIM_D22__GPIO_3_22,
	MX6Q_PAD_ENET_TXD1__GPIO_1_29,

	/* USB OC pin */
	MX6Q_PAD_EIM_D21__USBOH3_USBOTG_OC,
	MX6Q_PAD_EIM_D30__USBOH3_USBH1_OC,

	/* USDHC2 */
	MX6Q_PAD_SD2_CLK__USDHC2_CLK,
	MX6Q_PAD_SD2_CMD__USDHC2_CMD,
	MX6Q_PAD_SD2_DAT0__USDHC2_DAT0,
	MX6Q_PAD_SD2_DAT1__USDHC2_DAT1,
	MX6Q_PAD_SD2_DAT2__USDHC2_DAT2,
	MX6Q_PAD_SD2_DAT3__USDHC2_DAT3,
	MX6Q_PAD_NANDF_D4__USDHC2_DAT4,
	MX6Q_PAD_NANDF_D5__USDHC2_DAT5,
	MX6Q_PAD_NANDF_D6__USDHC2_DAT6,
	MX6Q_PAD_NANDF_D7__USDHC2_DAT7,
	MX6Q_PAD_NANDF_D2__GPIO_2_2,		/* SD2_CD */
	MX6Q_PAD_NANDF_D3__GPIO_2_3,		/* SD2_WP */

	/* USDHC3 */
	MX6Q_PAD_SD3_CLK__USDHC3_CLK_50MHZ,
	MX6Q_PAD_SD3_CMD__USDHC3_CMD_50MHZ,
	MX6Q_PAD_SD3_DAT0__USDHC3_DAT0_50MHZ,
	MX6Q_PAD_SD3_DAT1__USDHC3_DAT1_50MHZ,
	MX6Q_PAD_SD3_DAT2__USDHC3_DAT2_50MHZ,
	MX6Q_PAD_SD3_DAT3__USDHC3_DAT3_50MHZ,
	MX6Q_PAD_SD3_DAT4__USDHC3_DAT4_50MHZ,
	MX6Q_PAD_SD3_DAT5__USDHC3_DAT5_50MHZ,
	MX6Q_PAD_SD3_DAT6__USDHC3_DAT6_50MHZ,
	MX6Q_PAD_SD3_DAT7__USDHC3_DAT7_50MHZ,
	MX6Q_PAD_NANDF_D0__GPIO_2_0,		/* SD3_CD */
	MX6Q_PAD_NANDF_D1__GPIO_2_1,		/* SD3_WP */

	/* USDHC4 */
	MX6Q_PAD_SD4_CLK__USDHC4_CLK_50MHZ,
	MX6Q_PAD_SD4_CMD__USDHC4_CMD_50MHZ,
	MX6Q_PAD_SD4_DAT0__USDHC4_DAT0_50MHZ,
	MX6Q_PAD_SD4_DAT1__USDHC4_DAT1_50MHZ,
	MX6Q_PAD_SD4_DAT2__USDHC4_DAT2_50MHZ,
	MX6Q_PAD_SD4_DAT3__USDHC4_DAT3_50MHZ,
	MX6Q_PAD_SD4_DAT4__USDHC4_DAT4_50MHZ,
	MX6Q_PAD_SD4_DAT5__USDHC4_DAT5_50MHZ,
	MX6Q_PAD_SD4_DAT6__USDHC4_DAT6_50MHZ,
	MX6Q_PAD_SD4_DAT7__USDHC4_DAT7_50MHZ,

	/* Charge */
	MX6Q_PAD_EIM_A25__GPIO_5_2,  /* FLT_1_B */
	MX6Q_PAD_EIM_D23__GPIO_3_23, /* CHG_1_B */
	MX6Q_PAD_EIM_DA13__GPIO_3_13, /* CHG_2_B  */
	MX6Q_PAD_EIM_DA14__GPIO_3_14, /* FLT_2_B */

	MX6Q_PAD_ENET_RXD0__GPIO_1_27, /* UOK_B */
	MX6Q_PAD_EIM_CS1__GPIO_2_24,   /* DOK_B */

	/* Audio Codec */
	MX6Q_PAD_KEY_COL2__GPIO_4_10,		/* CODEC_PWR_EN */
	MX6Q_PAD_SD3_RST__GPIO_7_8,			/* HEADPHONE_DET */
	MX6Q_PAD_GPIO_9__GPIO_1_9,			/* MICROPHONE_DET */

	/*GPS AUX_3V15_EN*/
	MX6Q_PAD_NANDF_WP_B__GPIO_6_9,

	/* PCIE */
	MX6Q_PAD_EIM_D19__GPIO_3_19, /* PCIE_PWR_EN */

	MX6Q_PAD_GPIO_17__GPIO_7_12, /* PCIE_RST */
	MX6Q_PAD_KEY_COL4__GPIO_4_14, /* PCIE_DIS */

	/* DISP_RST_B */
	MX6Q_PAD_NANDF_CS0__GPIO_6_11,
	/* DISP_PWR_EN */
	MX6Q_PAD_NANDF_CS1__GPIO_6_14,
};

static iomux_v3_cfg_t mx6q_sabresd_csi0_sensor_pads[] = {
	/* IPU1 Camera */
	MX6Q_PAD_CSI0_DAT12__IPU1_CSI0_D_12,
	MX6Q_PAD_CSI0_DAT13__IPU1_CSI0_D_13,
	MX6Q_PAD_CSI0_DAT14__IPU1_CSI0_D_14,
	MX6Q_PAD_CSI0_DAT15__IPU1_CSI0_D_15,
	MX6Q_PAD_CSI0_DAT16__IPU1_CSI0_D_16,
	MX6Q_PAD_CSI0_DAT17__IPU1_CSI0_D_17,
	MX6Q_PAD_CSI0_DAT18__IPU1_CSI0_D_18,
	MX6Q_PAD_CSI0_DAT19__IPU1_CSI0_D_19,
	MX6Q_PAD_CSI0_DATA_EN__IPU1_CSI0_DATA_EN,
	MX6Q_PAD_CSI0_MCLK__IPU1_CSI0_HSYNC,
	MX6Q_PAD_CSI0_PIXCLK__IPU1_CSI0_PIXCLK,
	MX6Q_PAD_CSI0_VSYNC__IPU1_CSI0_VSYNC,

	MX6Q_PAD_GPIO_0__CCM_CLKO,		/* camera clk */

	MX6Q_PAD_SD1_DAT0__GPIO_1_16,		/* camera PWDN */
	MX6Q_PAD_SD1_DAT1__GPIO_1_17,		/* camera RESET */
};

static iomux_v3_cfg_t mx6q_sabresd_mipi_sensor_pads[] = {
	MX6Q_PAD_GPIO_0__CCM_CLKO,		/* camera clk */

	MX6Q_PAD_SD1_DAT2__GPIO_1_19,		/* camera PWDN */
	MX6Q_PAD_SD1_CLK__GPIO_1_20,		/* camera RESET */
};
#endif
