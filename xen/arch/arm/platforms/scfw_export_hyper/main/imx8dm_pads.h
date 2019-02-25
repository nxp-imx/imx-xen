/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * Header file used to configure SoC pad list.
 */

#ifndef SC_PADS_H
#define SC_PADS_H

/* Includes */

/* Defines */

/*!
 * @name Pad Definitions
 */
/*@{*/
#define SC_P_M40_I2C0_SCL                        0    /*!< M40.I2C0.SCL, M40.UART0.RX, M40.GPIO0.IO02, LSIO.GPIO0.IO06, SCU.UART0.RX */
#define SC_P_M40_GPIO0_00                        1    /*!< M40.GPIO0.IO00, M40.TPM0.CH0, DMA.UART2.RX, LSIO.GPIO0.IO08 */
#define SC_P_M40_I2C0_SDA                        2    /*!< M40.I2C0.SDA, M40.UART0.TX, M40.GPIO0.IO03, LSIO.GPIO0.IO07, SCU.UART0.TX */
#define SC_P_M40_GPIO0_01                        3    /*!< M40.GPIO0.IO01, M40.TPM0.CH1, DMA.UART2.TX, LSIO.GPIO0.IO09 */
#define SC_P_M41_I2C0_SCL                        4    /*!< M41.I2C0.SCL, M41.UART0.RX, M41.GPIO0.IO02, LSIO.GPIO0.IO10 */
#define SC_P_M41_GPIO0_00                        5    /*!< M41.GPIO0.IO00, M41.TPM0.CH0, DMA.UART3.RX, LSIO.GPIO0.IO12 */
#define SC_P_M41_I2C0_SDA                        6    /*!< M41.I2C0.SDA, M41.UART0.TX, M41.GPIO0.IO03, LSIO.GPIO0.IO11 */
#define SC_P_M41_GPIO0_01                        7    /*!< M41.GPIO0.IO01, M41.TPM0.CH1, DMA.UART3.TX, LSIO.GPIO0.IO13 */
#define SC_P_GPT0_CLK                            8    /*!< LSIO.GPT0.CLK, DMA.I2C1.SCL, LSIO.KPP0.COL4, LSIO.GPIO0.IO14 */
#define SC_P_GPT0_COMPARE                        9    /*!< LSIO.GPT0.COMPARE, LSIO.PWM3.OUT, LSIO.KPP0.COL6, LSIO.GPIO0.IO16 */
#define SC_P_GPT0_CAPTURE                        10   /*!< LSIO.GPT0.CAPTURE, DMA.I2C1.SDA, LSIO.KPP0.COL5, LSIO.GPIO0.IO15 */
#define SC_P_UART0_RX                            11   /*!< DMA.UART0.RX, SCU.UART0.RX, LSIO.GPIO0.IO20 */
#define SC_P_UART0_TX                            12   /*!< DMA.UART0.TX, SCU.UART0.TX, LSIO.GPIO0.IO21 */
#define SC_P_UART0_RTS_B                         13   /*!< DMA.UART0.RTS_B, LSIO.PWM0.OUT, DMA.UART2.RX, LSIO.GPIO0.IO22 */
#define SC_P_UART0_CTS_B                         14   /*!< DMA.UART0.CTS_B, LSIO.PWM1.OUT, DMA.UART2.TX, LSIO.GPIO0.IO23 */
#define SC_P_UART1_TX                            15   /*!< DMA.UART1.TX, DMA.SPI3.SCK, LSIO.GPIO0.IO24 */
#define SC_P_UART1_RX                            16   /*!< DMA.UART1.RX, DMA.SPI3.SDO, LSIO.GPIO0.IO25 */
#define SC_P_UART1_RTS_B                         17   /*!< DMA.UART1.RTS_B, DMA.SPI3.SDI, DMA.UART1.CTS_B, LSIO.GPIO0.IO26 */
#define SC_P_UART1_CTS_B                         18   /*!< DMA.UART1.CTS_B, DMA.SPI3.CS0, DMA.UART1.RTS_B, LSIO.GPIO0.IO27 */
#define SC_P_COMP_CTL_GPIO_1V8_3V3_GPIOLH        19   /*!<  */
#define SC_P_SCU_PMIC_MEMC_ON                    20   /*!< SCU.GPIO0.IOXX_PMIC_MEMC_ON */
#define SC_P_SCU_WDOG_OUT                        21   /*!< SCU.WDOG0.WDOG_OUT */
#define SC_P_PMIC_I2C_SDA                        22   /*!< SCU.PMIC_I2C.SDA */
#define SC_P_PMIC_I2C_SCL                        23   /*!< SCU.PMIC_I2C.SCL */
#define SC_P_PMIC_INT_B                          24   /*!< SCU.DSC.PMIC_INT_B */
#define SC_P_SCU_GPIO0_00                        25   /*!< SCU.GPIO0.IO00, SCU.UART0.RX, LSIO.GPIO0.IO28 */
#define SC_P_SCU_GPIO0_01                        26   /*!< SCU.GPIO0.IO01, SCU.UART0.TX, LSIO.GPIO0.IO29 */
#define SC_P_SCU_GPIO0_02                        27   /*!< SCU.GPIO0.IO02, SCU.GPIO0.IOXX_PMIC_GPU0_ON, LSIO.GPIO0.IO30 */
#define SC_P_SCU_GPIO0_03                        28   /*!< SCU.GPIO0.IO03, LSIO.GPIO0.IO31 */
#define SC_P_SCU_GPIO0_04                        29   /*!< SCU.GPIO0.IO04, SCU.GPIO0.IOXX_PMIC_A72_ON, LSIO.GPIO1.IO00 */
#define SC_P_SCU_GPIO0_05                        30   /*!< SCU.GPIO0.IO05, LSIO.GPIO1.IO01 */
#define SC_P_SCU_GPIO0_06                        31   /*!< SCU.GPIO0.IO06, SCU.TPM0.CH0, LSIO.GPIO1.IO02 */
#define SC_P_SCU_GPIO0_07                        32   /*!< SCU.GPIO0.IO07, SCU.TPM0.CH1, SCU.DSC.RTC_CLOCK_OUTPUT_32K, LSIO.GPIO1.IO03 */
#define SC_P_SCU_BOOT_MODE1                      33   /*!< SCU.DSC.BOOT_MODE1 */
#define SC_P_SCU_BOOT_MODE2                      34   /*!< SCU.DSC.BOOT_MODE2 */
#define SC_P_SCU_BOOT_MODE3                      35   /*!< SCU.DSC.BOOT_MODE3 */
#define SC_P_SCU_BOOT_MODE4                      36   /*!< SCU.DSC.BOOT_MODE4 */
#define SC_P_LVDS0_GPIO00                        37   /*!< LVDS0.GPIO0.IO00, LVDS0.PWM0.OUT, LSIO.GPIO1.IO04 */
#define SC_P_LVDS0_GPIO01                        38   /*!< LVDS0.GPIO0.IO01, LSIO.GPIO1.IO05 */
#define SC_P_LVDS0_I2C0_SCL                      39   /*!< LVDS0.I2C0.SCL, LVDS0.GPIO0.IO02, LSIO.GPIO1.IO06 */
#define SC_P_LVDS0_I2C0_SDA                      40   /*!< LVDS0.I2C0.SDA, LVDS0.GPIO0.IO03, LSIO.GPIO1.IO07 */
#define SC_P_LVDS0_I2C1_SCL                      41   /*!< LVDS0.I2C1.SCL, DMA.UART2.TX, LSIO.GPIO1.IO08 */
#define SC_P_LVDS0_I2C1_SDA                      42   /*!< LVDS0.I2C1.SDA, DMA.UART2.RX, LSIO.GPIO1.IO09 */
#define SC_P_COMP_CTL_GPIO_1V8_3V3_LVDSGPIO      43   /*!<  */
#define SC_P_MIPI_DSI0_I2C0_SCL                  44   /*!< MIPI_DSI0.I2C0.SCL, LSIO.GPIO1.IO16 */
#define SC_P_MIPI_DSI0_I2C0_SDA                  45   /*!< MIPI_DSI0.I2C0.SDA, LSIO.GPIO1.IO17 */
#define SC_P_MIPI_DSI0_GPIO0_00                  46   /*!< MIPI_DSI0.GPIO0.IO00, MIPI_DSI0.PWM0.OUT, LSIO.GPIO1.IO18 */
#define SC_P_MIPI_DSI0_GPIO0_01                  47   /*!< MIPI_DSI0.GPIO0.IO01, LSIO.GPIO1.IO19 */
#define SC_P_COMP_CTL_GPIO_1V8_3V3_MIPIDSIGPIO   48   /*!<  */
#define SC_P_MIPI_CSI0_MCLK_OUT                  49   /*!< MIPI_CSI0.ACM.MCLK_OUT, LSIO.GPIO1.IO24 */
#define SC_P_MIPI_CSI0_I2C0_SCL                  50   /*!< MIPI_CSI0.I2C0.SCL, LSIO.GPIO1.IO25 */
#define SC_P_MIPI_CSI0_I2C0_SDA                  51   /*!< MIPI_CSI0.I2C0.SDA, LSIO.GPIO1.IO26 */
#define SC_P_MIPI_CSI0_GPIO0_00                  52   /*!< MIPI_CSI0.GPIO0.IO00, DMA.I2C0.SCL, MIPI_CSI1.I2C0.SCL, LSIO.GPIO1.IO27 */
#define SC_P_MIPI_CSI0_GPIO0_01                  53   /*!< MIPI_CSI0.GPIO0.IO01, DMA.I2C0.SDA, MIPI_CSI1.I2C0.SDA, LSIO.GPIO1.IO28 */
#define SC_P_MIPI_CSI1_MCLK_OUT                  54   /*!< MIPI_CSI1.ACM.MCLK_OUT, LSIO.GPIO1.IO29 */
#define SC_P_MIPI_CSI1_GPIO0_00                  55   /*!< MIPI_CSI1.GPIO0.IO00, LSIO.GPIO1.IO30 */
#define SC_P_MIPI_CSI1_GPIO0_01                  56   /*!< MIPI_CSI1.GPIO0.IO01, LSIO.GPIO1.IO31 */
#define SC_P_MIPI_CSI1_I2C0_SCL                  57   /*!< MIPI_CSI1.I2C0.SCL, LSIO.GPIO2.IO00 */
#define SC_P_MIPI_CSI1_I2C0_SDA                  58   /*!< MIPI_CSI1.I2C0.SDA, LSIO.GPIO2.IO01 */
#define SC_P_HDMI_TX0_TS_SCL                     59   /*!< HDMI_TX0.I2C0.SCL, DMA.I2C0.SCL, DMA.I2C2.SCL, LSIO.GPIO2.IO02 */
#define SC_P_HDMI_TX0_TS_SDA                     60   /*!< HDMI_TX0.I2C0.SDA, DMA.I2C0.SDA, DMA.I2C2.SDA, LSIO.GPIO2.IO03 */
#define SC_P_COMP_CTL_GPIO_3V3_HDMIGPIO          61   /*!<  */
#define SC_P_ESAI1_FSR                           62   /*!< AUD.ESAI1.FSR, ADMA.LCDIF.D00, LSIO.GPIO2.IO04 */
#define SC_P_ESAI1_FST                           63   /*!< AUD.ESAI1.FST, AUD.SPDIF0.EXT_CLK, ADMA.LCDIF.D16, LSIO.GPIO2.IO05 */
#define SC_P_ESAI1_SCKR                          64   /*!< AUD.ESAI1.SCKR, ADMA.LCDIF.D01, LSIO.GPIO2.IO06 */
#define SC_P_ESAI1_SCKT                          65   /*!< AUD.ESAI1.SCKT, AUD.SAI2.RXC, AUD.SPDIF0.EXT_CLK, LSIO.GPIO2.IO07 */
#define SC_P_ESAI1_TX0                           66   /*!< AUD.ESAI1.TX0, AUD.SAI2.RXD, AUD.SPDIF0.RX, LSIO.GPIO2.IO08 */
#define SC_P_ESAI1_TX1                           67   /*!< AUD.ESAI1.TX1, AUD.SAI2.RXFS, AUD.SPDIF0.TX, LSIO.GPIO2.IO09 */
#define SC_P_ESAI1_TX2_RX3                       68   /*!< AUD.ESAI1.TX2_RX3, AUD.SPDIF0.RX, ADMA.LCDIF.D17, LSIO.GPIO2.IO10 */
#define SC_P_ESAI1_TX3_RX2                       69   /*!< AUD.ESAI1.TX3_RX2, AUD.SPDIF0.TX, ADMA.LCDIF.RESET, LSIO.GPIO2.IO11 */
#define SC_P_ESAI1_TX4_RX1                       70   /*!< AUD.ESAI1.TX4_RX1, ADMA.LCDIF.D02, LSIO.GPIO2.IO12 */
#define SC_P_ESAI1_TX5_RX0                       71   /*!< AUD.ESAI1.TX5_RX0, ADMA.LCDIF.D03, LSIO.GPIO2.IO13 */
#define SC_P_COMP_CTL_GPIO_1V8_3V3_GPIORHB       72   /*!<  */
#define SC_P_MCLK_IN0                            73   /*!< AUD.ACM.MCLK_IN0, AUD.ESAI1.RX_HF_CLK, LSIO.GPIO3.IO00, ADMA.LCDIF.EN */
#define SC_P_MCLK_OUT0                           74   /*!< AUD.ACM.MCLK_OUT0, AUD.ESAI1.TX_HF_CLK, LSIO.GPIO3.IO01 */
#define SC_P_COMP_CTL_GPIO_1V8_3V3_GPIORHC       75   /*!<  */
#define SC_P_SPI0_SCK                            76   /*!< DMA.SPI0.SCK, AUD.SAI0.RXC, ADMA.LCDIF.D04, LSIO.GPIO3.IO02 */
#define SC_P_SPI0_SDO                            77   /*!< DMA.SPI0.SDO, AUD.SAI0.TXD, ADMA.LCDIF.D05, LSIO.GPIO3.IO03 */
#define SC_P_SPI0_SDI                            78   /*!< DMA.SPI0.SDI, AUD.SAI0.RXD, ADMA.LCDIF.D06, LSIO.GPIO3.IO04 */
#define SC_P_SPI0_CS0                            79   /*!< DMA.SPI0.CS0, AUD.SAI0.RXFS, ADMA.LCDIF.D07, LSIO.GPIO3.IO05 */
#define SC_P_SPI0_CS1                            80   /*!< DMA.SPI0.CS1, AUD.SAI0.TXC, ADMA.LCDIF.D08, LSIO.GPIO3.IO06 */
#define SC_P_SPI2_SCK                            81   /*!< DMA.SPI2.SCK, DMA.DMA0.REQ_IN0, ADMA.LCDIF.D09, LSIO.GPIO3.IO07 */
#define SC_P_SPI2_SDO                            82   /*!< DMA.SPI2.SDO, DMA.FTM.CH0, ADMA.LCDIF.D10, LSIO.GPIO3.IO08 */
#define SC_P_SPI2_SDI                            83   /*!< DMA.SPI2.SDI, DMA.FTM.CH1, ADMA.LCDIF.D11, LSIO.GPIO3.IO09 */
#define SC_P_SPI2_CS0                            84   /*!< DMA.SPI2.CS0, DMA.FTM.CH2, ADMA.LCDIF.D12, LSIO.GPIO3.IO10 */
#define SC_P_SPI2_CS1                            85   /*!< DMA.SPI2.CS1, AUD.SAI0.TXFS, ADMA.LCDIF.D13, LSIO.GPIO3.IO11 */
#define SC_P_SAI1_RXC                            86   /*!< AUD.SAI1.RXC, AUD.SAI0.TXD, ADMA.LCDIF.HSYNC, LSIO.GPIO3.IO12 */
#define SC_P_SAI1_RXD                            87   /*!< AUD.SAI1.RXD, AUD.SAI0.TXFS, ADMA.LCDIF.D14, LSIO.GPIO3.IO13 */
#define SC_P_SAI1_RXFS                           88   /*!< AUD.SAI1.RXFS, AUD.SAI0.RXD, ADMA.LCDIF.EN, LSIO.GPIO3.IO14 */
#define SC_P_SAI1_TXC                            89   /*!< AUD.SAI1.TXC, AUD.SAI0.TXC, ADMA.LCDIF.VSYNC, LSIO.GPIO3.IO15 */
#define SC_P_SAI1_TXD                            90   /*!< AUD.SAI1.TXD, AUD.SAI1.RXC, ADMA.LCDIF.CLK, LSIO.GPIO3.IO16 */
#define SC_P_SAI1_TXFS                           91   /*!< AUD.SAI1.TXFS, AUD.SAI1.RXFS, ADMA.LCDIF.D15, LSIO.GPIO3.IO17 */
#define SC_P_COMP_CTL_GPIO_1V8_3V3_GPIORHT       92   /*!<  */
#define SC_P_ADC_IN0                             93   /*!< DMA.ADC.IN0, ADMA.LCDIF.D08, LSIO.KPP0.COL0, LSIO.GPIO3.IO18 */
#define SC_P_ADC_IN1                             94   /*!< DMA.ADC.IN1, ADMA.LCDIF.D13, LSIO.KPP0.COL1, LSIO.GPIO3.IO19 */
#define SC_P_ADC_IN2                             95   /*!< DMA.ADC.IN2, LSIO.KPP0.COL2, LSIO.GPIO3.IO20 */
#define SC_P_ADC_IN3                             96   /*!< DMA.ADC.IN3, DMA.SPI1.SCK, LSIO.KPP0.COL3, LSIO.GPIO3.IO21, ADMA.LCDIF.D14 */
#define SC_P_ADC_IN4                             97   /*!< DMA.ADC.IN4, DMA.SPI1.SDO, LSIO.KPP0.ROW0, LSIO.GPIO3.IO22, ADMA.LCDIF.D15 */
#define SC_P_ADC_IN5                             98   /*!< DMA.ADC.IN5, DMA.SPI1.SDI, LSIO.KPP0.ROW1, LSIO.GPIO3.IO23, ADMA.LCDIF.CLK */
#define SC_P_LSIO_GPIO3_24                       99   /*!< DMA.ADC.IN6_dummy, DMA.SPI1.CS0, LSIO.KPP0.ROW2, LSIO.GPIO3.IO24, ADMA.LCDIF.HSYNC */
#define SC_P_LSIO_GPIO3_25                       100  /*!< DMA.ADC.IN7_dummy, DMA.SPI1.CS1, LSIO.KPP0.ROW3, LSIO.GPIO3.IO25, ADMA.LCDIF.VSYNC */
#define SC_P_FLEXCAN0_RX                         101  /*!< DMA.FLEXCAN0.RX, LSIO.GPIO3.IO29 */
#define SC_P_FLEXCAN0_TX                         102  /*!< DMA.FLEXCAN0.TX, LSIO.GPIO3.IO30 */
#define SC_P_FLEXCAN1_RX                         103  /*!< DMA.FLEXCAN1.RX, LSIO.GPIO3.IO31 */
#define SC_P_FLEXCAN1_TX                         104  /*!< DMA.FLEXCAN1.TX, LSIO.GPIO4.IO00 */
#define SC_P_FLEXCAN2_RX                         105  /*!< DMA.FLEXCAN2.RX, LSIO.GPIO4.IO01 */
#define SC_P_FLEXCAN2_TX                         106  /*!< DMA.FLEXCAN2.TX, LSIO.GPIO4.IO02 */
#define SC_P_COMP_CTL_GPIO_1V8_3V3_GPIOTHR       107  /*!<  */
#define SC_P_MLB_SIG                             108  /*!< CONN.MLB.SIG, AUD.SAI3.RXC, LSIO.GPIO3.IO26 */
#define SC_P_MLB_CLK                             109  /*!< CONN.MLB.CLK, AUD.SAI3.RXFS, LSIO.GPIO3.IO27 */
#define SC_P_MLB_DATA                            110  /*!< CONN.MLB.DATA, AUD.SAI3.RXD, LSIO.GPIO3.IO28 */
#define SC_P_COMP_CTL_GPIO_1V8_3V3_GPIOLHT       111  /*!<  */
#define SC_P_USB_SS3_TC0                         112  /*!< DMA.I2C1.SCL, CONN.USB_OTG1.PWR, LSIO.QSPI1A.DATA1, LSIO.GPIO4.IO03 */
#define SC_P_USB_SS3_TC1                         113  /*!< DMA.I2C1.SCL, CONN.USB_OTG2.PWR, LSIO.QSPI1A.DATA2, LSIO.GPIO4.IO04 */
#define SC_P_USB_SS3_TC2                         114  /*!< DMA.I2C1.SDA, CONN.USB_OTG1.OC, LSIO.QSPI1A.DQS, LSIO.GPIO4.IO05 */
#define SC_P_USB_SS3_TC3                         115  /*!< DMA.I2C1.SDA, CONN.USB_OTG2.OC, LSIO.QSPI1A.DATA3, LSIO.GPIO4.IO06 */
#define SC_P_COMP_CTL_GPIO_3V3_USB3IO            116  /*!<  */
#define SC_P_ENET0_MDIO                          117  /*!< CONN.ENET0.MDIO, DMA.I2C2.SDA, LSIO.GPIO4.IO13 */
#define SC_P_ENET0_MDC                           118  /*!< CONN.ENET0.MDC, DMA.I2C2.SCL, LSIO.GPIO4.IO14 */
#define SC_P_ENET0_REFCLK_125M_25M               119  /*!< CONN.ENET0.REFCLK_125M_25M, CONN.ENET0.PPS, LSIO.GPIO4.IO15 */
#define SC_P_ENET1_REFCLK_125M_25M               120  /*!< CONN.ENET1.REFCLK_125M_25M, CONN.ENET1.PPS, LSIO.GPIO4.IO16 */
#define SC_P_ENET1_MDIO                          121  /*!< CONN.ENET1.MDIO, DMA.I2C3.SDA, LSIO.GPIO4.IO17 */
#define SC_P_ENET1_MDC                           122  /*!< CONN.ENET1.MDC, DMA.I2C3.SCL, LSIO.GPIO4.IO18 */
#define SC_P_COMP_CTL_GPIO_1V8_3V3_GPIOCT        123  /*!<  */
#define SC_P_QSPI1A_SS0_B                        124  /*!< LSIO.QSPI1A.SS0_B, LSIO.GPIO4.IO19 */
#define SC_P_QSPI1A_SCLK                         125  /*!< LSIO.QSPI1A.SCLK, LSIO.GPIO4.IO21 */
#define SC_P_QSPI1A_DATA0                        126  /*!< LSIO.QSPI1A.DATA0, LSIO.GPIO4.IO26 */
#define SC_P_COMP_CTL_GPIO_3V3_QSPI1             127  /*!<  */
#define SC_P_QSPI0A_DATA0                        128  /*!< LSIO.QSPI0A.DATA0 */
#define SC_P_QSPI0A_DATA1                        129  /*!< LSIO.QSPI0A.DATA1 */
#define SC_P_QSPI0A_DATA2                        130  /*!< LSIO.QSPI0A.DATA2 */
#define SC_P_QSPI0A_DATA3                        131  /*!< LSIO.QSPI0A.DATA3 */
#define SC_P_QSPI0A_DQS                          132  /*!< LSIO.QSPI0A.DQS */
#define SC_P_QSPI0A_SS0_B                        133  /*!< LSIO.QSPI0A.SS0_B */
#define SC_P_QSPI0A_SCLK                         134  /*!< LSIO.QSPI0A.SCLK */
#define SC_P_QSPI0B_SCLK                         135  /*!< LSIO.QSPI0B.SCLK */
#define SC_P_QSPI0B_DATA0                        136  /*!< LSIO.QSPI0B.DATA0 */
#define SC_P_QSPI0B_DATA1                        137  /*!< LSIO.QSPI0B.DATA1 */
#define SC_P_QSPI0B_DATA2                        138  /*!< LSIO.QSPI0B.DATA2 */
#define SC_P_QSPI0B_DATA3                        139  /*!< LSIO.QSPI0B.DATA3 */
#define SC_P_QSPI0B_DQS                          140  /*!< LSIO.QSPI0B.DQS */
#define SC_P_QSPI0B_SS0_B                        141  /*!< LSIO.QSPI0B.SS0_B */
#define SC_P_COMP_CTL_GPIO_1V8_3V3_QSPI0         142  /*!<  */
#define SC_P_PCIE_CTRL0_CLKREQ_B                 143  /*!< HSIO.PCIE0.CLKREQ_B, LSIO.GPIO4.IO27 */
#define SC_P_PCIE_CTRL0_WAKE_B                   144  /*!< HSIO.PCIE0.WAKE_B, LSIO.GPIO4.IO28 */
#define SC_P_PCIE_CTRL0_PERST_B                  145  /*!< HSIO.PCIE0.PERST_B, LSIO.GPIO4.IO29 */
#define SC_P_PCIE_CTRL1_CLKREQ_B                 146  /*!< HSIO.PCIE1.CLKREQ_B, DMA.I2C1.SDA, CONN.USB_OTG2.OC, LSIO.GPIO4.IO30 */
#define SC_P_PCIE_CTRL1_WAKE_B                   147  /*!< HSIO.PCIE1.WAKE_B, DMA.I2C1.SCL, CONN.USB_OTG2.PWR, LSIO.GPIO4.IO31 */
#define SC_P_PCIE_CTRL1_PERST_B                  148  /*!< HSIO.PCIE1.PERST_B, DMA.I2C1.SCL, CONN.USB_OTG1.PWR, LSIO.GPIO5.IO00 */
#define SC_P_COMP_CTL_GPIO_1V8_3V3_PCIESEP       149  /*!<  */
#define SC_P_EMMC0_CLK                           150  /*!< CONN.EMMC0.CLK, CONN.NAND.READY_B */
#define SC_P_EMMC0_CMD                           151  /*!< CONN.EMMC0.CMD, CONN.NAND.DQS, AUD.MQS.R, LSIO.GPIO5.IO03 */
#define SC_P_EMMC0_DATA0                         152  /*!< CONN.EMMC0.DATA0, CONN.NAND.DATA00, LSIO.GPIO5.IO04 */
#define SC_P_EMMC0_DATA1                         153  /*!< CONN.EMMC0.DATA1, CONN.NAND.DATA01, LSIO.GPIO5.IO05 */
#define SC_P_EMMC0_DATA2                         154  /*!< CONN.EMMC0.DATA2, CONN.NAND.DATA02, LSIO.GPIO5.IO06 */
#define SC_P_EMMC0_DATA3                         155  /*!< CONN.EMMC0.DATA3, CONN.NAND.DATA03, LSIO.GPIO5.IO07 */
#define SC_P_EMMC0_DATA4                         156  /*!< CONN.EMMC0.DATA4, CONN.NAND.DATA04, LSIO.GPIO5.IO08 */
#define SC_P_EMMC0_DATA5                         157  /*!< CONN.EMMC0.DATA5, CONN.NAND.DATA05, LSIO.GPIO5.IO09 */
#define SC_P_EMMC0_DATA6                         158  /*!< CONN.EMMC0.DATA6, CONN.NAND.DATA06, LSIO.GPIO5.IO10 */
#define SC_P_EMMC0_DATA7                         159  /*!< CONN.EMMC0.DATA7, CONN.NAND.DATA07, LSIO.GPIO5.IO11 */
#define SC_P_EMMC0_STROBE                        160  /*!< CONN.EMMC0.STROBE, CONN.NAND.CLE, LSIO.GPIO5.IO12 */
#define SC_P_EMMC0_RESET_B                       161  /*!< CONN.EMMC0.RESET_B, CONN.NAND.WP_B, CONN.USDHC1.VSELECT, LSIO.GPIO5.IO13 */
#define SC_P_COMP_CTL_GPIO_1V8_3V3_SD1FIX        162  /*!<  */
#define SC_P_USDHC1_CLK                          163  /*!< CONN.USDHC1.CLK, AUD.MQS.R */
#define SC_P_USDHC1_DATA1                        164  /*!< CONN.USDHC1.DATA1, CONN.NAND.RE_P, LSIO.GPIO5.IO16 */
#define SC_P_USDHC1_DATA0                        165  /*!< CONN.USDHC1.DATA0, CONN.NAND.RE_N, LSIO.GPIO5.IO15 */
#define SC_P_USDHC1_CMD                          166  /*!< CONN.USDHC1.CMD, AUD.MQS.L, LSIO.GPIO5.IO14 */
#define SC_P_CTL_NAND_RE_P_N                     167  /*!<  */
#define SC_P_USDHC1_DATA4                        168  /*!< CONN.USDHC1.DATA4, CONN.NAND.CE0_B, AUD.MQS.R, LSIO.GPIO5.IO19 */
#define SC_P_USDHC1_DATA3                        169  /*!< CONN.USDHC1.DATA3, CONN.NAND.DQS_P, LSIO.GPIO5.IO18 */
#define SC_P_CTL_NAND_DQS_P_N                    170  /*!<  */
#define SC_P_USDHC1_DATA2                        171  /*!< CONN.USDHC1.DATA2, CONN.NAND.DQS_N, LSIO.GPIO5.IO17 */
#define SC_P_USDHC1_DATA5                        172  /*!< CONN.USDHC1.DATA5, CONN.NAND.RE_B, AUD.MQS.L, LSIO.GPIO5.IO20 */
#define SC_P_USDHC1_DATA6                        173  /*!< CONN.USDHC1.DATA6, CONN.NAND.WE_B, CONN.USDHC1.WP, LSIO.GPIO5.IO21 */
#define SC_P_USDHC1_DATA7                        174  /*!< CONN.USDHC1.DATA7, CONN.NAND.ALE, CONN.USDHC1.CD_B, LSIO.GPIO5.IO22 */
#define SC_P_USDHC1_STROBE                       175  /*!< CONN.USDHC1.STROBE, CONN.NAND.CE1_B, CONN.USDHC1.RESET_B, LSIO.GPIO5.IO23 */
#define SC_P_COMP_CTL_GPIO_1V8_3V3_VSEL2         176  /*!<  */
#define SC_P_ENET0_RGMII_TXC                     177  /*!< CONN.ENET0.RGMII_TXC, CONN.ENET0.RCLK50M_OUT, CONN.ENET0.RCLK50M_IN, LSIO.GPIO5.IO30 */
#define SC_P_ENET0_RGMII_TX_CTL                  178  /*!< CONN.ENET0.RGMII_TX_CTL, LSIO.GPIO5.IO31 */
#define SC_P_ENET0_RGMII_TXD0                    179  /*!< CONN.ENET0.RGMII_TXD0, LSIO.GPIO6.IO00 */
#define SC_P_ENET0_RGMII_TXD1                    180  /*!< CONN.ENET0.RGMII_TXD1, LSIO.GPIO6.IO01 */
#define SC_P_ENET0_RGMII_TXD2                    181  /*!< CONN.ENET0.RGMII_TXD2, DMA.UART3.TX, LSIO.GPIO6.IO02 */
#define SC_P_ENET0_RGMII_TXD3                    182  /*!< CONN.ENET0.RGMII_TXD3, DMA.UART3.RTS_B, LSIO.GPIO6.IO03 */
#define SC_P_ENET0_RGMII_RXC                     183  /*!< CONN.ENET0.RGMII_RXC, DMA.UART3.CTS_B, LSIO.GPIO6.IO04 */
#define SC_P_ENET0_RGMII_RX_CTL                  184  /*!< CONN.ENET0.RGMII_RX_CTL, LSIO.GPIO6.IO05 */
#define SC_P_ENET0_RGMII_RXD0                    185  /*!< CONN.ENET0.RGMII_RXD0, LSIO.GPIO6.IO06 */
#define SC_P_ENET0_RGMII_RXD1                    186  /*!< CONN.ENET0.RGMII_RXD1, LSIO.GPIO6.IO07 */
#define SC_P_ENET0_RGMII_RXD2                    187  /*!< CONN.ENET0.RGMII_RXD2, CONN.ENET0.RMII_RX_ER, LSIO.GPIO6.IO08 */
#define SC_P_ENET0_RGMII_RXD3                    188  /*!< CONN.ENET0.RGMII_RXD3, DMA.UART3.RX, LSIO.GPIO6.IO09 */
#define SC_P_COMP_CTL_GPIO_1V8_3V3_ENETB         189  /*!<  */
#define SC_P_ENET1_RGMII_TXC                     190  /*!< CONN.ENET1.RGMII_TXC, CONN.ENET1.RCLK50M_OUT, CONN.ENET1.RCLK50M_IN, LSIO.GPIO6.IO10 */
#define SC_P_ENET1_RGMII_TX_CTL                  191  /*!< CONN.ENET1.RGMII_TX_CTL, LSIO.GPIO6.IO11 */
#define SC_P_ENET1_RGMII_TXD0                    192  /*!< CONN.ENET1.RGMII_TXD0, LSIO.GPIO6.IO12 */
#define SC_P_ENET1_RGMII_TXD1                    193  /*!< CONN.ENET1.RGMII_TXD1, LSIO.GPIO6.IO13 */
#define SC_P_ENET1_RGMII_TXD2                    194  /*!< CONN.ENET1.RGMII_TXD2, DMA.UART3.TX, LSIO.GPIO6.IO14 */
#define SC_P_ENET1_RGMII_TXD3                    195  /*!< CONN.ENET1.RGMII_TXD3, DMA.UART3.RTS_B, LSIO.GPIO6.IO15 */
#define SC_P_ENET1_RGMII_RXC                     196  /*!< CONN.ENET1.RGMII_RXC, DMA.UART3.CTS_B, LSIO.GPIO6.IO16 */
#define SC_P_ENET1_RGMII_RX_CTL                  197  /*!< CONN.ENET1.RGMII_RX_CTL, LSIO.GPIO6.IO17 */
#define SC_P_ENET1_RGMII_RXD0                    198  /*!< CONN.ENET1.RGMII_RXD0, LSIO.GPIO6.IO18 */
#define SC_P_ENET1_RGMII_RXD1                    199  /*!< CONN.ENET1.RGMII_RXD1, LSIO.GPIO6.IO19 */
#define SC_P_ENET1_RGMII_RXD3                    200  /*!< CONN.ENET1.RGMII_RXD3, DMA.UART3.RX, LSIO.GPIO6.IO21 */
#define SC_P_ENET1_RGMII_RXD2                    201  /*!< CONN.ENET1.RGMII_RXD2, CONN.ENET1.RMII_RX_ER, LSIO.GPIO6.IO20 */
#define SC_P_COMP_CTL_GPIO_1V8_3V3_ENETA         202  /*!<  */
/*@}*/

#endif /* SC_PADS_H */

