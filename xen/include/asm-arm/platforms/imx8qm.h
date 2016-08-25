#ifndef __ASM_ARM_PLATFORMS_IMX8QM_H
#define __ASM_ASM_PLATFORMS_IMX8QM_H

#define PWM_0_LPCG           0x5E400000
#define PWM_1_LPCG           0x5E410000
#define PWM_2_LPCG           0x5E420000
#define PWM_3_LPCG           0x5E430000
#define PWM_4_LPCG           0x5E440000
#define PWM_5_LPCG           0x5E450000
#define PWM_6_LPCG           0x5E460000
#define PWM_7_LPCG           0x5E470000
#define GPIO_0_LPCG          0x5E480000
#define GPIO_1_LPCG          0x5E490000
#define GPIO_2_LPCG          0x5E4A0000
#define GPIO_3_LPCG          0x5E4B0000
#define GPIO_4_LPCG          0x5E4C0000
#define GPIO_5_LPCG          0x5E4D0000
#define GPIO_6_LPCG          0x5E4E0000
#define GPIO_7_LPCG          0x5E4F0000
#define FLEX_SPI_0_LPCG      0x5E520000
#define FLEX_SPI_1_LPCG      0x5E530000
#define GPT_0_LPCG           0x5E540000
#define GPT_1_LPCG           0x5E550000
#define GPT_2_LPCG           0x5E560000
#define GPT_3_LPCG           0x5E570000
#define GPT_4_LPCG           0x5E580000
#define KPP_LPCG             0x5E5A0000
#define OCRAM_LPCG           0x5E590000
#define GPIO_LPCG            0x5F100000
#define CRR_5_LPCG           0x5F0F0000
#define CRR_4_LPCG           0x5F0E0000
#define CRR_3_LPCG           0x5F0D0000
#define CRR_2_LPCG           0x5F0C0000
#define CRR_1_LPCG           0x5F0B0000
#define CRR_0_LPCG           0x5F0A0000
#define PHY_1_LPCG           0x5F090000
#define PHY_2_LPCG           0x5F080000
#define SATA_0_LPCG          0x5F070000
#define PCIE_B_LPCG          0x5F060000
#define PCIE_A_LPCG          0x5F050000
#define FLEX_CAN_2_LPCG      0x5ACF0000
#define FLEX_CAN_1_LPCG      0x5ACE0000
#define FLEX_CAN_0_LPCG      0x5ACD0000
#define FTM_1_LPCG           0x5ACB0000
#define FTM_0_LPCG           0x5ACA0000
#define ADC_1_LPCG           0x5AC90000
#define ADC_0_LPCG           0x5AC80000
#define LPI2C_4_LPCG         0x5AC40000
#define LPI2C_3_LPCG         0x5AC30000
#define LPI2C_2_LPCG         0x5AC20000
#define LPI2C_1_LPCG         0x5AC10000
#define LPI2C_0_LPCG         0x5AC00000
#define EMVSIM_1_LPCG        0x5A4E0000
#define EMVSIM_0_LPCG        0x5A4D0000
#define LPUART_4_LPCG        0x5A4A0000

#define DC_0_LPCG            0x56010000
#define DC_1_LPCG            0x57010000
#define PCIE_PER_LPCG        0x5f050000
#define PCIE_PHY_LPCG        0x5f060000
#define LCD_LPCG             0x56230000
#define LVDS_0_LPCG          0x56243000
#define LVDS_1_LPCG          0x57243000
#define LVDS_2_LPCG          0x57210000
#define M4_0_I2C_LPCG        0x37630000
#define M4_0_LPUART_LPCG     0x37620000
#define M4_0_LPIT_LPCG       0x37610000
#define M4_1_I2C_LPCG        0x3B630000
#define M4_1_LPUART_LPCG     0x3B620000
#define M4_1_LPIT_LPCG       0x3B610000
#define CAPTURE_PL_0_LPCG    0x57303000
#define CAPTURE_PL_1_LPCG    0x57403000

#define LPCG_ARRAY \
    {\
        PWM_0_LPCG, \
        PWM_1_LPCG, \
        PWM_2_LPCG, \
        PWM_3_LPCG, \
        PWM_4_LPCG, \
        PWM_5_LPCG, \
        PWM_6_LPCG, \
        PWM_7_LPCG, \
        GPIO_0_LPCG, \
        GPIO_1_LPCG, \
        GPIO_2_LPCG, \
        GPIO_3_LPCG, \
        GPIO_4_LPCG, \
        GPIO_5_LPCG, \
        GPIO_6_LPCG, \
        GPIO_7_LPCG, \
        FLEX_SPI_0_LPCG, \
        FLEX_SPI_1_LPCG, \
        GPT_0_LPCG, \
        GPT_1_LPCG, \
        GPT_2_LPCG, \
        GPT_3_LPCG, \
        GPT_4_LPCG, \
        KPP_LPCG, \
        OCRAM_LPCG, \
        GPIO_LPCG, \
        CRR_5_LPCG, \
        CRR_4_LPCG, \
        CRR_3_LPCG, \
        CRR_2_LPCG, \
        CRR_1_LPCG, \
        CRR_0_LPCG, \
        PHY_1_LPCG, \
        PHY_2_LPCG, \
        SATA_0_LPCG, \
        PCIE_B_LPCG, \
        PCIE_A_LPCG, \
        FLEX_CAN_2_LPCG, \
        FLEX_CAN_1_LPCG, \
        FLEX_CAN_0_LPCG, \
        FTM_1_LPCG, \
        FTM_0_LPCG, \
        ADC_1_LPCG, \
        ADC_0_LPCG, \
        LPI2C_4_LPCG, \
        LPI2C_3_LPCG, \
        LPI2C_2_LPCG, \
        LPI2C_1_LPCG, \
        LPI2C_0_LPCG, \
        EMVSIM_1_LPCG, \
        EMVSIM_0_LPCG, \
        LPUART_4_LPCG, \
        DC_0_LPCG, \
        DC_1_LPCG, \
        PCIE_PER_LPCG, \
        PCIE_PHY_LPCG, \
        LCD_LPCG, \
        LVDS_0_LPCG, \
        LVDS_1_LPCG, \
        LVDS_2_LPCG, \
        M4_0_I2C_LPCG, \
        M4_0_LPUART_LPCG, \
        M4_0_LPIT_LPCG, \
        M4_1_I2C_LPCG, \
        M4_1_LPUART_LPCG, \
        M4_1_LPIT_LPCG, \
        CAPTURE_PL_0_LPCG, \
        CAPTURE_PL_1_LPCG \
    }

#endif /* __ASM_ARM_PLATFORMS_IMX8QM_H */

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
