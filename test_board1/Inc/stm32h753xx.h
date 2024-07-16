/*
 * stm32h753xx.h
 *
 *  Created on: Jun 25, 2024
 *      Author: Bharath
 */

#ifndef STM32H753XX_H_
#define STM32H753XX_H_
#include<stdint.h>
#include<stddef.h>



#define V           					 volatile
#define __weak    __attribute__((weak))



/**********************************************START:Processor Specific Details *********************************************************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register addresses
 */

#define NVIC_ISER0					((V uint32_t*)0xE00E100)
#define NVIC_ISER1					((V uint32_t*)0xE00E104)
#define NVIC_ISER2					((V uint32_t*)0xE00E108)
#define NVIC_ISER3					((V uint32_t*)0xE00E10C)
#define NVIC_ISER4					((V uint32_t*)0xE00E110)
#define NVIC_ISER5					((V uint32_t*)0xE00E114)
#define NVIC_ISER6					((V uint32_t*)0xE00E118)
#define NVIC_ISER7					((V uint32_t*)0xE00E11C)


/*
 * ARM CORTEX Mx Processor NVIC ICERx register addresses
 */

#define NVIC_ICER0					((V uint32_t*)0xE00E180)
#define NVIC_ICER1					((V uint32_t*)0xE00E184)
#define NVIC_ICER2					((V uint32_t*)0xE00E188)
#define NVIC_ICER3					((V uint32_t*)0xE00E18C)
#define NVIC_ICER4					((V uint32_t*)0xE00E190)
#define NVIC_ICER5					((V uint32_t*)0xE00E194)
#define NVIC_ICER6					((V uint32_t*)0xE00E198)
#define NVIC_ICER7					((V uint32_t*)0xE00E19C)

/*
 * ARM CORTEX Mx Processor Priority Register address calculation
 */
#define NVIC_PR1_BADDR   			((V uint32_t*)0xE000E400)

#define   NO_PR_BITS_IMPLEMENTED       4     // This value changes depending upon the board we are using
                                             // So make sure you equated right value when you are working with other Boards


/*******************************************************************************************************************
 *              PERIPHERAL MEMORY MAPPING
 ***************************************************************************************************************************/

/** @addtogroup Peripheral_memory_map
  * @{
  */
#define D1_ITCMRAM_BASE           (0x00000000UL) /*!< Base address of : 64KB RAM reserved for CPU execution/instruction accessible over ITCM  */
#define D1_ITCMICP_BASE           (0x00100000UL) /*!< Base address of : (up to 128KB) embedded Test FLASH memory accessible over ITCM         */
#define D1_DTCMRAM_BASE           (0x20000000UL) /*!< Base address of : 128KB system data RAM accessible over DTCM                            */
#define D1_AXIFLASH_BASE          (0x08000000UL) /*!< Base address of : (up to 2 MB) embedded FLASH memory accessible over AXI                */
#define D1_AXIICP_BASE            (0x1FF00000UL) /*!< Base address of : (up to 128KB) embedded Test FLASH memory accessible over AXI          */
#define D1_AXISRAM_BASE           (0x24000000UL) /*!< Base address of : (up to 512KB) system data RAM accessible over over AXI                */

#define D2_AXISRAM_BASE           (0x10000000UL) /*!< Base address of : (up to 288KB) system data RAM accessible over over AXI                */
#define D2_AHBSRAM_BASE           (0x30000000UL) /*!< Base address of : (up to 288KB) system data RAM accessible over over AXI->AHB Bridge    */

#define D3_BKPSRAM_BASE           (0x38800000UL) /*!< Base address of : Backup SRAM(4 KB) over AXI->AHB Bridge                                */
#define D3_SRAM_BASE              (0x38000000UL) /*!< Base address of : Backup SRAM(64 KB) over AXI->AHB Bridge                               */


#define PERIPH_BASE               (0x40000000UL) /*!< Base address of : AHB/APB Peripherals                                                    */
#define QSPI_BASE                 (0x90000000UL) /*!< Base address of : QSPI memories  accessible over AXI                                    */

#define FLASH_BANK1_BASE          (0x08000000UL) /*!< Base address of : (up to 1 MB) Flash Bank1 accessible over AXI                          */
#define FLASH_BANK2_BASE          (0x08100000UL) /*!< Base address of : (up to 1 MB) Flash Bank2 accessible over AXI                          */
#define FLASH_END                 (0x081FFFFFUL) /*!< FLASH end address                                                                       */

/* Legacy define */
#define FLASH_BASE                 FLASH_BANK1_BASE

/*!< Device electronic signature memory map */
#define UID_BASE                  (0x1FF1E800UL)            /*!< Unique device ID register base address */
#define FLASHSIZE_BASE            (0x1FF1E880UL)            /*!< FLASH Size register base address */


/*!< Peripheral memory map */
#define D2_APB1PERIPH_BASE        PERIPH_BASE
#define D2_APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000UL)
#define D2_AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000UL)
#define D2_AHB2PERIPH_BASE       (PERIPH_BASE + 0x08020000UL)

#define D1_APB1PERIPH_BASE       (PERIPH_BASE + 0x10000000UL)
#define D1_AHB1PERIPH_BASE       (PERIPH_BASE + 0x12000000UL)

#define D3_APB1PERIPH_BASE       (PERIPH_BASE + 0x18000000UL)
#define D3_AHB1PERIPH_BASE       (PERIPH_BASE + 0x18020000UL)

/*!< Legacy Peripheral memory map */
#define APB1PERIPH_BASE        PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000UL)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000UL)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x08000000UL)


/*!< D1_AHB1PERIPH peripherals */

#define MDMA_BASE             (D1_AHB1PERIPH_BASE + 0x0000UL)
#define DMA2D_BASE            (D1_AHB1PERIPH_BASE + 0x1000UL)
#define JPGDEC_BASE           (D1_AHB1PERIPH_BASE + 0x3000UL)
#define FLASH_R_BASE          (D1_AHB1PERIPH_BASE + 0x2000UL)
#define FMC_R_BASE            (D1_AHB1PERIPH_BASE + 0x4000UL)
#define QSPI_R_BASE           (D1_AHB1PERIPH_BASE + 0x5000UL)
#define DLYB_QSPI_BASE        (D1_AHB1PERIPH_BASE + 0x6000UL)
#define SDMMC1_BASE           (D1_AHB1PERIPH_BASE + 0x7000UL)
#define DLYB_SDMMC1_BASE      (D1_AHB1PERIPH_BASE + 0x8000UL)
#define RAMECC1_BASE          (D1_AHB1PERIPH_BASE + 0x9000UL)

/*!< D2_AHB1PERIPH peripherals */

#define DMA1_BASE               (D2_AHB1PERIPH_BASE + 0x0000UL)
#define DMA2_BASE               (D2_AHB1PERIPH_BASE + 0x0400UL)
#define DMAMUX1_BASE            (D2_AHB1PERIPH_BASE + 0x0800UL)
#define ADC1_BASE               (D2_AHB1PERIPH_BASE + 0x2000UL)
#define ADC2_BASE               (D2_AHB1PERIPH_BASE + 0x2100UL)
#define ADC12_COMMON_BASE       (D2_AHB1PERIPH_BASE + 0x2300UL)
#define ETH_BASE                (D2_AHB1PERIPH_BASE + 0x8000UL)
#define ETH_MAC_BASE            (ETH_BASE)

/*!< USB registers base address */
#define USB1_OTG_HS_PERIPH_BASE              (0x40040000UL)
#define USB2_OTG_FS_PERIPH_BASE              (0x40080000UL)
#define USB_OTG_GLOBAL_BASE                  (0x000UL)
#define USB_OTG_DEVICE_BASE                  (0x800UL)
#define USB_OTG_IN_ENDPOINT_BASE             (0x900UL)
#define USB_OTG_OUT_ENDPOINT_BASE            (0xB00UL)
#define USB_OTG_EP_REG_SIZE                  (0x20UL)
#define USB_OTG_HOST_BASE                    (0x400UL)
#define USB_OTG_HOST_PORT_BASE               (0x440UL)
#define USB_OTG_HOST_CHANNEL_BASE            (0x500UL)
#define USB_OTG_HOST_CHANNEL_SIZE            (0x20UL)
#define USB_OTG_PCGCCTL_BASE                 (0xE00UL)
#define USB_OTG_FIFO_BASE                    (0x1000UL)
#define USB_OTG_FIFO_SIZE                    (0x1000UL)

/*!< D2_AHB2PERIPH peripherals */

#define DCMI_BASE              (D2_AHB2PERIPH_BASE + 0x0000UL)
#define RNG_BASE               (D2_AHB2PERIPH_BASE + 0x1800UL)
#define SDMMC2_BASE            (D2_AHB2PERIPH_BASE + 0x2400UL)
#define DLYB_SDMMC2_BASE       (D2_AHB2PERIPH_BASE + 0x2800UL)
#define RAMECC2_BASE           (D2_AHB2PERIPH_BASE + 0x3000UL)

/*!< D3_AHB1PERIPH peripherals */
#define GPIOA_BASE            (D3_AHB1PERIPH_BASE + 0x0000UL)
#define GPIOB_BASE            (D3_AHB1PERIPH_BASE + 0x0400UL)
#define GPIOC_BASE            (D3_AHB1PERIPH_BASE + 0x0800UL)
#define GPIOD_BASE            (D3_AHB1PERIPH_BASE + 0x0C00UL)
#define GPIOE_BASE            (D3_AHB1PERIPH_BASE + 0x1000UL)
#define GPIOF_BASE            (D3_AHB1PERIPH_BASE + 0x1400UL)
#define GPIOG_BASE            (D3_AHB1PERIPH_BASE + 0x1800UL)
#define GPIOH_BASE            (D3_AHB1PERIPH_BASE + 0x1C00UL)
#define GPIOI_BASE            (D3_AHB1PERIPH_BASE + 0x2000UL)
#define GPIOJ_BASE            (D3_AHB1PERIPH_BASE + 0x2400UL)
#define GPIOK_BASE            (D3_AHB1PERIPH_BASE + 0x2800UL)



#define RCC_BASE              (D3_AHB1PERIPH_BASE + 0x4400UL)
#define PWR_BASE              (D3_AHB1PERIPH_BASE + 0x4800UL)
#define CRC_BASE              (D3_AHB1PERIPH_BASE + 0x4C00UL)
#define BDMA_BASE             (D3_AHB1PERIPH_BASE + 0x5400UL)
#define DMAMUX2_BASE          (D3_AHB1PERIPH_BASE + 0x5800UL)
#define ADC3_BASE             (D3_AHB1PERIPH_BASE + 0x6000UL)
#define ADC3_COMMON_BASE      (D3_AHB1PERIPH_BASE + 0x6300UL)
#define HSEM_BASE             (D3_AHB1PERIPH_BASE + 0x6400UL)
#define RAMECC3_BASE          (D3_AHB1PERIPH_BASE + 0x7000UL)



/*!< D1_APB1PERIPH peripherals */
#define LTDC_BASE             (D1_APB1PERIPH_BASE + 0x1000UL)
#define LTDC_Layer1_BASE      (LTDC_BASE + 0x84UL)
#define LTDC_Layer2_BASE      (LTDC_BASE + 0x104UL)
#define WWDG1_BASE            (D1_APB1PERIPH_BASE + 0x3000UL)

/*!< D2_APB1PERIPH peripherals */
#define TIM2_BASE             (D2_APB1PERIPH_BASE + 0x0000UL)
#define TIM3_BASE             (D2_APB1PERIPH_BASE + 0x0400UL)
#define TIM4_BASE             (D2_APB1PERIPH_BASE + 0x0800UL)
#define TIM5_BASE             (D2_APB1PERIPH_BASE + 0x0C00UL)
#define TIM6_BASE             (D2_APB1PERIPH_BASE + 0x1000UL)
#define TIM7_BASE             (D2_APB1PERIPH_BASE + 0x1400UL)
#define TIM12_BASE            (D2_APB1PERIPH_BASE + 0x1800UL)
#define TIM13_BASE            (D2_APB1PERIPH_BASE + 0x1C00UL)
#define TIM14_BASE            (D2_APB1PERIPH_BASE + 0x2000UL)
#define LPTIM1_BASE           (D2_APB1PERIPH_BASE + 0x2400UL)


#define SPI2_BASE             (D2_APB1PERIPH_BASE + 0x3800UL)
#define SPI3_BASE             (D2_APB1PERIPH_BASE + 0x3C00UL)
#define SPDIFRX_BASE          (D2_APB1PERIPH_BASE + 0x4000UL)
#define USART2_BASE           (D2_APB1PERIPH_BASE + 0x4400UL)
#define USART3_BASE           (D2_APB1PERIPH_BASE + 0x4800UL)
#define UART4_BASE            (D2_APB1PERIPH_BASE + 0x4C00UL)
#define UART5_BASE            (D2_APB1PERIPH_BASE + 0x5000UL)
#define I2C1_BASE             (D2_APB1PERIPH_BASE + 0x5400UL)
#define I2C2_BASE             (D2_APB1PERIPH_BASE + 0x5800UL)
#define I2C3_BASE             (D2_APB1PERIPH_BASE + 0x5C00UL)
#define CEC_BASE              (D2_APB1PERIPH_BASE + 0x6C00UL)
#define DAC1_BASE             (D2_APB1PERIPH_BASE + 0x7400UL)
#define UART7_BASE            (D2_APB1PERIPH_BASE + 0x7800UL)
#define UART8_BASE            (D2_APB1PERIPH_BASE + 0x7C00UL)
#define CRS_BASE              (D2_APB1PERIPH_BASE + 0x8400UL)
#define SWPMI1_BASE           (D2_APB1PERIPH_BASE + 0x8800UL)
#define OPAMP_BASE            (D2_APB1PERIPH_BASE + 0x9000UL)
#define OPAMP1_BASE           (D2_APB1PERIPH_BASE + 0x9000UL)
#define OPAMP2_BASE           (D2_APB1PERIPH_BASE + 0x9010UL)
#define MDIOS_BASE            (D2_APB1PERIPH_BASE + 0x9400UL)
#define FDCAN1_BASE           (D2_APB1PERIPH_BASE + 0xA000UL)
#define FDCAN2_BASE           (D2_APB1PERIPH_BASE + 0xA400UL)
#define FDCAN_CCU_BASE        (D2_APB1PERIPH_BASE + 0xA800UL)
#define SRAMCAN_BASE          (D2_APB1PERIPH_BASE + 0xAC00UL)

/*!< D2_APB2PERIPH peripherals */

#define TIM1_BASE             (D2_APB2PERIPH_BASE + 0x0000UL)
#define TIM8_BASE             (D2_APB2PERIPH_BASE + 0x0400UL)
#define USART1_BASE           (D2_APB2PERIPH_BASE + 0x1000UL)
#define USART6_BASE           (D2_APB2PERIPH_BASE + 0x1400UL)
#define SPI1_BASE             (D2_APB2PERIPH_BASE + 0x3000UL)
#define SPI4_BASE             (D2_APB2PERIPH_BASE + 0x3400UL)
#define TIM15_BASE            (D2_APB2PERIPH_BASE + 0x4000UL)
#define TIM16_BASE            (D2_APB2PERIPH_BASE + 0x4400UL)
#define TIM17_BASE            (D2_APB2PERIPH_BASE + 0x4800UL)
#define SPI5_BASE             (D2_APB2PERIPH_BASE + 0x5000UL)
#define SAI1_BASE             (D2_APB2PERIPH_BASE + 0x5800UL)
#define SAI1_Block_A_BASE     (SAI1_BASE + 0x004UL)
#define SAI1_Block_B_BASE     (SAI1_BASE + 0x024UL)
#define SAI2_BASE             (D2_APB2PERIPH_BASE + 0x5C00UL)
#define SAI2_Block_A_BASE     (SAI2_BASE + 0x004UL)
#define SAI2_Block_B_BASE     (SAI2_BASE + 0x024UL)
#define SAI3_BASE             (D2_APB2PERIPH_BASE + 0x6000UL)
#define SAI3_Block_A_BASE     (SAI3_BASE + 0x004UL)
#define SAI3_Block_B_BASE     (SAI3_BASE + 0x024UL)
#define DFSDM1_BASE           (D2_APB2PERIPH_BASE + 0x7000UL)
#define DFSDM1_Channel0_BASE  (DFSDM1_BASE + 0x00UL)
#define DFSDM1_Channel1_BASE  (DFSDM1_BASE + 0x20UL)
#define DFSDM1_Channel2_BASE  (DFSDM1_BASE + 0x40UL)
#define DFSDM1_Channel3_BASE  (DFSDM1_BASE + 0x60UL)
#define DFSDM1_Channel4_BASE  (DFSDM1_BASE + 0x80UL)
#define DFSDM1_Channel5_BASE  (DFSDM1_BASE + 0xA0UL)
#define DFSDM1_Channel6_BASE  (DFSDM1_BASE + 0xC0UL)
#define DFSDM1_Channel7_BASE  (DFSDM1_BASE + 0xE0UL)
#define DFSDM1_Filter0_BASE   (DFSDM1_BASE + 0x100UL)
#define DFSDM1_Filter1_BASE   (DFSDM1_BASE + 0x180UL)
#define DFSDM1_Filter2_BASE   (DFSDM1_BASE + 0x200UL)
#define DFSDM1_Filter3_BASE   (DFSDM1_BASE + 0x280UL)
#define HRTIM1_BASE           (D2_APB2PERIPH_BASE + 0x7400UL)
#define HRTIM1_TIMA_BASE      (HRTIM1_BASE + 0x00000080UL)
#define HRTIM1_TIMB_BASE      (HRTIM1_BASE + 0x00000100UL)
#define HRTIM1_TIMC_BASE      (HRTIM1_BASE + 0x00000180UL)
#define HRTIM1_TIMD_BASE      (HRTIM1_BASE + 0x00000200UL)
#define HRTIM1_TIME_BASE      (HRTIM1_BASE + 0x00000280UL)
#define HRTIM1_COMMON_BASE    (HRTIM1_BASE + 0x00000380UL)


/*!< D3_APB1PERIPH peripherals */
#define EXTI_BASE             (D3_APB1PERIPH_BASE + 0x0000UL)
#define EXTI_D1_BASE          (EXTI_BASE + 0x0080UL)
#define EXTI_D2_BASE          (EXTI_BASE + 0x00C0UL)
#define SYSCFG_BASE           (D3_APB1PERIPH_BASE + 0x0400UL)
#define LPUART1_BASE          (D3_APB1PERIPH_BASE + 0x0C00UL)
#define SPI6_BASE             (D3_APB1PERIPH_BASE + 0x1400UL)
#define I2C4_BASE             (D3_APB1PERIPH_BASE + 0x1C00UL)
#define LPTIM2_BASE           (D3_APB1PERIPH_BASE + 0x2400UL)
#define LPTIM3_BASE           (D3_APB1PERIPH_BASE + 0x2800UL)
#define LPTIM4_BASE           (D3_APB1PERIPH_BASE + 0x2C00UL)
#define LPTIM5_BASE           (D3_APB1PERIPH_BASE + 0x3000UL)
#define COMP12_BASE           (D3_APB1PERIPH_BASE + 0x3800UL)
#define COMP1_BASE            (COMP12_BASE + 0x0CUL)
#define COMP2_BASE            (COMP12_BASE + 0x10UL)
#define VREFBUF_BASE          (D3_APB1PERIPH_BASE + 0x3C00UL)
#define RTC_BASE              (D3_APB1PERIPH_BASE + 0x4000UL)
#define IWDG1_BASE            (D3_APB1PERIPH_BASE + 0x4800UL)


#define SAI4_BASE             (D3_APB1PERIPH_BASE + 0x5400UL)
#define SAI4_Block_A_BASE     (SAI4_BASE + 0x004UL)
#define SAI4_Block_B_BASE     (SAI4_BASE + 0x024UL)




#define BDMA_Channel0_BASE    (BDMA_BASE + 0x0008UL)
#define BDMA_Channel1_BASE    (BDMA_BASE + 0x001CUL)
#define BDMA_Channel2_BASE    (BDMA_BASE + 0x0030UL)
#define BDMA_Channel3_BASE    (BDMA_BASE + 0x0044UL)
#define BDMA_Channel4_BASE    (BDMA_BASE + 0x0058UL)
#define BDMA_Channel5_BASE    (BDMA_BASE + 0x006CUL)
#define BDMA_Channel6_BASE    (BDMA_BASE + 0x0080UL)
#define BDMA_Channel7_BASE    (BDMA_BASE + 0x0094UL)

#define DMAMUX2_Channel0_BASE    (DMAMUX2_BASE)
#define DMAMUX2_Channel1_BASE    (DMAMUX2_BASE + 0x0004UL)
#define DMAMUX2_Channel2_BASE    (DMAMUX2_BASE + 0x0008UL)
#define DMAMUX2_Channel3_BASE    (DMAMUX2_BASE + 0x000CUL)
#define DMAMUX2_Channel4_BASE    (DMAMUX2_BASE + 0x0010UL)
#define DMAMUX2_Channel5_BASE    (DMAMUX2_BASE + 0x0014UL)
#define DMAMUX2_Channel6_BASE    (DMAMUX2_BASE + 0x0018UL)
#define DMAMUX2_Channel7_BASE    (DMAMUX2_BASE + 0x001CUL)

#define DMAMUX2_RequestGenerator0_BASE  (DMAMUX2_BASE + 0x0100UL)
#define DMAMUX2_RequestGenerator1_BASE  (DMAMUX2_BASE + 0x0104UL)
#define DMAMUX2_RequestGenerator2_BASE  (DMAMUX2_BASE + 0x0108UL)
#define DMAMUX2_RequestGenerator3_BASE  (DMAMUX2_BASE + 0x010CUL)
#define DMAMUX2_RequestGenerator4_BASE  (DMAMUX2_BASE + 0x0110UL)
#define DMAMUX2_RequestGenerator5_BASE  (DMAMUX2_BASE + 0x0114UL)
#define DMAMUX2_RequestGenerator6_BASE  (DMAMUX2_BASE + 0x0118UL)
#define DMAMUX2_RequestGenerator7_BASE  (DMAMUX2_BASE + 0x011CUL)

#define DMAMUX2_ChannelStatus_BASE      (DMAMUX2_BASE + 0x0080UL)
#define DMAMUX2_RequestGenStatus_BASE   (DMAMUX2_BASE + 0x0140UL)

#define DMA1_Stream0_BASE     (DMA1_BASE + 0x010UL)
#define DMA1_Stream1_BASE     (DMA1_BASE + 0x028UL)
#define DMA1_Stream2_BASE     (DMA1_BASE + 0x040UL)
#define DMA1_Stream3_BASE     (DMA1_BASE + 0x058UL)
#define DMA1_Stream4_BASE     (DMA1_BASE + 0x070UL)
#define DMA1_Stream5_BASE     (DMA1_BASE + 0x088UL)
#define DMA1_Stream6_BASE     (DMA1_BASE + 0x0A0UL)
#define DMA1_Stream7_BASE     (DMA1_BASE + 0x0B8UL)

#define DMA2_Stream0_BASE     (DMA2_BASE + 0x010UL)
#define DMA2_Stream1_BASE     (DMA2_BASE + 0x028UL)
#define DMA2_Stream2_BASE     (DMA2_BASE + 0x040UL)
#define DMA2_Stream3_BASE     (DMA2_BASE + 0x058UL)
#define DMA2_Stream4_BASE     (DMA2_BASE + 0x070UL)
#define DMA2_Stream5_BASE     (DMA2_BASE + 0x088UL)
#define DMA2_Stream6_BASE     (DMA2_BASE + 0x0A0UL)
#define DMA2_Stream7_BASE     (DMA2_BASE + 0x0B8UL)

#define DMAMUX1_Channel0_BASE    (DMAMUX1_BASE)
#define DMAMUX1_Channel1_BASE    (DMAMUX1_BASE + 0x0004UL)
#define DMAMUX1_Channel2_BASE    (DMAMUX1_BASE + 0x0008UL)
#define DMAMUX1_Channel3_BASE    (DMAMUX1_BASE + 0x000CUL)
#define DMAMUX1_Channel4_BASE    (DMAMUX1_BASE + 0x0010UL)
#define DMAMUX1_Channel5_BASE    (DMAMUX1_BASE + 0x0014UL)
#define DMAMUX1_Channel6_BASE    (DMAMUX1_BASE + 0x0018UL)
#define DMAMUX1_Channel7_BASE    (DMAMUX1_BASE + 0x001CUL)
#define DMAMUX1_Channel8_BASE    (DMAMUX1_BASE + 0x0020UL)
#define DMAMUX1_Channel9_BASE    (DMAMUX1_BASE + 0x0024UL)
#define DMAMUX1_Channel10_BASE   (DMAMUX1_BASE + 0x0028UL)
#define DMAMUX1_Channel11_BASE   (DMAMUX1_BASE + 0x002CUL)
#define DMAMUX1_Channel12_BASE   (DMAMUX1_BASE + 0x0030UL)
#define DMAMUX1_Channel13_BASE   (DMAMUX1_BASE + 0x0034UL)
#define DMAMUX1_Channel14_BASE   (DMAMUX1_BASE + 0x0038UL)
#define DMAMUX1_Channel15_BASE   (DMAMUX1_BASE + 0x003CUL)

#define DMAMUX1_RequestGenerator0_BASE  (DMAMUX1_BASE + 0x0100UL)
#define DMAMUX1_RequestGenerator1_BASE  (DMAMUX1_BASE + 0x0104UL)
#define DMAMUX1_RequestGenerator2_BASE  (DMAMUX1_BASE + 0x0108UL)
#define DMAMUX1_RequestGenerator3_BASE  (DMAMUX1_BASE + 0x010CUL)
#define DMAMUX1_RequestGenerator4_BASE  (DMAMUX1_BASE + 0x0110UL)
#define DMAMUX1_RequestGenerator5_BASE  (DMAMUX1_BASE + 0x0114UL)
#define DMAMUX1_RequestGenerator6_BASE  (DMAMUX1_BASE + 0x0118UL)
#define DMAMUX1_RequestGenerator7_BASE  (DMAMUX1_BASE + 0x011CUL)

#define DMAMUX1_ChannelStatus_BASE      (DMAMUX1_BASE + 0x0080UL)
#define DMAMUX1_RequestGenStatus_BASE   (DMAMUX1_BASE + 0x0140UL)

/*!< FMC Banks registers base  address */
#define FMC_Bank1_R_BASE      (FMC_R_BASE + 0x0000UL)
#define FMC_Bank1E_R_BASE     (FMC_R_BASE + 0x0104UL)
#define FMC_Bank2_R_BASE      (FMC_R_BASE + 0x0060UL)
#define FMC_Bank3_R_BASE      (FMC_R_BASE + 0x0080UL)
#define FMC_Bank5_6_R_BASE    (FMC_R_BASE + 0x0140UL)

/* Debug MCU registers base address */
#define DBGMCU_BASE           (0x5C001000UL)

#define MDMA_Channel0_BASE    (MDMA_BASE + 0x00000040UL)
#define MDMA_Channel1_BASE    (MDMA_BASE + 0x00000080UL)
#define MDMA_Channel2_BASE    (MDMA_BASE + 0x000000C0UL)
#define MDMA_Channel3_BASE    (MDMA_BASE + 0x00000100UL)
#define MDMA_Channel4_BASE    (MDMA_BASE + 0x00000140UL)
#define MDMA_Channel5_BASE    (MDMA_BASE + 0x00000180UL)
#define MDMA_Channel6_BASE    (MDMA_BASE + 0x000001C0UL)
#define MDMA_Channel7_BASE    (MDMA_BASE + 0x00000200UL)
#define MDMA_Channel8_BASE    (MDMA_BASE + 0x00000240UL)
#define MDMA_Channel9_BASE    (MDMA_BASE + 0x00000280UL)
#define MDMA_Channel10_BASE   (MDMA_BASE + 0x000002C0UL)
#define MDMA_Channel11_BASE   (MDMA_BASE + 0x00000300UL)
#define MDMA_Channel12_BASE   (MDMA_BASE + 0x00000340UL)
#define MDMA_Channel13_BASE   (MDMA_BASE + 0x00000380UL)
#define MDMA_Channel14_BASE   (MDMA_BASE + 0x000003C0UL)
#define MDMA_Channel15_BASE   (MDMA_BASE + 0x00000400UL)

#define RAMECC1_Monitor1_BASE (RAMECC1_BASE + 0x20UL)
#define RAMECC1_Monitor2_BASE (RAMECC1_BASE + 0x40UL)
#define RAMECC1_Monitor3_BASE (RAMECC1_BASE + 0x60UL)
#define RAMECC1_Monitor4_BASE (RAMECC1_BASE + 0x80UL)
#define RAMECC1_Monitor5_BASE (RAMECC1_BASE + 0xA0UL)

#define RAMECC2_Monitor1_BASE (RAMECC2_BASE + 0x20UL)
#define RAMECC2_Monitor2_BASE (RAMECC2_BASE + 0x40UL)
#define RAMECC2_Monitor3_BASE (RAMECC2_BASE + 0x60UL)
#define RAMECC2_Monitor4_BASE (RAMECC2_BASE + 0x80UL)
#define RAMECC2_Monitor5_BASE (RAMECC2_BASE + 0xA0UL)

#define RAMECC3_Monitor1_BASE (RAMECC3_BASE + 0x20UL)
#define RAMECC3_Monitor2_BASE (RAMECC3_BASE + 0x40UL)





/** @addtogroup Peripheral_declaration
  * @{
  */
#define TIM2                ((TIM_RegDef_t *) TIM2_BASE)
#define TIM3                ((TIM_RegDef_t *) TIM3_BASE)
#define TIM4                ((TIM_RegDef_t *) TIM4_BASE)
#define TIM5                ((TIM_RegDef_t *) TIM5_BASE)
#define TIM6                ((TIM_RegDef_t *) TIM6_BASE)
#define TIM7                ((TIM_RegDef_t *) TIM7_BASE)
#define TIM13               ((TIM_RegDef_t *) TIM13_BASE)
#define TIM14               ((TIM_RegDef_t *) TIM14_BASE)
#define VREFBUF             ((VREFBUF_RegDef_t *) VREFBUF_BASE)
#define RTC                 ((RTC_RegDef_t *) RTC_BASE)
#define WWDG1               ((WWDG_RegDef_t *) WWDG1_BASE)


#define IWDG1               ((IWDG_RegDef_t *) IWDG1_BASE)
#define SPI2                ((SPI_RegDef_t *) SPI2_BASE)
#define SPI3                ((SPI_RegDef_t *) SPI3_BASE)
#define SPI4                ((SPI_RegDef_t *) SPI4_BASE)
#define SPI5                ((SPI_RegDef_t *) SPI5_BASE)
#define SPI6                ((SPI_RegDef_t *) SPI6_BASE)
#define USART2              ((USART_RegDef_t *) USART2_BASE)
#define USART3              ((USART_RegDef_t *) USART3_BASE)
#define USART6              ((USART_RegDef_t *) USART6_BASE)
#define UART7               ((USART_RegDef_t *) UART7_BASE)
#define UART8               ((USART_RegDef_t *) UART8_BASE)
#define CRS                 ((CRS_RegDef_t *) CRS_BASE)
#define UART4               ((USART_RegDef_t *) UART4_BASE)
#define UART5               ((USART_RegDef_t *) UART5_BASE)
#define I2C1                ((I2C_RegDef_t *) I2C1_BASE)
#define I2C2                ((I2C_RegDef_t *) I2C2_BASE)
#define I2C3                ((I2C_RegDef_t *) I2C3_BASE)
#define I2C4                ((I2C_RegDef_t *) I2C4_BASE)
#define FDCAN1              ((FDCAN_GlobalRegDef_t *) FDCAN1_BASE)
#define FDCAN2              ((FDCAN_GlobalRegDef_t *) FDCAN2_BASE)
#define FDCAN_CCU           ((FDCAN_ClockCalibrationUnit_RegDef_t *) FDCAN_CCU_BASE)
#define CEC                 ((CEC_RegDef_t *) CEC_BASE)
#define LPTIM1              ((LPTIM_RegDef_t *) LPTIM1_BASE)
#define PWR                 ((PWR_RegDef_t *) PWR_BASE)
#define DAC1                ((DAC_RegDef_t *) DAC1_BASE)
#define LPUART1             ((USART_RegDef_t *) LPUART1_BASE)
#define SWPMI1              ((SWPMI_RegDef_t *) SWPMI1_BASE)
#define LPTIM2              ((LPTIM_RegDef_t *) LPTIM2_BASE)
#define LPTIM3              ((LPTIM_RegDef_t *) LPTIM3_BASE)
#define LPTIM4              ((LPTIM_RegDef_t *) LPTIM4_BASE)
#define LPTIM5              ((LPTIM_RegDef_t *) LPTIM5_BASE)

#define SYSCFG              ((SYSCFG_RegDef_t *) SYSCFG_BASE)
#define COMP12              ((COMPOPT_RegDef_t *) COMP12_BASE)
#define COMP1               ((COMP_RegDef_t *) COMP1_BASE)
#define COMP2               ((COMP_RegDef_t *) COMP2_BASE)
#define COMP12_COMMON       ((COMP_Common_RegDef_t *) COMP2_BASE)
#define OPAMP               ((OPAMP_RegDef_t *) OPAMP_BASE)
#define OPAMP1              ((OPAMP_RegDef_t *) OPAMP1_BASE)
#define OPAMP2              ((OPAMP_RegDef_t *) OPAMP2_BASE)


#define EXTI                ((EXTI_RegDef_t *) EXTI_BASE)
#define EXTI_D1             ((EXTI_Core_RegDef_t *) EXTI_D1_BASE)
#define EXTI_D2             ((EXTI_Core_RegDef_t *) EXTI_D2_BASE)
#define TIM1                ((TIM_RegDef_t *) TIM1_BASE)
#define SPI1                ((SPI_RegDef_t *) SPI1_BASE)
#define TIM8                ((TIM_RegDef_t *) TIM8_BASE)
#define USART1              ((USART_RegDef_t *) USART1_BASE)
#define TIM12               ((TIM_RegDef_t *) TIM12_BASE)
#define TIM15               ((TIM_RegDef_t *) TIM15_BASE)
#define TIM16               ((TIM_RegDef_t *) TIM16_BASE)
#define TIM17               ((TIM_RegDef_t *) TIM17_BASE)
#define HRTIM1              ((HRTIM_RegDef_t *) HRTIM1_BASE)
#define HRTIM1_TIMA         ((HRTIM_Timerx_RegDef_t *) HRTIM1_TIMA_BASE)
#define HRTIM1_TIMB         ((HRTIM_Timerx_RegDef_t *) HRTIM1_TIMB_BASE)
#define HRTIM1_TIMC         ((HRTIM_Timerx_RegDef_t *) HRTIM1_TIMC_BASE)
#define HRTIM1_TIMD         ((HRTIM_Timerx_RegDef_t *) HRTIM1_TIMD_BASE)
#define HRTIM1_TIME         ((HRTIM_Timerx_RegDef_t *) HRTIM1_TIME_BASE)
#define HRTIM1_COMMON       ((HRTIM_Common_RegDef_t *) HRTIM1_COMMON_BASE)
#define SAI1                ((SAI_RegDef_t *) SAI1_BASE)
#define SAI1_Block_A        ((SAI_Block_RegDef_t *)SAI1_Block_A_BASE)
#define SAI1_Block_B        ((SAI_Block_RegDef_t *)SAI1_Block_B_BASE)
#define SAI2                ((SAI_RegDef_t *) SAI2_BASE)
#define SAI2_Block_A        ((SAI_Block_RegDef_t *)SAI2_Block_A_BASE)
#define SAI2_Block_B        ((SAI_Block_RegDef_t *)SAI2_Block_B_BASE)
#define SAI3                ((SAI_RegDef_t *) SAI3_BASE)
#define SAI3_Block_A        ((SAI_Block_RegDef_t *)SAI3_Block_A_BASE)
#define SAI3_Block_B        ((SAI_Block_RegDef_t *)SAI3_Block_B_BASE)
#define SAI4                ((SAI_RegDef_t *) SAI4_BASE)
#define SAI4_Block_A        ((SAI_Block_RegDef_t *)SAI4_Block_A_BASE)
#define SAI4_Block_B        ((SAI_Block_RegDef_t *)SAI4_Block_B_BASE)

#define SPDIFRX             ((SPDIFRX_RegDef_t *) SPDIFRX_BASE)
#define DFSDM1_Channel0     ((DFSDM_Channel_RegDef_t *) DFSDM1_Channel0_BASE)
#define DFSDM1_Channel1     ((DFSDM_Channel_RegDef_t *) DFSDM1_Channel1_BASE)
#define DFSDM1_Channel2     ((DFSDM_Channel_RegDef_t *) DFSDM1_Channel2_BASE)
#define DFSDM1_Channel3     ((DFSDM_Channel_RegDef_t *) DFSDM1_Channel3_BASE)
#define DFSDM1_Channel4     ((DFSDM_Channel_RegDef_t *) DFSDM1_Channel4_BASE)
#define DFSDM1_Channel5     ((DFSDM_Channel_RegDef_t *) DFSDM1_Channel5_BASE)
#define DFSDM1_Channel6     ((DFSDM_Channel_RegDef_t *) DFSDM1_Channel6_BASE)
#define DFSDM1_Channel7     ((DFSDM_Channel_RegDef_t *) DFSDM1_Channel7_BASE)
#define DFSDM1_Filter0      ((DFSDM_Filter_RegDef_t *) DFSDM1_Filter0_BASE)
#define DFSDM1_Filter1      ((DFSDM_Filter_RegDef_t *) DFSDM1_Filter1_BASE)
#define DFSDM1_Filter2      ((DFSDM_Filter_RegDef_t *) DFSDM1_Filter2_BASE)
#define DFSDM1_Filter3      ((DFSDM_Filter_RegDef_t *) DFSDM1_Filter3_BASE)
#define DMA2D               ((DMA2D_RegDef_t *) DMA2D_BASE)
#define DCMI                ((DCMI_RegDef_t *) DCMI_BASE)
#define RCC                 ((RCC_RegDef_t *) RCC_BASE)
#define FLASH               ((FLASH_RegDef_t *) FLASH_R_BASE)
#define CRC                 ((CRC_RegDef_t *) CRC_BASE)

#define GPIOA               ((GPIO_RegDef_t *) GPIOA_BASE)
#define GPIOB               ((GPIO_RegDef_t *) GPIOB_BASE)
#define GPIOC               ((GPIO_RegDef_t *) GPIOC_BASE)
#define GPIOD               ((GPIO_RegDef_t *) GPIOD_BASE)
#define GPIOE               ((GPIO_RegDef_t *) GPIOE_BASE)
#define GPIOF               ((GPIO_RegDef_t *) GPIOF_BASE)
#define GPIOG               ((GPIO_RegDef_t *) GPIOG_BASE)
#define GPIOH               ((GPIO_RegDef_t *) GPIOH_BASE)
#define GPIOI               ((GPIO_RegDef_t *) GPIOI_BASE)
#define GPIOJ               ((GPIO_RegDef_t *) GPIOJ_BASE)
#define GPIOK               ((GPIO_RegDef_t *) GPIOK_BASE)

#define ADC1                ((ADC_RegDef_t *) ADC1_BASE)
#define ADC2                ((ADC_RegDef_t *) ADC2_BASE)
#define ADC3                ((ADC_RegDef_t *) ADC3_BASE)
#define ADC3_COMMON         ((ADC_Common_RegDef_t *) ADC3_COMMON_BASE)
#define ADC12_COMMON        ((ADC_Common_RegDef_t *) ADC12_COMMON_BASE)

#define RNG                 ((RNG_RegDef_t *) RNG_BASE)
#define SDMMC2              ((SDMMC_RegDef_t *) SDMMC2_BASE)
#define DLYB_SDMMC2         ((DLYB_RegDef_t *) DLYB_SDMMC2_BASE)

#define BDMA                ((BDMA_RegDef_t *) BDMA_BASE)
#define BDMA_Channel0       ((BDMA_Channel_RegDef_t *) BDMA_Channel0_BASE)
#define BDMA_Channel1       ((BDMA_Channel_RegDef_t *) BDMA_Channel1_BASE)
#define BDMA_Channel2       ((BDMA_Channel_RegDef_t *) BDMA_Channel2_BASE)
#define BDMA_Channel3       ((BDMA_Channel_RegDef_t *) BDMA_Channel3_BASE)
#define BDMA_Channel4       ((BDMA_Channel_RegDef_t *) BDMA_Channel4_BASE)
#define BDMA_Channel5       ((BDMA_Channel_RegDef_t *) BDMA_Channel5_BASE)
#define BDMA_Channel6       ((BDMA_Channel_RegDef_t *) BDMA_Channel6_BASE)
#define BDMA_Channel7       ((BDMA_Channel_RegDef_t *) BDMA_Channel7_BASE)

#define RAMECC1              ((RAMECC_RegDef_t *)RAMECC1_BASE)
#define RAMECC1_Monitor1     ((RAMECC_MonitorRegDef_t *)RAMECC1_Monitor1_BASE)
#define RAMECC1_Monitor2     ((RAMECC_MonitorRegDef_t *)RAMECC1_Monitor2_BASE)
#define RAMECC1_Monitor3     ((RAMECC_MonitorRegDef_t *)RAMECC1_Monitor3_BASE)
#define RAMECC1_Monitor4     ((RAMECC_MonitorRegDef_t *)RAMECC1_Monitor4_BASE)
#define RAMECC1_Monitor5     ((RAMECC_MonitorRegDef_t *)RAMECC1_Monitor5_BASE)

#define RAMECC2              ((RAMECC_RegDef_t *)RAMECC2_BASE)
#define RAMECC2_Monitor1     ((RAMECC_MonitorRegDef_t *)RAMECC2_Monitor1_BASE)
#define RAMECC2_Monitor2     ((RAMECC_MonitorRegDef_t *)RAMECC2_Monitor2_BASE)
#define RAMECC2_Monitor3     ((RAMECC_MonitorRegDef_t *)RAMECC2_Monitor3_BASE)
#define RAMECC2_Monitor4     ((RAMECC_MonitorRegDef_t *)RAMECC2_Monitor4_BASE)
#define RAMECC2_Monitor5     ((RAMECC_MonitorRegDef_t *)RAMECC2_Monitor5_BASE)

#define RAMECC3              ((RAMECC_RegDef_t *)RAMECC3_BASE)
#define RAMECC3_Monitor1     ((RAMECC_MonitorRegDef_t *)RAMECC3_Monitor1_BASE)
#define RAMECC3_Monitor2     ((RAMECC_MonitorRegDef_t *)RAMECC3_Monitor2_BASE)

#define DMAMUX2                ((DMAMUX_Channel_RegDef_t *) DMAMUX2_BASE)
#define DMAMUX2_Channel0       ((DMAMUX_Channel_RegDef_t *) DMAMUX2_Channel0_BASE)
#define DMAMUX2_Channel1       ((DMAMUX_Channel_RegDef_t *) DMAMUX2_Channel1_BASE)
#define DMAMUX2_Channel2       ((DMAMUX_Channel_RegDef_t *) DMAMUX2_Channel2_BASE)
#define DMAMUX2_Channel3       ((DMAMUX_Channel_RegDef_t *) DMAMUX2_Channel3_BASE)
#define DMAMUX2_Channel4       ((DMAMUX_Channel_RegDef_t *) DMAMUX2_Channel4_BASE)
#define DMAMUX2_Channel5       ((DMAMUX_Channel_RegDef_t *) DMAMUX2_Channel5_BASE)
#define DMAMUX2_Channel6       ((DMAMUX_Channel_RegDef_t *) DMAMUX2_Channel6_BASE)
#define DMAMUX2_Channel7       ((DMAMUX_Channel_RegDef_t *) DMAMUX2_Channel7_BASE)


#define DMAMUX2_RequestGenerator0  ((DMAMUX_RequestGen_RegDef_t *) DMAMUX2_RequestGenerator0_BASE)
#define DMAMUX2_RequestGenerator1  ((DMAMUX_RequestGen_RegDef_t *) DMAMUX2_RequestGenerator1_BASE)
#define DMAMUX2_RequestGenerator2  ((DMAMUX_RequestGen_RegDef_t *) DMAMUX2_RequestGenerator2_BASE)
#define DMAMUX2_RequestGenerator3  ((DMAMUX_RequestGen_RegDef_t *) DMAMUX2_RequestGenerator3_BASE)
#define DMAMUX2_RequestGenerator4  ((DMAMUX_RequestGen_RegDef_t *) DMAMUX2_RequestGenerator4_BASE)
#define DMAMUX2_RequestGenerator5  ((DMAMUX_RequestGen_RegDef_t *) DMAMUX2_RequestGenerator5_BASE)
#define DMAMUX2_RequestGenerator6  ((DMAMUX_RequestGen_RegDef_t *) DMAMUX2_RequestGenerator6_BASE)
#define DMAMUX2_RequestGenerator7  ((DMAMUX_RequestGen_RegDef_t *) DMAMUX2_RequestGenerator7_BASE)

#define DMAMUX2_ChannelStatus      ((DMAMUX_ChannelStatus_RegDef_t *) DMAMUX2_ChannelStatus_BASE)
#define DMAMUX2_RequestGenStatus   ((DMAMUX_RequestGenStatus_RegDef_t *) DMAMUX2_RequestGenStatus_BASE)

#define DMA2                ((DMA_RegDef_t *) DMA2_BASE)
#define DMA2_Stream0        ((DMA_Stream_RegDef_t *) DMA2_Stream0_BASE)
#define DMA2_Stream1        ((DMA_Stream_RegDef_t *) DMA2_Stream1_BASE)
#define DMA2_Stream2        ((DMA_Stream_RegDef_t *) DMA2_Stream2_BASE)
#define DMA2_Stream3        ((DMA_Stream_RegDef_t *) DMA2_Stream3_BASE)
#define DMA2_Stream4        ((DMA_Stream_RegDef_t *) DMA2_Stream4_BASE)
#define DMA2_Stream5        ((DMA_Stream_RegDef_t *) DMA2_Stream5_BASE)
#define DMA2_Stream6        ((DMA_Stream_RegDef_t *) DMA2_Stream6_BASE)
#define DMA2_Stream7        ((DMA_Stream_RegDef_t *) DMA2_Stream7_BASE)

#define DMA1                ((DMA_RegDef_t *) DMA1_BASE)
#define DMA1_Stream0        ((DMA_Stream_RegDef_t *) DMA1_Stream0_BASE)
#define DMA1_Stream1        ((DMA_Stream_RegDef_t *) DMA1_Stream1_BASE)
#define DMA1_Stream2        ((DMA_Stream_RegDef_t *) DMA1_Stream2_BASE)
#define DMA1_Stream3        ((DMA_Stream_RegDef_t *) DMA1_Stream3_BASE)
#define DMA1_Stream4        ((DMA_Stream_RegDef_t *) DMA1_Stream4_BASE)
#define DMA1_Stream5        ((DMA_Stream_RegDef_t *) DMA1_Stream5_BASE)
#define DMA1_Stream6        ((DMA_Stream_RegDef_t *) DMA1_Stream6_BASE)
#define DMA1_Stream7        ((DMA_Stream_RegDef_t *) DMA1_Stream7_BASE)


#define DMAMUX1              ((DMAMUX_Channel_RegDef_t *) DMAMUX1_BASE)
#define DMAMUX1_Channel0     ((DMAMUX_Channel_RegDef_t *) DMAMUX1_Channel0_BASE)
#define DMAMUX1_Channel1     ((DMAMUX_Channel_RegDef_t *) DMAMUX1_Channel1_BASE)
#define DMAMUX1_Channel2     ((DMAMUX_Channel_RegDef_t *) DMAMUX1_Channel2_BASE)
#define DMAMUX1_Channel3     ((DMAMUX_Channel_RegDef_t *) DMAMUX1_Channel3_BASE)
#define DMAMUX1_Channel4     ((DMAMUX_Channel_RegDef_t *) DMAMUX1_Channel4_BASE)
#define DMAMUX1_Channel5     ((DMAMUX_Channel_RegDef_t *) DMAMUX1_Channel5_BASE)
#define DMAMUX1_Channel6     ((DMAMUX_Channel_RegDef_t *) DMAMUX1_Channel6_BASE)
#define DMAMUX1_Channel7     ((DMAMUX_Channel_RegDef_t *) DMAMUX1_Channel7_BASE)
#define DMAMUX1_Channel8     ((DMAMUX_Channel_RegDef_t *) DMAMUX1_Channel8_BASE)
#define DMAMUX1_Channel9     ((DMAMUX_Channel_RegDef_t *) DMAMUX1_Channel9_BASE)
#define DMAMUX1_Channel10    ((DMAMUX_Channel_RegDef_t *) DMAMUX1_Channel10_BASE)
#define DMAMUX1_Channel11    ((DMAMUX_Channel_RegDef_t *) DMAMUX1_Channel11_BASE)
#define DMAMUX1_Channel12    ((DMAMUX_Channel_RegDef_t *) DMAMUX1_Channel12_BASE)
#define DMAMUX1_Channel13    ((DMAMUX_Channel_RegDef_t *) DMAMUX1_Channel13_BASE)
#define DMAMUX1_Channel14    ((DMAMUX_Channel_RegDef_t *) DMAMUX1_Channel14_BASE)
#define DMAMUX1_Channel15    ((DMAMUX_Channel_RegDef_t *) DMAMUX1_Channel15_BASE)

#define DMAMUX1_RequestGenerator0  ((DMAMUX_RequestGen_RegDef_t *) DMAMUX1_RequestGenerator0_BASE)
#define DMAMUX1_RequestGenerator1  ((DMAMUX_RequestGen_RegDef_t *) DMAMUX1_RequestGenerator1_BASE)
#define DMAMUX1_RequestGenerator2  ((DMAMUX_RequestGen_RegDef_t *) DMAMUX1_RequestGenerator2_BASE)
#define DMAMUX1_RequestGenerator3  ((DMAMUX_RequestGen_RegDef_t *) DMAMUX1_RequestGenerator3_BASE)
#define DMAMUX1_RequestGenerator4  ((DMAMUX_RequestGen_RegDef_t *) DMAMUX1_RequestGenerator4_BASE)
#define DMAMUX1_RequestGenerator5  ((DMAMUX_RequestGen_RegDef_t *) DMAMUX1_RequestGenerator5_BASE)
#define DMAMUX1_RequestGenerator6  ((DMAMUX_RequestGen_RegDef_t *) DMAMUX1_RequestGenerator6_BASE)
#define DMAMUX1_RequestGenerator7  ((DMAMUX_RequestGen_RegDef_t *) DMAMUX1_RequestGenerator7_BASE)

#define DMAMUX1_ChannelStatus      ((DMAMUX_ChannelStatus_RegDef_t *)    DMAMUX1_ChannelStatus_BASE)
#define DMAMUX1_RequestGenStatus   ((DMAMUX_RequestGenStatus_RegDef_t *) DMAMUX1_RequestGenStatus_BASE)


#define FMC_Bank1_R           ((FMC_Bank1_RegDef_t *) FMC_Bank1_R_BASE)
#define FMC_Bank1E_R          ((FMC_Bank1E_RegDef_t *) FMC_Bank1E_R_BASE)
#define FMC_Bank2_R           ((FMC_Bank2_RegDef_t *) FMC_Bank2_R_BASE)
#define FMC_Bank3_R           ((FMC_Bank3_RegDef_t *) FMC_Bank3_R_BASE)
#define FMC_Bank5_6_R         ((FMC_Bank5_6_RegDef_t *) FMC_Bank5_6_R_BASE)


#define QUADSPI               ((QUADSPI_RegDef_t_t *) QSPI_R_BASE)
#define DLYB_QUADSPI          ((DLYB_RegDef_t *) DLYB_QSPI_BASE)
#define SDMMC1                ((SDMMC_RegDef_t *) SDMMC1_BASE)
#define DLYB_SDMMC1           ((DLYB_RegDef_t *) DLYB_SDMMC1_BASE)

#define DBGMCU              ((DBGMCU_RegDef_t *) DBGMCU_BASE)

#define JPEG                ((JPEG_RegDef_t *) JPGDEC_BASE)
#define HSEM                ((HSEM_RegDef_t *) HSEM_BASE)
#define HSEM_COMMON         ((HSEM_Common_RegDef_t *) (HSEM_BASE + 0x100UL))

#define LTDC                ((LTDC_RegDef_t *)LTDC_BASE)
#define LTDC_Layer1         ((LTDC_Layer_RegDef_t *)LTDC_Layer1_BASE)
#define LTDC_Layer2         ((LTDC_Layer_RegDef_t *)LTDC_Layer2_BASE)

#define MDIOS               ((MDIOS_RegDef_t *) MDIOS_BASE)

#define ETH                 ((ETH_RegDef_t *)ETH_BASE)
#define MDMA                ((MDMA_RegDef_t *)MDMA_BASE)
#define MDMA_Channel0       ((MDMA_Channel_RegDef_t *)MDMA_Channel0_BASE)
#define MDMA_Channel1       ((MDMA_Channel_RegDef_t *)MDMA_Channel1_BASE)
#define MDMA_Channel2       ((MDMA_Channel_RegDef_t *)MDMA_Channel2_BASE)
#define MDMA_Channel3       ((MDMA_Channel_RegDef_t *)MDMA_Channel3_BASE)
#define MDMA_Channel4       ((MDMA_Channel_RegDef_t *)MDMA_Channel4_BASE)
#define MDMA_Channel5       ((MDMA_Channel_RegDef_t *)MDMA_Channel5_BASE)
#define MDMA_Channel6       ((MDMA_Channel_RegDef_t *)MDMA_Channel6_BASE)
#define MDMA_Channel7       ((MDMA_Channel_RegDef_t *)MDMA_Channel7_BASE)
#define MDMA_Channel8       ((MDMA_Channel_RegDef_t *)MDMA_Channel8_BASE)
#define MDMA_Channel9       ((MDMA_Channel_RegDef_t *)MDMA_Channel9_BASE)
#define MDMA_Channel10      ((MDMA_Channel_RegDef_t *)MDMA_Channel10_BASE)
#define MDMA_Channel11      ((MDMA_Channel_RegDef_t *)MDMA_Channel11_BASE)
#define MDMA_Channel12      ((MDMA_Channel_RegDef_t *)MDMA_Channel12_BASE)
#define MDMA_Channel13      ((MDMA_Channel_RegDef_t *)MDMA_Channel13_BASE)
#define MDMA_Channel14      ((MDMA_Channel_RegDef_t *)MDMA_Channel14_BASE)
#define MDMA_Channel15      ((MDMA_Channel_RegDef_t *)MDMA_Channel15_BASE)


#define USB1_OTG_HS         ((USB_OTG_GlobalRegDef_t *) USB1_OTG_HS_PERIPH_BASE)
#define USB2_OTG_FS         ((USB_OTG_GlobalRegDef_t *) USB2_OTG_FS_PERIPH_BASE)



/*******************************************************************************************************************
 *                REGISTER DEFINITION OF VARIOUS PERIPHERALS
 ****************************************************************************************************************/




/* Peripheral Register Definition Structure for GPIO */

typedef struct{
	V uint32_t MODER;       // Base_Address  0x00
	V uint32_t OTYPER;             // Base_Address  0x04
	V uint32_t OSPEEDER;// Base_Address  0x08
	V uint32_t PUPDR;// Base_Address  0x0C
	V uint32_t IDR;                // Base_Address  0x10
	V uint32_t ODR;                // Base_Address  0x14
	V uint32_t BSRR;               // Base_Address  0x18
	V uint32_t LCKR;               // Base_Address  0x1C
	V uint32_t AFR[2];      /* AFR[0] : Alternate Function low register    Base_Address  0x20
	                           AFR[1] : Alternate Function high register  Base_Address  0x24  */
}GPIO_RegDef_t;


/**
  * @brief Serial Peripheral Interface
  */

typedef struct
{
   V uint32_t CR1;           /*!< SPI/I2S Control register 1,                      Address offset: 0x00 */
  V uint32_t CR2;           /*!< SPI Control register 2,                          Address offset: 0x04 */
  V uint32_t CFG1;          /*!< SPI Configuration register 1,                    Address offset: 0x08 */
  V uint32_t CFG2;          /*!< SPI Configuration register 2,                    Address offset: 0x0C */
  V uint32_t IER;           /*!< SPI/I2S Interrupt Enable register,               Address offset: 0x10 */
  V uint32_t SR;            /*!< SPI/I2S Status register,                         Address offset: 0x14 */
  V uint32_t IFCR;          /*!< SPI/I2S Interrupt/Status flags clear register,   Address offset: 0x18 */
  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                        */
  V uint32_t TXDR;          /*!< SPI/I2S Transmit data register,                  Address offset: 0x20 */
  uint32_t      RESERVED1[3];  /*!< Reserved, 0x24-0x2C                                                   */
  V uint32_t RXDR;          /*!< SPI/I2S Receive data register,                   Address offset: 0x30 */
  uint32_t      RESERVED2[3];  /*!< Reserved, 0x34-0x3C                                                   */
  V uint32_t CRCPOLY;       /*!< SPI CRC Polynomial register,                     Address offset: 0x40 */
  V uint32_t TXCRC;         /*!< SPI Transmitter CRC register,                    Address offset: 0x44 */
  V uint32_t RXCRC;         /*!< SPI Receiver CRC register,                       Address offset: 0x48 */
  V uint32_t UDRDR;         /*!< SPI Underrun data register,                      Address offset: 0x4C */
  V uint32_t I2SCFGR;       /*!< I2S Configuration register,                      Address offset: 0x50 */

}SPI_RegDef_t;

/**
  * @brief System configuration controller
  */

typedef struct
{
  uint32_t RESERVED1;           /*!< Reserved,                                           Address offset: 0x00        */
 V uint32_t PMCR;           /*!< SYSCFG peripheral mode configuration register,      Address offset: 0x04        */
 V uint32_t EXTICR[4];      /*!< SYSCFG external interrupt configuration registers,  Address offset: 0x08-0x14   */
 V uint32_t CFGR;           /*!< SYSCFG configuration registers,                     Address offset: 0x18        */
 uint32_t RESERVED2;           /*!< Reserved,                                           Address offset: 0x1C        */
 V uint32_t CCCSR;          /*!< SYSCFG compensation cell control/status register,   Address offset: 0x20        */
 V uint32_t CCVR;           /*!< SYSCFG compensation cell value register,            Address offset: 0x24        */
 V uint32_t CCCR;           /*!< SYSCFG compensation cell code register,             Address offset: 0x28        */
 V uint32_t PWRCR;          /*!< PWR control register,                               Address offset: 0x2C        */
  uint32_t     RESERVED3[61];  /*!< Reserved, 0x30-0x120                                                            */
  V uint32_t PKGR;          /*!< SYSCFG package register,                            Address offset: 0x124       */
  uint32_t     RESERVED4[118]; /*!< Reserved, 0x128-0x2FC                                                           */
 V uint32_t UR0;            /*!< SYSCFG user register 0,                             Address offset: 0x300       */
 V uint32_t UR1;            /*!< SYSCFG user register 1,                             Address offset: 0x304       */
 V uint32_t UR2;            /*!< SYSCFG user register 2,                             Address offset: 0x308       */
 V uint32_t UR3;            /*!< SYSCFG user register 3,                             Address offset: 0x30C       */
 V uint32_t UR4;            /*!< SYSCFG user register 4,                             Address offset: 0x310       */
 V uint32_t UR5;            /*!< SYSCFG user register 5,                             Address offset: 0x314       */
 V uint32_t UR6;            /*!< SYSCFG user register 6,                             Address offset: 0x318       */
 V uint32_t UR7;            /*!< SYSCFG user register 7,                             Address offset: 0x31C       */
 V uint32_t UR8;            /*!< SYSCFG user register 8,                             Address offset: 0x320       */
 V uint32_t UR9;            /*!< SYSCFG user register 9,                             Address offset: 0x324       */
 V uint32_t UR10;           /*!< SYSCFG user register 10,                            Address offset: 0x328       */
 V uint32_t UR11;           /*!< SYSCFG user register 11,                            Address offset: 0x32C       */
 V uint32_t UR12;           /*!< SYSCFG user register 12,                            Address offset: 0x330       */
 V uint32_t UR13;           /*!< SYSCFG user register 13,                            Address offset: 0x334       */
 V uint32_t UR14;           /*!< SYSCFG user register 14,                            Address offset: 0x338       */
 V uint32_t UR15;           /*!< SYSCFG user register 15,                            Address offset: 0x33C       */
 V uint32_t UR16;           /*!< SYSCFG user register 16,                            Address offset: 0x340       */
 V uint32_t UR17;           /*!< SYSCFG user register 17,                            Address offset: 0x344       */

} SYSCFG_RegDef_t;

/**
  * @brief Reset and Clock Control
  */

typedef struct
{
 V uint32_t CR;             /*!< RCC clock control register,                                              Address offset: 0x00  */
 V uint32_t ICSCR;
 V uint32_t HSICFGR;        /*!< HSI Clock Calibration Register,                                          Address offset: 0x04  */
 V uint32_t CRRCR;          /*!< Clock Recovery RC  Register,                                             Address offset: 0x08  */
 V uint32_t CSICFGR;        /*!< CSI Clock Calibration Register,                                          Address offset: 0x0C  */
 V uint32_t CFGR;           /*!< RCC clock configuration register,                                        Address offset: 0x10  */
 uint32_t     RESERVED1;       /*!< Reserved,                                                                Address offset: 0x14  */
 V uint32_t D1CFGR;         /*!< RCC Domain 1 configuration register,                                     Address offset: 0x18  */
 V uint32_t D2CFGR;         /*!< RCC Domain 2 configuration register,                                     Address offset: 0x1C  */
 V uint32_t D3CFGR;         /*!< RCC Domain 3 configuration register,                                     Address offset: 0x20  */
 uint32_t     RESERVED2;       /*!< Reserved,                                                                Address offset: 0x24  */
 V uint32_t PLLCKSELR;      /*!< RCC PLLs Clock Source Selection Register,                                Address offset: 0x28  */
 V uint32_t PLLCFGR;        /*!< RCC PLLs  Configuration Register,                                        Address offset: 0x2C  */
 V uint32_t PLL1DIVR;       /*!< RCC PLL1 Dividers Configuration Register,                                Address offset: 0x30  */
 V uint32_t PLL1FRACR;      /*!< RCC PLL1 Fractional Divider Configuration Register,                      Address offset: 0x34  */
 V uint32_t PLL2DIVR;       /*!< RCC PLL2 Dividers Configuration Register,                                Address offset: 0x38  */
 V uint32_t PLL2FRACR;      /*!< RCC PLL2 Fractional Divider Configuration Register,                      Address offset: 0x3C  */
 V uint32_t PLL3DIVR;       /*!< RCC PLL3 Dividers Configuration Register,                                Address offset: 0x40  */
 V uint32_t PLL3FRACR;      /*!< RCC PLL3 Fractional Divider Configuration Register,                      Address offset: 0x44  */
 uint32_t      RESERVED3;      /*!< Reserved,                                                                Address offset: 0x48  */
 V uint32_t  D1CCIPR;       /*!< RCC Domain 1 Kernel Clock Configuration Register                         Address offset: 0x4C  */
 V uint32_t  D2CCIP1R;      /*!< RCC Domain 2 Kernel Clock Configuration Register                         Address offset: 0x50  */
 V uint32_t  D2CCIP2R;      /*!< RCC Domain 2 Kernel Clock Configuration Register                         Address offset: 0x54  */
 V uint32_t  D3CCIPR;       /*!< RCC Domain 3 Kernel Clock Configuration Register                         Address offset: 0x58  */
 uint32_t      RESERVED4;      /*!< Reserved,                                                                Address offset: 0x5C  */
 V uint32_t  CIER;          /*!< RCC Clock Source Interrupt Enable Register                               Address offset: 0x60  */
 V uint32_t  CIFR;          /*!< RCC Clock Source Interrupt Flag Register                                 Address offset: 0x64  */
 V uint32_t  CICR;          /*!< RCC Clock Source Interrupt Clear Register                                Address offset: 0x68  */
 uint32_t     RESERVED5;       /*!< Reserved,                                                                Address offset: 0x6C  */
 V uint32_t  BDCR;          /*!< RCC Vswitch Backup Domain Control Register,                              Address offset: 0x70  */
 V uint32_t  CSR;           /*!< RCC clock control & status register,                                     Address offset: 0x74  */
 uint32_t     RESERVED6;       /*!< Reserved,                                                                Address offset: 0x78  */
 V uint32_t AHB3RSTR;       /*!< RCC AHB3 peripheral reset register,                                      Address offset: 0x7C  */
 V uint32_t AHB1RSTR;       /*!< RCC AHB1 peripheral reset register,                                      Address offset: 0x80  */
 V uint32_t AHB2RSTR;       /*!< RCC AHB2 peripheral reset register,                                      Address offset: 0x84  */
 V uint32_t AHB4RSTR;       /*!< RCC AHB4 peripheral reset register,                                      Address offset: 0x88  */
 V uint32_t APB3RSTR;       /*!< RCC APB3 peripheral reset register,                                      Address offset: 0x8C  */
 V uint32_t APB1LRSTR;      /*!< RCC APB1 peripheral reset Low Word register,                             Address offset: 0x90  */
 V uint32_t APB1HRSTR;      /*!< RCC APB1 peripheral reset High Word register,                            Address offset: 0x94  */
 V uint32_t APB2RSTR;       /*!< RCC APB2 peripheral reset register,                                      Address offset: 0x98  */
 V uint32_t APB4RSTR;       /*!< RCC APB4 peripheral reset register,                                      Address offset: 0x9C  */
 V uint32_t GCR;            /*!< RCC RCC Global Control  Register,                                        Address offset: 0xA0  */
 uint32_t     RESERVED8;       /*!< Reserved,                                                                Address offset: 0xA4  */
 V uint32_t D3AMR;          /*!< RCC Domain 3 Autonomous Mode Register,                                   Address offset: 0xA8  */
 uint32_t     RESERVED11[8];    /*!< Reserved, 0xAC-0xCC                                                      Address offset: 0xAC  */
 V uint32_t RSR;            /*!< RCC Reset status register,                                               Address offset: 0xD0  */
 V uint32_t AHB3ENR;        /*!< RCC AHB3 peripheral clock  register,                                     Address offset: 0xD4  */
 V uint32_t AHB1ENR;        /*!< RCC AHB1 peripheral clock  register,                                     Address offset: 0xD8  */
 V uint32_t AHB2ENR;        /*!< RCC AHB2 peripheral clock  register,                                     Address offset: 0xDC  */
 V uint32_t AHB4ENR;        /*!< RCC AHB4 peripheral clock  register,                                     Address offset: 0xE0  */
 V uint32_t APB3ENR;        /*!< RCC APB3 peripheral clock  register,                                     Address offset: 0xE4  */
 V uint32_t APB1LENR;       /*!< RCC APB1 peripheral clock  Low Word register,                            Address offset: 0xE8  */
 V uint32_t APB1HENR;       /*!< RCC APB1 peripheral clock  High Word register,                           Address offset: 0xEC  */
 V uint32_t APB2ENR;        /*!< RCC APB2 peripheral clock  register,                                     Address offset: 0xF0  */
 V uint32_t APB4ENR;        /*!< RCC APB4 peripheral clock  register,                                     Address offset: 0xF4  */
 uint32_t      RESERVED12;      /*!< Reserved,                                                                Address offset: 0xF8  */
 V uint32_t AHB3LPENR;      /*!< RCC AHB3 peripheral sleep clock  register,                               Address offset: 0xFC  */
 V uint32_t AHB1LPENR;      /*!< RCC AHB1 peripheral sleep clock  register,                               Address offset: 0x100 */
 V uint32_t AHB2LPENR;      /*!< RCC AHB2 peripheral sleep clock  register,                               Address offset: 0x104 */
 V uint32_t AHB4LPENR;      /*!< RCC AHB4 peripheral sleep clock  register,                               Address offset: 0x108 */
 V uint32_t APB3LPENR;      /*!< RCC APB3 peripheral sleep clock  register,                               Address offset: 0x10C */
 V uint32_t APB1LLPENR;     /*!< RCC APB1 peripheral sleep clock  Low Word register,                      Address offset: 0x110 */
 V uint32_t APB1HLPENR;     /*!< RCC APB1 peripheral sleep clock  High Word register,                     Address offset: 0x114 */
 V uint32_t APB2LPENR;      /*!< RCC APB2 peripheral sleep clock  register,                               Address offset: 0x118 */
 V uint32_t APB4LPENR;      /*!< RCC APB4 peripheral sleep clock  register,                               Address offset: 0x11C */
 uint32_t     RESERVED13[4];   /*!< Reserved, 0x120-0x12C                                                    Address offset: 0x120 */
 V uint32_t C1_AHB3ENR;
 V uint32_t C1_AHB1ENR;
 V uint32_t C1_AHB2ENR;
 V uint32_t C1_AHB4ENR;
 V uint32_t C1_APB3ENR;
 V uint32_t C1_APB1ENR;
 V uint32_t C1_APB1LENR;
 V uint32_t C1_APB1HENR;
 V uint32_t C1_APB2ENR;
 V uint32_t C1_APB4ENR;
  uint32_t RESERVED14;
 V uint32_t C1_AHB3LPENR;
 V uint32_t C1_AHB1LPENR;
 V uint32_t C1_AHB2LPENR;
 V uint32_t C1_AHB4LPENR;
 V uint32_t C1_APB3LPENR;
 V uint32_t C1_APB1LLPENR;
 V uint32_t C1_APB1HLPENR;
 V uint32_t C1_APB2LPENR;
 V uint32_t C1_APB4LPENR;
 V uint32_t RESERVED15[18];


} RCC_RegDef_t;


/**
  * @brief External Interrupt/Event Controller
  */

typedef struct
{
V uint32_t RTSR1;               /*!< EXTI Rising trigger selection register,          Address offset: 0x00 */
V uint32_t FTSR1;               /*!< EXTI Falling trigger selection register,         Address offset: 0x04 */
V uint32_t SWIER1;              /*!< EXTI Software interrupt event register,          Address offset: 0x08 */
V uint32_t D3PMR1;              /*!< EXTI D3 Pending mask register, (same register as to SRDPMR1) Address offset: 0x0C */
V uint32_t D3PCR1L;             /*!< EXTI D3 Pending clear selection register low, (same register as to SRDPCR1L)     Address offset: 0x10 */
V uint32_t D3PCR1H;             /*!< EXTI D3 Pending clear selection register High, (same register as to SRDPCR1H)   Address offset: 0x14 */
uint32_t      RESERVED1[2];        /*!< Reserved,                                        0x18 to 0x1C         */
V uint32_t RTSR2;               /*!< EXTI Rising trigger selection register,          Address offset: 0x20 */
V uint32_t FTSR2;               /*!< EXTI Falling trigger selection register,         Address offset: 0x24 */
V uint32_t SWIER2;              /*!< EXTI Software interrupt event register,          Address offset: 0x28 */
V uint32_t D3PMR2;              /*!< EXTI D3 Pending mask register, (same register as to SRDPMR2) Address offset: 0x2C */
V uint32_t D3PCR2L;             /*!< EXTI D3 Pending clear selection register low, (same register as to SRDPCR2L)  Address offset: 0x30 */
V uint32_t D3PCR2H;             /*!< EXTI D3 Pending clear selection register High, (same register as to SRDPCR2H) Address offset: 0x34 */
 uint32_t      RESERVED2[2];        /*!< Reserved,                                        0x38 to 0x3C         */
V uint32_t RTSR3;               /*!< EXTI Rising trigger selection register,          Address offset: 0x40 */
V uint32_t FTSR3;               /*!< EXTI Falling trigger selection register,         Address offset: 0x44 */
V uint32_t SWIER3;              /*!< EXTI Software interrupt event register,          Address offset: 0x48 */
V uint32_t D3PMR3;              /*!< EXTI D3 Pending mask register, (same register as to SRDPMR3) Address offset: 0x4C */
V uint32_t D3PCR3L;             /*!< EXTI D3 Pending clear selection register low, (same register as to SRDPCR3L) Address offset: 0x50 */
V uint32_t D3PCR3H;             /*!< EXTI D3 Pending clear selection register High, (same register as to SRDPCR3H) Address offset: 0x54 */
uint32_t      RESERVED3[10];       /*!< Reserved,                                        0x58 to 0x7C         */
V uint32_t IMR1;                /*!< EXTI Interrupt mask register,                    Address offset: 0x80 */
V uint32_t EMR1;                /*!< EXTI Event mask register,                        Address offset: 0x84 */
V uint32_t PR1;                 /*!< EXTI Pending register,                           Address offset: 0x88 */
uint32_t      RESERVED4;           /*!< Reserved,                                        0x8C                 */
V uint32_t IMR2;                /*!< EXTI Interrupt mask register,                    Address offset: 0x90 */
V uint32_t EMR2;                /*!< EXTI Event mask register,                        Address offset: 0x94 */
V uint32_t PR2;                 /*!< EXTI Pending register,                           Address offset: 0x98 */
uint32_t      RESERVED5;           /*!< Reserved,                                        0x9C                 */
V uint32_t IMR3;                /*!< EXTI Interrupt mask register,                    Address offset: 0xA0 */
V uint32_t EMR3;                /*!< EXTI Event mask register,                        Address offset: 0xA4 */
V uint32_t PR3;                 /*!< EXTI Pending register,                           Address offset: 0xA8 */
uint32_t      RESERVED6[5];           /*!< Reserved,                                   0xAC to 0xBC         */
}EXTI_RegDef_t;

typedef struct
{
V uint32_t IMR1;                /*!< EXTI Interrupt mask register,                Address offset: 0x00 */
V uint32_t EMR1;                /*!< EXTI Event mask register,                    Address offset: 0x04 */
V uint32_t PR1;                 /*!< EXTI Pending register,                       Address offset: 0x08 */
uint32_t      RESERVED1;           /*!< Reserved, 0x0C                                                    */
V uint32_t IMR2;                /*!< EXTI Interrupt mask register,                Address offset: 0x10 */
V uint32_t EMR2;                /*!< EXTI Event mask register,                    Address offset: 0x14 */
V uint32_t PR2;                 /*!< EXTI Pending register,                       Address offset: 0x18 */
uint32_t      RESERVED2;           /*!< Reserved, 0x1C                                                    */
V uint32_t IMR3;                /*!< EXTI Interrupt mask register,                Address offset: 0x20 */
V uint32_t EMR3;                /*!< EXTI Event mask register,                    Address offset: 0x24 */
V uint32_t PR3;                 /*!< EXTI Pending register,                       Address offset: 0x28 */
}EXTI_Core_RegDef_t;


/* Legacy defines */
#define USB_OTG_HS                   USB1_OTG_HS
#define USB_OTG_HS_PERIPH_BASE       USB1_OTG_HS_PERIPH_BASE
#define USB_OTG_FS                   USB2_OTG_FS
#define USB_OTG_FS_PERIPH_BASE       USB2_OTG_FS_PERIPH_BASE




  /*
   * clock enable macros for GPIOx peripherals
   */

  #define GPIOA_PCLK_EN()  	(RCC-> AHB4ENR |= (1 << 0))
  #define GPIOB_PCLK_EN()  	(RCC-> AHB4ENR |= (1 << 1))
  #define GPIOC_PCLK_EN()  	(RCC-> AHB4ENR |= (1 << 2))
  #define GPIOD_PCLK_EN()  	(RCC-> AHB4ENR |= (1 << 3))
  #define GPIOE_PCLK_EN()  	(RCC-> AHB4ENR |= (1 << 4))
  #define GPIOF_PCLK_EN()  	(RCC-> AHB4ENR |= (1 << 5))
  #define GPIOG_PCLK_EN()  	(RCC-> AHB4ENR |= (1 << 6))
  #define GPIOH_PCLK_EN()  	(RCC-> AHB4ENR |= (1 << 7))
  #define GPIOI_PCLK_EN()  	(RCC-> AHB4ENR |= (1 << 8))
 #define GPIOJ_PCLK_EN()  	(RCC-> AHB4ENR |= (1 << 9))
 #define GPIOK_PCLK_EN()  	(RCC-> AHB4ENR |= (1 << 10))


/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()      ( RCC->APB2ENR |= (1 << 12) ) /* SPI1 peripheral clock enabled */
#define SPI2_PCLK_EN()      ( RCC->APB1LENR |= (1 << 14) ) /* SPI2 peripheral clock enabled */
#define SPI3_PCLK_EN()      ( RCC->APB1LENR |= (1 << 15) ) /* SPI3 peripheral clock enabled */
#define SPI4_PCLK_EN()      ( RCC->APB2ENR |= (1 << 13) ) /* SPI4 peripheral clock enabled */
#define SPI5_PCLK_EN()      ( RCC->APB2ENR |= (1 << 20) ) /* SPI5 peripheral clock enabled */
#define SPI6_PCLK_EN()      ( RCC->APB4ENR |= (1 << 5) ) /* SPI6 peripheral clock enabled */
  /*
   * clock enable macros for SYSCFG peripherals
   */

  #define SYSCFG_PCLK_EN()  (RCC-> APB4ENR |= (1<<2) )




  /*
   * clock disable macros for GPIO peripherals
   */

  #define GPIOA_PCLK_DEN()  ~(RCC-> AHB4ENR &= (1 << 0))
  #define GPIOB_PCLK_DEN()  ~(RCC-> AHB4ENR &= (1<<1) )
  #define GPIOC_PCLK_DEN()  ~(RCC-> AHB4ENR &= (1<<2) )
  #define GPIOD_PCLK_DEN()  ~(RCC-> AHB4ENR &= (1<<3) )
  #define GPIOE_PCLK_DEN()  ~(RCC-> AHB4ENR &= (1<<4) )
  #define GPIOF_PCLK_DEN()  ~(RCC-> AHB4ENR &= (1<<5) )
  #define GPIOG_PCLK_DEN()  ~(RCC-> AHB4ENR &= (1<<6) )
  #define GPIOH_PCLK_DEN()  ~(RCC-> AHB4ENR &= (1<<7) )
  #define GPIOI_PCLK_DEN()  ~(RCC-> AHB4ENR &= (1<<8) )
  #define GPIOJ_PCLK_DEN()  ~(RCC-> AHB4ENR &= (1<<9) )
  #define GPIOK_PCLK_DEN()  ~(RCC-> APB4ENR &= (1<<10) )


/*
 * Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DEN()      ( RCC->APB2ENR &= ~(1 << 12) ) /* SPI1 peripheral clock disabled */
#define SPI2_PCLK_DEN()      ( RCC->APB1LENR &= ~(1 << 14) ) /* SPI2 peripheral clock disabled */
#define SPI3_PCLK_DEN()      ( RCC->APB1LENR &= ~(1 << 15) ) /* SPI3 peripheral clock disabled */
#define SPI4_PCLK_DEN()      ( RCC->APB2ENR &= ~(1 << 13) ) /* SPI4 peripheral clock disabled */
#define SPI5_PCLK_DEN()      ( RCC->APB2ENR &= ~(1 << 20) ) /* SPI5 peripheral clock disabled */
#define SPI6_PCLK_DEN()      ( RCC->APB4ENR &= ~(1 << 5) ) /* SPI6 peripheral clock disabled */

  /*
   * clock disable macros for SYSCFG peripherals
   */

  #define SYSCFG_PCLK_DEN()  (RCC-> APB2ENR &= (0<<14) )






  /*
   * Macros to reset GPIOx peripherals
   */

  #define GPIOA_REG_RESET()         do{ (RCC-> AHB4RSTR |= (1<<0)) ;   (RCC-> AHB4RSTR &= ~(1<<0));} while(0)
  #define GPIOB_REG_RESET()         do{ (RCC-> AHB4RSTR |= (1<<0)) ;   (RCC-> AHB4RSTR &= ~(1<<0));} while(0)
  #define GPIOC_REG_RESET()         do{ (RCC-> AHB4RSTR |= (1<<0)) ;   (RCC-> AHB4RSTR &= ~(1<<0));} while(0)
  #define GPIOD_REG_RESET()         do{ (RCC-> AHB4RSTR |= (1<<0)) ;   (RCC-> AHB4RSTR &= ~(1<<0));} while(0)
  #define GPIOE_REG_RESET()         do{ (RCC-> AHB4RSTR |= (1<<0)) ;   (RCC-> AHB4RSTR &= ~(1<<0));} while(0)
  #define GPIOF_REG_RESET()         do{ (RCC-> AHB4RSTR |= (1<<0)) ;   (RCC-> AHB4RSTR &= ~(1<<0));} while(0)
  #define GPIOG_REG_RESET()         do{ (RCC-> AHB4RSTR |= (1<<0)) ;   (RCC-> AHB4RSTR &= ~(1<<0));} while(0)
  #define GPIOH_REG_RESET()         do{ (RCC-> AHB4RSTR |= (1<<0)) ;   (RCC-> AHB4RSTR &= ~(1<<0));} while(0)
 #define GPIOI_REG_RESET()         do{ (RCC-> AHB4RSTR |= (1<<0)) ;   (RCC-> AHB4RSTR &= ~(1<<0));} while(0)
 #define GPIOJ_REG_RESET()         do{ (RCC-> AHB4RSTR |= (1<<0)) ;   (RCC-> AHB4RSTR &= ~(1<<0));} while(0)
 #define GPIOK_REG_RESET()         do{ (RCC-> AHB4RSTR |= (1<<0)) ;   (RCC-> AHB4RSTR &= ~(1<<0));} while(0)


/*
 * Macros to reset SPIx peripherals
 */
#define SPI1_REG_RESET()    do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()    do{ (RCC->APB1LRSTR |= (1 << 14)); (RCC->APB1LRSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()    do{ (RCC->APB1LRSTR |= (1 << 15)); (RCC->APB1LRSTR &= ~(1 << 15)); }while(0)
#define SPI4_REG_RESET()    do{ (RCC->APB2RSTR |= (1 << 13)); (RCC->APB2RSTR &= ~(1 << 13)); }while(0)
#define SPI5_REG_RESET()    do{ (RCC->APB2RSTR |= (1 << 20)); (RCC->APB2RSTR &= ~(1 << 20)); }while(0)
#define SPI6_REG_RESET()    do{ (RCC->APB4RSTR |= (1 << 5)); (RCC->AHB4RSTR &= ~(1 << 5)); }while(0)


#define GPIO_BADDR_TO_CODE(x)      ((x == GPIOA) ? 0 :\
								   (x == GPIOB) ? 1 :\
								   (x == GPIOC) ? 2 :\
								   (x == GPIOD) ? 3 :\
								   (x == GPIOE) ? 4 :\
								   (x == GPIOF) ? 5 :\
								   (x == GPIOG) ? 6 :\
								   (x == GPIOH) ? 7 :\
                                   (x == GPIOI) ? 8 :\
								   (x == GPIOJ) ? 9 :\
								   (x == GPIOK) ? 10:0 )


/** @addtogroup Peripheral_interrupt_number_definition
  * @{
  */

/**
 * @brief STM32H7XX Interrupt Number Definition, according to the selected device
 *        in @ref Library_configuration_section
 */
typedef enum
{
/******  Cortex-M Processor Exceptions Numbers *****************************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                                          */
  HardFault_IRQn              = -13,    /*!< 4 Cortex-M Memory Management Interrupt                            */
  MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M Memory Management Interrupt                            */
  BusFault_IRQn               = -11,    /*!< 5 Cortex-M Bus Fault Interrupt                                    */
  UsageFault_IRQn             = -10,    /*!< 6 Cortex-M Usage Fault Interrupt                                  */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M SV Call Interrupt                                     */
  DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M Debug Monitor Interrupt                               */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M Pend SV Interrupt                                     */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M System Tick Interrupt                                 */
/******  STM32 specific Interrupt Numbers **********************************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt ( wwdg1_it, wwdg2_it)                   */
  PVD_AVD_IRQn                = 1,      /*!< PVD/AVD through EXTI Line detection Interrupt                     */
  TAMP_STAMP_IRQn             = 2,      /*!< Tamper and TimeStamp interrupts through the EXTI line             */
  RTC_WKUP_IRQn               = 3,      /*!< RTC Wakeup interrupt through the EXTI line                        */
  FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                                            */
  RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                              */
  EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                              */
  EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                              */
  EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                              */
  EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                              */
  EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                              */
  DMA1_Stream0_IRQn           = 11,     /*!< DMA1 Stream 0 global Interrupt                                    */
  DMA1_Stream1_IRQn           = 12,     /*!< DMA1 Stream 1 global Interrupt                                    */
  DMA1_Stream2_IRQn           = 13,     /*!< DMA1 Stream 2 global Interrupt                                    */
  DMA1_Stream3_IRQn           = 14,     /*!< DMA1 Stream 3 global Interrupt                                    */
  DMA1_Stream4_IRQn           = 15,     /*!< DMA1 Stream 4 global Interrupt                                    */
  DMA1_Stream5_IRQn           = 16,     /*!< DMA1 Stream 5 global Interrupt                                    */
  DMA1_Stream6_IRQn           = 17,     /*!< DMA1 Stream 6 global Interrupt                                    */
  ADC_IRQn                    = 18,     /*!< ADC1 and  ADC2 global Interrupts                                  */
  FDCAN1_IT0_IRQn             = 19,     /*!< FDCAN1 Interrupt line 0                                           */
  FDCAN2_IT0_IRQn             = 20,     /*!< FDCAN2 Interrupt line 0                                           */
  FDCAN1_IT1_IRQn             = 21,     /*!< FDCAN1 Interrupt line 1                                           */
  FDCAN2_IT1_IRQn             = 22,     /*!< FDCAN2 Interrupt line 1                                           */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                                     */
  TIM1_BRK_IRQn               = 24,     /*!< TIM1 Break Interrupt                                              */
  TIM1_UP_IRQn                = 25,     /*!< TIM1 Update Interrupt                                             */
  TIM1_TRG_COM_IRQn           = 26,     /*!< TIM1 Trigger and Commutation Interrupt                            */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                                    */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                             */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                             */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                             */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                              */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                              */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                              */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                              */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                             */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                             */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                                           */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                                           */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                                           */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
  TIM8_BRK_TIM12_IRQn         = 43,     /*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
  TIM8_UP_TIM13_IRQn          = 44,     /*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
  TIM8_TRG_COM_TIM14_IRQn     = 45,     /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
  TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare Interrupt                                    */
  DMA1_Stream7_IRQn           = 47,     /*!< DMA1 Stream7 Interrupt                                            */
  FMC_IRQn                    = 48,     /*!< FMC global Interrupt                                              */
  SDMMC1_IRQn                 = 49,     /*!< SDMMC1 global Interrupt                                           */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                             */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                                            */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                                            */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
  TIM7_IRQn                   = 55,     /*!< TIM7 global interrupt                                             */
  DMA2_Stream0_IRQn           = 56,     /*!<   DMA2 Stream 0 global Interrupt                                  */
  DMA2_Stream1_IRQn           = 57,     /*!<   DMA2 Stream 1 global Interrupt                                  */
  DMA2_Stream2_IRQn           = 58,     /*!<   DMA2 Stream 2 global Interrupt                                  */
  DMA2_Stream3_IRQn           = 59,     /*!<   DMA2 Stream 3 global Interrupt                                  */
  DMA2_Stream4_IRQn           = 60,     /*!<   DMA2 Stream 4 global Interrupt                                  */
  ETH_IRQn                    = 61,     /*!< Ethernet global Interrupt                                         */
  ETH_WKUP_IRQn               = 62,     /*!< Ethernet Wakeup through EXTI line Interrupt                       */
  FDCAN_CAL_IRQn              = 63,     /*!< FDCAN Calibration unit Interrupt                                  */
  DMA2_Stream5_IRQn           = 68,     /*!< DMA2 Stream 5 global interrupt                                    */
  DMA2_Stream6_IRQn           = 69,     /*!< DMA2 Stream 6 global interrupt                                    */
  DMA2_Stream7_IRQn           = 70,     /*!< DMA2 Stream 7 global interrupt                                    */
  USART6_IRQn                 = 71,     /*!< USART6 global interrupt                                           */
  I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
  OTG_HS_EP1_OUT_IRQn         = 74,     /*!< USB OTG HS End Point 1 Out global interrupt                       */
  OTG_HS_EP1_IN_IRQn          = 75,     /*!< USB OTG HS End Point 1 In global interrupt                        */
  OTG_HS_WKUP_IRQn            = 76,     /*!< USB OTG HS Wakeup through EXTI interrupt                          */
  OTG_HS_IRQn                 = 77,     /*!< USB OTG HS global interrupt                                       */
  DCMI_IRQn                   = 78,     /*!< DCMI global interrupt                                             */
  RNG_IRQn                    = 80,     /*!< RNG global interrupt                                              */
  FPU_IRQn                    = 81,     /*!< FPU global interrupt                                              */
  UART7_IRQn                  = 82,     /*!< UART7 global interrupt                                            */
  UART8_IRQn                  = 83,     /*!< UART8 global interrupt                                            */
  SPI4_IRQn                   = 84,     /*!< SPI4 global Interrupt                                             */
  SPI5_IRQn                   = 85,     /*!< SPI5 global Interrupt                                             */
  SPI6_IRQn                   = 86,     /*!< SPI6 global Interrupt                                             */
  SAI1_IRQn                   = 87,     /*!< SAI1 global Interrupt                                             */
  LTDC_IRQn                   = 88,     /*!< LTDC global Interrupt                                             */
  LTDC_ER_IRQn                = 89,     /*!< LTDC Error global Interrupt                                       */
  DMA2D_IRQn                  = 90,     /*!< DMA2D global Interrupt                                            */
  SAI2_IRQn                   = 91,     /*!< SAI2 global Interrupt                                             */
  QUADSPI_IRQn                = 92,     /*!< Quad SPI global interrupt                                         */
  LPTIM1_IRQn                 = 93,     /*!< LP TIM1 interrupt                                                 */
  CEC_IRQn                    = 94,     /*!< HDMI-CEC global Interrupt                                         */
  I2C4_EV_IRQn                = 95,     /*!< I2C4 Event Interrupt                                              */
  I2C4_ER_IRQn                = 96,     /*!< I2C4 Error Interrupt                                              */
  SPDIF_RX_IRQn               = 97,     /*!< SPDIF-RX global Interrupt                                         */
  OTG_FS_EP1_OUT_IRQn         = 98,     /*!< USB OTG HS2 global interrupt                                      */
  OTG_FS_EP1_IN_IRQn          = 99,     /*!< USB OTG HS2 End Point 1 Out global interrupt                      */
  OTG_FS_WKUP_IRQn            = 100,    /*!< USB OTG HS2 End Point 1 In global interrupt                       */
  OTG_FS_IRQn                 = 101,    /*!< USB OTG HS2 Wakeup through EXTI interrupt                         */
  DMAMUX1_OVR_IRQn            = 102,    /*!<DMAMUX1 Overrun interrupt                                          */
  HRTIM1_Master_IRQn          = 103,    /*!< HRTIM Master Timer global Interrupts                              */
  HRTIM1_TIMA_IRQn            = 104,    /*!< HRTIM Timer A global Interrupt                                    */
  HRTIM1_TIMB_IRQn            = 105,    /*!< HRTIM Timer B global Interrupt                                    */
  HRTIM1_TIMC_IRQn            = 106,    /*!< HRTIM Timer C global Interrupt                                    */
  HRTIM1_TIMD_IRQn            = 107,    /*!< HRTIM Timer D global Interrupt                                    */
  HRTIM1_TIME_IRQn            = 108,    /*!< HRTIM Timer E global Interrupt                                    */
  HRTIM1_FLT_IRQn             = 109,    /*!< HRTIM Fault global Interrupt                                      */
  DFSDM1_FLT0_IRQn            = 110,    /*!<DFSDM Filter1 Interrupt                                            */
  DFSDM1_FLT1_IRQn            = 111,    /*!<DFSDM Filter2 Interrupt                                            */
  DFSDM1_FLT2_IRQn            = 112,    /*!<DFSDM Filter3 Interrupt                                            */
  DFSDM1_FLT3_IRQn            = 113,    /*!<DFSDM Filter4 Interrupt                                            */
  SAI3_IRQn                   = 114,    /*!< SAI3 global Interrupt                                             */
  SWPMI1_IRQn                 = 115,    /*!< Serial Wire Interface 1 global interrupt                          */
  TIM15_IRQn                  = 116,    /*!< TIM15 global Interrupt                                            */
  TIM16_IRQn                  = 117,    /*!< TIM16 global Interrupt                                            */
  TIM17_IRQn                  = 118,    /*!< TIM17 global Interrupt                                            */
  MDIOS_WKUP_IRQn             = 119,    /*!< MDIOS Wakeup  Interrupt                                           */
  MDIOS_IRQn                  = 120,    /*!< MDIOS global Interrupt                                            */
  JPEG_IRQn                   = 121,    /*!< JPEG global Interrupt                                             */
  MDMA_IRQn                   = 122,    /*!< MDMA global Interrupt                                             */
  SDMMC2_IRQn                 = 124,    /*!< SDMMC2 global Interrupt                                           */
  HSEM1_IRQn                  = 125,    /*!< HSEM1 global Interrupt                                            */
  ADC3_IRQn                   = 127,    /*!< ADC3 global Interrupt                                             */
  DMAMUX2_OVR_IRQn            = 128,    /*!<DMAMUX2 Overrun interrupt                                          */
  BDMA_Channel0_IRQn          = 129,    /*!< BDMA Channel 0 global Interrupt                                   */
  BDMA_Channel1_IRQn          = 130,    /*!< BDMA Channel 1 global Interrupt                                   */
  BDMA_Channel2_IRQn          = 131,    /*!< BDMA Channel 2 global Interrupt                                   */
  BDMA_Channel3_IRQn          = 132,    /*!< BDMA Channel 3 global Interrupt                                   */
  BDMA_Channel4_IRQn          = 133,    /*!< BDMA Channel 4 global Interrupt                                   */
  BDMA_Channel5_IRQn          = 134,    /*!< BDMA Channel 5 global Interrupt                                   */
  BDMA_Channel6_IRQn          = 135,    /*!< BDMA Channel 6 global Interrupt                                   */
  BDMA_Channel7_IRQn          = 136,    /*!< BDMA Channel 7 global Interrupt                                   */
  COMP_IRQn                   = 137 ,   /*!< COMP global Interrupt                                             */
  LPTIM2_IRQn                 = 138,    /*!< LP TIM2 global interrupt                                          */
  LPTIM3_IRQn                 = 139,    /*!< LP TIM3 global interrupt                                          */
  LPTIM4_IRQn                 = 140,    /*!< LP TIM4 global interrupt                                          */
  LPTIM5_IRQn                 = 141,    /*!< LP TIM5 global interrupt                                          */
  LPUART1_IRQn                = 142,    /*!< LP UART1 interrupt                                                */
  CRS_IRQn                    = 144,    /*!< Clock Recovery Global Interrupt                                   */
  ECC_IRQn                    = 145,    /*!< ECC diagnostic Global Interrupt                                   */
  SAI4_IRQn                   = 146,    /*!< SAI4 global interrupt                                             */
  WAKEUP_PIN_IRQn             = 149,    /*!< Interrupt for all 6 wake-up pins                                  */
} IRQn_Type;

/**
  * @}
  */

/*
 * macros for all the possible priority levels
 */

#define NVIC_IRQ_PRI0          0
#define NVIC_IRQ_PRI15         15


#define Enable			 1
#define Disable 	     0
#define SET				 Enable
#define RESET 			 Disable
#define	GPIO_PIN_SET     SET
#define	GPIO_PIN_RESET   RESET
#define	FLAG_SET         SET
#define	FLAG_RESET         RESET

/************************************************************************************************************************************************************
 *                                        BIT POSITION DEFINITION FOR SPI PHERIPERAL REGISTERS
 **************************************************************************************************************************************************************/

/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_SPE        		0       // serial peripheral enable
#define SPI_CR1_MASRX         	8       //enables master to automatically suspends in receiver mode if overrun condition happens
#define SPI_CR1_CSTART         	9       // master transfer start(starts spi/i12s communication
#define SPI_CR1_CSUSP     		10		// master suspend request
#define SPI_CR1_HDDIR         	11      //  Rx/TX direction at half duplex mode
#define SPI_CR1_SSI   			12		// internal ss signal input level(this bit has an effect only when the ssm bit is set
#define SPI_CR1_CRC33_17     	13		// crc config
#define SPI_CR1_RCRCINI     	14		// crc calculation config for receiver
#define SPI_CR1_TCRCINI   		15		// crc calculation config for transmitter
#define SPI_CR1_IOLOCK   		16		// to lock AF config of associated I/Os

/*
 * Bit position definitions SPI_CFG1
 */

#define SPI_CFG1_DSIZE        	0     //  decides no. of bits at single SPI data frame
#define SPI_CFG1_FTHLV        	5     //  FIFO threshold level(defines no. of data frames at single data packet)
#define SPI_CFG1_UDRCFG        	9     //  behaviour of slave transmitter at underrun condition
#define SPI_CFG1_UDRDET        	11    //  to detect underrun condition at slave transmitter
#define SPI_CFG1_RxDMAEN        14    //  AS THE NAME ITSELF SAYS, Rx DMA STREAM ENABLE REGISTER
#define SPI_CFG1_TxDMAEN        15    //  Rx DMA STREAM ENABLE REGISTER
#define SPI_CFG1_CRCSIZE        16    //  LENGTH OF CRC FRAME TO BE TRANSACTED AND COMPARED
#define SPI_CFG1_CRCEN 			22    //  HARDWARE CRC COMPUTATION ENABLE?1:0
#define SPI_CFG1_MBR        	29    //  MASTER BAUD RATE


/*
 * Bit position definitions SPI_CFG2
 */

#define SPI_CFG2_MSSI       	0		// master SS idleness
#define SPI_CFG2_MIDI     		4       // master inter-data idleness
#define SPI_CFG2_IOSWP     		15		// swap functionality of MISO and MOSI pins
#define SPI_CFG2_COMM     		17		// communication mode(FD,HD,SIMPLEX)
#define SPI_CFG2_SP     		19		// seraial protocol( motorola or TI)
#define SPI_CFG2_MASTER     	22		// spi master or slave select register
#define SPI_CFG2_LSBFRST     	23		// data frame format(lsb or msb transmitted first)
#define SPI_CFG2_CPHA     		24		// clock phase
#define SPI_CFG2_CPOL     		25		// clock polarity
#define SPI_CFG2_SSM     		26		// software management of SS signal input
#define SPI_CFG2_SSIOP     		28		// ss input/output polarity
#define SPI_CFG2_SSOE     		29		//ss output enable
#define SPI_CFG2_SSOM     		30		// ss ouput management in master mode
#define SPI_CFG2_AFCNTR     	31		// alternate function control(taen into account only when spe=0)


/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXP       	    0		//rx packet available
#define SPI_SR_TXP       	    1		// tx packet space available
#define SPI_SR_DXP		        2		// duplex packet
#define SPI_SR_EOT  	        3		// end of transfer
#define SPI_SR_TXTF     		4		// transmission tranfer filled
#define SPI_SR_UDR       	    5		// underrun
#define SPI_SR_OVR        		6		// overrun
#define SPI_SR_CRCE        	  	7		// CRC error
#define SPI_SR_TIFRE         	8		// ti frame format error
#define SPI_SR_MODF         	9		// mode fault
#define SPI_SR_TSERF         	10
#define SPI_SR_SUSP             11		// suspension status
#define SPI_SR_TXC             	12		// TxFIFO transmission complete
#define SPI_SR_RXPLVL         	13		// RxFIFO packing level
#define SPI_SR_RXWNE         	14		// RxFIFO word not empty
#define SPI_SR_CTSIZE         	15      // no. of data frames remaining in current TSIZE session


/*
 * Bit position definitions SPI_IER
 */
#define SPI_IER_RXPIE             0
#define SPI_IER_TXPIE             1
#define SPI_IER_OVRIE             6





#include "stm32h753_gpio_driver.h"
#include "stm32h753_spi_driver.h"


#endif /* STM32H753XX_H_ */















