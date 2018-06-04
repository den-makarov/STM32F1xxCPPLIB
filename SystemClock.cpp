#include "SystemClock.h"

namespace STM32F107VC_DRIVER {
namespace SYS_CLK {

SystemClock *SystemClock::instance = nullptr;

SystemClock::SystemClock()
{
#ifdef USE_HAL_DRIVER
    HAL_Init();
#endif /* USE_HAL_DRIVER */
}

SystemClock *SystemClock::getInstance()
{
    if(instance)
    {
        instance = new SystemClock();
    }
    return instance;
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 72000000
  *            HCLK(Hz)                       = 72000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 25000000
  *            HSE PREDIV1                    = 5
  *            HSE PREDIV2                    = 5
  *            PLL2MUL                        = 8
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
void SystemClock::systemClockConfig(void)
{
    RCC_ClkInitTypeDef clkinitstruct = {0, 0, 0, 0, 0};
    RCC_OscInitTypeDef oscinitstruct = {0, 0, 0, 0, 0, 0, 0, 0, {0, 0, 0}, {0, 0, 0}};

    /* Configure PLLs ------------------------------------------------------*/
    /* PLL2 configuration: PLL2CLK = (HSE / HSEPrediv2Value) * PLL2MUL = (25 / 5) * 8 = 40 MHz */
    /* PREDIV1 configuration: PREDIV1CLK = PLL2CLK / HSEPredivValue = 40 / 5 = 8 MHz */
    /* PLL configuration: PLLCLK = PREDIV1CLK * PLLMUL = 8 * 9 = 72 MHz */

    /* Enable HSE Oscillator and activate PLL with HSE as source */
    oscinitstruct.OscillatorType        = RCC_OSCILLATORTYPE_HSE;
    oscinitstruct.HSEState              = RCC_HSE_ON;
    oscinitstruct.HSEPredivValue        = RCC_HSE_PREDIV_DIV5;
    oscinitstruct.Prediv1Source         = RCC_PREDIV1_SOURCE_PLL2;
    oscinitstruct.PLL.PLLState          = RCC_PLL_ON;
    oscinitstruct.PLL.PLLSource         = RCC_PLLSOURCE_HSE;
    oscinitstruct.PLL.PLLMUL            = RCC_PLL_MUL9;
    oscinitstruct.PLL2.PLL2State        = RCC_PLL2_ON;
    oscinitstruct.PLL2.PLL2MUL          = RCC_PLL2_MUL8;
    oscinitstruct.PLL2.HSEPrediv2Value  = RCC_HSE_PREDIV2_DIV5;

#ifdef USE_HAL_DRIVER
    if(HAL_RCC_OscConfig(&oscinitstruct) != HAL_OK)
    {
        /* Initialization Error */
        while(1);
    }
#endif /* USE_HAL_DRIVER */

    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
    clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
    clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;

#ifdef USE_HAL_DRIVER
    if(HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2)!= HAL_OK)
    {
        /* Initialization Error */
        while(1);
    }
#endif /* USE_HAL_DRIVER */
}

void SystemClock::periphClockEnable(PeripheryClockEnum clk)
{
#ifdef USE_HAL_DRIVER
    switch(clk)
    {
        case CRC_CLK: __HAL_RCC_CRC_CLK_ENABLE(); break;
        case FLITF_CLK: __HAL_RCC_FLITF_CLK_ENABLE(); break;
        case SRAM_CLK: __HAL_RCC_SRAM_CLK_ENABLE(); break;
        case AFIO_CLK: __HAL_RCC_AFIO_CLK_ENABLE(); break;

        case GPIOA_CLK: __HAL_RCC_GPIOA_CLK_ENABLE(); break;
        case GPIOB_CLK: __HAL_RCC_GPIOB_CLK_ENABLE(); break;
        case GPIOC_CLK: __HAL_RCC_GPIOC_CLK_ENABLE(); break;
        case GPIOD_CLK: __HAL_RCC_GPIOD_CLK_ENABLE(); break;
        case GPIOE_CLK: __HAL_RCC_GPIOE_CLK_ENABLE(); break;

        case ADC1_CLK: __HAL_RCC_ADC1_CLK_ENABLE(); break;
        case ADC2_CLK: __HAL_RCC_ADC2_CLK_ENABLE(); break;
        case DAC_CLK: __HAL_RCC_DAC_CLK_ENABLE(); break;

        case TIM1_CLK: __HAL_RCC_TIM1_CLK_ENABLE(); break;
        case TIM2_CLK: __HAL_RCC_TIM2_CLK_ENABLE(); break;
        case TIM3_CLK: __HAL_RCC_TIM3_CLK_ENABLE(); break;
        case TIM4_CLK: __HAL_RCC_TIM4_CLK_ENABLE(); break;
        case TIM5_CLK: __HAL_RCC_TIM5_CLK_ENABLE(); break;
        case TIM6_CLK: __HAL_RCC_TIM6_CLK_ENABLE(); break;
        case TIM7_CLK: __HAL_RCC_TIM7_CLK_ENABLE(); break;

        case WWDG_CLK: __HAL_RCC_WWDG_CLK_ENABLE(); break;

        case SPI1_CLK: __HAL_RCC_SPI1_CLK_ENABLE(); break;
        case SPI2_CLK: __HAL_RCC_SPI2_CLK_ENABLE(); break;
        case SPI3_CLK: __HAL_RCC_SPI3_CLK_ENABLE(); break;

        case USART1_CLK: __HAL_RCC_USART1_CLK_ENABLE(); break;
        case USART2_CLK: __HAL_RCC_USART2_CLK_ENABLE(); break;
        case USART3_CLK: __HAL_RCC_USART3_CLK_ENABLE(); break;
        case UART4_CLK: __HAL_RCC_UART4_CLK_ENABLE(); break;
        case UART5_CLK: __HAL_RCC_UART5_CLK_ENABLE(); break;

        case I2C1_CLK: __HAL_RCC_I2C1_CLK_ENABLE(); break;
        case I2C2_CLK: __HAL_RCC_I2C2_CLK_ENABLE(); break;

        case CAN1_CLK: __HAL_RCC_CAN1_CLK_ENABLE(); break;
        case CAN2_CLK: __HAL_RCC_CAN2_CLK_ENABLE(); break;

        case BKP_CLK: __HAL_RCC_BKP_CLK_ENABLE(); break;
        case PWR_CLK: __HAL_RCC_PWR_CLK_ENABLE(); break;

        case DMA1_CLK: __HAL_RCC_DMA1_CLK_ENABLE(); break;
        case DMA2_CLK: __HAL_RCC_DMA2_CLK_ENABLE(); break;

        case USB_CLK: __HAL_RCC_USB_OTG_FS_CLK_ENABLE(); break;
        case ETH_CLK: __HAL_RCC_ETH_CLK_ENABLE(); break;

    default: systemClockError(UNDEFINED_PERIPH_CLOCK);
    }
#else /* USE_HAL_DRIVER */
#endif /* USE_HAL_DRIVER */
}

void SystemClock::periphClockDisable(PeripheryClockEnum clk)
{
#ifdef USE_HAL_DRIVER
    switch(clk)
    {
        case CRC_CLK: __HAL_RCC_CRC_CLK_DISABLE(); break;
        case FLITF_CLK: __HAL_RCC_FLITF_CLK_DISABLE(); break;
        case SRAM_CLK: __HAL_RCC_SRAM_CLK_DISABLE(); break;
        case AFIO_CLK: __HAL_RCC_AFIO_CLK_DISABLE(); break;

        case GPIOA_CLK: __HAL_RCC_GPIOA_CLK_DISABLE(); break;
        case GPIOB_CLK: __HAL_RCC_GPIOB_CLK_DISABLE(); break;
        case GPIOC_CLK: __HAL_RCC_GPIOC_CLK_DISABLE(); break;
        case GPIOD_CLK: __HAL_RCC_GPIOD_CLK_DISABLE(); break;
        case GPIOE_CLK: __HAL_RCC_GPIOE_CLK_DISABLE(); break;

        case ADC1_CLK: __HAL_RCC_ADC1_CLK_DISABLE(); break;
        case ADC2_CLK: __HAL_RCC_ADC2_CLK_DISABLE(); break;
        case DAC_CLK: __HAL_RCC_DAC_CLK_DISABLE(); break;

        case TIM1_CLK: __HAL_RCC_TIM1_CLK_DISABLE(); break;
        case TIM2_CLK: __HAL_RCC_TIM2_CLK_DISABLE(); break;
        case TIM3_CLK: __HAL_RCC_TIM3_CLK_DISABLE(); break;
        case TIM4_CLK: __HAL_RCC_TIM4_CLK_DISABLE(); break;
        case TIM5_CLK: __HAL_RCC_TIM5_CLK_DISABLE(); break;
        case TIM6_CLK: __HAL_RCC_TIM6_CLK_DISABLE(); break;
        case TIM7_CLK: __HAL_RCC_TIM7_CLK_DISABLE(); break;

        case WWDG_CLK: __HAL_RCC_WWDG_CLK_DISABLE(); break;

        case SPI1_CLK: __HAL_RCC_SPI1_CLK_DISABLE(); break;
        case SPI2_CLK: __HAL_RCC_SPI2_CLK_DISABLE(); break;
        case SPI3_CLK: __HAL_RCC_SPI3_CLK_DISABLE(); break;

        case USART1_CLK: __HAL_RCC_USART1_CLK_DISABLE(); break;
        case USART2_CLK: __HAL_RCC_USART2_CLK_DISABLE(); break;
        case USART3_CLK: __HAL_RCC_USART3_CLK_DISABLE(); break;
        case UART4_CLK: __HAL_RCC_UART4_CLK_DISABLE(); break;
        case UART5_CLK: __HAL_RCC_UART5_CLK_DISABLE(); break;

        case I2C1_CLK: __HAL_RCC_I2C1_CLK_DISABLE(); break;
        case I2C2_CLK: __HAL_RCC_I2C2_CLK_DISABLE(); break;

        case CAN1_CLK: __HAL_RCC_CAN1_CLK_DISABLE(); break;
        case CAN2_CLK: __HAL_RCC_CAN2_CLK_DISABLE(); break;

        case BKP_CLK: __HAL_RCC_BKP_CLK_DISABLE(); break;
        case PWR_CLK: __HAL_RCC_PWR_CLK_DISABLE(); break;

        case DMA1_CLK: __HAL_RCC_DMA1_CLK_DISABLE(); break;
        case DMA2_CLK: __HAL_RCC_DMA2_CLK_DISABLE(); break;

        case USB_CLK: __HAL_RCC_USB_OTG_FS_CLK_DISABLE(); break;
        case ETH_CLK: __HAL_RCC_ETH_CLK_DISABLE(); break;

    default: systemClockError(UNDEFINED_PERIPH_CLOCK);
    }
#else /* USE_HAL_DRIVER */
#endif /* USE_HAL_DRIVER */
}

void SystemClock::systemClockError(SysClockErrorEnum error)
{
    if(error)
    {
        while(1);
    }
}

} // namespace SYS_CLK
} // namespace STM32F107VC_DRIVER
