#ifndef SYSTEMCLOCK_H
#define SYSTEMCLOCK_H

#include "stm32f1xx.h"

namespace STM32F107VC_DRIVER {
namespace SYS_CLK {

enum PeripheryClockEnum
{
// STM32F107VC APB2 bus peripheral group
    AFIO_CLK = 0u,
    GPIOA_CLK,
    GPIOB_CLK,
    GPIOC_CLK,
    GPIOD_CLK,
    GPIOE_CLK,
    ADC1_CLK,
    ADC2_CLK,
    TIM1_CLK,
    SPI1_CLK,
    USART1_CLK,
// STM32F107VC APB1 bus peripheral group
    TIM2_CLK,
    TIM3_CLK,
    TIM4_CLK,
    TIM5_CLK,
    TIM6_CLK,
    TIM7_CLK,
    WWDG_CLK,
    SPI2_CLK,
    SPI3_CLK,
    USART2_CLK,
    USART3_CLK,
    UART4_CLK,
    UART5_CLK,
    DAC_CLK,
    I2C1_CLK,
    I2C2_CLK,
    BKP_CLK,
    PWR_CLK,
    CAN1_CLK,
    CAN2_CLK,
// STM32F107VC AHB bus peripheral group
    DMA1_CLK,
    DMA2_CLK,
    USB_CLK,
    ETH_CLK,
    SRAM_CLK,
    FLITF_CLK,
    CRC_CLK
};

class SystemClock
{
private:
    enum SysClockErrorEnum
    {
        NO_ERROR = 0U,
        UNDEFINED_ERROR,
        GPIOD_CLK_ERROR,
        UNDEFINED_PERIPH_CLOCK
    };

public:
    static SystemClock *getInstance();

    void systemClockConfig(void);
    void periphClockEnable(PeripheryClockEnum clk);
    void periphClockDisable(PeripheryClockEnum clk);
private:
    SystemClock();
    SystemClock(const SystemClock &sysClk) = delete;
    SystemClock &operator=(const SystemClock &sysClk) = delete;
    ~SystemClock() {}

    void systemClockError(SysClockErrorEnum error);

    static SystemClock *instance;
};

} // namespace SYS_CLK
} // namespace STM32F107VC_DRIVER

#endif // SYSTEMCLOCK_H
