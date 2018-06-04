#include "PinControl.h"

namespace STM32F107VC_DRIVER {
namespace GPIO {

PinControl::PinControl(PinPort pinPort, PinNumberEnum pinNumber,
                       PinControl::PinModeEnum pinMode, PinPullMode pinPull,
                       PinSpeedMode pinSpeed):
    m_pinPort(pinPort),
    m_pinNumber(pinNumber),
    m_pinMode(pinMode),
    m_pinPull(pinPull),
    m_pinSpeed(pinSpeed)
{
    /*#ifdef DEBUG
    assert_param(IS_GPIO_AVAILABLE(m_pinPort));
    #endif*/ /* DEBUG */
    switch(m_pinPort)
    {
    case STM32F107VC_DRIVER::GPIO::PORT_A:
        m_pinPortAddress = GPIOA; break;
    case STM32F107VC_DRIVER::GPIO::PORT_B:
        m_pinPortAddress = GPIOB; break;
    case STM32F107VC_DRIVER::GPIO::PORT_C:
        m_pinPortAddress = GPIOC; break;
    case STM32F107VC_DRIVER::GPIO::PORT_D:
        m_pinPortAddress = GPIOD; break;
    case STM32F107VC_DRIVER::GPIO::PORT_E:
        m_pinPortAddress = GPIOE; break;
    default:
        pinControlError(STM32F107VC_DRIVER::GPIO::UNAVAILABLE_PIN_PORT);
    }
}

/*PinControl::PinControl()
{

}*/

void PinControl::setPinMode(PinModeEnum pinMode)
{
    m_pinMode = pinMode;

    GPIO_InitTypeDef  gpioInitStruct = {0, 0, 0, 0};

    gpioInitStruct.Pin    = m_pinNumber;
    gpioInitStruct.Mode   = m_pinMode;
    gpioInitStruct.Pull   = m_pinPull;
    gpioInitStruct.Speed  = m_pinSpeed;

#ifdef HAL_GPIO_MODULE_ENABLED
    HAL_GPIO_Init(m_pinPortAddress, &gpioInitStruct);
#else /* HAL_GPIO_MODULE_ENABLED */
#endif /* HAL_GPIO_MODULE_ENABLED */
}

void PinControl::setPinPull(PinPullMode pinPull)
{
    m_pinPull = pinPull;

    GPIO_InitTypeDef  gpioInitStruct = {0, 0, 0, 0};

    gpioInitStruct.Pin    = m_pinNumber;
    gpioInitStruct.Mode   = m_pinMode;
    gpioInitStruct.Pull   = m_pinPull;
    gpioInitStruct.Speed  = m_pinSpeed;

#ifdef HAL_GPIO_MODULE_ENABLED
    HAL_GPIO_Init(m_pinPortAddress, &gpioInitStruct);
#else /* HAL_GPIO_MODULE_ENABLED */
#endif /* HAL_GPIO_MODULE_ENABLED */
}

void PinControl::setPinSpeed(PinSpeedMode pinSpeed)
{
    m_pinSpeed = pinSpeed;

    GPIO_InitTypeDef  gpioInitStruct = {0, 0, 0, 0};

    gpioInitStruct.Pin    = m_pinNumber;
    gpioInitStruct.Mode   = m_pinMode;
    gpioInitStruct.Pull   = m_pinPull;
    gpioInitStruct.Speed  = m_pinSpeed;

#ifdef HAL_GPIO_MODULE_ENABLED
    HAL_GPIO_Init(m_pinPortAddress, &gpioInitStruct);
#else /* HAL_GPIO_MODULE_ENABLED */
#endif /* HAL_GPIO_MODULE_ENABLED */
}

void PinControl::pinControlError(pinPortError error) const
{
    if(error)
    {
        while(1);
    }
}

void PinControl::updatePinState()
{
#ifdef HAL_GPIO_MODULE_ENABLED
    if(HAL_GPIO_ReadPin(m_pinPortAddress, m_pinNumber))
    {
        m_pinState = STM32F107VC_DRIVER::GPIO::HIGH;
    }
    else
    {
        m_pinState = STM32F107VC_DRIVER::GPIO::LOW;
    }
#else /* HAL_GPIO_MODULE_ENABLED */
#endif /* HAL_GPIO_MODULE_ENABLED */
}

/*void PinControl::setPinPortAddress(GPIO_TypeDef *pinPortAddress)
{

}

void PinControl::setPinPort(PinPort pinPort)
{

}

void PinControl::setPinNumber(PinNumberEnum pinNumber)
{

}*/

} // namespace GPIO
} // namespace STM32F107VC_DRIVER
