#include "OutputPin.h"

namespace STM32F107VC_DRIVER {
namespace GPIO {

OutputPin::OutputPin(PinPort pinPort, PinNumberEnum pinNumber,
                     PinOutputMode pinMode, PinSpeedMode pinSpeed):
    PinControl(pinPort, pinNumber, PinControl::PinModeEnum::OUT_OD,
               STM32F107VC_DRIVER::GPIO::HIGH_Z, pinSpeed)
{
    setPinMode(pinMode);
}

/*OutputPin::OutputPin()
{

}*/

void OutputPin::setPinMode(PinOutputMode pinMode)
{
    switch(pinMode)
    {
    case STM32F107VC_DRIVER::GPIO::PUSH_PULL:
        PinControl::setPinMode(PinControl::PinModeEnum::OUT_PP); break;
    case STM32F107VC_DRIVER::GPIO::OPEN_DRAIN:
        PinControl::setPinMode(PinControl::PinModeEnum::OUT_OD); break;
    case STM32F107VC_DRIVER::GPIO::AF_PUSH_PULL:
        PinControl::setPinMode(PinControl::PinModeEnum::AF_PP); break;
    case STM32F107VC_DRIVER::GPIO::AF_OPEN_DRAIN:
        PinControl::setPinMode(PinControl::PinModeEnum::AF_OD); break;
    default:
        PinControl::pinControlError(STM32F107VC_DRIVER::GPIO::INCORRECT_PIN_MODE);
    }
}

PinOutputMode OutputPin::getPinMode() const
{
    PinOutputMode mode;

    switch(m_pinMode)
    {
    case PinControl::PinModeEnum::OUT_PP:
        mode = STM32F107VC_DRIVER::GPIO::PUSH_PULL; break;
    case PinControl::PinModeEnum::OUT_OD:
        mode = STM32F107VC_DRIVER::GPIO::OPEN_DRAIN; break;
    case PinControl::PinModeEnum::AF_PP:
        mode = STM32F107VC_DRIVER::GPIO::AF_PUSH_PULL; break;
    case PinControl::PinModeEnum::AF_OD:
        mode = STM32F107VC_DRIVER::GPIO::AF_OPEN_DRAIN; break;
    default:
        PinControl::pinControlError(STM32F107VC_DRIVER::GPIO::INCORRECT_PIN_MODE);
    }

    return mode;
}

void OutputPin::setPinSpeed(PinSpeedMode pinSpeed)
{
    PinControl::setPinSpeed(pinSpeed);
}

void OutputPin::setPinState(PinState pinState)
{
    m_pinState = pinState;
#ifdef HAL_GPIO_MODULE_ENABLED
    HAL_GPIO_WritePin(m_pinPortAddress, m_pinNumber, (GPIO_PinState)m_pinState);
#else /* HAL_GPIO_MODULE_ENABLED */
#endif /* HAL_GPIO_MODULE_ENABLED */
}

PinState OutputPin::getPinState() const
{
/** @attention  Next method changes class's member m_pinState
  *             that is restricted with modifier 'const'
  */
    //PinControl::updatePinState();
    return m_pinState;
}

void OutputPin::tooglePinState()
{
    if(m_pinState == STM32F107VC_DRIVER::GPIO::HIGH)
    {
        m_pinState = STM32F107VC_DRIVER::GPIO::LOW;
    }
    else
    {
        m_pinState = STM32F107VC_DRIVER::GPIO::HIGH;
    }
#ifdef HAL_GPIO_MODULE_ENABLED
    HAL_GPIO_WritePin(m_pinPortAddress, m_pinNumber, (GPIO_PinState)m_pinState);
#else /* HAL_GPIO_MODULE_ENABLED */
#endif /* HAL_GPIO_MODULE_ENABLED */
}

} // namespace GPIO
} // namespace STM32F107VC_DRIVER
