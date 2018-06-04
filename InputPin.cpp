#include "InputPin.h"

namespace STM32F107VC_DRIVER {
namespace GPIO {

InputPin::InputPin(PinPort pinPort, PinNumberEnum pinNumber,
                   PinInputMode pinMode, PinPullMode pinPull):
    PinControl(pinPort, pinNumber, PinControl::PinModeEnum::IN,
               pinPull, STM32F107VC_DRIVER::GPIO::SPEED_LOW)
{
    setPinMode(pinMode);
}

/*InputPin::InputPin()
{

}*/

void InputPin::setPinMode(PinInputMode pinMode)
{
    switch(pinMode)
    {
    case STM32F107VC_DRIVER::GPIO::INPUT:
        PinControl::setPinMode(PinControl::PinModeEnum::IN); break;
    case STM32F107VC_DRIVER::GPIO::AF_INPUT:
        PinControl::setPinMode(PinControl::PinModeEnum::AF_IN); break;
    default:
        PinControl::pinControlError(STM32F107VC_DRIVER::GPIO::INCORRECT_PIN_MODE);
    }
}

PinInputMode InputPin::getPinMode() const
{
    PinInputMode mode;

#ifdef HAL_GPIO_MODULE_ENABLED
    if(m_pinMode == PinControl::PinModeEnum::IN)
    {
        mode = STM32F107VC_DRIVER::GPIO::INPUT;
    }
    else
    {
        PinControl::pinControlError(STM32F107VC_DRIVER::GPIO::INCORRECT_PIN_MODE);
    }
#else /* HAL_GPIO_MODULE_ENABLED */
    switch(m_pinMode)
    {
    case PinControl::PinModeEnum::IN:
        mode = STM32F107VC_DRIVER::GPIO::INPUT; break;
    case PinControl::PinModeEnum::AF_IN:
        mode = STM32F107VC_DRIVER::GPIO::AF_INPUT; break;
    default:
        PinControl::pinControlError(STM32F107VC_DRIVER::GPIO::INCORRECT_PIN_MODE);
    }
#endif /* HAL_GPIO_MODULE_ENABLED */
    return mode;
}

void InputPin::setPinPull(PinPullMode pinPull)
{
    PinControl::setPinPull(pinPull);
}

PinState InputPin::getPinState()
{
    PinControl::updatePinState();
    return m_pinState;
}

} // namespace GPIO
} // namespace STM32F107VC_DRIVER
