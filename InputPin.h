#ifndef INPUTPIN_H
#define INPUTPIN_H

#include "PinControl.h"

namespace STM32F107VC_DRIVER {
namespace GPIO {

enum PinInputMode {
    INPUT = 0U,
    AF_INPUT
};

class InputPin : private PinControl
{
public:
    InputPin(PinPort pinPort, PinNumberEnum pinNumber, PinInputMode pinMode = INPUT,
             PinPullMode pinPull = HIGH_Z);
    ~InputPin() {}
    InputPin(const InputPin &pin) = default;
    InputPin &operator=(const InputPin &pin) = default;

    void setPinMode(PinInputMode pinMode);
    PinInputMode getPinMode() const;
    void setPinPull(PinPullMode pinPull);
    PinPullMode getPinPull() const {return m_pinPull;}
    PinState getPinState();

private:
    InputPin();
    void setPinState() {}
};

} // namespace GPIO
} // namespace STM32F107VC_DRIVER

#endif // INPUTPIN_H
