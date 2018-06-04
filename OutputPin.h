#ifndef OUTPUTPIN_H
#define OUTPUTPIN_H

#include "PinControl.h"

namespace STM32F107VC_DRIVER {
namespace GPIO {

enum PinOutputMode {
    OPEN_DRAIN = 0U,
    PUSH_PULL,
    AF_OPEN_DRAIN,
    AF_PUSH_PULL
};

class OutputPin : public PinControl
{
public:
    OutputPin(PinPort pinPort, PinNumberEnum pinNumber, PinOutputMode pinMode,
              PinSpeedMode pinSpeed);
    ~OutputPin() {}
    OutputPin(const OutputPin &pin) = default;
    OutputPin &operator=(const OutputPin &pin) = default;

    void setPinMode(PinOutputMode pinMode);
    PinOutputMode getPinMode() const;
    void setPinSpeed(PinSpeedMode pinSpeed);
    PinSpeedMode getPinSpeed() const {return m_pinSpeed;}
    void setPinState(PinState pinState);
    PinState getPinState() const;
    void tooglePinState();

private:
    OutputPin();
};

} // namespace GPIO
} // namespace STM32F107VC_DRIVER

#endif // OUTPUTPIN_H
