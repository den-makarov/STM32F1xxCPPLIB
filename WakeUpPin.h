#ifndef WAKEUPPIN_H
#define WAKEUPPIN_H

#include "PinControl.h"

namespace STM32F107VC_DRIVER {
namespace GPIO {

class WakeUpPin : private PinControl
{
public:
    WakeUpPin(PinPort pinPort, PinNumberEnum pinNumber,
              EdgeType edgeType = BOTH_EDGE, PinPullMode pinPull = PULL_UP);
    ~WakeUpPin() {}
    WakeUpPin(const WakeUpPin &pin) = default;
    WakeUpPin &operator=(const WakeUpPin &pin) = default;

    void setEdgeType(EdgeType edgeType);
    EdgeType getEdgeType() const;
    void setPinPull(PinPullMode pinPull);
    PinPullMode getPinPull() const {return m_pinPull;}
    PinState getPinState();

private:
    WakeUpPin();
};

} // namespace GPIO
} // namespace STM32F107VC_DRIVER

#endif // WAKEUPPIN_H
