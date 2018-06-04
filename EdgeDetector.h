#ifndef EDGEDETECTOR_H
#define EDGEDETECTOR_H

#include "PinControl.h"

namespace STM32F107VC_DRIVER {
namespace GPIO {

class EdgeDetector : private PinControl
{
public:
    EdgeDetector(PinPort pinPort, PinNumberEnum pinNumber,
                 EdgeType edgeType = BOTH_EDGE, PinPullMode pinPull = PULL_UP);
    ~EdgeDetector() {}
    EdgeDetector(const EdgeDetector &pin) = default;
    EdgeDetector &operator=(const EdgeDetector &pin) = default;

    void setEdgeType(EdgeType edgeType);
    EdgeType getEdgeType() const;
    void setPinPull(PinPullMode pinPull);
    PinPullMode getPinPull() const {return m_pinPull;}
    PinState getPinState();

private:
    EdgeDetector();
};

} // namespace GPIO
} // namespace STM32F107VC_DRIVER

#endif // EDGEDETECTOR_H
