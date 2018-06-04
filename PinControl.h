#ifndef PINCONTROL_H
#define PINCONTROL_H

#include "stm32f1xx.h"

namespace STM32F107VC_DRIVER {
namespace GPIO {

enum PinNumberEnum {
#ifdef HAL_GPIO_MODULE_ENABLED
    PIN_0 = GPIO_PIN_0,
    PIN_1 = GPIO_PIN_1,
    PIN_2 = GPIO_PIN_2,
    PIN_3 = GPIO_PIN_3,
    PIN_4 = GPIO_PIN_4,
    PIN_5 = GPIO_PIN_5,
    PIN_6 = GPIO_PIN_6,
    PIN_7 = GPIO_PIN_7,
    PIN_8 = GPIO_PIN_8,
    PIN_9 = GPIO_PIN_9,
    PIN_10 = GPIO_PIN_10,
    PIN_11 = GPIO_PIN_11,
    PIN_12 = GPIO_PIN_12,
    PIN_13 = GPIO_PIN_13,
    PIN_14 = GPIO_PIN_14,
    PIN_15 = GPIO_PIN_15,
    PIN_ALL = GPIO_PIN_All,
    FIRST_PIN = GPIO_PIN_0,
    LAST_PIN = GPIO_PIN_15
#else   /* HAL_GPIO_MODULE_ENABLED */
    PIN_0 = 0x0001,
    PIN_1 = 0x0002,
    PIN_2 = 0x0004,
    PIN_3 = 0x0008,
    PIN_4 = 0x0010,
    PIN_5 = 0x0020,
    PIN_6 = 0x0040,
    PIN_7 = 0x0080,
    PIN_8 = 0x0100,
    PIN_9 = 0x0200,
    PIN_10 = 0x0400,
    PIN_11 = 0x0800,
    PIN_12 = 0x1000,
    PIN_13 = 0x2000,
    PIN_14 = 0x4000,
    PIN_15 = 0x8000,
    FIRST_PIN = PIN_0,
    LAST_PIN = PIN_15
#endif  /* HAL_GPIO_MODULE_ENABLED */
};

//#define PIN_HIGH true
//#define PIN_LOW false

enum PinState {
#ifdef HAL_GPIO_MODULE_ENABLED
    LOW = GPIO_PIN_RESET,
    HIGH = GPIO_PIN_SET
#else   /* HAL_GPIO_MODULE_ENABLED */
    LOW = PIN_LOW,
    HIGH = !PIN_LOW
#endif  /* HAL_GPIO_MODULE_ENABLED */
};

enum PinPullMode {
#ifdef HAL_GPIO_MODULE_ENABLED
    HIGH_Z = GPIO_NOPULL,
    PULL_UP = GPIO_PULLUP,
    PULL_DOWN = GPIO_PULLDOWN
#else   /* HAL_GPIO_MODULE_ENABLED */
    HIGH_Z = 0x0000U,
    PULL_UP = 0x0001U,
    PULL_DOWN = 0x0002U
#endif  /* HAL_GPIO_MODULE_ENABLED */
};

enum PinSpeedMode {
#ifdef HAL_GPIO_MODULE_ENABLED
    SPEED_LOW = GPIO_SPEED_FREQ_LOW,
    SPEED_MED = GPIO_SPEED_FREQ_MEDIUM,
    SPEED_FULL = GPIO_SPEED_FREQ_HIGH
#else   /* HAL_GPIO_MODULE_ENABLED */
    SPEED_LOW = 0x0002U,
    SPEED_MED = 0x0001U,
    SPEED_FULL = 0x0003U
#endif  /* HAL_GPIO_MODULE_ENABLED */
};

enum PinPort {
    PORT_A = 0x0001,
    PORT_B = 0x0002,
    PORT_C = 0x0003,
    PORT_D = 0x0004,
    PORT_E = 0x0005,
    PORT_FIRST = PORT_A,
    PORT_LAST = PORT_E
};

enum pinPortError
{
    NO_ERROR = 0,
    UNAVAILABLE_PIN_PORT,
    UNAVAILABLE_PIN_NUMBER,
    INCORRECT_PIN_MODE,
    INCORRECT_PIN_SPEED,
    INCORRECT_PIN_PULL_MODE,
    UNDEFINED_ERROR
};

enum EdgeType
{
    RISING_EDGE = 0U,
    FALLING_EDGE,
    BOTH_EDGE
};

/******************************* GPIO Instances *******************************/
/*#define IS_GPIO_AVAILABLE(INSTANCE) (((INSTANCE) == (PinPort)PORT_A) || \
                                        ((INSTANCE) == (PinPort)PORT_B) || \
                                        ((INSTANCE) == (PinPort)PORT_C) || \
                                        ((INSTANCE) == (PinPort)PORT_D) || \
                                        ((INSTANCE) == (PinPort)PORT_E))*/

class PinControl
{
protected:
    enum PinModeEnum {
    #ifdef HAL_GPIO_MODULE_ENABLED
        IN = GPIO_MODE_INPUT,
        OUT_PP = GPIO_MODE_OUTPUT_PP,
        OUT_OD = GPIO_MODE_OUTPUT_OD,
        AF_PP = GPIO_MODE_AF_PP,
        AF_OD = GPIO_MODE_AF_OD,
        AF_IN = GPIO_MODE_AF_INPUT,
        ANALOG = GPIO_MODE_ANALOG,
        WFE_RISING_FRONT = GPIO_MODE_EVT_RISING,
        WFE_FALLING_FRONT = GPIO_MODE_EVT_FALLING,
        WFE_ANY_FRONT = GPIO_MODE_EVT_RISING_FALLING,
        IRQ_RISING_FRONT = GPIO_MODE_IT_RISING,
        IRQ_FALLING_FRONT = GPIO_MODE_IT_FALLING,
        IRQ_ANY_FRONT = GPIO_MODE_IT_RISING_FALLING
    #else   /* HAL_GPIO_MODULE_ENABLED */
        IN = 0x0000,
        OUT_PP = 0x0003,
        OUT_OD = 0x0005,
        AF_PP = 0x000B,
        AF_OD = 0x000D,
        AF_IN = 0x0008,
        ANALOG = 0x0010,
        WFE_RISING_FRONT = 0x0100,
        WFE_FALLING_FRONT = 0x0200,
        WFE_ANY_FRONT = 0x0300,
        IRQ_RISING_FRONT = 0x1000,
        IRQ_FALLING_FRONT = 0x2000,
        IRQ_ANY_FRONT = 0x3000,
        FIRST_MODE = INPUT,
        LAST_MODE = IRQ_ANY_FRONT
    #endif  /* HAL_GPIO_MODULE_ENABLED */
    };

public:
    PinControl(PinPort pinPort, PinNumberEnum pinNumber,
               PinControl::PinModeEnum pinMode = IN,
               PinPullMode pinPull = HIGH_Z,
               PinSpeedMode pinSpeed = SPEED_LOW);
    PinControl(const PinControl &pin) = default;
    PinControl &operator=(const PinControl &pin) = default;
    virtual ~PinControl() {}

protected:
    void pinControlError(pinPortError error) const;
    void setPinMode(PinModeEnum pinMode);
    PinModeEnum getPinMode(void) const {return m_pinMode;}
    void setPinPull(PinPullMode pinPull);
    PinPullMode getPinPull() const {return m_pinPull;}
    void setPinSpeed(PinSpeedMode pinSpeed);
    PinSpeedMode getPinSpeed() const {return m_pinSpeed;}
    void updatePinState();

    virtual void setPinState(PinState pinState) {m_pinState = pinState;}
    virtual PinState getPinState() {updatePinState(); return m_pinState;}
    virtual void tooglePinState() {}
    /*void setPinPortAddress(GPIO_TypeDef *pinPortAddress);
    GPIO_TypeDef *getPinPortAddress(void) const {return m_pinPortAddress;}
    void setPinPort(PinPort pinPort);
    PinPort getPinPort(void) const {return m_pinPort;}
    void setPinNumber(PinNumberEnum pinNumber);
    PinNumberEnum getPinNumber(void) const {return m_pinNumber;}*/

    PinPort m_pinPort;
    PinNumberEnum m_pinNumber;
    PinModeEnum m_pinMode;
    PinPullMode m_pinPull;
    PinSpeedMode m_pinSpeed;
    PinState m_pinState;
    GPIO_TypeDef *m_pinPortAddress;

private:
    PinControl();
};

} // namespase GPIO
} // namespase STM32F107VC_DRIVER
#endif // PINCONTROL_H
