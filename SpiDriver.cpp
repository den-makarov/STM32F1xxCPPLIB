#include "SpiDriver.h"

namespace STM32F107VC_DRIVER {
namespace SPI {

SpiDriver::SpiDriver(SPI_Channel channel, const SPI_Config &config):
    m_channel(channel),
    m_txBuffer(new SPI_Buffer()),
    m_rxBuffer(new SPI_Buffer()),
    m_locked(true),
    m_state(BUSY),
    m_error(NO_ERROR),
    m_sysClk(STM32F107VC_DRIVER::SYS_CLK::SystemClock::getInstance())
{
    if(nullptr != m_config)
    {
        return;
    }

    m_config = new SPI_Config(config);

    if((SPI_1RM == m_channel) || (SPI_3RM == m_channel))
    {
        remapSpi();
    }

    initSpiPins();

    setClock(true);

    disableSpi();

    setConfig();

#if defined(SPI_I2SCFGR_I2SMOD)
    switchInstance2SpiMode();
#endif

    m_state = READY;
    m_locked = false;
}

SpiDriver::~SpiDriver()
{
    if(m_locked)
    {
        return;
    }

    m_locked = true;
    m_state = RESET;

    if(nullptr != m_pinSCK)
    {
        delete m_pinSCK;
        m_pinSCK = nullptr;
    }

    if(nullptr != m_pinMISO)
    {
        delete m_pinMISO;
        m_pinMISO = nullptr;
    }

    if(nullptr != m_pinMOSI)
    {
        delete m_pinMOSI;
        m_pinMOSI = nullptr;
    }

    if(nullptr != m_pinNSS)
    {
        delete m_pinNSS;
        m_pinNSS = nullptr;
    }

    if(nullptr != m_config)
    {
        delete m_config;
        m_config = nullptr;
    }

    if(nullptr != m_txBuffer)
    {
        delete m_txBuffer;
        m_txBuffer = nullptr;
    }

    if(nullptr != m_rxBuffer)
    {
        delete m_rxBuffer;
        m_rxBuffer = nullptr;
    }

#ifdef HAL_SPI_MODULE_ENABLED
    m_hSPI = nullptr;
#endif /* HAL_SPI_MODULE_ENABLED */
    m_locked = false;
}

#ifdef HAL_SPI_MODULE_ENABLED
SpiDriver::SpiDriver(SPI_HandleTypeDef *hal_handler_spi, bool remaped):
    m_hSPI(hal_handler_spi),
    m_locked(true),
    m_state(BUSY),
    m_error(NO_ERROR),
    m_sysClk(STM32F107VC_DRIVER::SYS_CLK::SystemClock::getInstance())
{
    if(nullptr == m_config)
    {
        m_config = new SPI_Config();
        m_config->timeout = TIMEOUT_1S;
        m_txBuffer = new SPI_Buffer();
        m_rxBuffer = new SPI_Buffer();
/**
  * @todo Complete all fields of m_config
  * @note or relax and forget it
  */
    }

    if(SPI1 == m_hSPI->Instance)
    {
        m_channel = (!remaped) ? SPI_1 : SPI_1RM;
    }
    else if(SPI2 == m_hSPI->Instance)
    {
        m_channel = (!remaped) ? SPI_2 : SPI_2RM;
    }
    else if(SPI3 == m_hSPI->Instance)
    {
        m_channel = (!remaped) ? SPI_3 : SPI_3RM;
    }
    else
    {
        spiError(INIT_ERROR);
        return;
    }

    if(remaped)
    {
        remapSpi();
    }

    initSpiPins();

    setClock(true);

    if (HAL_SPI_Init(m_hSPI) != HAL_OK)
    {
        while(1) {};
    }
    m_state = READY;
    m_locked = false;
}
#endif /* HAL_SPI_MODULE_ENABLED */

void SpiDriver::setConfig(const SPI_Config &config)
{
    /**
     * @todo CHECK ALL CONFIGURATION PARAMETERS
     */
    if(nullptr != m_config)
    {
        delete m_config;
    }
    m_config = new SPI_Config(config);
    setConfig();
}

const SPI_Config *SpiDriver::getConfig()
{
    return m_config;
}

void SpiDriver::setConfig()
{
    getSpiPeriphAddr()->CR1 = (m_config->mode |
                               m_config->dir |
                               m_config->dSize |
                               m_config->clkCfg |
                               m_config->divider |
                               m_config->firstBit);

    if(m_config->crcCfg.enabled)
    {
        getSpiPeriphAddr()->CR1 |= SPI_CRC_ENABLE;
        setCrcPolynom();
    }
    else
    {
        getSpiPeriphAddr()->CR1 &= ~SPI_CRC_ENABLE;
    }

    if(NSS_SW == m_config->nssMode)
    {
        getSpiPeriphAddr()->CR1 |= m_config->nssMode;
        getSpiPeriphAddr()->CR2 &= ~STM32F107VC_DRIVER::SPI::NSS_HW_OUT;
        /**
         * @brief initSpiNSS
         * @todo Refer to STM32F107 reference manual for NSS configuration
         */
        initSpiNSS();
    }
    else
    {
        getSpiPeriphAddr()->CR1 &= ~STM32F107VC_DRIVER::SPI::NSS_SW;
        getSpiPeriphAddr()->CR2 |= m_config->nssMode;
    }
}

void SpiDriver::enableSpi()
{
    getSpiPeriphAddr()->CR1 |= SPI_CR1_SPE;
}

void SpiDriver::disableSpi()
{
    getSpiPeriphAddr()->CR1 &= (~SPI_CR1_SPE);
}

SPI_TypeDef *SpiDriver::getSpiPeriphAddr()
{
    SPI_TypeDef *instance;
    switch(m_channel)
    {
    case SPI_1:
    case SPI_1RM: instance = SPI1; break;
    case SPI_2: instance = SPI2; break;
    case SPI_3:
    case SPI_3RM: instance = SPI3; break;
    default: spiError(INIT_ERROR);
    }

    return instance;
}

void SpiDriver::setClock(bool enabled)
{
    STM32F107VC_DRIVER::SYS_CLK::PeripheryClockEnum spiClock;

    switch(m_channel)
    {
    case SPI_1:
    case SPI_1RM: spiClock = STM32F107VC_DRIVER::SYS_CLK::SPI1_CLK; break;
    case SPI_2: spiClock = STM32F107VC_DRIVER::SYS_CLK::SPI2_CLK; break;
    case SPI_3:
    case SPI_3RM: spiClock = STM32F107VC_DRIVER::SYS_CLK::SPI3_CLK; break;
    default: spiError(INIT_ERROR);
    }

    if(enabled)
    {
        m_sysClk->periphClockEnable(spiClock);
    }
    else
    {
        m_sysClk->periphClockDisable(spiClock);
    }
}

void SpiDriver::setCrcPolynom(uint16_t polynom)
{
    m_config->crcCfg.polynom = polynom;
    setCrcPolynom();
}

void SpiDriver::setCrcPolynom()
{
    if(0U == m_config->crcCfg.polynom)
    {
        spiError(CRC_POLYNOM_ZERO);
    }
    getSpiPeriphAddr()->CRCPR = m_config->crcCfg.polynom;
}

uint16_t SpiDriver::getCrcPolynom() const
{
    return m_config->crcCfg.polynom;
}

#if defined(SPI_I2SCFGR_I2SMOD)
void SpiDriver::switchInstance2SpiMode()
{
    getSpiPeriphAddr()->I2SCFGR &= ~SPI_I2SCFGR_I2SMOD;
}
#endif /* SPI_I2SCFGR_I2SMOD */

void SpiDriver::remapSpi()
{
    m_sysClk->periphClockEnable(STM32F107VC_DRIVER::SYS_CLK::AFIO_CLK);
#ifdef HAL_SPI_MODULE_ENABLED
    switch(m_channel)
    {
    case SPI_1RM: __HAL_AFIO_REMAP_SPI1_ENABLE(); break;
    case SPI_3RM: __HAL_AFIO_REMAP_SPI3_ENABLE(); break;
    default: spiError(INIT_ERROR);
    }
#else /* HAL_SPI_MODULE_ENABLED */
#endif /* HAL_SPI_MODULE_ENABLED */
}

void SpiDriver::initSpiPins()
{
    enablePinsClock();

    STM32F107VC_DRIVER::GPIO::PinPort sckPort, misoPort, mosiPort;
    STM32F107VC_DRIVER::GPIO::PinNumberEnum sckPin, misoPin, mosiPin;

    switch(m_channel)
    {
    case SPI_1:
        sckPort = STM32F107VC_DRIVER::GPIO::PORT_A;
        misoPort = STM32F107VC_DRIVER::GPIO::PORT_A;
        mosiPort = STM32F107VC_DRIVER::GPIO::PORT_A;
        sckPin = STM32F107VC_DRIVER::GPIO::PIN_5;
        misoPin = STM32F107VC_DRIVER::GPIO::PIN_6;
        mosiPin = STM32F107VC_DRIVER::GPIO::PIN_7;
        break;
    case SPI_2:
        sckPort = STM32F107VC_DRIVER::GPIO::PORT_B;
        misoPort = STM32F107VC_DRIVER::GPIO::PORT_B;
        mosiPort = STM32F107VC_DRIVER::GPIO::PORT_B;
        sckPin = STM32F107VC_DRIVER::GPIO::PIN_13;
        misoPin = STM32F107VC_DRIVER::GPIO::PIN_14;
        mosiPin = STM32F107VC_DRIVER::GPIO::PIN_15;
        break;
    case SPI_3:
        sckPort = STM32F107VC_DRIVER::GPIO::PORT_B;
        misoPort = STM32F107VC_DRIVER::GPIO::PORT_B;
        mosiPort = STM32F107VC_DRIVER::GPIO::PORT_B;
        sckPin = STM32F107VC_DRIVER::GPIO::PIN_3;
        misoPin = STM32F107VC_DRIVER::GPIO::PIN_4;
        mosiPin = STM32F107VC_DRIVER::GPIO::PIN_5;
        break;
    case SPI_1RM:
        sckPort = STM32F107VC_DRIVER::GPIO::PORT_B;
        misoPort = STM32F107VC_DRIVER::GPIO::PORT_B;
        mosiPort = STM32F107VC_DRIVER::GPIO::PORT_B;
        sckPin = STM32F107VC_DRIVER::GPIO::PIN_3;
        misoPin = STM32F107VC_DRIVER::GPIO::PIN_4;
        mosiPin = STM32F107VC_DRIVER::GPIO::PIN_5;
        break;
    case SPI_3RM:
        sckPort = STM32F107VC_DRIVER::GPIO::PORT_C;
        misoPort = STM32F107VC_DRIVER::GPIO::PORT_C;
        mosiPort = STM32F107VC_DRIVER::GPIO::PORT_C;
        sckPin = STM32F107VC_DRIVER::GPIO::PIN_10;
        misoPin = STM32F107VC_DRIVER::GPIO::PIN_11;
        mosiPin = STM32F107VC_DRIVER::GPIO::PIN_12;
        break;
    default: spiError(INIT_ERROR);
    }

    m_pinSCK = new STM32F107VC_DRIVER::GPIO::OutputPin(sckPort, sckPin,
                STM32F107VC_DRIVER::GPIO::AF_PUSH_PULL,
                STM32F107VC_DRIVER::GPIO::SPEED_FULL);

    m_pinMISO = new STM32F107VC_DRIVER::GPIO::OutputPin(misoPort, misoPin,
                STM32F107VC_DRIVER::GPIO::AF_PUSH_PULL,
                STM32F107VC_DRIVER::GPIO::SPEED_FULL);

    m_pinMOSI = new STM32F107VC_DRIVER::GPIO::OutputPin(mosiPort, mosiPin,
                STM32F107VC_DRIVER::GPIO::AF_PUSH_PULL,
                STM32F107VC_DRIVER::GPIO::SPEED_FULL);
}

void SpiDriver::initSpiNSS()
{
    enableNSSClock();

    STM32F107VC_DRIVER::GPIO::PinPort nssPort;
    STM32F107VC_DRIVER::GPIO::PinNumberEnum nssPin;

    switch(m_channel)
    {
    case SPI_1:
        nssPort = STM32F107VC_DRIVER::GPIO::PORT_A;
        nssPin = STM32F107VC_DRIVER::GPIO::PIN_4;
        break;
    case SPI_2:
        nssPort = STM32F107VC_DRIVER::GPIO::PORT_B;
        nssPin = STM32F107VC_DRIVER::GPIO::PIN_12;
        break;
    case SPI_3:
        nssPort = STM32F107VC_DRIVER::GPIO::PORT_A;
        nssPin = STM32F107VC_DRIVER::GPIO::PIN_15;
        break;
    case SPI_1RM:
        nssPort = STM32F107VC_DRIVER::GPIO::PORT_A;
        nssPin = STM32F107VC_DRIVER::GPIO::PIN_15;
        break;
    case SPI_3RM:
        nssPort = STM32F107VC_DRIVER::GPIO::PORT_A;
        nssPin = STM32F107VC_DRIVER::GPIO::PIN_4;
        break;
    default: spiError(INIT_ERROR);
    }
    m_pinNSS = new STM32F107VC_DRIVER::GPIO::OutputPin(nssPort, nssPin,
                STM32F107VC_DRIVER::GPIO::AF_PUSH_PULL,
                STM32F107VC_DRIVER::GPIO::SPEED_FULL);
}

void SpiDriver::enableNSSClock()
{
    STM32F107VC_DRIVER::SYS_CLK::PeripheryClockEnum nssPortClock;

    switch(m_channel)
    {
    case SPI_1:
        nssPortClock = STM32F107VC_DRIVER::SYS_CLK::GPIOA_CLK;
        break;
    case SPI_2:
        nssPortClock = STM32F107VC_DRIVER::SYS_CLK::GPIOB_CLK;
        break;
    case SPI_3:
        nssPortClock = STM32F107VC_DRIVER::SYS_CLK::GPIOA_CLK;
        break;
    case SPI_1RM:
        nssPortClock = STM32F107VC_DRIVER::SYS_CLK::GPIOA_CLK;
        break;
    case SPI_3RM:
        nssPortClock = STM32F107VC_DRIVER::SYS_CLK::GPIOA_CLK;
        break;
    default: spiError(INIT_ERROR);
    }
    m_sysClk->periphClockEnable(nssPortClock);
}

void SpiDriver::enablePinsClock()
{
    STM32F107VC_DRIVER::SYS_CLK::PeripheryClockEnum pinsPortClock;

    switch(m_channel)
    {
    case SPI_1:
        pinsPortClock = STM32F107VC_DRIVER::SYS_CLK::GPIOA_CLK;
        break;
    case SPI_2:
        pinsPortClock = STM32F107VC_DRIVER::SYS_CLK::GPIOB_CLK;
        break;
    case SPI_3:
        pinsPortClock = STM32F107VC_DRIVER::SYS_CLK::GPIOB_CLK;
        break;
    case SPI_1RM:
        pinsPortClock = STM32F107VC_DRIVER::SYS_CLK::GPIOB_CLK;
        break;
    case SPI_3RM:
        pinsPortClock = STM32F107VC_DRIVER::SYS_CLK::GPIOC_CLK;
        break;
    default: spiError(INIT_ERROR);
    }
    m_sysClk->periphClockEnable(pinsPortClock);
}

void SpiDriver::spiError(SPI_ErrorCode error)
{
    m_error = error;
    m_state = ERROR;
    m_locked = false;
}

const SPI_Channel &SpiDriver::getChannel() const
{
    return m_channel;
}

const SPI_StateCode &SpiDriver::getState()
{
#ifdef HAL_SPI_MODULE_ENABLED
    switch(m_hSPI->State)
    {
    case HAL_SPI_STATE_RESET: m_state = RESET; break;
    case HAL_SPI_STATE_READY: m_state = READY; break;
    case HAL_SPI_STATE_BUSY: m_state = BUSY; break;
    case HAL_SPI_STATE_BUSY_TX: m_state = TRANSFER_TX; break;
    case HAL_SPI_STATE_BUSY_RX: m_state = TRANSFER_RX; break;
    case HAL_SPI_STATE_BUSY_TX_RX: m_state = TRANSFER_TX_RX; break;
    default: m_state = ERROR;
    }
#endif /* HAL_SPI_MODULE_ENABLED */
    return m_state;
}

const SPI_ErrorCode &SpiDriver::getError()
{
#ifdef HAL_SPI_MODULE_ENABLED
    switch(m_hSPI->ErrorCode)
    {
    case HAL_SPI_ERROR_NONE: m_error = NO_ERROR; break;
    case HAL_SPI_ERROR_MODF: m_error = MODF_ERROR; break;
    case HAL_SPI_ERROR_CRC: m_error = CRC_ERROR; break;
    case HAL_SPI_ERROR_OVR: m_error = OVR_ERROR; break;
    case HAL_SPI_ERROR_FRE: m_error = FRE_ERROR; break;
    case HAL_SPI_ERROR_DMA: m_error = DMA_ERROR; break;
    case HAL_SPI_ERROR_FLAG: m_error = FLAG_ERROR; break;
    default: m_error = UNDEFINED_ERROR;
    }
#endif /* HAL_SPI_MODULE_ENABLED */
    return m_error;
}

bool SpiDriver::isLocked()
{
#ifdef HAL_SPI_MODULE_ENABLED
    m_locked = (HAL_LOCKED == m_hSPI->Lock) ? true : false;
#endif /* HAL_SPI_MODULE_ENABLED */
    return m_locked;
}

bool SpiDriver::setTimeout(SPI_Timeout timeout)
{
    if(READY != m_state)
    {
        return false;
    }

    if(nullptr == m_config)
    {
        return false;
    }

    m_config->timeout = timeout;
    return true;
}

SPI_Timeout SpiDriver::getTimeout() const
{
    return m_config->timeout;
}

void SpiDriver::transmitData(const uint8_t *txData, uint16_t size)
{
    if((READY != m_state) || (true == m_locked))
    {
        spiError(TRANSFER_ERROR);
        return;
    }

    m_locked = true;
    m_state = TRANSFER_TX;

    if((nullptr == txData) || (0 == size))
    {
        spiError(EMPTY_BUFFER);
        return;
    }

    m_txBuffer->pBuffer = const_cast<uint8_t *>(txData);
    m_txBuffer->bufferSize = size;
    m_txBuffer->transferCounter = 0;

#ifdef HAL_SPI_MODULE_ENABLED
    HAL_StatusTypeDef status = HAL_OK;

    status = HAL_SPI_Transmit(m_hSPI,
                              m_txBuffer->pBuffer,
                              m_txBuffer->bufferSize,
                              m_config->timeout);

    if(status != HAL_OK)
    {
        spiError(TIMEOUT_TX);
    }
#endif /* HAL_SPI_MODULE_ENABLED */
    m_locked = false;
    m_state = READY;
}

void SpiDriver::receiveData(uint8_t *rxData, uint16_t size)
{
    if((READY != m_state) || (true == m_locked))
    {
        spiError(TRANSFER_ERROR);
        return;
    }

    m_locked = true;
    m_state = TRANSFER_RX;

    if((nullptr == rxData) || (0 == size))
    {
        spiError(EMPTY_BUFFER);
        return;
    }

    m_rxBuffer->pBuffer = rxData;
    m_rxBuffer->bufferSize = size;
    m_rxBuffer->transferCounter = 0;

#ifdef HAL_SPI_MODULE_ENABLED
    HAL_StatusTypeDef status = HAL_OK;

    status = HAL_SPI_Receive(m_hSPI,
                             m_rxBuffer->pBuffer,
                             m_rxBuffer->bufferSize,
                             m_config->timeout);

    if(status != HAL_OK)
    {
        spiError(TIMEOUT_RX);
    }
#endif /* HAL_SPI_MODULE_ENABLED */
    m_locked = false;
    m_state = READY;
}

void SpiDriver::exchangeData(const uint8_t *txData,
                             uint8_t *rxData,
                             uint16_t size)
{
    if((READY != m_state) || (true == m_locked))
    {
        spiError(TRANSFER_ERROR);
        return;
    }

    m_locked = true;
    m_state = TRANSFER_TX_RX;

    if((nullptr == txData) || (nullptr == rxData)  || (0 == size))
    {
        spiError(EMPTY_BUFFER);
        return;
    }

    m_txBuffer->pBuffer = const_cast<uint8_t *>(txData);
    m_txBuffer->bufferSize = size;
    m_txBuffer->transferCounter = 0;

    m_rxBuffer->pBuffer = rxData;
    m_rxBuffer->bufferSize = size;
    m_rxBuffer->transferCounter = 0;

#ifdef HAL_SPI_MODULE_ENABLED
    HAL_StatusTypeDef status = HAL_OK;

    status = HAL_SPI_TransmitReceive(m_hSPI,
                                     m_txBuffer->pBuffer,
                                     m_rxBuffer->pBuffer,
                                     m_txBuffer->bufferSize,
                                     m_config->timeout);

    if(status != HAL_OK)
    {
        spiError(TIMEOUT_TX_RX);
    }
#endif /* HAL_SPI_MODULE_ENABLED */
    m_locked = false;
    m_state = READY;
}

SpiDriver &SpiDriver::operator<<(const uint8_t *txData)
{
    transmitData(txData, 1U);
    return *this;
}

SpiDriver &SpiDriver::operator>>(uint8_t *rxData)
{
    receiveData(rxData, 1U);
    return *this;
}

} // namespace SPI
} // namespace STM32F107VC_DRIVER
