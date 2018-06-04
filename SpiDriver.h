#ifndef SPIDRIVER_H
#define SPIDRIVER_H

#include "stm32f1xx.h"
#include "SystemClock.h"
#include "OutputPin.h"

namespace STM32F107VC_DRIVER {
namespace SPI {

#define SPI_CRC_ENABLE SPI_CR1_CRCEN

enum SPI_Channel {
    SPI_1 = 0u,
    SPI_2,
    SPI_3,
    SPI_1RM, /*!< Remapped SPI1 pins */
    SPI_3RM, /*!< Remapped SPI3 pins */
    SPI_2RM = SPI_2, /*!< STM32F107VC doesn't provide remapping SPI2 */
    SPI_FIRST = SPI_1,
    SPI_LAST = SPI_3
};

enum SPI_Direction {
#ifdef HAL_SPI_MODULE_ENABLED
    DUPLEX = SPI_DIRECTION_2LINES,
    SIMPLEX = SPI_DIRECTION_1LINE,
    DUPLEX_RxONLY = SPI_DIRECTION_2LINES_RXONLY
#else /* HAL_SPI_MODULE_ENABLED */
    DUPLEX = 0U,
    SIMPLEX = SPI_CR1_BIDIMODE,
    DUPLEX_RxONLY = SPI_CR1_RXONLY
#endif /* HAL_SPI_MODULE_ENABLED */
};

enum SPI_Mode {
#ifdef HAL_SPI_MODULE_ENABLED
    SLAVE = SPI_MODE_SLAVE,
    MASTER = SPI_MODE_MASTER
#else /* HAL_SPI_MODULE_ENABLED */
    SLAVE = 0U,
    MASTER = (SPI_CR1_MSTR | SPI_CR1_SSI)
#endif /* HAL_SPI_MODULE_ENABLED */
};

enum SPI_DataSize {
#ifdef HAL_SPI_MODULE_ENABLED
    D08 = SPI_DATASIZE_8BIT,
    D16 = SPI_DATASIZE_16BIT
#else /* HAL_SPI_MODULE_ENABLED */
    D08 = 0U,
    D16 = SPI_CR1_DFF
#endif /* HAL_SPI_MODULE_ENABLED */
};

enum SPI_ClockConfig {
    RISING_FIRST = 0U, /*!< SPI clock = low polarity & phase 1 edge */
    RISING_SECOND = SPI_CR1_CPHA,      /*!< SPI clock = low polarity & phase 2 edge */
    FALLING_FIRST = SPI_CR1_CPOL,      /*!< SPI clock = high polarity & phase 1 edge */
    FALLING_DECOND = (SPI_CR1_CPHA | SPI_CR1_CPOL)/*!< SPI clock = high polarity & phase 2 edge */
};

enum SPI_NSSConfig {
#ifdef HAL_SPI_MODULE_ENABLED
    NSS_HW_IN = SPI_NSS_HARD_INPUT,
    NSS_HW_OUT = SPI_NSS_HARD_OUTPUT,
    NSS_SW = SPI_NSS_SOFT,
#else /* HAL_SPI_MODULE_ENABLED */
    NSS_HW_IN = 0U,
    NSS_HW_OUT = SPI_CR2_SSOE,
    NSS_SW = SPI_CR1_SSM,
#endif /* HAL_SPI_MODULE_ENABLED */
};

enum SPI_ClockDividerConfig {
#ifdef HAL_SPI_MODULE_ENABLED
    DIVIDER_2 = SPI_BAUDRATEPRESCALER_2,
    DIVIDER_4 = SPI_BAUDRATEPRESCALER_4,
    DIVIDER_8 = SPI_BAUDRATEPRESCALER_8,
    DIVIDER_16 = SPI_BAUDRATEPRESCALER_16,
    DIVIDER_32 = SPI_BAUDRATEPRESCALER_32,
    DIVIDER_64 = SPI_BAUDRATEPRESCALER_64,
    DIVIDER_128 = SPI_BAUDRATEPRESCALER_128,
    DIVIDER_256 = SPI_BAUDRATEPRESCALER_256,
    DIVIDER_MIN = DIVIDER_2,
    DIVIDER_MAX = DIVIDER_256
#else /* HAL_SPI_MODULE_ENABLED */
    DIVIDER_2 = 0U,
    DIVIDER_4 = SPI_CR1_BR_0,
    DIVIDER_8 = SPI_CR1_BR_1,
    DIVIDER_16 = (uint32_t)(SPI_CR1_BR_1 | SPI_CR1_BR_0),
    DIVIDER_32 = SPI_CR1_BR_2,
    DIVIDER_64 = (uint32_t)(SPI_CR1_BR_2 | SPI_CR1_BR_0),
    DIVIDER_128 = (uint32_t)(SPI_CR1_BR_2 | SPI_CR1_BR_1),
    DIVIDER_256 = SPI_CR1_BR,
    DIVIDER_MIN = DIVIDER_2,
    DIVIDER_MAX = DIVIDER_256
#endif /* HAL_SPI_MODULE_ENABLED */
};

enum SPI_FirstBitConfig {
#ifdef HAL_SPI_MODULE_ENABLED
    MSB = SPI_FIRSTBIT_MSB,
    LSB = SPI_FIRSTBIT_LSB
#else /* HAL_SPI_MODULE_ENABLED */
    MSB = 0U,
    LSB = SPI_CR1_LSBFIRST
#endif /* HAL_SPI_MODULE_ENABLED */
};

enum SPI_StateCode {
#ifdef HAL_SPI_MODULE_ENABLED
    RESET = HAL_SPI_STATE_RESET,
    READY = HAL_SPI_STATE_READY,
    BUSY = HAL_SPI_STATE_BUSY,
    TRANSFER_TX = HAL_SPI_STATE_BUSY_TX,
    TRANSFER_RX = HAL_SPI_STATE_BUSY_RX,
    TRANSFER_TX_RX = HAL_SPI_STATE_BUSY_TX_RX,
    ERROR = HAL_SPI_STATE_ERROR
#else /* HAL_SPI_MODULE_ENABLED */
    RESET = 0U,
    READY,
    BUSY,
    TRANSFER_TX,
    TRANSFER_RX,
    TRANSFER_TX_RX,
    ERROR
#endif /* HAL_SPI_MODULE_ENABLED */
};

enum SPI_ErrorCode {
#ifdef HAL_SPI_MODULE_ENABLED
    NO_ERROR = HAL_SPI_ERROR_NONE,
    MODF_ERROR = HAL_SPI_ERROR_MODF,
    CRC_ERROR = HAL_SPI_ERROR_CRC,
    OVR_ERROR = HAL_SPI_ERROR_OVR,
    FRE_ERROR = HAL_SPI_ERROR_FRE,
    DMA_ERROR = HAL_SPI_ERROR_DMA,
    FLAG_ERROR = HAL_SPI_ERROR_FLAG,
    INIT_ERROR,
    CRC_POLYNOM_ZERO,
    SYS_CLOCK_ERROR,
    EMPTY_BUFFER,
    TIMEOUT_TX,
    TIMEOUT_RX,
    TIMEOUT_TX_RX,
    TRANSFER_ERROR,
    UNDEFINED_ERROR
#else /* HAL_SPI_MODULE_ENABLED */
    NO_ERROR = 0U,
    MODF_ERROR,
    CRC_ERROR,
    OVR_ERROR,
    FRE_ERROR,
    DMA_ERROR,
    FLAG_ERROR,
    INIT_ERROR,
    SYS_CLOCK_ERROR,
    EMPTY_BUFFER,
    TIMEOUT_TX,
    TIMEOUT_RX,
    TIMEOUT_TX_RX,
    TRANSFER_ERROR,
    UNDEFINED_ERROR
#endif /* HAL_SPI_MODULE_ENABLED */
};

struct SPI_CRCConfig {
    bool enabled;
    uint16_t polynom;
};

/**
 * @brief The SPI_Timeout enum
 * @remark each value with respect to 1ms period of
 * @remark SysTyck Cortex-M timer.
 */
enum SPI_Timeout {
    TIMEOUT_1mS = 01U,
    TIMEOUT_10mS = 10U,
    TIMEOUT_100mS = 100U,
    TIMEOUT_250mS = 250U,
    TIMEOUT_500mS = 500U,
    TIMEOUT_750mS = 750U,
    TIMEOUT_1S = 1000U,
    TIMEOUT_2p5S = 2500U,
    TIMEOUT_5S = 5000U,
    TIMEOUT_7p5S = 7500U,
    TIMEOUT_10S = 10000U,
    TIMEOUT_30S = 30000U,
    TIMEOUT_60S = 60000U,
    TIMEOUT_MIN = TIMEOUT_1mS,
    TIMEOUT_MAX = TIMEOUT_60S
};

struct SPI_Config {
    SPI_Mode mode;
    SPI_Direction dir;
    SPI_DataSize dSize;
    SPI_ClockConfig clkCfg;
    SPI_ClockDividerConfig divider;
    SPI_NSSConfig nssMode;
    SPI_FirstBitConfig firstBit;
    SPI_CRCConfig crcCfg;
    SPI_Timeout timeout;
};

struct SPI_Buffer {
    uint8_t *pBuffer;
    uint16_t bufferSize;
    uint16_t transferCounter;
};

class SpiDriver
{
public:
    SpiDriver(SPI_Channel channel, const SPI_Config &config);
#ifdef HAL_SPI_MODULE_ENABLED
    SpiDriver(SPI_HandleTypeDef *hal_handler_spi, bool remaped);
#endif /* HAL_SPI_MODULE_ENABLED */
    ~SpiDriver();

    const SPI_Config *getConfig();
    void setConfig(const SPI_Config &config);
    const SPI_Channel &getChannel() const;
    const SPI_StateCode &getState();
    const SPI_ErrorCode &getError();
    bool isLocked();
    bool setTimeout(SPI_Timeout timeout);
    SPI_Timeout getTimeout() const;
    void setCrcPolynom(uint16_t polynom);
    uint16_t getCrcPolynom() const;

/**
  * @brief SPI data transfer methods
  * @param rxData is received data pointer
  * @param txData is data pointer to be transmitted
  * @param size of a transaction
  */
    void transmitData(const uint8_t *txData, uint16_t size);
    SpiDriver &operator<<(const uint8_t *txData);
    void receiveData(uint8_t *rxData, uint16_t size);
    SpiDriver &operator>>(uint8_t *rxData);
    void exchangeData(const uint8_t *txData, uint8_t *rxData, uint16_t size);

protected:
    void setConfig();
    void enableSpi();
    void disableSpi();
    SPI_TypeDef *getSpiPeriphAddr();
    void setClock(bool enabled);
    void setCrcPolynom();
    void remapSpi();
    void initSpiPins();
    void initSpiNSS();
    void enablePinsClock();
    void enableNSSClock();
    void spiError(SPI_ErrorCode error);

private:
    SpiDriver() {}
#if defined(SPI_I2SCFGR_I2SMOD)
    void switchInstance2SpiMode();
#endif

#ifdef HAL_SPI_MODULE_ENABLED
    SPI_HandleTypeDef *m_hSPI;
#endif /* HAL_SPI_MODULE_ENABLED */

    SPI_Channel m_channel;
    SPI_Config *m_config = nullptr;
    SPI_Buffer *m_txBuffer;
    SPI_Buffer *m_rxBuffer;
    bool m_locked;
    SPI_StateCode m_state;
    SPI_ErrorCode m_error;
    STM32F107VC_DRIVER::GPIO::OutputPin *m_pinSCK;
    STM32F107VC_DRIVER::GPIO::OutputPin *m_pinMISO;
    STM32F107VC_DRIVER::GPIO::OutputPin *m_pinMOSI;
    STM32F107VC_DRIVER::GPIO::PinControl *m_pinNSS;
    STM32F107VC_DRIVER::SYS_CLK::SystemClock *m_sysClk;

    //void (*onDataTransmitted)(SpiDriver *spi);
    //void (*onDataRecieved)(SpiDriver *spi);
    //DmaDriver *m_txDMA;
    //DmaDriver *m_rxDMA;
};

} // namespace SPI
} // namespace STM32F107VC_DRIVER

#endif // SPIDRIVER_H
