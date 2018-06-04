#include "stm3210c_eval_cppAPI.h"
#include "SpiDriver.h"

SpiDriver_t SpiDriver_init(SPI_HandleTypeDef *handler_spi)
{
    return new STM32F107VC_DRIVER::SPI::SpiDriver(handler_spi, true);
}

void SpiDriver_destroy(SpiDriver_t untyped_ptr)
{
    STM32F107VC_DRIVER::SPI::SpiDriver *spi = static_cast<STM32F107VC_DRIVER::SPI::SpiDriver *>(untyped_ptr);
	delete spi;
}

void SpiDriver_transmitData(SpiDriver_t untyped_ptr, const uint8_t *txData, uint16_t size)
{
    STM32F107VC_DRIVER::SPI::SpiDriver *spi = static_cast<STM32F107VC_DRIVER::SPI::SpiDriver *>(untyped_ptr);
    spi->transmitData(txData, size);
}

void SpiDriver_receiveData(SpiDriver_t untyped_ptr, uint8_t *rxData, uint16_t size)
{
    STM32F107VC_DRIVER::SPI::SpiDriver *spi = static_cast<STM32F107VC_DRIVER::SPI::SpiDriver *>(untyped_ptr);
    spi->receiveData(rxData, size);
}

void SpiDriver_exchangeData(SpiDriver_t untyped_ptr, const uint8_t *txData, uint8_t *rxData, uint16_t size)
{
    STM32F107VC_DRIVER::SPI::SpiDriver *spi = static_cast<STM32F107VC_DRIVER::SPI::SpiDriver *>(untyped_ptr);
    spi->exchangeData(txData, rxData, size);
}
