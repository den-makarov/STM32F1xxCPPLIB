#ifndef SPIDRIVERCPPAPI_H
#define SPIDRIVERCPPAPI_H

#define SPI_DRIVER
//#undef SPI_DRIVER

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

typedef void *SpiDriver_t;

EXTERNC SpiDriver_t SpiDriver_init(SPI_HandleTypeDef *handler_spi);
EXTERNC void SpiDriver_destroy(SpiDriver_t untyped_ptr);
EXTERNC void SpiDriver_exchangeData(SpiDriver_t untyped_ptr, const uint8_t *txData, uint8_t *rxData, uint16_t size);
EXTERNC void SpiDriver_receiveData(SpiDriver_t untyped_ptr, uint8_t *rxData, uint16_t size);
EXTERNC void SpiDriver_transmitData(SpiDriver_t untyped_ptr, const uint8_t *txData, uint16_t size);

#undef EXTERNC
#endif // SPIDRIVERCPPAPI_H
