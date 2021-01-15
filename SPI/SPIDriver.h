#ifndef DRIVERS_SPIDRIVER_H_
#define DRIVERS_SPIDRIVER_H_

#include <ti/drivers/SPI.h>

#define SPI_SEL_LORA   (uint8_t)0
#define SPI_SEL_OTHER  (uint8_t)1

#define SPI_TXRX_BUF_LEN    100

int32_t SPI_Init();

/**
 * SPI_TxRx - send and receive over SPI,
 * @param idx_dev  - lora = 0, other = 1
 * @param tx_buffer - buffer to be sent
 * @param txex_len - transmission length
 * @return rx length
 */
int32_t SPI_TxRx(uint8_t idx_dev, uint8_t* tx_buffer, uint8_t txrx_len);

#endif /* DRIVERS_SPIDRIVER_H_ */
