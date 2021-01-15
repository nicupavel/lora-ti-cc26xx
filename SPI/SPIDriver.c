#include "SPIDriver.h"
#include <ti/drivers/GPIO.h>
#include <ti/drivers/spi/SPICC26X2DMA.h>
#include <ti_drivers_config.h>
#include "ESPUart.h"
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>

#include "../Application/simple_peripheral.h"

#include <string.h>

SPI_Handle SPIHandle;
SPI_Params SPIParams;
SPI_Transaction SPITransaction;

uint8_t SPIBufTX[SPI_TXRX_BUF_LEN] = {0};
uint8_t SPIBufRX[SPI_TXRX_BUF_LEN] = {0};

int32_t SPI_Init()
{
    // ChipSelect
    GPIO_setConfig(CONFIG_GPIO_LORA_EEPROM_CS, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    SPI_Params_init(&SPIParams);
    SPIParams.bitRate     = 400000;
    SPIParams.mode        = SPI_MASTER;

    // Configure the transaction
    SPITransaction.count = SPI_TXRX_BUF_LEN;
    SPITransaction.txBuf = SPIBufTX;
    SPITransaction.rxBuf = SPIBufRX;

    // Open the SPI and perform the transfer
    SPIHandle = SPI_open(CONFIG_SPI_LORA, &SPIParams);

    return 0;
}

void user_delay_us(uint32_t period)
{
    Task_sleep( (period) / Clock_tickPeriod );
}

int32_t SPI_TxRx(uint8_t idx_dev, uint8_t* tx_buffer, uint8_t txrx_len)
{
    if (idx_dev == SPI_SEL_LORA)
    {
        SPIParams.frameFormat = SPI_POL0_PHA0;
        GPIO_write(CONFIG_GPIO_LORA_EEPROM_CS, !DEVBOARD); // 0 on TI Devboard, 1 on Prototype
    }
    else
    {
        SPIParams.frameFormat = SPI_POL1_PHA1;
        GPIO_write(CONFIG_GPIO_LORA_EEPROM_CS, 1);
        user_delay_us(10);
        GPIO_write(CONFIG_GPIO_LORA_EEPROM_CS, 0);
    }

    if (tx_buffer != SPIBufTX) // permits reusing the same buffer for message composing
        memcpy(SPIBufTX, tx_buffer, txrx_len);

    SPITransaction.count = txrx_len;

    SPI_transfer(SPIHandle, &SPITransaction);

    if (select_lora == SPI_SEL_LORA) {
        //user_delay_us(10);
        GPIO_write(CONFIG_GPIO_LORA_EEPROM_CS, !!DEVBOARD); // 1 on TI Devboard, 0 on Prototype
    }

    return txrx_len;
}

