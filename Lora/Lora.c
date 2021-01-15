#include "../SPI/SPIDriver.h"
#include "../Application/simple_peripheral.h"

#include "Lora.h"

#include <ti/drivers/GPIO.h>
#include <ti_drivers_config.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <stdlib.h>
#include <string.h>

#include <time.h>

#define YIELD delay(10)

#define LOGGING_DUMMY(...)

// Change this to your logging function
#define ESPUart_WriteDebugPrintf LOGGING_DUMMY

extern uint8_t SPIBufTX[SPI_TXRX_BUF_LEN];
extern uint8_t SPIBufRX[SPI_TXRX_BUF_LEN];

void (*_lora_rx_handler)(void) = NULL;
void (*_lora_tx_handler)(void) = NULL;

void Lora_write(uint8_t reg, uint8_t val)
{
    SPIBufTX[0] = reg | RF95_WRITE_MASK;
    SPIBufTX[1] = val;
    SPI_TxRx(SPI_SEL_LORA, SPIBufTX, 2);
}

uint8_t Lora_read(uint8_t reg)
{

    SPIBufTX[0] = reg;
    SPIBufTX[1] = 0xFF;

    memset(SPIBufRX, 0, SPI_TXRX_BUF_LEN);

    SPI_TxRx(SPI_SEL_LORA, SPIBufTX, 2);

    //ESPUart_WriteDebugPrintf(DEBUG_LEVEL_SPI, "Lora_read: Register 0x%02x: 0x%02x", reg, SPIBufRX[1]);
    return SPIBufRX[1];
}

uint8_t Lora_read_bulk(uint8_t reg, uint8_t* dest, uint8_t len) {
    uint8_t status = 0;

    if (len + 1 <= SPI_TXRX_BUF_LEN) {
        memset(SPIBufRX, 0, SPI_TXRX_BUF_LEN);
        SPIBufTX[0] = reg;
        status = SPI_TxRx(SPI_SEL_LORA, g_txSP1Buf, len);
        memcpy(dest, SPIBufRX, len);
    }

    return status;
}

uint8_t Lora_write_bulk(uint8_t reg, const uint8_t* src, uint8_t len) {
    uint8_t status = 0;

    SPIBufTX[0] = reg | RF95_WRITE_MASK;
    memcpy(SPIBufTX + 1, src, len);
    memset(SPIBufRX, 0, SPI_TXRX_BUF_LEN);

    status = SPI_TxRx(SPI_SEL_LORA, SPIBufTX, len);

    return status;
}

/**
 * @brief Provide a delay in milliseconds.
 * @param ms The number of Milli Seconds to delay.
 */
void delay(unsigned long ms)
{
    Task_sleep((ms * 1000) / Clock_tickPeriod);
}


void udelay(uint32_t us)
{
    Task_sleep( (us) / Clock_tickPeriod );
}

/**
 * @brief Get the number of elapsed milliseconds since the last boot.
 */
uint32_t millis(void)
{
    struct timespec now;

    clock_gettime(CLOCK_MONOTONIC, &now);
    return (now.tv_sec * 1000U) + ((now.tv_nsec / 1000000U) % 1000);
}

/**
 * @brief Generate a random number between limits.
 * @param min The minimum random value to be generated.
 * @param max The maximum random value to be generated.
 */
long random(long min, long max)
{
    return (rand() % (max + 1 - min) + min);
}

/**
 * @brief Set the state of a GPIO pin.
 * @param pin the Pin whose state is to be set.
 * @param value The state of the pin.
 */
void digitalWrite(unsigned char pin, unsigned char value)
{
    GPIO_write(pin, value);
}

/**
 * @brief Read the state of a GPIO pin.
 * @param pin the Pin whose state is to be set.
 * @return 1 If high, 0 if low.
 */
uint8_t digitalRead(uint8_t pin)
{
    return GPIO_read(pin);
}
/**
 * @brief Set the direction of a GPIO pin.
 * @param pin the Pin whose direction is to be set.
 * @param mode The direction of the pin (OUTPUT or INPUT)
 **/
void pinMode(uint8_t pin, WiringPinMode mode)
{
    if (mode == OUTPUT)
    {
        GPIO_setConfig(pin, GPIO_CFG_OUTPUT);
    }
    else if (mode == OUTPUT_OPEN_DRAIN)
    {
        GPIO_setConfig(pin, GPIO_CFG_OUT_OD_PD);
    }
    else if (mode == INPUT || mode == INPUT_FLOATING )
    {
        GPIO_setConfig(pin, GPIO_CFG_INPUT);
    }
    else if (mode == INPUT_ANALOG)
    {
        GPIO_setConfig(pin, GPIO_CFG_INPUT);
    }
    else if (mode == INPUT_PULLUP)
    {
        GPIO_setConfig(pin, GPIO_CFG_IN_PU);
    }
    else if (mode == INPUT_PULLDOWN)
    {
        GPIO_setConfig(pin, GPIO_CFG_IN_PD);
    }
    else if (mode == PWM)
    {
        GPIO_setConfig(pin, GPIO_CFG_INPUT);
    }
    else if (mode == PWM_OPEN_DRAIN) {
        GPIO_setConfig(pin, GPIO_CFG_INPUT);
    }
}

void Lora_Callback(uint_least8_t index)
{
    SimplePeripheral_enqueueMsgExternal(SP_LORA_RXTX_MESSAGE, 0);
}

void attachInterrupt()
{
   //GPIO_setConfig(CONFIG_GPIO_LORA_IRQ, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
   GPIO_setConfig(CONFIG_GPIO_LORA_IRQ, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_RISING);
   GPIO_setCallback(CONFIG_GPIO_LORA_IRQ, Lora_Callback);
   GPIO_enableInt(CONFIG_GPIO_LORA_IRQ);
   ESPUart_WriteDebugPrintf(DEBUG_LEVEL_SPI, "LoRa init: callback attached");
}


// Wait until no channel activity detected or timeout
bool Lora_waitCAD()
{
    if (!_cad_timeout)
        return true;

    // Wait for any channel activity to finish or timeout
    // Sophisticated DCF function...
    // DCF : BackoffTime = random() x aSlotTime
    // 100 - 1000 ms
    // 10 sec timeout
    unsigned long t = millis();
    ESPUart_WriteDebugPrintf(DEBUG_LEVEL_SPI, "LoRa waitCAD: start");
    while (Lora_isChannelActive())
    {
         if (millis() - t > _cad_timeout) {
             ESPUart_WriteDebugPrintf(DEBUG_LEVEL_SPI, "LoRa waitCAD: timeout");
             return false;
         }
         delay(random(1, 10) * 100); // Should these values be configurable? Macros?

    }
    ESPUart_WriteDebugPrintf(DEBUG_LEVEL_SPI, "LoRa waitCAD: end");
    return true;
}

bool Lora_waitPacketSent()
{
    //TODO: Check
    ESPUart_WriteDebugPrintf(DEBUG_LEVEL_SPI, "LoRa waitPacketSent: start");
    while (_mode == RHModeTx)
        YIELD; // Wait for any previous transmit to finish
    ESPUart_WriteDebugPrintf(DEBUG_LEVEL_SPI, "LoRa waitPacketSent: end");
    return true;
}

bool Lora_waitPacketSentTimeout(uint16_t timeout)
{
    unsigned long starttime = millis();
    while ((millis() - starttime) < timeout)
    {
        if (_mode != RHModeTx) // Any previous transmit finished?
           return true;
        YIELD;
    }
    return false;
}

void Lora_reset() {
    // Reset Chip  Section 7.2 RFM96W-V2.0.pdf
    GPIO_write(CONFIG_GPIO_LORA_RST, 1);
    GPIO_write(CONFIG_GPIO_LORA_RST, 0);
    //udelay(200); // >100us
    delay(10); // >100us
    GPIO_write(CONFIG_GPIO_LORA_RST, 1);
    delay(50); // 5ms
}

uint32_t Lora_init()
{
    Lora_write(RH_RF95_REG_01_OP_MODE,
               RH_RF95_MODE_SLEEP | RH_RF95_LONG_RANGE_MODE);

    delay(20); // Wait for sleep mode to take over from say, CAD

    // Workaround for Lora not getting initialized correctly on non-oad project
    Lora_write(RH_RF95_REG_01_OP_MODE,
                   RH_RF95_MODE_SLEEP | RH_RF95_LONG_RANGE_MODE);

    // Check we are in sleep mode, with LORA set
    uint8_t val = Lora_read(RH_RF95_REG_01_OP_MODE);

    ESPUart_WriteDebugPrintf(DEBUG_LEVEL_SPI, "LoRa init: Reg 0x01: 0x%02x: expected: 0x%02x", val, (RH_RF95_MODE_SLEEP | RH_RF95_LONG_RANGE_MODE) );

    // Set up interrupt
    attachInterrupt();

    if (val != (RH_RF95_MODE_SLEEP | RH_RF95_LONG_RANGE_MODE))
    {
        return 1; // No device present?
    }

    // Set up FIFO
    // We configure so that we can use the entire 256 byte FIFO for either receive
    // or transmit, but not both at the same time
    Lora_write(RH_RF95_REG_0E_FIFO_TX_BASE_ADDR, 0);
    Lora_write(RH_RF95_REG_0F_FIFO_RX_BASE_ADDR, 0);

    // Packet format is preamble + explicit-header + payload + crc
    // Explicit Header Mode
    // payload is TO + FROM + ID + FLAGS + message data
    // RX mode is implmented with RXCONTINUOUS
    // max message data length is 255 - 4 = 251 octets
    Lora_setModeIdle();
    // Set up default configuration
    // No Sync Words in LORA mode.
    //Lora_setModemConfig(Bw125Cr45Sf128); // Radio default
    Lora_setModemConfig(Bw125Cr45Sf4094); // Big Range and slow
    Lora_setPreambleLength(8); // Default is 8
    // An innocuous ISM frequency, same as RF22's
    Lora_setFrequency(868.5);
    //Lora_setFrequency(433.0);
    // Lowish power
    Lora_setTxPower(23, false);

    return 0;
}

// C level interrupt handler for this instance
// LORA is unusual in that it has several interrupt lines, and not a single, combined one.
// On MiniWirelessLoRa, only one of the several interrupt lines (DI0) from the RFM95 is usefully
// connected to the processor.
// We use this to get RxDone and TxDone interrupts
void Lora_handleInterrupt()
{
    // Read the interrupt register
    uint8_t irq_flags = Lora_read(RH_RF95_REG_12_IRQ_FLAGS);
    // Read the RegHopChannel register to check if CRC presence is signaled
    // in the header. If not it might be a stray (noise) packet.*
    uint8_t crc_present = Lora_read(RH_RF95_REG_1C_HOP_CHANNEL);

    ESPUart_WriteDebugPrintf(DEBUG_LEVEL_SPI, "Lora: IRQ Enter: mode: %d, irq_flags: 0x%02x crc: 0x%02x", _mode, irq_flags, crc_present);

    if (_mode == RHModeRx
            && ((irq_flags & (RH_RF95_RX_TIMEOUT | RH_RF95_PAYLOAD_CRC_ERROR))
                    | !(crc_present & RH_RF95_RX_PAYLOAD_CRC_IS_ON)))
//    if (_mode == RHModeRx && irq_flags & (RH_RF95_RX_TIMEOUT | RH_RF95_PAYLOAD_CRC_ERROR))
    {
        _rxBad++;
        ESPUart_WriteDebugMessage(DEBUG_LEVEL_SPI, "Lora: IRQ: ModeRx Bad");
    }
    else if (_mode == RHModeRx && irq_flags & RH_RF95_RX_DONE)
    {
        // Have received a packet
        uint8_t len = Lora_read(RH_RF95_REG_13_RX_NB_BYTES);

        // Reset the fifo read ptr to the beginning of the packet
        Lora_write(RH_RF95_REG_0D_FIFO_ADDR_PTR,
                   Lora_read(RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR));
        Lora_read_bulk(RH_RF95_REG_00_FIFO, _buf, len);
        _bufLen = len;
        Lora_write(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags

        // Remember the last signal to noise ratio, LORA mode
        // Per page 111, SX1276/77/78/79 datasheet
        _lastSNR = (int8_t) Lora_read(RH_RF95_REG_19_PKT_SNR_VALUE) / 4;

        // Remember the RSSI of this packet, LORA mode
        // this is according to the doc, but is it really correct?
        // weakest receivable signals are reported RSSI at about -66
        _lastRssi = Lora_read(RH_RF95_REG_1A_PKT_RSSI_VALUE);
        // Adjust the RSSI, datasheet page 87
        if (_lastSNR < 0)
            _lastRssi = _lastRssi + _lastSNR;
        else
            _lastRssi = (int) _lastRssi * 16 / 15;
        if (_usingHFport)
            _lastRssi -= 157;
        else
            _lastRssi -= 164;

        ESPUart_WriteDebugPrintf(DEBUG_LEVEL_SPI, "Lora: IRQ: ModeRx DONE, Receive RSSI: %d, length: %d", _lastRssi, len);
        // We have received a message.
        Lora_validateRxBuf();
        if (_rxBufValid)
            Lora_setModeIdle(); // Got one

        if(_lora_rx_handler != NULL) {
            _lora_rx_handler();
        }
    }
    else if (_mode == RHModeTx && irq_flags & RH_RF95_TX_DONE)
    {
        _txGood++;
        ESPUart_WriteDebugMessage(DEBUG_LEVEL_SPI, "Lora: IRQ: ModeTx DONE");
        Lora_setModeIdle();
        if (_lora_tx_handler != NULL) {
            _lora_tx_handler();
        }
    }
    else if (_mode == RHModeCad && irq_flags & RH_RF95_CAD_DONE)
    {
        _cad = irq_flags & RH_RF95_CAD_DETECTED;
        ESPUart_WriteDebugMessage(DEBUG_LEVEL_SPI, "Lora: IRQ: ModeCad DONE");
        Lora_setModeIdle();
    }
    // Sigh: on some processors, for some unknown reason, doing this only once does not actually
    // clear the radio's interrupt flag. So we do it twice. Why?
    Lora_write(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags
    Lora_write(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags
}

// Check whether the latest received message is complete and uncorrupted
void Lora_validateRxBuf()
{
    if (_bufLen < 4)
        return; // Too short to be a real message
    // Extract the 4 headers
    _rxHeaderTo = _buf[0];
    _rxHeaderFrom = _buf[1];
    _rxHeaderId = _buf[2];
    _rxHeaderFlags = _buf[3];

    ESPUart_WriteDebugPrintf(DEBUG_LEVEL_SPI, "Lora: promisc: %d, rxTo: %x", _promiscuous, _rxHeaderTo);

    if (_promiscuous || _rxHeaderTo == _thisAddress
            || _rxHeaderTo == RH_BROADCAST_ADDRESS)
    {
        _rxGood++;
        _rxBufValid = true;
        ESPUart_WriteDebugPrintf(DEBUG_LEVEL_SPI, "Lora: Valid Buffer");

    }
}

bool Lora_available()
{
    if (_mode == RHModeTx)
        return false;
    ESPUart_WriteDebugMessage(DEBUG_LEVEL_SPI, "LoRa: available");
    Lora_setModeRx();
    return _rxBufValid; // Will be set by the interrupt handler when a good message is received
}

void Lora_clearRxBuf()
{
    //ATOMIC_BLOCK_START;
    _rxBufValid = false;
    _bufLen = 0;
    //ATOMIC_BLOCK_END;
}

bool Lora_recv(uint8_t* buf, uint8_t* len)
{
    if (!Lora_available())
        return false;

    if (buf && len)
    {
        //ATOMIC_BLOCK_START;
        // Skip the 4 headers that are at the beginning of the rxBuf
        if (*len > _bufLen - RH_RF95_HEADER_LEN)
            *len = _bufLen - RH_RF95_HEADER_LEN;
        memcpy(buf, _buf + RH_RF95_HEADER_LEN + 1, *len);
        //ATOMIC_BLOCK_END;
    }
    Lora_clearRxBuf(); // This message accepted and cleared
    return true;
}

bool Lora_send(const uint8_t* data, uint8_t len, int timeout)
{
    if (len > RH_RF95_MAX_MESSAGE_LEN)
        return false;

    if (timeout > 0) {
        Lora_waitPacketSentTimeout(timeout); // Make sure we dont interrupt an outgoing message
    } else {
        Lora_waitPacketSent(); // Make sure we dont interrupt an outgoing message
    }

    Lora_setModeIdle();

    if (!Lora_waitCAD())
        return false;  // Check channel activity

    // Position at the beginning of the FIFO
    Lora_write(RH_RF95_REG_0D_FIFO_ADDR_PTR, 0);
    // The headers
    Lora_write(RH_RF95_REG_00_FIFO, _txHeaderTo);
    Lora_write(RH_RF95_REG_00_FIFO, _txHeaderFrom);
    Lora_write(RH_RF95_REG_00_FIFO, _txHeaderId);
    Lora_write(RH_RF95_REG_00_FIFO, _txHeaderFlags);
    // The message data
    Lora_write_bulk(RH_RF95_REG_00_FIFO, data, len);
    Lora_write(RH_RF95_REG_22_PAYLOAD_LENGTH, len + RH_RF95_HEADER_LEN);

    Lora_setModeTx(); // Start the transmitter
    // when Tx is done, interruptHandler will fire and radio mode will return to STANDBY
    return true;
}

bool Lora_printRegisters()
{
    uint8_t registers[] = { 0x01, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c,
                            0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x014,
                            0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c,
                            0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24,
                            0x25, 0x26, 0x27, 0x40, 0x41, 0x9F };

    uint8_t i;
    for (i = 0; i < sizeof(registers); i++)
    {
        ESPUart_WriteDebugPrintf(DEBUG_LEVEL_SPI, "Lora: Register 0x%02x: 0x%02x",
                                 registers[i], Lora_read(registers[i]));
    }

    return true;
}

void Lora_setModeIdle()
{
    if (_mode != RHModeIdle)
    {
        ESPUart_WriteDebugMessage(DEBUG_LEVEL_SPI, "Lora: setModeIdle");
        Lora_write(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_STDBY);
        _mode = RHModeIdle;
    }
    ESPUart_WriteDebugPrintf(DEBUG_LEVEL_SPI, "LoRa setModeIdle: Already in Idle mode");
}

uint8_t Lora_maxMessageLength()
{
    return RH_RF95_MAX_MESSAGE_LEN;
}

bool Lora_setFrequency(float centre)
{
    // Frf = FRF / FSTEP
    uint32_t frf = (centre * 1000000.0) / RH_RF95_FSTEP;
    Lora_write(RH_RF95_REG_06_FRF_MSB, (frf >> 16) & 0xff);
    Lora_write(RH_RF95_REG_07_FRF_MID, (frf >> 8) & 0xff);
    Lora_write(RH_RF95_REG_08_FRF_LSB, frf & 0xff);
    _usingHFport = (centre >= 779.0);
    ESPUart_WriteDebugMessage(DEBUG_LEVEL_SPI, "Lora: setFrequency");
    return true;
}

bool Lora_sleep()
{
    if (_mode != RHModeSleep)
    {
        ESPUart_WriteDebugMessage(DEBUG_LEVEL_SPI, "Lora: sleep");
        Lora_write(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_SLEEP);
        _mode = RHModeSleep;
    }
    return true;
}

void Lora_setModeRx()
{
    if (_mode != RHModeRx)
    {
        ESPUart_WriteDebugMessage(DEBUG_LEVEL_SPI, "Lora: setModeRx");
        Lora_write(RH_RF95_REG_40_DIO_MAPPING1, 0x00); // Interrupt on RxDone
        Lora_write(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_RXCONTINUOUS);
        _mode = RHModeRx;
    }
}

void Lora_setModeTx()
{
    if (_mode != RHModeTx)
    {
        ESPUart_WriteDebugMessage(DEBUG_LEVEL_SPI, "LoRa: setModeTx");
        Lora_write(RH_RF95_REG_40_DIO_MAPPING1, 0x40); // Interrupt on TxDone
        Lora_write(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_TX);
        _mode = RHModeTx;
    }
    ESPUart_WriteDebugMessage(DEBUG_LEVEL_SPI, "LoRa: setModeTx: Already in TX Mode");
}

void Lora_setTxPower(int8_t power, bool useRFO)
{
    // Sigh, different behaviors depending on whether the module use PA_BOOST or the RFO pin
    // for the transmitter output
    if (useRFO)
    {
        if (power > 14)
            power = 14;
        if (power < -1)
            power = -1;

        Lora_write(RH_RF95_REG_09_PA_CONFIG, RH_RF95_MAX_POWER | (power + 1));
    }
    else
    {
        if (power > 23)
            power = 23;
        if (power < 5)
            power = 5;

        // For RH_RF95_PA_DAC_ENABLE, manual says '+20dBm on PA_BOOST when OutputPower=0xf'
        // RH_RF95_PA_DAC_ENABLE actually adds about 3dBm to all power levels. We will us it
        // for 21, 22 and 23dBm
        if (power > 20)
        {
            Lora_write(RH_RF95_REG_4D_PA_DAC, RH_RF95_PA_DAC_ENABLE);
            power -= 3;
        }
        else
        {
            Lora_write(RH_RF95_REG_4D_PA_DAC, RH_RF95_PA_DAC_DISABLE);
        }

        // RFM95/96/97/98 does not have RFO pins connected to anything. Only PA_BOOST
        // pin is connected, so must use PA_BOOST
        // Pout = 2 + OutputPower.
        // The documentation is pretty confusing on this topic: PaSelect says the max power is 20dBm,
        // but OutputPower claims it would be 17dBm.
        // My measurements show 20dBm is correct
        Lora_write(RH_RF95_REG_09_PA_CONFIG, RH_RF95_PA_SELECT | (power - 5));
    }
    ESPUart_WriteDebugMessage(DEBUG_LEVEL_SPI, "LoRa: setTxPower");
}

// Sets registers from a canned modem configuration structure
void Lora_setModemRegisters(const ModemConfig* config)
{
    Lora_write(RH_RF95_REG_1D_MODEM_CONFIG1, config->reg_1d);
    Lora_write(RH_RF95_REG_1E_MODEM_CONFIG2, config->reg_1e);
    Lora_write(RH_RF95_REG_26_MODEM_CONFIG3, config->reg_26);
    ESPUart_WriteDebugMessage(DEBUG_LEVEL_SPI, "LoRa: setModemRegisters");
}

// Set one of the canned FSK Modem configs
// Returns true if its a valid choice
bool Lora_setModemConfig(ModemConfigChoice index)
{
    if (index > (signed int) (sizeof(MODEM_CONFIG_TABLE) / sizeof(ModemConfig)))
        return false;

    ModemConfig cfg;
    memcpy(&cfg, &MODEM_CONFIG_TABLE[index], sizeof(ModemConfig));
    Lora_setModemRegisters(&cfg);

    return true;
}

void Lora_setPreambleLength(uint16_t bytes)
{
    Lora_write(RH_RF95_REG_20_PREAMBLE_MSB, bytes >> 8);
    Lora_write(RH_RF95_REG_21_PREAMBLE_LSB, bytes & 0xff);
    ESPUart_WriteDebugMessage(DEBUG_LEVEL_SPI, "LoRa: setPreambleLength");
}

bool Lora_isChannelActive()
{
    // Set mode RHModeCad
    if (_mode != RHModeCad)
    {
        Lora_write(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_CAD);
        Lora_write(RH_RF95_REG_40_DIO_MAPPING1, 0x80); // Interrupt on CadDone
        _mode = RHModeCad;
    }

    ESPUart_WriteDebugMessage(DEBUG_LEVEL_SPI, "LoRa: isChannelActive");

    while (_mode == RHModeCad)
        YIELD;

    ESPUart_WriteDebugMessage(DEBUG_LEVEL_SPI, "LoRa: isChannelActive DONE");
    return _cad;
}

void Lora_enableTCXO()
{
    ESPUart_WriteDebugMessage(DEBUG_LEVEL_SPI, "LoRa: enableTCXO");

    while ((Lora_read(RH_RF95_REG_4B_TCXO) & RH_RF95_TCXO_TCXO_INPUT_ON)
            != RH_RF95_TCXO_TCXO_INPUT_ON)
    {
        Lora_sleep();
        Lora_write(
                RH_RF95_REG_4B_TCXO,
                (Lora_read(RH_RF95_REG_4B_TCXO) | RH_RF95_TCXO_TCXO_INPUT_ON));
    }
    ESPUart_WriteDebugMessage(DEBUG_LEVEL_SPI, "LoRa: enableTCXO DONE");
}

// From section 4.1.5 of SX1276/77/78/79
// Ferror = FreqError * 2**24 * BW / Fxtal / 500
int Lora_frequencyError()
{
    int32_t freqerror = 0;

    // Convert 2.5 bytes (5 nibbles, 20 bits) to 32 bit signed int
    // Caution: some C compilers make errors with eg:
    // freqerror = spiRead(RH_RF95_REG_28_FEI_MSB) << 16
    // so we go more carefully.
    freqerror = Lora_read(RH_RF95_REG_28_FEI_MSB);
    freqerror <<= 8;
    freqerror |= Lora_read(RH_RF95_REG_29_FEI_MID);
    freqerror <<= 8;
    freqerror |= Lora_read(RH_RF95_REG_2A_FEI_LSB);
    // Sign extension into top 3 nibbles
    if (freqerror & 0x80000)
        freqerror |= 0xfff00000;

    int error = 0; // In hertz
    float bw_tab[] = { 7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250, 500 };
    uint8_t bwindex = Lora_read(RH_RF95_REG_1D_MODEM_CONFIG1) >> 4;
    if (bwindex < (sizeof(bw_tab) / sizeof(float)))
        error = (float) freqerror * bw_tab[bwindex]
                * ((float) (1L << 24) / (float) RH_RF95_FXOSC / 500.0);
    // else not defined

    return error;
}

int Lora_lastSNR()
{
    return _lastSNR;
}

void Lora_setSpreadingFactor(uint8_t sf)
{
    if (sf <= 6)
        sf = RH_RF95_SPREADING_FACTOR_64CPS;
    else if (sf == 7)
        sf = RH_RF95_SPREADING_FACTOR_128CPS;
    else if (sf == 8)
        sf = RH_RF95_SPREADING_FACTOR_256CPS;
    else if (sf == 9)
        sf = RH_RF95_SPREADING_FACTOR_512CPS;
    else if (sf == 10)
        sf = RH_RF95_SPREADING_FACTOR_1024CPS;
    else if (sf == 11)
        sf = RH_RF95_SPREADING_FACTOR_2048CPS;
    else if (sf >= 12)
        sf = RH_RF95_SPREADING_FACTOR_4096CPS;

    // set the new spreading factor
    Lora_write(
            RH_RF95_REG_1E_MODEM_CONFIG2,
            (Lora_read(RH_RF95_REG_1E_MODEM_CONFIG2) & ~RH_RF95_SPREADING_FACTOR)
                    | sf);
    // check if Low data Rate bit should be set or cleared
    Lora_setLowDatarate();
}

void Lora_setSignalBandwidth(long sbw)
{
    uint8_t bw; //register bit pattern

    if (sbw <= 7800)
        bw = RH_RF95_BW_7_8KHZ;
    else if (sbw <= 10400)
        bw = RH_RF95_BW_10_4KHZ;
    else if (sbw <= 15600)
        bw = RH_RF95_BW_15_6KHZ;
    else if (sbw <= 20800)
        bw = RH_RF95_BW_20_8KHZ;
    else if (sbw <= 31250)
        bw = RH_RF95_BW_31_25KHZ;
    else if (sbw <= 41700)
        bw = RH_RF95_BW_41_7KHZ;
    else if (sbw <= 62500)
        bw = RH_RF95_BW_62_5KHZ;
    else if (sbw <= 125000)
        bw = RH_RF95_BW_125KHZ;
    else if (sbw <= 250000)
        bw = RH_RF95_BW_250KHZ;
    else
        bw = RH_RF95_BW_500KHZ;

    // top 4 bits of reg 1D control bandwidth
    Lora_write(RH_RF95_REG_1D_MODEM_CONFIG1,
               (Lora_read(RH_RF95_REG_1D_MODEM_CONFIG1) & ~RH_RF95_BW) | bw);
    // check if low data rate bit should be set or cleared
    Lora_setLowDatarate();
}

void Lora_setCodingRate4(uint8_t denominator)
{
    int cr = RH_RF95_CODING_RATE_4_5;

//    if (denominator <= 5)
//  cr = RH_RF95_CODING_RATE_4_5;
    if (denominator == 6)
        cr = RH_RF95_CODING_RATE_4_6;
    else if (denominator == 7)
        cr = RH_RF95_CODING_RATE_4_7;
    else if (denominator >= 8)
        cr = RH_RF95_CODING_RATE_4_8;

    // CR is bits 3..1 of RH_RF95_REG_1D_MODEM_CONFIG1
    Lora_write(
            RH_RF95_REG_1D_MODEM_CONFIG1,
            (Lora_read(RH_RF95_REG_1D_MODEM_CONFIG1) & ~RH_RF95_CODING_RATE)
                    | cr);
}

void Lora_setLowDatarate()
{
    // called after changing bandwidth and/or spreading factor
    //  Semtech modem design guide AN1200.13 says
    // "To avoid issues surrounding  drift  of  the  crystal  reference  oscillator  due  to  either  temperature  change
    // or  motion,the  low  data  rate optimization  bit  is  used. Specifically for 125  kHz  bandwidth  and  SF  =  11  and  12,
    // this  adds  a  small  overhead  to increase robustness to reference frequency variations over the timescale of the LoRa packet."

    // read current value for BW and SF
    uint8_t BW = Lora_read(RH_RF95_REG_1D_MODEM_CONFIG1) >> 4; // bw is in bits 7..4
    uint8_t SF = Lora_read(RH_RF95_REG_1E_MODEM_CONFIG2) >> 4; // sf is in bits 7..4

    // calculate symbol time (see Semtech AN1200.22 section 4)
    float bw_tab[] = { 7800, 10400, 15600, 20800, 31250, 41700, 62500, 125000,
                       250000, 500000 };

    float bandwidth = bw_tab[BW];

    float symbolTime = 1000.0 * (1 << SF) / bandwidth;  // ms

    // the symbolTime for SF 11 BW 125 is 16.384ms.
    // and, according to this :-
    // https://www.thethingsnetwork.org/forum/t/a-point-to-note-lora-low-data-rate-optimisation-flag/12007
    // the LDR bit should be set if the Symbol Time is > 16ms
    // So the threshold used here is 16.0ms

    // the LDR is bit 3 of RH_RF95_REG_26_MODEM_CONFIG3
    uint8_t current = Lora_read(
    RH_RF95_REG_26_MODEM_CONFIG3) & ~RH_RF95_LOW_DATA_RATE_OPTIMIZE; // mask off the LDR bit
    if (symbolTime > 16.0)
        Lora_write(RH_RF95_REG_26_MODEM_CONFIG3,
                   current | RH_RF95_LOW_DATA_RATE_OPTIMIZE);
    else
        Lora_write(RH_RF95_REG_26_MODEM_CONFIG3, current);

}

void Lora_setPayloadCRC(bool on)
{
    // Payload CRC is bit 2 of register 1E
    uint8_t current = Lora_read(
    RH_RF95_REG_1E_MODEM_CONFIG2) & ~RH_RF95_PAYLOAD_CRC_ON; // mask off the CRC

    if (on)
        Lora_write(RH_RF95_REG_1E_MODEM_CONFIG2,
                   current | RH_RF95_PAYLOAD_CRC_ON);
    else
        Lora_write(RH_RF95_REG_1E_MODEM_CONFIG2, current);
}

/* Functions to be called from Lora_handleInterrupt() function */
void Lora_setRXHandler(void(*func)(void)) {
    _lora_rx_handler = func;
}

void Lora_setTXHandler(void(*func)(void)) {
    _lora_tx_handler = func;
}
