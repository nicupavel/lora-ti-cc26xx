#ifndef DRIVERS_LORA_H_
#define DRIVERS_LORA_H_
#include <stdint.h>

#include "Lora_RF95.h"

#define SP_LORA_RXTX_MESSAGE                 33

#define MODULE_DISABLED         0
#define MODULE_ENABLED          1
#define MODULE_ENABLED_IN_RESET 2

#define ENABLE_LORA     MODULE_ENABLED

#define DEVBOARD        1
#define GATEWAY_MODE    1
#define LORA_HAS_IRQ    1

void Lora_write(uint8_t reg, uint8_t val);
uint8_t Lora_read(uint8_t reg);
void delay(unsigned long ms);

void Lora_reset(void);
uint32_t Lora_init(void);

void Lora_Callback(uint_least8_t index);
void Lora_handleInterrupt();

void Lora_validateRxBuf(void);
bool Lora_available(void);
void Lora_clearRxBuf();
bool Lora_recv(uint8_t* buf, uint8_t* len);
bool Lora_send(const uint8_t* data, uint8_t len, int timeout);

bool Lora_printRegisters(void);
void Lora_setModeIdle(void);
uint8_t Lora_maxMessageLength(void);
bool Lora_setFrequency(float centre);
bool Lora_sleep(void);
void Lora_setModeRx(void);
void Lora_setModeTx(void);
void Lora_setTxPower(int8_t power, bool useRFO);
void Lora_setModemRegisters(const ModemConfig* config);
bool Lora_setModemConfig(ModemConfigChoice index);
void Lora_setPreambleLength(uint16_t bytes);
bool Lora_isChannelActive(void);
void Lora_enableTCXO(void);
int Lora_frequencyError(void);
int Lora_lastSNR(void);
void Lora_setSpreadingFactor(uint8_t sf);
void Lora_setSignalBandwidth(long sbw);
void Lora_setCodingRate4(uint8_t denominator);
void Lora_setLowDatarate(void);
void Lora_setPayloadCRC(bool on);
void Lora_setRXHandler(void(*func)(void));
void Lora_setTXHandler(void(*func)(void));

typedef enum WiringPinMode {
    OUTPUT, /**< Basic digital output: when the pin is HIGH, the
               voltage is held at +3.3v (Vcc) and when it is LOW, it
               is pulled down to ground. */

    OUTPUT_OPEN_DRAIN, /**< In open drain mode, the pin indicates
                          "low" by accepting current flow to ground
                          and "high" by providing increased
                          impedance. An example use would be to
                          connect a pin to a bus line (which is pulled
                          up to a positive voltage by a separate
                          supply through a large resistor). When the
                          pin is high, not much current flows through
                          to ground and the line stays at positive
                          voltage; when the pin is low, the bus
                          "drains" to ground with a small amount of
                          current constantly flowing through the large
                          resistor from the external supply. In this
                          mode, no current is ever actually sourced
                          from the pin. */

    INPUT, /**< Basic digital input. The pin voltage is sampled; when
              it is closer to 3.3v (Vcc) the pin status is high, and
              when it is closer to 0v (ground) it is low. If no
              external circuit is pulling the pin voltage to high or
              low, it will tend to randomly oscillate and be very
              sensitive to noise (e.g., a breath of air across the pin
              might cause the state to flip). */

    INPUT_ANALOG, /**< This is a special mode for when the pin will be
                     used for analog (not digital) reads.  Enables ADC
                     conversion to be performed on the voltage at the
                     pin. */

    INPUT_PULLUP, /**< The state of the pin in this mode is reported
                     the same way as with INPUT, but the pin voltage
                     is gently "pulled up" towards +3.3v. This means
                     the state will be high unless an external device
                     is specifically pulling the pin down to ground,
                     in which case the "gentle" pull up will not
                     affect the state of the input. */

    INPUT_PULLDOWN, /**< The state of the pin in this mode is reported
                       the same way as with INPUT, but the pin voltage
                       is gently "pulled down" towards 0v. This means
                       the state will be low unless an external device
                       is specifically pulling the pin up to 3.3v, in
                       which case the "gentle" pull down will not
                       affect the state of the input. */

    INPUT_FLOATING, /**< Synonym for INPUT. */

    PWM, /**< This is a special mode for when the pin will be used for
            PWM output (a special case of digital output). */

    PWM_OPEN_DRAIN, /**< Like PWM, except that instead of alternating
                       cycles of LOW and HIGH, the voltage on the pin
                       consists of alternating cycles of LOW and
                       floating (disconnected). */
} WiringPinMode;


#endif /* DRIVERS_LORA_H_ */
