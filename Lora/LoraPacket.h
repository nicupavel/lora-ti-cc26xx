#ifndef DRIVERS_LORA_LORAPACKET_H_
#define DRIVERS_LORA_LORAPACKET_H_

#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define LORA_VERSION 0x01

#define LORA_VERSION_MASK 0xF0
#define LORA_FLAGS_MASK  0x0F

#define LORA_UID_SIZE 0x03
#define LORA_HEADER_SIZE (0x02 + LORA_UID_SIZE)

#define LORA_PACKET_MAX_SIZE 251
#define LORA_PACKET_PAYLOAD_SIZE (LORA_PACKET_MAX_SIZE - LORA_HEADER_SIZE)

#define LORA_PACKET_UNKNOWN_VALUE 0xffffffff

#ifdef HAS_PRINTF
#define lora_debug(format, ...) fprintf(stderr, format, ##__VA_ARGS__)
#else 
#include "Drivers/ESPUart.h"
#define lora_debug(format, ...) ESPUart_WriteDebugPrintf(DEBUG_LEVEL_SPI1, format, ##__VA_ARGS__)
#endif

typedef enum _lora_data_type {
    UNKNOWN = 0x00,
    TEMPERATURE = 0x01,
    PRESSURE,
    HUMIDITY,
    LASER_RANGE,
    AIR_QUALITY,
    ACCELEROMETER,
    BATTERY,
    THERMAL_RAW,
    HISTORICAL_SYNC,
    DATATYPE_END,
} lora_data_type_t;

typedef enum _lora_payload_type {
    SIMPLE = 0x01,
    EXTENDED = 0x02
} lora_payload_type_t;

typedef struct _lora_sensor_thermal_raw { 
    uint8_t row;
    uint8_t data[32];
    struct _lora_sensor_thermal_raw *next;
} lora_sensor_thermal_raw_t;

typedef struct _lora_dataparsed {
    uint8_t historic_offset;
    float temperature;
    float pressure;
    float humidity;
    float laser_range;
    float air_quality;
    float accel_x;
    float accel_y;
    float accel_z;
    float battery;
    lora_sensor_thermal_raw_t *thermal_raw;
} lora_dataparsed_t;

typedef struct _lora_packet {
    uint8_t preamble;
    uint8_t uid[3];
    uint8_t size;
    uint8_t payload[LORA_PACKET_PAYLOAD_SIZE];    
} lora_packet_t;


lora_packet_t *lora_packet_new(unsigned char *uid, lora_payload_type_t payload_type);
lora_packet_t *lora_packet_from_buf(uint8_t *buf);
uint8_t *lora_packet_to_buf(lora_packet_t *packet);

void lora_packet_put_datatype(lora_packet_t *packet, lora_data_type_t type, float data);
void lora_packet_put_datatype_multi(lora_packet_t *packet, lora_data_type_t type, float *data, uint8_t data_len);

void lora_packet_get_datatype(lora_packet_t *packet, lora_data_type_t type, uint8_t offset, float *data);
lora_dataparsed_t *lora_packet_parse(lora_packet_t *packet);

void _lora_packet_display(lora_packet_t *packet);
void _lora_packet_clear(lora_packet_t *packet);
void _lora_packet_put8(lora_packet_t *packet, uint8_t byte);
void _lora_packet_put16(lora_packet_t *packet, uint16_t data);
void _lora_packet_put32(lora_packet_t *packet, uint32_t data);

void _lora_packet_get8(lora_packet_t *packet, size_t pos, uint8_t *ret);
void _lora_packet_get16(lora_packet_t *packet, size_t pos, uint16_t *ret);
void _lora_packet_get32(lora_packet_t *packet, size_t pos, uint32_t *ret);

#endif /* DRIVERS_LORA_LORAPACKET_H_ */
