#include <stdio.h>
#include "LoraPacket.h"

lora_packet_t *lora_packet_new(uint8_t *uid, lora_payload_type_t payload_type) {
    lora_packet_t *packet = (lora_packet_t *) malloc(sizeof(lora_packet_t));

    if (packet) {
	    packet->preamble = (payload_type) | (LORA_VERSION & ~LORA_VERSION_MASK) << 4;                
	    memcpy(packet->uid, uid, LORA_UID_SIZE);
        packet->size = 0;
        memset(packet->payload, DATATYPE_END, LORA_PACKET_PAYLOAD_SIZE);        
    }

    return packet;
}

lora_packet_t *lora_packet_from_buf(uint8_t *buf) {
    lora_packet_t *packet = (lora_packet_t *) malloc(sizeof(lora_packet_t));

    if (packet && buf) {
	    packet->preamble = buf[0];                
	    memcpy(packet->uid, &buf[1], LORA_UID_SIZE);

        packet->size = buf[1 + LORA_UID_SIZE];
        if (packet->size > LORA_PACKET_PAYLOAD_SIZE) {
            packet->size = LORA_PACKET_PAYLOAD_SIZE;
        }

        memcpy(packet->payload, &buf[ 1 + LORA_UID_SIZE + 1], packet->size);        
    }

    return packet;
}

uint8_t *lora_packet_to_buf(lora_packet_t *packet) {
    uint8_t *buf = (uint8_t *) malloc(sizeof(uint8_t) * LORA_PACKET_MAX_SIZE);
    
    if (packet && buf) {
	    buf[0] = packet->preamble;                
	    memcpy(&buf[1], packet->uid, LORA_UID_SIZE);

        if (packet->size > LORA_PACKET_PAYLOAD_SIZE) {
            packet->size = LORA_PACKET_PAYLOAD_SIZE;
        }

        buf[1 + LORA_UID_SIZE] = packet->size;        
        memcpy(&buf[ 1 + LORA_UID_SIZE + 1], packet->payload, packet->size);        
    }

    return buf;
}

void _lora_packet_display(lora_packet_t *packet) {
    if (!packet) return;
    lora_debug("0x%02x:[%d]:", (packet->preamble & LORA_FLAGS_MASK),  (packet->size + LORA_HEADER_SIZE));
    lora_debug("{0x%02x}", packet->preamble);
    /*
    lora_debug("(");
    for (int i = 0; i < LORA_UID_SIZE; i++) {
        lora_debug("0x%02x", packet->uid[i]);
        if (i < LORA_UID_SIZE) {
            lora_debug(":");
        }
    }
    lora_debug(")");
    */
    
    for (int i = 0; i < packet->size; i++) {
        lora_debug("[0x%02x]", packet->payload[i]);
    }

    lora_debug("\n");
}

//TODO: Return a list of lora_dataparsed_t for historical data
lora_dataparsed_t *lora_packet_parse(lora_packet_t *packet) {
    lora_dataparsed_t *parsed;
    parsed = (lora_dataparsed_t *) malloc(sizeof(lora_dataparsed_t));
    
    if (!parsed || !packet || !packet->payload) {
        return NULL;
    }

    parsed->thermal_raw = (lora_sensor_thermal_raw_t *) malloc(sizeof(lora_sensor_thermal_raw_t));
    if (!parsed->thermal_raw) {
        free(parsed);
        return NULL;
    }

    /* Default unknown values */
    parsed->historic_offset = 0;
    parsed->temperature = LORA_PACKET_UNKNOWN_VALUE;
    parsed->pressure = LORA_PACKET_UNKNOWN_VALUE;
    parsed->humidity = LORA_PACKET_UNKNOWN_VALUE;
    parsed->laser_range = LORA_PACKET_UNKNOWN_VALUE;
    parsed->air_quality = LORA_PACKET_UNKNOWN_VALUE;
    parsed->accel_x = LORA_PACKET_UNKNOWN_VALUE;
    parsed->accel_y = LORA_PACKET_UNKNOWN_VALUE;
    parsed->accel_z = LORA_PACKET_UNKNOWN_VALUE;
    parsed->battery = LORA_PACKET_UNKNOWN_VALUE;    

    uint8_t offset = 0;

    while(offset < packet->size) {
        uint8_t datatype;
        _lora_packet_get8(packet, offset, &datatype);

        if (&datatype == NULL || datatype <= UNKNOWN || datatype >= DATATYPE_END) {
            lora_debug("Unknown datatype 0x%02x at pos %d\n", datatype, offset);
            offset += 1;
            continue;
        };

        switch (datatype)
        {
        case TEMPERATURE:
            lora_packet_get_datatype(packet, datatype, offset + 1, &parsed->temperature);            
        break;
        case PRESSURE:
            lora_packet_get_datatype(packet, datatype, offset + 1, &parsed->pressure);
        break;        
        case HUMIDITY:
            lora_packet_get_datatype(packet, datatype, offset + 1, &parsed->humidity);
        break;
        case LASER_RANGE:
            lora_packet_get_datatype(packet, datatype, offset + 1, &parsed->laser_range);
        break;
        case AIR_QUALITY:
            lora_packet_get_datatype(packet, datatype, offset + 1, &parsed->air_quality);
        break;
        case ACCELEROMETER:
            lora_packet_get_datatype(packet, datatype, offset + 1, &parsed->accel_x);
            offset += 2;
            lora_packet_get_datatype(packet, datatype, offset + 1, &parsed->accel_y);
            offset += 2;
            lora_packet_get_datatype(packet, datatype, offset + 1, &parsed->accel_z);
        break;
        case BATTERY:
            lora_packet_get_datatype(packet, datatype, offset + 1, &parsed->battery);
        break;
        default:
            break;
        }

        offset += 3;        
    }
    return parsed;
}

/**
 * Add data to packet
 */

void _lora_packet_clear(lora_packet_t *packet) {
    if (!packet) return;
    packet->size = LORA_HEADER_SIZE; 
    memset(packet->uid, 0, LORA_UID_SIZE);
    memset(packet->payload, 0, LORA_PACKET_PAYLOAD_SIZE);
}

void lora_packet_put_datatype(lora_packet_t *packet, lora_data_type_t type, float data) {
    float _data[] = { data };
    return lora_packet_put_datatype_multi(packet, type, _data, 1);
}

void lora_packet_put_datatype_multi(lora_packet_t *packet, lora_data_type_t type, float *data, uint8_t data_len) {

    uint16_t *_data = (uint16_t *) malloc(sizeof(uint16_t) * data_len);    

    if (!packet || !data || !_data) {
        return;
    }
    
    _lora_packet_put8(packet, type);

    if (type == AIR_QUALITY || type == ACCELEROMETER) {
        for (int i = 0; i < data_len; i++) {
            _data[i] = data[i] * 100;
            _lora_packet_put16(packet, _data[i]);
        }
    } else {
        if (type == TEMPERATURE) {
            _data[0] = (data[0] * 100) + 4000;
            lora_debug("%f=[0x%04x]\n", data[0], _data[0]);
        } else {
            _data[0] = data[0] * 100;
        }

        if (type != HISTORICAL_SYNC) {
            _lora_packet_put16(packet, _data[0]);
        }
    }
}

/**
 * Add data to packet (low level)
 */
void _lora_packet_put8(lora_packet_t *packet, uint8_t byte) {
    if (!packet || packet->size >= LORA_PACKET_PAYLOAD_SIZE) return;
    packet->payload[packet->size++] = byte;
}

void _lora_packet_put16(lora_packet_t *packet, uint16_t data) {
    _lora_packet_put8(packet, data & 0x00FF);
    _lora_packet_put8(packet, data >> 8);
}

void _lora_packet_put32(lora_packet_t *packet, uint32_t data) {
    _lora_packet_put16(packet, data & 0x0000FFFF);
    _lora_packet_put16(packet, data >> 16);
}


/**
 * Get data from packet
 */

void lora_packet_get_datatype(lora_packet_t *packet, lora_data_type_t type, uint8_t offset, float *data) {
    uint16_t ret16;

    _lora_packet_get16(packet, offset, &ret16);

    if (type == TEMPERATURE) {
        *data = (ret16 - 4000) / 100.0;
        //lora_debug("Found TEMPERATURE (%.2f (0x%04x)) at offset %d\n", *data, ret16, offset);
    } else {
        *data = ret16 / 100.0;
        //lora_debug("Found TYPE %d (%.2f (0x%04x)) at offset %d\n", type, *data, ret16, offset);
    }           
}

/**
 * Get data from packet (low level)
 */

void _lora_packet_get8(lora_packet_t *packet, size_t pos, uint8_t *ret) {
    if (!packet || pos > packet->size) {
	ret = NULL;
	return;
    }
    *ret = packet->payload[pos];
}

void _lora_packet_get16(lora_packet_t *packet, size_t pos, uint16_t *ret) {
    if (!packet || pos >= packet->size) {
	ret = NULL;
	return;
    }
    uint16_t tmp = packet->payload[pos];
    *ret = tmp | packet->payload[pos+1] << 8;
}

void _lora_packet_get32(lora_packet_t *packet, size_t pos, uint32_t *ret) {
    if (!packet || pos + 3 >= packet->size) {
	ret = NULL;
	return;
    }

    uint16_t b1, b2;
    _lora_packet_get16(packet, pos, &b1);
    _lora_packet_get16(packet, pos + 2, &b2);

    if (!b1 || !b2) {
	ret = NULL;
	return;
    }

    *ret =  b1 | (b2 << 16);
}
