# LoRa for Texas Instruments CC26xx MCUs
A LoRa library for SemTech LoRa (or compatible RFM95, HopeRF, etc) chips  working with TI CC26xx MCUs. Also includes packet handling
for sending sensor data like temperature, humidity, etc via LoRa. 

## Integrating into TI CCS projects
The references below to *simple_peripheral* considers a project started from TI BLE Simple Peripheral Example.

1. Configure LoRa chip SPI pins
Open simple_peripheral.syscfg in Code Composer Studio and on TI Drivers > SPI > Add a new entry for your LoRa Chip. Call this ```CONFIG_SPI_LORA```. 
Configure the SPI pins, example in the screenshot below

2. Configure LoRa chip GPIO pins (RESET and IRQ)
Open simple_peripheral.syscfg in Code Composer Studio and on TI Drivers > GPIO > Add 2 new entries for RESET and IRQ: ```CONFIG_GPIO_LORA_RST``` and ```CONFIG_GPIO_LORA_IRQ```
The PIN MUX should correspond to where you connected the RESET and IRQ pins from LoRa chip to your MCU. The IRQ pin on LoRa Chip is **DIO_0** in mapping mode 00 + packet mode. Consult chip datasheet.

**Important**:  For ```CONFIG_GPIO_LORA_IRQ``` at *Callback Function* configuration for GPIO write *Lora_Callback*. This function exists in Lora.c and handles the IRQ. Example in screenshot below.

3. Add SPI code that handles SPI communication
Add the SPI/ folder to your CCS project. You can use your own SPI implementation but Lora.c uses SPI_TxRx() function to write/read from SPI and buffers as defined in the working SPIDriver.c example. 

4. Add LoRa code
Add Lora folder to your CCS project. Edit Lora.c and define/add your logging function (Display_printf, Log_info, etc ):

```
#define LOGGING_DUMMY(...)
// Change this to your logging function
#define ESPUart_WriteDebugPrintf LOGGING_DUMMY
```

5. Integrate into your project

In your *simple_peripheral.c* file add a function to handle LoRa packet receive. Example:

```
void SimplePeripheral_loraRecvHandler(void) {
    uint8_t recvLen = LORA_PACKET_MAX_SIZE;
    uint8_t *buf = (uint8_t *) malloc(recvLen * sizeof(uint8_t));
    _promiscuous = true;
    bool hasRecv = Lora_recv(buf, &recvLen);
    ESPUart_WriteDebugPrintf(DEBUG_LEVEL_SPI1,"LoRa: hasRecv: 0x%02x length: %d", hasRecv, recvLen);
    if (hasRecv) {
        lora_packet_t *packet = lora_packet_from_buf(buf);
        lora_dataparsed_t *parsed = lora_packet_parse(packet);
        //_lora_packet_display(packet);

        ESPUart_WriteDebugPrintf(DEBUG_LEVEL_SPI1,"LoRa: temperature: %f", parsed->temperature);
        ESPUart_WriteDebugPrintf(DEBUG_LEVEL_SPI1,"LoRa: pressure: %f", parsed->pressure);
        ESPUart_WriteDebugPrintf(DEBUG_LEVEL_SPI1,"LoRa: humidity: %f", parsed->humidity);
        ESPUart_WriteDebugPrintf(DEBUG_LEVEL_SPI1,"LoRa: accel x: %f y: %f z:%f", parsed->accel_x, parsed->accel_y, parsed->accel_z);

        free(packet);
        free(parsed);
    }

    free(buf);
}
```

Add initialisation code to your *SimplePeripheral_Init()*. Example:

```
static void SimplePeripheral_init(void)
{
     GPIO_init();
     Timer_init();
     SPI_Init();

     /* SPI LoRa Tests */
     Lora_printRegisters();
     Lora_reset();
     uint32_t lora_err = Lora_init();
     Lora_printRegisters();

#if GATEWAY_MODE
     Lora_setModeRx();
     Lora_setRXHandler(SimplePeripheral_loraRecvHandler);
#endif

  uint8_t lora_msg[] = "Hello from Lora";
  Lora_send(lora_msg, sizeof(lora_msg), 0);
  uint8_t recvLen = 0;
  bool hasRecv = Lora_recv(&lora_msg[0], &recvLen);
}
```

In your *SimplePeripheral_processAppMsg(...)* function add a case for LoRa messages that come from Lora_Callback() function. Example:

```
  switch (pMsg->event)
  {
    
    case SP_CONN_EVT:
      SimplePeripheral_processConnEvt((Gap_ConnEventRpt_t *)(pMsg->pData));
      break;

    case SP_LORA_RXTX_MESSAGE:
        Lora_handleInterrupt();
      break;
```

## Using library

1. Set internal chip registers to default:  Lora_reset() 
2. Init chip and set proper register values: Lora_init()
3. Send and/or receive packets with Lora_send() / Lora_recv()
4. As described above you should set your callback functions for packet receive using  Lora_setRXHandler(). A callback for transmit finished is set using Lora_setTXHandler()

## Using sensor data packets

LoraPacket.c uses a custom protocol to send data. See protocol [description](docs/LoraSensorsProtocol.html).

1. Create a new packet: ```lora_packet_t *packet = lora_packet_new(uid, MODE)```
2. Put data into packet: ```lora_packet_put_datatype(packet, TYPE, VALUE)```
3. To parse a received packet: ```lora_packet_from_buf(); lora_packet_parse()```

See below for a more complete example

## LoraPacket Example

```
    unsigned char uid[3] = {0xaa, 0xbb, 0xcc};
    lora_packet_t *packet;

    packet = lora_packet_new(uid, EXTENDED);

    lora_packet_put_datatype(packet, TEMPERATURE, 22.70);
    lora_packet_put_datatype(packet, PRESSURE, 101.70);
    lora_packet_put_datatype(packet, HUMIDITY, 85.6);

    float accell_data[] = {20.1, 30.1, 0.55 };
    lora_packet_put_datatype_multi(packet, ACCELEROMETER, accell_data, 3);

    lora_packet_t *recv_packet = lora_packet_from_buf(lora_packet_to_buf(packet));
    _lora_packet_display(recv_packet);
    lora_dataparsed_t *parsed = lora_packet_parse(recv_packet);

    printf("Temperature: %.2f\n", parsed->temperature);
    printf("Pressure: %.2f\n", parsed->pressure);
    printf("Humidity: %.2f\n", parsed->humidity);
    printf("Laser Range: %.2f\n", parsed->laser_range);
    printf("Air Quality: %.2f\n", parsed->air_quality);
    printf("Accelerometer x: %.2f y: %.2f z: %.2f\n", parsed->accel_x, parsed->accel_y, parsed->accel_z);
```

## Code
    Parts of Lora.c code borrows from RadioHead library but it has been rewritten in C to integrate easier with TI projects.
