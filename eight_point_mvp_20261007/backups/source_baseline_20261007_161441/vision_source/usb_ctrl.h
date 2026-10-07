#ifndef __USB_CTRL_H__
#define __USB_CTRL_H__

#include "Arduino.h"
#include "freertos/message_buffer.h"
#include "CommProtocol.h"

#define SERIAL_TX_PIN         1
#define SERIAL_RX_PIN         3
#define MAX_MSG_BUF_SIZE    512

class SerialPort_t {
public:
    HardwareSerial* uart; 
    CommProtocol_t protocol;

    void begin(HardwareSerial& uart, uint32_t baudrate, uint8_t tx_pin = SERIAL_TX_PIN, uint8_t rx_pin = SERIAL_RX_PIN);
    void register_ops_callback(ProtocolSuccessCallback cb);
    void rec_handler(void);

private:
    uint8_t rec_buffer[MAX_MSG_BUF_SIZE];
    uint16_t real_msg_rec_size;
};

extern SerialPort_t serial_port; 

#endif