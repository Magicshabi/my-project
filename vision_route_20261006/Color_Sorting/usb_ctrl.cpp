#include "usb_ctrl.h"

static MessageBufferHandle_t  xReceiveBuffer;
SerialPort_t serial_port;

void SerialEvent()
{
    uint8_t size = serial_port.uart->available();
    if(size) {
        uint8_t buff[size];
        serial_port.uart->readBytes(buff, size);
        xMessageBufferSend(xReceiveBuffer, buff, size, 0);
    }
}

void SerialPort_t::begin(HardwareSerial& uart, uint32_t baudrate, uint8_t tx_pin, uint8_t rx_pin)
{
    this->uart = &uart;
    this->uart->setTxBufferSize(512);
    this->uart->setRxBufferSize(512);
    xReceiveBuffer = xMessageBufferCreate(MAX_MSG_BUF_SIZE);
    this->uart->onReceive(SerialEvent, true);
    this->uart->begin(baudrate, SERIAL_8N1, rx_pin, tx_pin);
    protocol.begin();
}

void SerialPort_t::register_ops_callback(ProtocolSuccessCallback cb)
{   
    protocol.register_success_callback(cb);
}

void SerialPort_t::rec_handler(void)
{
    /* 一次可能积压多段 UART 回调写入的消息，需排空队列，否则 STOP 等控制帧会延迟 */
    do {
        real_msg_rec_size = xMessageBufferReceive(xReceiveBuffer, &rec_buffer, sizeof(rec_buffer), 0);
        if(real_msg_rec_size > 0) {
            protocol.parsing(rec_buffer, real_msg_rec_size);
        }
    } while(real_msg_rec_size > 0);
}
// void SerialPort_t::rec_handler(void)
// {
//     real_msg_rec_size = xMessageBufferReceive(xReceiveBuffer, &rec_buffer, sizeof(rec_buffer), 0);
//     if(real_msg_rec_size > 0) {
//         Serial.print("USB Recv: ");
//         for(int i=0; i<real_msg_rec_size; i++) {
//             Serial.printf("%02X ", rec_buffer[i]);
//         }
//         Serial.println();
//         protocol.parsing(rec_buffer, real_msg_rec_size);
//     }
// }