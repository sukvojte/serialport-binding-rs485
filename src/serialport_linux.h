#ifndef PACKAGES_SERIALPORT_SRC_SERIALPORT_LINUX_H_
#define PACKAGES_SERIALPORT_SRC_SERIALPORT_LINUX_H_

#include <stdint.h>

int linuxSetCustomBaudRate(const int fd, const unsigned int baudrate);
int linuxGetSystemBaudRate(const int fd, int* const outbaud);

struct Rs485Config {
    bool rs485_rts_on_send = false;
    bool rs485_rts_after_send = true;
    uint8_t rs485_delay_before_send = 0;
    uint8_t rs485_delay_after_send = 0;
    bool rs485_rx_during_send = false;
    bool rs485_enabled = false;
};

int linuxSetRs485(const int fd, Rs485Config *config);

#endif  // PACKAGES_SERIALPORT_SRC_SERIALPORT_LINUX_H_

