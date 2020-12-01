#if defined(__linux__)

#include <sys/ioctl.h>
#include <asm/ioctls.h>
#include <asm/termbits.h>
#include <linux/serial.h>
#include "serialport_linux.h"

// Uses the termios2 interface to set nonstandard baud rates
int linuxSetCustomBaudRate(const int fd, const unsigned int baudrate) {
    struct termios2 t;

    if (ioctl(fd, TCGETS2, &t)) {
      return -1;
    }

    t.c_cflag &= ~CBAUD;
    t.c_cflag |= BOTHER;
    t.c_ospeed = t.c_ispeed = baudrate;

    if (ioctl(fd, TCSETS2, &t)) {
      return -2;
    }

    return 0;
}

// Uses termios2 interface to retrieve system reported baud rate
int linuxGetSystemBaudRate(const int fd, int* const outbaud) {
  struct termios2 t;

  if (ioctl(fd, TCGETS2, &t)) {
    return -1;
  }

  *outbaud = static_cast<int>(t.c_ospeed);

  return 0;
}

int linuxSetRs485(const int fd, Rs485Config *config) {
  struct serial_rs485 rs485conf;
  
  if (ioctl(fd, TIOCGRS485, &rs485conf) < 0)
	{
		return -1;
	} 


  rs485conf.flags &= ~(SER_RS485_ENABLED | SER_RS485_RTS_ON_SEND | SER_RS485_RTS_AFTER_SEND | SER_RS485_RX_DURING_TX);

  if(config->rs485_enabled)
	  rs485conf.flags |= SER_RS485_ENABLED;

  if(config->rs485_rts_on_send)
	  rs485conf.flags |= SER_RS485_RTS_ON_SEND;

  if(config->rs485_rts_after_send)
	  rs485conf.flags |= SER_RS485_RTS_AFTER_SEND;

	rs485conf.delay_rts_before_send = config->rs485_delay_before_send;

	rs485conf.delay_rts_after_send = config->rs485_delay_after_send;

  if(config->rs485_rx_during_send)
	  rs485conf.flags |= SER_RS485_RX_DURING_TX;

	if (ioctl(fd, TIOCSRS485, &rs485conf) < 0)
	{
		return -1;
	}

  return 0;
}

#endif
