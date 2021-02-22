/*
 * OS independent serial interface
 *
 * Heavily based on Pirate-Loader:
 * http://the-bus-pirate.googlecode.com/svn/trunk/bootloader-v4/pirate-loader/source/pirate-loader.c
 *
 */
#ifndef MYSERIAL_H_
#define MYSERIAL_H_

#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>

#ifdef USE_WINDOWS_OS
#include <windows.h>
#include <time.h>

#define B115200 115200
#define B921600 921600


typedef HANDLE SerialHandle;

typedef long speed_t;
#else // LINUX

#include <unistd.h>
#include <termios.h>
#include <sys/select.h>
#include <sys/types.h>
#include <sys/time.h>

typedef int SerialHandle;
#ifndef INVALID_HANDLE_VALUE
#define INVALID_HANDLE_VALUE -1
#endif

#endif

#ifdef IS_DARWIN
#include <IOKit/serial/ioss.h>
#include <sys/ioctl.h>

#define B1500000 1500000
#define B1000000 1000000
#define B921600  921600
#endif

int serial_setup(SerialHandle fd, unsigned long speed);
int serial_write(SerialHandle fd, const char *buf, int size);
int serial_read(SerialHandle fd, char *buf, int max_size, int timeout);
SerialHandle serial_open(const char *port);
SerialHandle serial_close(SerialHandle fd);

#ifdef __cplusplus
}
#endif

#endif
