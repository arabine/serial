/*
 * OS independent serial interface
 *
 * Heavily based on Pirate-Loader:
 * http://the-bus-pirate.googlecode.com/svn/trunk/bootloader-v4/pirate-loader/source/pirate-loader.c
 *
 */


#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#ifdef USE_WINDOWS_OS
#else
#include <unistd.h>     // for read/write
#include <sys/ioctl.h>
#endif
#include <string.h>

#include "serial.h"

int serial_setup(int fd, unsigned long speed)
{
#ifdef USE_WINDOWS_OS

#define FC_DTRDSR       0x01
#define FC_RTSCTS       0x02
#define FC_XONXOFF      0x04
#define ASCII_BEL       0x07
#define ASCII_BS        0x08
#define ASCII_LF        0x0A
#define ASCII_CR        0x0D
#define ASCII_XON       0x11
#define ASCII_XOFF      0x13

    COMMTIMEOUTS CommTimeOuts;
    CommTimeOuts.ReadIntervalTimeout = 0xFFFFFFFF;
    CommTimeOuts.ReadTotalTimeoutMultiplier = 0;
    CommTimeOuts.ReadTotalTimeoutConstant = 0;
    CommTimeOuts.WriteTotalTimeoutMultiplier = 0;
    CommTimeOuts.WriteTotalTimeoutConstant = 5000;
    SetCommTimeouts( fd, &CommTimeOuts );

    DCB dcb;
    dcb.DCBlength = sizeof( DCB );
    GetCommState( fd, &dcb );
    dcb.BaudRate = speed;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    unsigned char ucSet;
    ucSet = (unsigned char) ( ( FC_RTSCTS & FC_DTRDSR ) != 0 );
    ucSet = (unsigned char) ( ( FC_RTSCTS & FC_RTSCTS ) != 0 );
    ucSet = (unsigned char) ( ( FC_RTSCTS & FC_XONXOFF ) != 0 );
    if( !SetCommState( fd, &dcb ) ||
        !SetupComm( fd, 10000, 10000 ))
    {
        DWORD dwError = GetLastError();

        CloseHandle( fd );
        return( 1 );
    }

	return 0;
#else
	struct termios t_opt;
	speed_t baud;

	switch (speed) {
		case 921600:
			baud = B921600;
			break;
		case 2400:
            baud = B2400;
            break;
		case 4800:
            baud = B4800;
            break;
		case 9600:
		    baud = B9600;
		    break;
		case 19200:
		    baud = B19200;
		    break;
		case 38400:
		    baud = B38400;
		    break;
        case 57600:
            baud = B57600;
            break;
		case 115200:
			baud = B115200;
			break;
		case 1000000:
			baud = B1000000;
			break;
		case 1500000:
			baud = B1500000;
			break;
		default:
			printf("unknown speed setting \n");
			return -1;
			break;
	}

	/* set the serial port parameters */
	fcntl(fd, F_SETFL, 0);
	tcgetattr(fd, &t_opt);
	cfsetispeed(&t_opt, baud);
	cfsetospeed(&t_opt, baud);
	t_opt.c_cflag |= (CLOCAL | CREAD);
	t_opt.c_cflag &= ~PARENB;
	t_opt.c_cflag &= ~CSTOPB;
	t_opt.c_cflag &= ~CSIZE;
	t_opt.c_cflag |= CS8;
	t_opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	t_opt.c_iflag &= ~(IXON | IXOFF | IXANY);
	t_opt.c_iflag &= ~(ICRNL | INLCR);
	t_opt.c_oflag &= ~(OCRNL | ONLCR);
	t_opt.c_oflag &= ~OPOST;
    t_opt.c_cc[VMIN] = 1; // blocking read until N chars received
    t_opt.c_cc[VTIME] = 1;

#ifdef IS_DARWIN
	if( tcsetattr(fd, TCSANOW, &t_opt) < 0 ) {
		return -1;
	}

	return ioctl( fd, IOSSIOSPEED, &baud );
#else
	tcflush(fd, TCIOFLUSH);

	return tcsetattr(fd, TCSANOW, &t_opt);
#endif
#endif
}

int serial_write(int fd, const char *buf, int size)
{
	int ret = -1;
#ifdef USE_WINDOWS_OS
	HANDLE hCom = (HANDLE)fd;
	unsigned long bwritten = 0;

    OVERLAPPED osWrite = {0};
    DWORD dwRes;

    // Create this write operation's OVERLAPPED structure's hEvent.
    osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    if (osWrite.hEvent != NULL)
    {
        // Issue write.
        if (!WriteFile(hCom, buf, size, &bwritten, &osWrite))
        {
            if (GetLastError() == ERROR_IO_PENDING)
            {
                 // Write is pending.
                 dwRes = WaitForSingleObject(osWrite.hEvent, INFINITE);
                 switch(dwRes)
                 {
                    // OVERLAPPED structure's event has been signaled.
                    case WAIT_OBJECT_0:
                         if (GetOverlappedResult(hCom, &osWrite, &bwritten, FALSE))
                         {
                             ret = bwritten;
                         }
                         break;

                    default:
                         // An error has occurred in WaitForSingleObject.
                         // This usually indicates a problem with the
                        // OVERLAPPED structure's event handle.
                         break;
                 }
            }
            else
            {
              // WriteFile failed, but isn't delayed. Report error and abort.
            }
        }
        else
        {
          // WriteFile completed immediately.
           ret = bwritten;
        }
    }

#else
	ret = write(fd, buf, size);
#endif

#ifdef DEBUG
	if (ret != size)
		fprintf(stderr, "Error sending data (written %d should have written %d)\n", ret, size);
#endif

	return ret;
}

// timeout in seconds
int serial_read(int fd, char *buf, int max_size, int timeout)
{
	int len = 0;

#ifdef USE_WINDOWS_OS

    BOOL bReadStatus;
    DWORD dwBytesRead, dwErrorFlags;
    COMSTAT ComStat;
    OVERLAPPED m_OverlappedRead;
    memset( &m_OverlappedRead, 0, sizeof( OVERLAPPED ) );
    m_OverlappedRead.hEvent = CreateEvent( NULL, TRUE, FALSE, NULL );

    ClearCommError( fd, &dwErrorFlags, &ComStat );
    if( !ComStat.cbInQue ) return( 0 );

    dwBytesRead = 0;
//    if( size < (int) dwBytesRead ) dwBytesRead = (DWORD) size;

    bReadStatus = ReadFile( fd, buf, max_size, &dwBytesRead, &m_OverlappedRead );
    if( !bReadStatus )
    {
        if( GetLastError() == ERROR_IO_PENDING )
        {
            WaitForSingleObject( m_OverlappedRead.hEvent, timeout * 1000 );
            return( (int) dwBytesRead );
        }
        return( 0 );
    }

    len = dwBytesRead;

//	ret = ReadFile(hCom, buf, size, &bread, NULL);
//
//	if( ret == FALSE || ret==-1 ) {
//		len = -1;
//	} else {
//		len = bread;
//	}

#else
    int ret = 0;

	fd_set readfs;
	int    maxfd;     /* maximum file desciptor used */
	maxfd = fd + 1;  /* maximum bit entry (fd) to test */

    struct timeval Timeout;

    /* set timeout value within input loop */
    Timeout.tv_usec = 0;  /* milliseconds */
    Timeout.tv_sec  = timeout;  /* seconds */

    FD_ZERO(&readfs);
    FD_SET(fd, &readfs);  /* set testing for source 1 */

    ret = select(maxfd, &readfs, NULL, NULL, &Timeout);

    if ((ret > 0) && FD_ISSET(fd, &readfs))
    {
        size_t read_len = 0;
        ioctl(fd, FIONREAD, &read_len);
       // errsv = errno;
       //  printf("prog_name: zero read from the device: %s.", strerror(errsv));


        if (read_len == 0)
        {
            len = -2; // error during read, maybe serial port was disconnected
        }
        else
        {
            ret = read(fd, buf, max_size);
            len = ret;
        }
    }
    else if (ret == 0)
    {
        // Timeout
        len = 0;
    }
    else
    {
        len = -1;
    }

#endif

	return len;
}

int serial_open(const char *port)
{
	int fd = -1;
#ifdef USE_WINDOWS_OS
	static char full_path[32] = {0};

	HANDLE hCom = NULL;

	if( port[0] != '\\' ) {
        _snprintf(full_path, sizeof(full_path) - 1, "\\\\.\\%s", port);
		port = full_path;
	}

    hCom = CreateFileA(port, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, NULL );

	if( !hCom || hCom == INVALID_HANDLE_VALUE ) {
        printf("serial error %d\r\n", GetLastError());
		fd = -1;
	} else {
		fd = (int)hCom;
    }


#else
    fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1) {
		//fprintf(stderr, "Could not open serial port.\n");
		return -1;
	}

	/* Make the file descriptor asynchronous (the manual page says only        |
	|           O_APPEND and O_NONBLOCK, will work with F_SETFL...) */
	fcntl(fd, F_SETFL, O_ASYNC);
#endif
	return fd;
}

int serial_close(int fd)
{
#ifdef USE_WINDOWS_OS
	HANDLE hCom = (HANDLE)fd;

	CloseHandle(hCom);
#else
	close(fd);
#endif
    return -1;
}
