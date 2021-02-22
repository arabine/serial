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

int serial_setup(SerialHandle fd, unsigned long speed)
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
    CommTimeOuts.ReadIntervalTimeout = MAXDWORD;
    CommTimeOuts.ReadTotalTimeoutMultiplier = 0;
    CommTimeOuts.ReadTotalTimeoutConstant = 0;
    CommTimeOuts.WriteTotalTimeoutMultiplier = 0;
    CommTimeOuts.WriteTotalTimeoutConstant = 0;
    SetCommTimeouts( fd, &CommTimeOuts );

    DCB dcb;
    dcb.DCBlength = sizeof( DCB );
    GetCommState( fd, &dcb );
    dcb.BaudRate = speed;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits          = ONESTOPBIT;   // 0,1,2 = 1, 1.5, 2

    dcb.fBinary           = TRUE;         // Binary mode; no EOF check
    dcb.fParity           = FALSE;        // Enable parity checking
    dcb.fOutxCtsFlow      = FALSE;        // No CTS output flow control
    dcb.fOutxDsrFlow      = FALSE;        // No DSR output flow control
    dcb.fDtrControl       = DTR_CONTROL_DISABLE; // DTR flow control type
    dcb.fDsrSensitivity   = FALSE;        // DSR sensitivity
    dcb.fTXContinueOnXoff = FALSE;        // XOFF continues Tx
    dcb.fOutX             = FALSE;        // No XON/XOFF out flow control
    dcb.fInX              = FALSE;        // No XON/XOFF in flow control
    dcb.fErrorChar        = FALSE;        // Disable error replacement
    dcb.fNull             = FALSE;        // Disable null stripping
    dcb.fRtsControl       = RTS_CONTROL_DISABLE; // RTS flow control
    dcb.fAbortOnError     = FALSE;        // Do not abort reads/writes on error
    dcb.EvtChar           = 0x7E;         // Flag

    if( !SetCommState( fd, &dcb ) ||
        !SetupComm( fd, 10000, 10000 ))
    {
        (void) GetLastError();
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



int sendpckt(HANDLE hComm, unsigned length, const char * pckt)
{
    BOOL result;
    DWORD dwCommEvent;
    OVERLAPPED oWrite = { 0 };
    DWORD errCode;

    result = SetCommMask(hComm, EV_TXEMPTY);
    if (!result) printf("Err: %d\n", (int)GetLastError());

    result = WriteFile(hComm, pckt, length, NULL, &oWrite);
    if (result == FALSE)
    {
        errCode = GetLastError();

        if (errCode != ERROR_IO_PENDING)
            printf("Error! Setting CommMask\n");
    }

    OVERLAPPED commOverlapped = { 0 };

    commOverlapped.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    if (commOverlapped.hEvent == NULL)
        return FALSE; // Error creating overlapped event handle.

    result = WaitCommEvent(hComm, &dwCommEvent, &commOverlapped);
    if (!dwCommEvent)
        printf("Error Setting WaitCommEvent()%d\n", (int) GetLastError());
    else
    {
        // If WaitCommEvent() == TRUE then Read the received data
        if (dwCommEvent & EV_TXEMPTY)
        {
            printf("Send complete.\n");
        }

    }

    CloseHandle(oWrite.hEvent);
    CloseHandle(commOverlapped.hEvent);

    return 0;
};
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
int recvpckt(HANDLE hComm, char * pckt)
{
    BOOL result;
    int len = 0;
    OVERLAPPED oRead = { 0 };
    DWORD errCode;
    DWORD dwCommEvent;

    result = SetCommMask(hComm, EV_RXCHAR);
    if (!result) printf("Err: %d\n", (int)GetLastError());

    result = ReadFile(hComm, pckt, 2048, NULL, &oRead);
    if (result == FALSE)
    {
        errCode = GetLastError();

        if (errCode != ERROR_IO_PENDING)
            printf("nError! Setting CommMask\n");
    }

    OVERLAPPED commOverlapped = { 0 };

    commOverlapped.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    if (commOverlapped.hEvent == NULL)
        return FALSE; // Error creating overlapped event handle.

    result = WaitCommEvent(hComm, &dwCommEvent, &commOverlapped);
    if (!dwCommEvent)
        printf("Error Setting WaitCommEvent()%d\n", (int)GetLastError());
    else
    {
        if (dwCommEvent & EV_TXEMPTY)
        {
            printf("Chars Recveived\n");
            len = oRead.InternalHigh;
        }

    }

    CloseHandle(oRead.hEvent);
    CloseHandle(commOverlapped.hEvent);

    return len;
};


int serial_write(SerialHandle fd, const char *buf, int size)
{
	int ret = -1;
#ifdef USE_WINDOWS_OS


    return sendpckt(fd, size, buf);
    /*
	unsigned long bwritten = 0;

    OVERLAPPED osWrite = {0};
    DWORD dwRes;

    // Create this write operation's OVERLAPPED structure's hEvent.
    osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    if (osWrite.hEvent != NULL)
    {
        // Issue write.
        if (!WriteFile(fd, buf, size, &bwritten, &osWrite))
        {
            if (GetLastError() == ERROR_IO_PENDING)
            {
                 // Write is pending.
                 dwRes = WaitForSingleObject(osWrite.hEvent, INFINITE);
                 switch(dwRes)
                 {
                    // OVERLAPPED structure's event has been signaled.
                    case WAIT_OBJECT_0:
                         if (GetOverlappedResult(fd, &osWrite, &bwritten, FALSE))
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
            */

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
int serial_read(SerialHandle fd, char *buf, int max_size, int timeout)
{
	int len = 0;

#ifdef USE_WINDOWS_OS
    DWORD dwRead;
    BOOL fWaitingOnRead = FALSE;
    OVERLAPPED osReader = {0};


    COMMTIMEOUTS commTimeout;

    if(GetCommTimeouts(fd, &commTimeout))
    {
        commTimeout.ReadIntervalTimeout     = 1000 * timeout;
        commTimeout.ReadTotalTimeoutConstant     = 50;
        commTimeout.ReadTotalTimeoutMultiplier     = 50;
    }
    else {
        //Handle Error Condition
    }
    (void) SetCommTimeouts(fd, &commTimeout);

    // Create the overlapped event. Must be closed before exiting
    // to avoid a handle leak.
    osReader.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

    if (osReader.hEvent == NULL) {
       // Error creating overlapped event; abort.
        return 0;
    }

    if (!fWaitingOnRead) {
       // Issue read operation.
       if (!ReadFile(fd, buf, max_size, &dwRead, &osReader)) {
          if (GetLastError() != ERROR_IO_PENDING) {    // read not delayed?
             // Error in communications; report it.
              len = 0;
          } else
             fWaitingOnRead = TRUE;
       }
       else {
          // read completed immediately
          //HandleASuccessfulRead(lpBuf, dwRead);
           len = dwRead;
           fWaitingOnRead = TRUE;
        }
    }

    DWORD dwRes;

    if (fWaitingOnRead) {
       dwRes = WaitForSingleObject(osReader.hEvent, timeout * 1000);
       switch(dwRes)
       {
          // Read completed.
          case WAIT_OBJECT_0:
              if (!GetOverlappedResult(fd, &osReader, &dwRead, FALSE)) {
                  len = 0;
                 // Error in communications; report it.
              } else {
                 // Read completed successfully.
                  len = dwRead;
                // HandleASuccessfulRead(lpBuf, dwRead);
              }
              //  Reset flag so that another opertion can be issued.
              fWaitingOnRead = FALSE;
              break;

          case WAIT_TIMEOUT:
              // Operation isn't complete yet. fWaitingOnRead flag isn't
              // changed since I'll loop back around, and I don't want
              // to issue another read until the first one finishes.
              //
              // This is a good time to do some background work.
                len = 0;
              break;

          default:
              // Error in the WaitForSingleObject; abort.
              // This indicates a problem with the OVERLAPPED structure's
              // event handle.
              break;
       }
    }

    CloseHandle(osReader.hEvent);

#else
    int ret = 0;

	fd_set readfs;
	int    maxfd;     /* maximum file desciptor used */
	maxfd = fd + 1;  /* maximum bit entry (fd) to test */

    struct timeval Timeout;

    /* set timeout value within input loop */
    Timeout.tv_usec = timeout * 1000;  /* milliseconds */
    Timeout.tv_sec  = 0;  /* seconds */

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

SerialHandle serial_open(const char *port)
{
    SerialHandle fd = INVALID_HANDLE_VALUE;
#ifdef USE_WINDOWS_OS
	static char full_path[32] = {0};

	if( port[0] != '\\' ) {
        _snprintf(full_path, sizeof(full_path) - 1, "\\\\.\\%s", port);
		port = full_path;
	}

    fd = CreateFileA(port, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, NULL );

    if( !fd || fd == INVALID_HANDLE_VALUE ) {
        printf("serial error %d\r\n", (int)GetLastError());
        fd = INVALID_HANDLE_VALUE;
    } else {
        if(!SetCommMask(fd, EV_RXCHAR)){ // set a mask for incoming characters event.
            fd = INVALID_HANDLE_VALUE;
        }
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

SerialHandle serial_close(SerialHandle fd)
{
#ifdef USE_WINDOWS_OS

    CloseHandle(fd);
#else
	close(fd);
#endif
    return INVALID_HANDLE_VALUE;
}
