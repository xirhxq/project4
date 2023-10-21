#include "uart.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <strings.h>
#include <pthread.h>
#include <semaphore.h>
#include <string.h>
#include <assert.h>
#include <semaphore.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>

static signal_handler_t sci_recv_signal_hander;

Uart::Uart(const char *l_path, int p_baudrate, int p_datalength, int p_stopbit, int p_parity, signal_handler_t hander) {
    if (strlen(l_path) <= 1 || strlen(l_path) >= MAX_PATH_LENGTH) {
        strcpy(path, "/dev/ttyTHS1");
    }
    strcpy(path, l_path);
    baudrate = p_baudrate;
    datalength = p_datalength;
    stopbit = p_stopbit;
    parity = p_parity;

    name = path;
    if (hander != NULL)
        sci_recv_signal_hander = hander;
    else {
        sci_recv_signal_hander = NULL;
    }
    isopen = 0;
}
///******************************************
// �źŴ��������豸wait_flag=FASLE
// ******************************************************/
//static void signal_handler_IO(int status)
//{
//   sci_recv_signal_hander(status);
//   printf("received SIGIO signale.\n");
//}
/*-------------------------------------------------------------------
 * function	:	uart init function
 * parameter	:	fd,bouadrate,datalength,stopbit,parity
 * return	:	o:success -1:failed
 * ***************************************************************/
int Uart::set_uart(void) {
    struct termios newtio, oldtio;

    if (tcgetattr(fd, &oldtio) != 0) {
        perror("tcgetattr err...\r\n");
        return -1;
    }
    memset(&newtio, 0, sizeof(newtio));

    //
    newtio.c_cflag |= CLOCAL;
    newtio.c_cflag |= CREAD;
    newtio.c_cflag &= ~CSIZE;

    switch (datalength) {
        case 7:
            newtio.c_cflag |= CS7;
            break;
        case 8:
            newtio.c_cflag |= CS8;
            break;
        default:
            newtio.c_cflag |= CS8;
            break;
    }

    //------------------------------------------------------------------------
    switch (parity) {
        case 0:
            newtio.c_cflag &= ~PARENB;// Clear parity enable
            newtio.c_iflag &= ~INPCK;// Disable parity check
            break;
        case 1:
            newtio.c_cflag |= PARENB;    //parity enable bit
            newtio.c_cflag |= PARODD;     //odd bit
            newtio.c_iflag |= INPCK;    //enable function
            break;
        case 2:
            newtio.c_cflag |= PARENB; //Enable parity
            newtio.c_cflag &= ~PARODD; //
            newtio.c_iflag |= INPCK;    //enable function
            break;
        default:
            newtio.c_cflag &= ~PARENB;    //
            newtio.c_iflag &= ~INPCK;    //
            break;

    }

    //------------------------------------------------------
    switch (baudrate) {
        case 0:
            cfsetispeed(&newtio, B0);
            cfsetospeed(&newtio, B0);
            break;
        case 2400:
            cfsetispeed(&newtio, B2400);
            cfsetospeed(&newtio, B2400);
            break;
        case 4800:
            cfsetispeed(&newtio, B4800);
            cfsetospeed(&newtio, B4800);
            break;
        case 9600:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
        case 19200:
            cfsetispeed(&newtio, B19200);
            cfsetospeed(&newtio, B19200);
            break;
        case 38400:
            cfsetispeed(&newtio, B38400);
            cfsetospeed(&newtio, B38400);
            break;
        case 57600:
            cfsetispeed(&newtio, B57600);
            cfsetospeed(&newtio, B57600);
            break;
        case 115200:
            cfsetispeed(&newtio, B115200);
            cfsetospeed(&newtio, B115200);
            break;
        case 230400:
            cfsetispeed(&newtio, B230400);
            cfsetospeed(&newtio, B230400);
            break;
        default:
            cfsetispeed(&newtio, B115200);
            cfsetospeed(&newtio, B115200);
            break;

    }

    //--------------------------------------------------------------------------------
    switch (stopbit) {
        case 1:
            newtio.c_cflag &= ~CSTOPB;
            break;
        case 2:
            newtio.c_cflag |= CSTOPB;
            break;
        default:
            newtio.c_cflag &= ~CSTOPB;
            break;
    }

    //---------------------------------------------------------------------------
    newtio.c_cc[VTIME] = 150; //set timeout
    newtio.c_cc[VMIN] = 0; /* Update the options and do it NOW */
    tcflush(fd, TCIFLUSH); //flush data
    //set uart
    if (tcsetattr(fd, TCSANOW, &newtio) != 0) {
        perror("Setup Serial Failed!\n");
        return -1;
    }
    return 0;
}

/*-----------------------------------------
 * function     :	int Uart::close_uart_dev()
 * parameter	:	the device path
 * return       :	file decription
 * ***************************************/
int Uart::close_uart_dev() {
    close(fd);
    return 0;
}

/*-----------------------------------------
 * function     :	static int open_uart_dev(const char *path,uart_device_t *uart_dev_p)
 * parameter	:	the device path
 * return       :	file decription
 * ***************************************/
int Uart::open_uart_dev() {
    fd = open(path, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        printf("open serial failed! \n");
        return -1;
    } else {
#if defined(UART_DEBUG)
        printf("device open secced...\r\n");
#endif
        return fd;
    }
}

/*-----------------------------------------
 * function	:	open sci device
 * parameter	:	the device path
 * return 	:	file decription
 * ***************************************/
int Uart::saio_init() {
    /* install the signal handle before making the device asynchronous*/
    saio.sa_handler = sci_recv_signal_hander;
    //uart_dev_p->saio.sa_mask = 0;������sigemptyset������ʼ��act�ṹ��sa_mask��Ա
    sigemptyset(&saio.sa_mask);
    saio.sa_flags = 0;
    saio.sa_restorer = NULL;
    sigaction(SIGIO, &saio, NULL);
    /* allow the process to recevie SIGIO*/
    fcntl(fd, F_SETOWN, getpid());
    /* Make the file descriptor asynchronous*/
    fcntl(fd, F_SETFL, FASYNC);//0Ϊ���� FNDELAY������

    return 0;
}

/*-----------------------------------------
 * function     :	int Uart::open(char *path)
 * parameter	:	the device path
 * return       :	file decription
 * ***************************************/
int Uart::open_sci() {
    if (isopen == 1)
        return -1;
    //�򿪴�����ز���
    if (open_uart_dev() < 0)
        return -1;
    if (sci_recv_signal_hander != NULL)
        saio_init();
    set_uart();
    isopen = 1;
    return 0;
}

/*-----------------------------------------
 * function     :	int Uart::open(char *path)
 * parameter	:	the device path
 * return       :	file decription
 * ***************************************/
int Uart::close_sci(void) {
    if (isopen == 0)
        return -1;
    close_uart_dev();
    return 0;
}

int Uart::child_signal(class Object *) {
    return 0;
}

/*-----------------------------------------
 * function     :	int Uart::read_sci(unsigned char *buf,unsigned int size)
 * parameter	:	the device path
 * return       :	file decription
 * ***************************************/
int Uart::read_data(unsigned char *buf, unsigned int size) {
    int ret = 0;
    if (buf == NULL || isopen == 0)
        return 0;
    ret = read(fd, buf, size);
    if (ret == -1) {
        perror("serial send failed\n");
        return 0;
    } else {
        return ret;
    }
    return 0;
}

/*-----------------------------------------
 * function     :	int Uart::write_sci(unsigned char *buf,unsigned int size)
 * parameter	:	the device path
 * return       :	file decription
 * ***************************************/
int Uart::write_data(const unsigned char *buf, unsigned int size) {
    int ret = 0;
    int total = 0;
    if (buf == NULL || isopen == 0)
        return 0;

    while (total != size) {
        if ((ret = write(fd, buf + total, size - total)) == -1) {
            perror("serial send failed\n");
            total = -1;
            break;
        } else
            total += ret;
    }
    return total;
}


//int Uart::register_recv_handler(signal_handler_t signal_handler)
//{
//    if(signal_handler!=NULL||isopen==0)
//    {
//        sci_recv_signal_hander=signal_handler;
//        return 0;
//    }
//    return 1;


//}
