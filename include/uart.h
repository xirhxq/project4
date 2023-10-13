#ifndef UART_H
#define UART_H

#include "object.h"
#include <string.h>
#include <signal.h>

#define MAX_PATH_LENGTH 300

class Uart:public Object
{
public:
    /*
    *
    *
    */
    Uart(const char *l_path,int p_baudrate,int p_datalength,int p_stopbit,int p_parity,signal_handler_t hander);

    int open_sci(void);
    int close_sci(void);
    int read_data(unsigned char *buf,unsigned int size) override;
    int write_data(const unsigned char *buf,unsigned int size) override;
    int child_signal(class Object *)override;
    //int register_recv_handler(signal_handler_t signal_handler);
public:

private:
    int saio_init();
    /*-----------------------------------------
     * function	:	static int open_uart_dev(const char *path,uart_device_t *uart_dev_p)
     * parameter	:	the device path
     * return 	:	file decription
     * ***************************************/
    int open_uart_dev();
    /*-----------------------------------------
     * function     :	int close_uart_dev();
     * parameter	:	the device path
     * return       :	file decription
     * ***************************************/
    int close_uart_dev();
    /*-------------------------------------------------------------------
     * function	:	uart init function
     * parameter	:	fd,bouadrate,datalength,stopbit,parity
     * return	:	o:success -1:failed
     * ***************************************************************/
    int set_uart();

private:
    int fd;
    char path[MAX_PATH_LENGTH];
    int baudrate;
    int datalength;
    int stopbit;
    int parity;
    struct sigaction 				saio;

    int                             isopen;
};

#endif // UART_H
