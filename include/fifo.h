#ifndef FIFO_H
#define FIFO_H

#include "object.h"

typedef struct
{
    unsigned int head:10;
    unsigned int tail:10;
    unsigned int legnth:10;
    unsigned int  full_err;
    unsigned int write_length;
    unsigned int read_length;
    unsigned char  buff[1024+10];
}fifo_1024_t;

class Fifo:public Object
{
public:
    Fifo();

    int read_data(unsigned char *buf,unsigned int size) override;
    int write_data(const unsigned char *buf,unsigned int size) override;
    int length();
    int child_signal(class Object *)override;

private:
    fifo_1024_t fifo_data;
};

#endif // FIFO_H
