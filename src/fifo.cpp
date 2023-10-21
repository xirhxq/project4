#include "fifo.h"

Fifo::Fifo() {
    // memset(&fifo_data,0,sizeof(fifo_data));
}

int Fifo::child_signal(class Object *) {
    return 0;
}

int Fifo::length() {
    fifo_data.legnth = fifo_data.head - fifo_data.tail;
    return fifo_data.legnth;
}

int Fifo::read_data(unsigned char *msg, unsigned int length) {
    unsigned int recNum;
    fifo_data.legnth = fifo_data.head - fifo_data.tail;//usb_buff.rec_Length_USB;
    recNum = fifo_data.legnth < length ? fifo_data.legnth : length;

    for (int i = 0; i < recNum; i++) {
        msg[i] = fifo_data.buff[(fifo_data.tail)];
        fifo_data.tail++;
        fifo_data.read_length++;
    }
    return (int) recNum;
}

int Fifo::write_data(const unsigned char *msg, unsigned int length) {
    int write_length = 0;
    for (int i = 0; i < length; i++) {
        fifo_data.buff[fifo_data.head] = msg[i];
        if (fifo_data.head + 1 != fifo_data.tail) {
            fifo_data.head++;
            write_length++;
            fifo_data.write_length++;
        } else {
            fifo_data.full_err++;
            //���������л�
            return write_length;
        }
    }
    //���������л�
    return write_length;
}
