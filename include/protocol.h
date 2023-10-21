#ifndef PROTOCOL_H
#define PROTOCOL_H

#include "object.h"

typedef void (*protocol_parsed_ok_func_t)(unsigned char *buff, int length);


class Protocol : public Object {
public:
    Protocol(Object *obj);

    Object *parent;
    unsigned char header[5];
    unsigned char send_frame_id;
    unsigned int frame_length;
    unsigned int check_num;
    int data_length_pos;
    unsigned char tx_count;//record frame transfer
    Buff parsedbuf;

    int read_data(unsigned char *buf, unsigned int size) override;

    int write_data(const unsigned char *buf, unsigned int size) override;

    int child_signal(class Object *) override;

private:
    unsigned int data_length;
    int rx_num;

    unsigned char header_flag[5];

    unsigned char frame_id;
    unsigned char *data_p;
    unsigned char frame_buff[1024];


    unsigned char frame_check();

};

#endif // PROTOCOL_H
