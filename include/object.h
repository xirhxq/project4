#ifndef OBJECT_H
#define OBJECT_H

#include "stdio.h"

typedef void (*signal_handler_t)(int status);


class Object {
public:
    Object();

    virtual int read_data(unsigned char *buf, unsigned int size);

    virtual int write_data(const unsigned char *buf, unsigned int size);

    virtual int child_signal(class Object *);

    char *name;
private:

protected:

};

class Buff : public Object {
public:
    Buff();

    int length();

    int read_data(unsigned char *buf, unsigned int size) override;

    int write_data(const unsigned char *buf, unsigned int size) override;

    unsigned char *buff();

    int child_signal(class Object *) override;

private:
    unsigned char buff_temp[500];
    int buff_length;
};

#endif // OBJECT_H
