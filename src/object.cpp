#include "object.h"
#include "string.h"

Buff::Buff()
{
    buff_length=0;
}
unsigned char *Buff::buff()
{
    return this->buff_temp;
}
int Buff::length()
{
    return buff_length;
}
int Buff::read_data(unsigned char *buf,unsigned int size)
{
    if(size>500)
        return 1;
    memcpy(buf,this->buff_temp,size);
    return 0;
}
int Buff::write_data(const unsigned char *buf,unsigned int size)
{
    if(size>500)
        return 1;
    memcpy(this->buff_temp,buf,size);
    buff_length=size;
    return 0;
}

int Buff::child_signal(class Object *)
{
    return 0;
}

Object::Object()
{

}

int Object::child_signal(class Object *)
{
    printf("Object::child_signal...\r\n");
    return 0;
}

int Object::read_data(unsigned char *buf,unsigned int size)
{
    size=0;
    printf("read_data vitual...\r\n");
    return 0;
}
int Object::write_data(const unsigned char *buf,unsigned int size)
{
    size=0;
    printf("write_data vitual...\r\n");
    return 0;
}
