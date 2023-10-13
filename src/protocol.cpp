#include "protocol.h"
#include "string.h"

#define printf

Protocol::Protocol(Object *obj)
{
    header[0]=0xEB;
    header[1]=0x90;


    data_length_pos=4;
    rx_num=0;
    frame_length=6;
    check_num=0;
    tx_count=0;
    parent=obj;

}

unsigned char Protocol::frame_check()
{
    unsigned char check=0;
    if(check_num==0)
    {
        //用户自校验 暂时返回成功
        return 0;
    }
    for (int j=0;j<data_length-1;j++) {
        check+=frame_buff[j];
        printf("check:0x%x\r\n",check);
    }
    if(check==frame_buff[data_length-1])
    {
        return 0;
    }
    else {
        printf("data_length+frame_length-1:%d \tframe_buff[data_length+frame_length-1]:0x%x\r\n",data_length-1,frame_buff[data_length-1]);
        return 1;
    }
}
int Protocol::child_signal(class Object *)
{
    return 0;
}

int Protocol::read_data(unsigned char *buf,unsigned int size)
{
    buf[0]=header[0];
    buf[1]=header[1];
    buf[2]=send_frame_id;
    buf[3]=tx_count;
    tx_count++;


    buf[4]=(size)&0xff;
    unsigned char check=0;
    for (unsigned int i=0;i<size-1;i++) {
        check   +=buf[i];
    }

    buf[size-1] =   check;

    return 0;
}
int Protocol::write_data(const unsigned char *buf,unsigned int size)
{
    //累加校验
    unsigned int  i=0;
    unsigned char flag=0;
    for (i= 0; i<size; i++)
    {
        printf("i:0x%x\t buf:0x%x\t rx_num:0%x\r\n",i,buf[i],rx_num);
        if(rx_num>=2)
        {
            frame_buff[rx_num]=buf[i];
            if(data_length_pos!=0&&data_length_pos==rx_num)
            {
                //找帧长度
                data_length=frame_buff[rx_num];
            }
            else if(data_length_pos==0){
                data_length=0;
            }
            if(rx_num>=(data_length-1))
            {
                printf("rx_num 0x%x \t frame_length+data_length: 0x%x\r\n",rx_num,data_length);
                //接收到了整包数据，开始校验
                if(frame_check()==0)
                {
                    rx_num=0;
                    parsedbuf.write_data(frame_buff,data_length);
                    //调用解析成功函数
                    if(parent!=NULL)
                    {
                        parent->child_signal(this);
                    }
                    else {
                        printf("jiexi ok...\r\n");
                    }
                    continue;
                }
                else {
                    //去除第一个字节继续 递归
                    rx_num=0;
                    printf("check err\r\n");
                    printf("digui qian i:%d\r\n",i);
                    write_data(&frame_buff[1],data_length-1);
                    printf("digui return rx_num:%d\r\n",rx_num);
                    printf("digui hou i:%d\r\n",i);
                    continue;
                }
            }
            rx_num++;
            continue;
        }
        data_length = 500;
        /*判断第一个枕头*/
        if(buf[i]==header[0]&&header_flag[0]==0)
        {
            header_flag[0]=1;
            printf("header1 0x%x\r\n",buf[i]);
        }

        /*判断第二个枕头*/
        if(header_flag[0]==1&&buf[i]==header[1])
        {
            header_flag[0]=0;
            header_flag[1]=0;
            frame_buff[0]=header[0];
            frame_buff[1]=header[1];
            flag=0;
            printf("header2 0x%x\r\n",buf[i]);
            rx_num=2;
            continue;
        }
        if(header_flag[0])
        {
            if(flag++>=1)
            {
                header_flag[0]=0;
                flag=0;
                /*判断第一个枕头*/
                if(buf[i]==header[0]&&header_flag[0]==0)
                {
                    header_flag[0]=1;
                    printf("header1 0x%x\r\n",buf[i]);
                }
                if(header_flag[0])
                {
                    flag++;
                }
            }
        }

    }
    return 0;
}
