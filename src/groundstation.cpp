#include <iostream>
#include "groundstation.h"
#include "string.h"
#include <stdio.h>
#include <stdlib.h>


#define printf
using namespace std;
//control_infor_out_t control_infor_out;
XY_DATA xy_data;

GroundStation::GroundStation()
{
    gs_tx2_protocl_up = new Protocol(this);
    gs_pod_protocl = new Protocol(this);
    gs_tx2_protocl_douwn = new Protocol(this);
    newctrl_to_xy_protocl = new Protocol(this);

    gs_tx2_protocl_douwn->header[0]=0x5A;
    gs_tx2_protocl_douwn->header[1]=0xFB;
    gs_tx2_protocl_douwn->data_length_pos=4;
    gs_tx2_protocl_douwn->frame_length=6;
    gs_tx2_protocl_douwn->check_num=1;




     gs_tx2_protocl_up->header[0]=0xEA;
     gs_tx2_protocl_up->header[1]=0x9B;
     gs_tx2_protocl_up->data_length_pos=4;
     gs_tx2_protocl_up->frame_length=6;
     gs_tx2_protocl_up->check_num=1;
     gs_tx2_protocl_douwn->send_frame_id=USR_MODE_CTR_CMD;

     /* add by wg
      * data:2021-07-12
      * add new protocol*/
     newctrl_to_xy_protocl->header[0]=0x5A;
     newctrl_to_xy_protocl->header[1]=0xFB;
     newctrl_to_xy_protocl->data_length_pos=4;
     newctrl_to_xy_protocl->frame_length=6;
     newctrl_to_xy_protocl->check_num=1;
     newctrl_to_xy_protocl->send_frame_id=NEW_CTR_CMD;
    //==============================================================

    // gs_pod_protocl->header[0]=0x55;
    // gs_pod_protocl->header[1]=0xaa;
    // gs_pod_protocl->data_length_pos=0;
    // gs_pod_protocl->frame_length=20;

    memset(&fifo,0,sizeof(fifo));
    
}

int GroundStation::child_signal(class Object * child)
{
    //printf("child signal\r\n");
    if(child==  gs_tx2_protocl_douwn)
    {
        //printf("gs_tx2_protocl_douwn signal\r\n");
    }
    else if(child==gs_tx2_protocl_up){
       printf("--------------------------buff:\n");
       for (int i = 0; i < 10; i++) {
              printf("%x ", gs_tx2_protocl_up->parsedbuf.buff()[i]);
       }
       printf("\n");
        if(gs_tx2_protocl_up->parsedbuf.buff()[2]==FLIIGHT_PARAM_CMD )
        {
           control_infor_buff.write_data(gs_tx2_protocl_up->parsedbuf.buff(),gs_tx2_protocl_up->parsedbuf.length());
           //����
           int pos = 5;
           memcpy(&control_infor_in.cur_horizonal_control_mode,&(control_infor_buff.buff()[pos])
                  ,1);xy_data.horizonal_mode = int(control_infor_in.cur_horizonal_control_mode);
           pos  +=  1;
           memcpy(&control_infor_in.cur_vertical_control_mode,&(control_infor_buff.buff()[pos])
                  ,1);xy_data.vertical_mode = int(control_infor_in.cur_vertical_control_mode);
           pos  +=  1;
           memcpy(&control_infor_in.cur_heading_control_mode,&(control_infor_buff.buff()[pos])
                  ,1);xy_data.yaw_mode = int(control_infor_in.cur_heading_control_mode);
           pos  +=  1;
           memcpy(&control_infor_in.lon,&(control_infor_buff.buff()[pos])
                  ,4);
                  xy_data.UAV_lon = double(control_infor_in.lon)/1000000.0 ;
           pos  +=  4;
           memcpy(&control_infor_in.lat,&(control_infor_buff.buff()[pos])
                  ,4);
                  xy_data.UAV_lat = double(control_infor_in.lat)/1000000.0 ;
           pos  +=  4;
           memcpy(&control_infor_in.alt_satelite,&(control_infor_buff.buff()[pos]),4); 
           
           
           xy_data.UAV_height = float(control_infor_in.alt_satelite)/100.0 ;
           
           pos  +=  4;
           memcpy(&control_infor_in.barometer_satelite,&(control_infor_buff.buff()[pos])
                  ,4);
                  
                  
                  xy_data.baro_height = float(control_infor_in.barometer_satelite)/100.0 ;
           pos  +=  4;
           memcpy(&control_infor_in.ve,&(control_infor_buff.buff()[pos])
                  ,2);xy_data.UAV_Vel[0] = float(control_infor_in.ve)/10.0 ;
           pos  +=  2;
           memcpy(&control_infor_in.vn,&(control_infor_buff.buff()[pos])
                  ,2);xy_data.UAV_Vel[1] = float(control_infor_in.vn)/10.0 ;
           pos  +=  2;
           memcpy(&control_infor_in.vu,&(control_infor_buff.buff()[pos])
                  ,2);xy_data.UAV_Vel[2] = float(control_infor_in.vu)/10.0 ;
           pos  +=  2;
           memcpy(&control_infor_in.position_x_cur,&(control_infor_buff.buff()[pos])
                  ,4);xy_data.UAV_pos[0] = float(control_infor_in.position_x_cur) ;
           pos  +=  4;
           memcpy(&control_infor_in.position_y_cur,&(control_infor_buff.buff()[pos])
                  ,4);xy_data.UAV_pos[1] = float(control_infor_in.position_y_cur) ;
           pos  +=  4;
           memcpy(&control_infor_in.position_z_cur,&(control_infor_buff.buff()[pos])
                  ,4);
                 // printf("1111111111111111---------------:%f",control_infor_in.position_z_cur);
                  
                  xy_data.UAV_pos[2] = float(control_infor_in.position_z_cur) ;
           pos  +=  4;
           memcpy(&control_infor_in.yaw,&(control_infor_buff.buff()[pos])
                  ,2);xy_data.UAV_Euler[2] = float(control_infor_in.yaw)/1000.0;
           pos  +=  2;           
           memcpy(&control_infor_in.pitch,&(control_infor_buff.buff()[pos])
                  ,2);  xy_data.UAV_Euler[1] = float(control_infor_in.pitch)/1000.0;
           pos  +=  2;

           memcpy(&control_infor_in.roll,&(control_infor_buff.buff()[pos])
                  ,2);xy_data.UAV_Euler[0] = float(control_infor_in.roll)/1000.0;
           pos  +=  2;
           memcpy(&control_infor_in.throttle_percentage,&(control_infor_buff.buff()[pos])
                  ,1); xy_data.cur_throttle = float(control_infor_in.throttle_percentage);
           pos  +=  1;
           memcpy(&control_infor_in.ax,&(control_infor_buff.buff()[pos])
                  ,2);xy_data.UAV_acc[0] = float(control_infor_in.ax)/100.0;
           pos  +=  2;
           memcpy(&control_infor_in.ay,&(control_infor_buff.buff()[pos])
                  ,2);xy_data.UAV_acc[1] = float(control_infor_in.ay)/100.0;
           pos  +=  2;
           memcpy(&control_infor_in.az,&(control_infor_buff.buff()[pos])
                  ,2);xy_data.UAV_acc[2] = float(control_infor_in.az)/100.0;
           pos  +=  2;
           memcpy(&control_infor_in.gx,&(control_infor_buff.buff()[pos])
                  ,2);xy_data.UAV_gyro[0] = float(control_infor_in.gx)/100.0;
           pos  +=  2;
           memcpy(&control_infor_in.gy,&(control_infor_buff.buff()[pos])
                  ,2);xy_data.UAV_gyro[1] = float(control_infor_in.gy)/100.0;
           pos  +=  2;
           memcpy(&control_infor_in.gz,&(control_infor_buff.buff()[pos])
                  ,2);xy_data.UAV_gyro[2] = float(control_infor_in.gz)/100.0;
           pos  +=  2;
           memcpy(&control_infor_in.uav_status,&(control_infor_buff.buff()[pos])
                  ,1);xy_data.UAV_status = int(control_infor_in.uav_status);
          pos  +=  1;

           gs_state|=GS_RECV_CONTROL_MODE;
           printf("copy succed...\r\n");
        }
        //printf("gs_tx2_protocl_up signal\r\n");
       
    }
    else if(child==gs_pod_protocl)
    {
    }
    return 0;
}

int GroundStation::read_data(unsigned char *buf,unsigned int size)
{
    return fifo.read_data(buf,size);
}
int GroundStation::write_data(const unsigned char *buf,unsigned int size)
{
    return fifo.write_data(buf,size);
}

int GroundStation::pack_process()
{
    int ret = 0;

    unsigned int pos = 5;

    unsigned char buff[500];

//    control_infor_out.horizonal_control_mode=1;
    memcpy(&(buff[pos])
          ,&control_infor_out.horizonal_control_mode,
           1);
    pos  +=  1;
//    control_infor_out.vertical_control_mode=2;
    memcpy(&(buff[pos])
          ,&control_infor_out.vertical_control_mode,
           1);
    pos  +=  1;
//    control_infor_out.heading_control_mode=3;
    memcpy(&(buff[pos])
          ,&control_infor_out.heading_control_mode,
           1);
    pos  +=  1;
//    control_infor_out.position_x_ctr=4;
    memcpy(&(buff[pos])
          ,&control_infor_out.position_x_ctr,
           4);
    pos  +=  4;
//    control_infor_out.position_y_ctr=4;
    memcpy(&(buff[pos])
          ,&control_infor_out.position_y_ctr,
           4);
    pos  +=  4;
//    control_infor_out.position_z_ctr=4;
    memcpy(&(buff[pos])
          ,&control_infor_out.position_z_ctr,
           4);
    pos  +=  4;
//    control_infor_out.ve_ctr=5;
    memcpy(&(buff[pos])
          ,&control_infor_out.ve_ctr,
           2);
    pos  +=  2;
//    control_infor_out.vn_ctr=6;
    memcpy(&(buff[pos])
          ,&control_infor_out.vn_ctr,
           2);
    pos  +=  2;
//    control_infor_out.vu_ctr=7;
    memcpy(&(buff[pos])
          ,&control_infor_out.vu_ctr,
           2);
    pos  +=  2;
//    control_infor_out.yaw_ctr=8;
    memcpy(&(buff[pos])
          ,&control_infor_out.yaw_ctr,
           2);
    pos  +=  2;
//    control_infor_out.pitch_ctr=9;
    memcpy(&(buff[pos])
          ,&control_infor_out.pitch_ctr,
           2);
    pos  +=  2;
//    control_infor_out.roll_ctr=10;
    memcpy(&(buff[pos])
          ,&control_infor_out.roll_ctr,
           2);
    pos  +=  2;
//    control_infor_out.throttle_percentage=11;
    memcpy(&(buff[pos])
          ,&control_infor_out.acx_forward,
           2);
    pos  +=  2;
    memcpy(&(buff[pos])
          ,&control_infor_out.acy_lateral,
           2);
    pos  +=  2;
    memcpy(&(buff[pos])
          ,&control_infor_out.acz_up,
           2);
    pos  +=  2;
//    printf("horizonal_control_mode: %d\r\n",control_infor_out.horizonal_control_mode);
//    printf("position_x_ctr: %d\r\n",control_infor_out.position_x_ctr);
//    printf("ve_ctr: %d\r\n",control_infor_out.ve_ctr);
//    printf("roll_ctr: %d\r\n",control_infor_out.roll_ctr);
//    printf("guidance throttle: %d\r\n",control_infor_out.throttle_percentage);
//   cout<<"~~2~guidance throttle=="<<control_infor_out.throttle_percentage<<endl;
    memcpy(&(buff[pos])
          ,&control_infor_out.throttle_percentage,
           1);
    pos  +=  1;
    memcpy(&(buff[pos])
          ,&control_infor_out.angleRateX,
           2);
    pos  +=  2;
    memcpy(&(buff[pos])
          ,&control_infor_out.angleRateY,
           2);
    pos  +=  2;
    memcpy(&(buff[pos])
          ,&control_infor_out.angleRateZ,
           2);
    pos  +=  2;
    memcpy(&(buff[pos])
          ,&control_infor_out.param1,
           2);
    pos  +=  2;
    memcpy(&(buff[pos])
          ,&control_infor_out.param2,
           2);
    pos  +=  2;

    if(gs_tx2_protocl_douwn->read_data(buff,pos+1)==0)
    {
        if(control_infor_out_buff.write_data(buff,pos+1)==0)
            ret = 0;
        else {
            ret = 1;
        }
    }


    return ret;

}

int GroundStation::pack_process_N2()
{
    int ret = 0;

    unsigned int pos = 5;

    unsigned char buff[100];
    /* add by wg
    * data:2021-07-12
    * add new protocol*/
   // pos=0;
    /*pos:0*/
    // memcpy((unsigned char *)&buff[pos],(unsigned char *)&newctrl_infor_out.head1,1);
    // pos+=1;
    // /*pos:1*/
    // memcpy((unsigned char *)&buff[pos],(unsigned char *)&newctrl_infor_out.head2,1);
    // pos+=1;
    // /*pos:2*/
    // memcpy((unsigned char *)&buff[pos],(unsigned char *)&newctrl_infor_out.ID,1);
    // pos+=1;
    // /*pos:3*/
    // memcpy((unsigned char *)&buff[pos],(unsigned char *)&newctrl_infor_out.frame_count,1);
    // pos+=1;
    // /*pos:4*/
    // memcpy((unsigned char *)&buff[pos],(unsigned char *)&newctrl_infor_out.length,1);
    // pos+=1;
    /*pos:5*/
    memcpy((unsigned char *)&buff[pos],(unsigned char *)&newctrl_infor_out.main_cmd,1);
    pos+=1;
    /*pos:6*/
    memcpy((unsigned char *)&buff[pos],(unsigned char *)&newctrl_infor_out.sub_command,1);
    pos+=1;
    /*pos:7*/
    memcpy((unsigned char *)&buff[pos],(unsigned char *)&newctrl_infor_out.param1,2);
    pos+=2;
    /*pos:9*/
    memcpy((unsigned char *)&buff[pos],(unsigned char *)&newctrl_infor_out.param2,2);
    pos+=2;

    if(newctrl_to_xy_protocl->read_data(buff,pos+1)==0)
    {
        if(newctrl_infor_out_buff.write_data(buff,pos+1)==0)
            ret = 0;
        else {
            ret = 1;
        }
    }
    //==============================================================

    return ret;

}


int GroundStation::unpack_process()
{
    int ret=0;
    ret=read_data(buff,1000);
    if(ret>0)
    {
        //gs_tx2_protocl_douwn->write_data(buff,ret);
        gs_tx2_protocl_up->write_data(buff,ret);
        //gs_pod_protocl->write_data(buff,ret);
    }

    return 0;
}
