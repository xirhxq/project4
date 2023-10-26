//#include <QCoreApplication>
#include "uart.h"
#include "common.h"
#include "groundstation.h"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
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
#include <stdlib.h>
#include <signal.h>

#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/BatteryState.h"
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>

//#define printf

using namespace std;
XY_CMD_MODE xy_cmd_mode;

std_msgs::Float32MultiArray msg;
ros::Publisher gs_data_pub;
ros::Subscriber guidance_command_sub;
GroundStation gs;

void *read_thread(void *arg);

void *write_thread(void *arg);

static void ground_station_sci_recv_sig(int status);

Uart uart("/dev/ttyUSB0", 230400, 8, 1, 0, ground_station_sci_recv_sig);

static sem_t ground_station_sem_r;//read senmphore
static unsigned char recv_ground_station_signal;

void guidanceCallback(const std_msgs::Float32MultiArray &msg) {
    if (msg.data.size() != 15) return;
    xy_cmd_mode.horizonal_control_mode = msg.data[0];
    xy_cmd_mode.vertical_control_mode = msg.data[1];
    xy_cmd_mode.heading_control_mode = msg.data[2];
    xy_cmd_mode.position_x_ctr = msg.data[3];
    xy_cmd_mode.position_y_ctr = msg.data[4];
    xy_cmd_mode.position_z_ctr = msg.data[5];
    xy_cmd_mode.ve_ctr = (short) (msg.data[6] * 10);
    xy_cmd_mode.vn_ctr = (short) (msg.data[7] * 10);
    xy_cmd_mode.vu_ctr = (short) (msg.data[8] * 10);
    xy_cmd_mode.yaw_ctr = (short) ((msg.data[9]) * 1000);
    xy_cmd_mode.pitch_ctr = (short) (msg.data[10] * 1000);
    xy_cmd_mode.roll_ctr = (short) (msg.data[11] * 1000);
    xy_cmd_mode.angleRateZ = (short) (msg.data[11] * 100);
    xy_cmd_mode.throttle_percentage = (unsigned char) (msg.data[12]);
    xy_cmd_mode.flight_cmd_mode = msg.data[13];
    xy_cmd_mode.landing_cmd = msg.data[14];
}


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "xy_fcu", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::Rate rate(100);   //100Hz

    ros::Publisher gs_data_pub = nh.advertise<std_msgs::Float32MultiArray>("/suav/xy_fcu/flight_data", 10);
    guidance_command_sub = nh.subscribe("/suav/xy_fcu/xy_cmd", 10, guidanceCallback);

    uart.open_sci();
    pthread_t r_thread, w_thread;

    pthread_create(&r_thread, NULL, read_thread, 0);
    pthread_create(&w_thread, NULL, write_thread, 0);
    int ret = 0;

    ret = sem_init(&ground_station_sem_r, 0, 0);
    if (ret < 0) {
        perror("ground_station_sem_r init err...\r\n");
        exit(1);
    }
    cout << "XY_data!!!" << endl;

    while (nh.ok()) {
        ros::spinOnce();

        msg.data.resize(47);
        msg.data[0] = xy_data.horizonal_mode;
        msg.data[1] = xy_data.vertical_mode;
        msg.data[2] = xy_data.yaw_mode;
        msg.data[3] = xy_data.UAV_lon;
        msg.data[4] = xy_data.UAV_lat;
        msg.data[5] = xy_data.UAV_height;
        msg.data[6] = xy_data.baro_height;
        msg.data[7] = xy_data.UAV_Vel[0];
        msg.data[8] = xy_data.UAV_Vel[1];
        msg.data[9] = xy_data.UAV_Vel[2];
        msg.data[10] = xy_data.UAV_pos[0];
        msg.data[11] = xy_data.UAV_pos[1];
        msg.data[12] = xy_data.UAV_pos[2];
        msg.data[13] = xy_data.UAV_Euler[0];
        msg.data[14] = xy_data.UAV_Euler[1];
        msg.data[15] = xy_data.UAV_Euler[2];
        msg.data[16] = xy_data.cur_throttle;
        msg.data[17] = xy_data.UAV_acc[0];
        msg.data[18] = xy_data.UAV_acc[1];
        msg.data[19] = xy_data.UAV_acc[2];
        msg.data[20] = xy_data.UAV_gyro[0];
        msg.data[21] = xy_data.UAV_gyro[1];
        msg.data[22] = xy_data.UAV_gyro[2];
        msg.data[23] = xy_data.UAV_status;
        msg.data[24] = xy_data.uavStage;
        msg.data[25] = xy_data.uavHealthStatus1;
        msg.data[26] = xy_data.uavHealthStatus2;
        msg.data[27] = xy_data.systemClock;
        msg.data[28] = xy_data.year;
        msg.data[29] = xy_data.month;
        msg.data[30] = xy_data.day;
        msg.data[31] = xy_data.hour;
        msg.data[32] = xy_data.minute;
        msg.data[33] = xy_data.second;
        for (int _ = 0; _ < 10; _++) {
            msg.data[_ + 34] = xy_data.pwm[_];
        }
        msg.data[44] = xy_data.expectedAngleRate[0];
        msg.data[45] = xy_data.expectedAngleRate[1];
        msg.data[46] = xy_data.expectedAngleRate[2];

        gs_data_pub.publish(msg);

        printf("************\n");
        xy_data.print();
        printf("------\n");
        xy_cmd_mode.print();
        
        rate.sleep();
    }

    while (true) {
        pthread_join(r_thread, NULL);
        break;
    }
}


static void ground_station_sci_recv_sig(int status) {
    status = 0;
    recv_ground_station_signal = 1;
    sem_post(&ground_station_sem_r);
    return;
}

void *read_thread(void *arg) {
    int ret = 0;
    unsigned char buff[1024];

    ros::NodeHandle nhRead;
    while (nhRead.ok()) {
        //  sem_wait(&ground_station_sem_r);

        if (sem_trywait(&ground_station_sem_r) >= 0) {
            //  cout<<"1ground_station_sem_r come..."<<endl;

            if (recv_ground_station_signal == 1) {
                //  cout<<"2ground_station_sem_r come..."<<endl;
                ret = uart.read_data(buff, 120);
                /*//
                printf("recv data:\r\n");
                for (int i = 0; i < ret; i++) {
                    printf("0x%x ", buff[i]);
                }
                printf("\r\n");
                //*/
                recv_ground_station_signal = 0;
                if (ret > 0) {

                    gs.write_data(buff, ret);
                    gs.unpack_process();

                    if (gs.gs_state & GS_RECV_CONTROL_MODE) {
                        //xy_data.print();
                    }
                    //printf("for test : recv_ok the num is %d\r\n",ret);
                }
            }
        } else {
            usleep(10000);
        }
    }
}


void *write_thread(void *arg)                    //write cmd to FCU
{
    ros::NodeHandle nhWrite;

    while (nhWrite.ok()) {
        gs.control_infor_out.horizonal_control_mode = xy_cmd_mode.horizonal_control_mode;
        gs.control_infor_out.vertical_control_mode = xy_cmd_mode.vertical_control_mode;
        gs.control_infor_out.heading_control_mode = xy_cmd_mode.heading_control_mode;
        gs.control_infor_out.position_x_ctr = xy_cmd_mode.position_x_ctr;
        gs.control_infor_out.position_y_ctr = xy_cmd_mode.position_y_ctr;
        gs.control_infor_out.position_z_ctr = xy_cmd_mode.position_z_ctr;
        gs.control_infor_out.ve_ctr = xy_cmd_mode.ve_ctr;
        gs.control_infor_out.vn_ctr = xy_cmd_mode.vn_ctr;
        gs.control_infor_out.vu_ctr = xy_cmd_mode.vu_ctr;
        gs.control_infor_out.yaw_ctr = xy_cmd_mode.yaw_ctr;
        gs.control_infor_out.pitch_ctr = xy_cmd_mode.pitch_ctr;
        gs.control_infor_out.roll_ctr = xy_cmd_mode.roll_ctr;
        gs.control_infor_out.throttle_percentage = xy_cmd_mode.throttle_percentage;
        gs.control_infor_out.angleRateZ = xy_cmd_mode.angleRateZ;

        gs.newctrl_infor_out.sub_command = xy_cmd_mode.landing_cmd;        //0x10：就地降落; 0x28：立即关车


        // 11:USER_MODE; 22: landing_cmd; 33: no ctr
        // cout<<"flight_cmd_mode="<<xy_cmd_mode.flight_cmd_mode<<endl;

        if (xy_cmd_mode.flight_cmd_mode == 11) {
            gs.pack_process();
//            printf("send user cmd data:\n");
//            for (int i = 0; i < gs.control_infor_out_buff.length(); i++) {
//                printf("0x%x ", gs.control_infor_out_buff.buff()[i]);
//            }
//            printf("\n");
        } else if (xy_cmd_mode.flight_cmd_mode == 22) {
            gs.pack_process_N2();
//            printf("send landing cmd data:\n");
//            for (int i = 0; i < gs.newctrl_infor_out_buff.length(); i++) {
//                printf("0x%x ", gs.newctrl_infor_out_buff.buff()[i]);
//            }
//            printf("\n");
        }


        int total = uart.write_data(gs.newctrl_infor_out_buff.buff(), gs.newctrl_infor_out_buff.length());

        total = uart.write_data(gs.control_infor_out_buff.buff(), gs.control_infor_out_buff.length());
//        printf("total %d\r\n", total);
        usleep(10000);
    }


}




