#ifndef GROUNDSTATION_H
#define GROUNDSTATION_H

#include "object.h"
#include "protocol.h"
#include "fifo.h"

typedef void (*protocol_parsed_ok_func_t)(unsigned char *buff, int length);

#define USR_MODE_CTR_CMD            0XB2//User mode controls the frame
#define FLIIGHT_PARAM_CMD           0XC5//Flight control flight parameters frame flight parameters 2	0xC4	·É¿Ø·ÉÐÐ²ÎÊýÖ¡	·ÉÐÐ²ÎÊý
#define NEW_CTR_CMD                 0XD6  //202107: add auto_landing and propeller_armed function to user mode  
#define FUSE_CMD            0X07
#define LOAD_CMD            0X09

typedef enum {
    GS_IDLE = 0x0,
    GS_RECV_POD = 0x01,
    GS_RECV_YX_POWER = 0x02,
    GS_RECV_CONTROL_MODE = 0x04,
} GS_State;

typedef struct {
    unsigned char horizonal_control_mode;
    unsigned char vertical_control_mode;
    unsigned char heading_control_mode;
    float position_x_ctr;
    float position_y_ctr;
    float position_z_ctr;
    short ve_ctr;
    short vn_ctr;
    short vu_ctr;
    short yaw_ctr;
    short pitch_ctr;
    short roll_ctr;
    short acx_forward;
    short acy_lateral;
    short acz_up;
    unsigned char throttle_percentage;
    short angleRateX;
    short angleRateY;
    short angleRateZ;
    unsigned short param1;
    unsigned short param2;
} control_infor_out_t;

typedef struct {
    unsigned char cur_horizonal_control_mode;
    unsigned char cur_vertical_control_mode;
    unsigned char cur_heading_control_mode;
    int lon;
    int lat;
    int alt_satelite;
    int barometer_satelite;
    short ve;
    short vn;
    short vu;
    float position_x_cur;
    float position_y_cur;
    float position_z_cur;
    short yaw;
    short pitch;
    short roll;
    unsigned char throttle_percentage;
    short ax;
    short ay;
    short az;
    short gx;
    short gy;
    short gz;
    unsigned char uav_status;
    unsigned char uavStage;
    unsigned short uavHealthStatus1, uavHealthStatus2;
    unsigned int systemClock;
    unsigned char year, month, day, hour, minute, second;
    unsigned short pwm[10];
    short expectedAngleRate[3];

} control_infor_in_t;

/* add by wg
 * data:2021-07-12
 * add new protocol*/
#define NEWCTRL_TXLENGTH (11+1)
#define NEWCTRL_RXLENGTH (11+1)
typedef struct {
    unsigned char head1;
    unsigned char head2;
    unsigned char ID;
    unsigned char frame_count;
    unsigned char length;
    unsigned char main_cmd = 243;
    unsigned char sub_command;
    unsigned short param1;
    unsigned short param2;
    unsigned char check;
} newctrl_infor_out_t;

class GroundStation : public Object {
public:

    unsigned int gs_state;

    GroundStation();

    Protocol *gs_tx2_protocl_up;
    Protocol *gs_tx2_protocl_down;
    /* add by wg
     * data:2021-07-12
     * add new protocol*/
    Protocol *newctrl_to_xy_protocl;
    //==============================================================

    Protocol *gs_pod_protocl;

    int read_data(unsigned char *buf, unsigned int size) override;

    int write_data(const unsigned char *buf, unsigned int size) override;

    int child_signal(class Object *) override;

    int unpack_process();

    int pack_process();

    int pack_process_N2();

    Fifo podbuf;
    Buff fuze_power_buf;
    Buff control_infor_buff;
    Buff control_infor_out_buff;
    Buff newctrl_infor_out_buff;
    control_infor_in_t control_infor_in;
    control_infor_out_t control_infor_out;
    newctrl_infor_out_t newctrl_infor_out;

private:
    Fifo fifo;
    unsigned char buff[1024];


};

struct XY_DATA {
    int horizonal_mode;
    int vertical_mode;
    int yaw_mode;
    double UAV_lat;
    double UAV_lon;
    float UAV_height;
    float baro_height;
    float UAV_Vel[3];
    float UAV_pos[3];
    float UAV_Euler[3];
    float cur_throttle;
    float UAV_acc[3];
    float UAV_gyro[3];
    int UAV_status;
    int uavStage;
    int uavHealthStatus1;
    int uavHealthStatus2;
    int systemClock;
    int year, month, day, hour, minute, second;
    int pwm[10];
    double expectedAngleRate[3];
};
extern XY_DATA xy_data;

#endif // GROUNDSTATION_H


struct XY_CMD_MODE {
    unsigned char horizonal_control_mode;
    unsigned char vertical_control_mode;
    unsigned char heading_control_mode;
    float position_x_ctr;
    float position_y_ctr;
    float position_z_ctr;
    short ve_ctr;
    short vn_ctr;
    short vu_ctr;
    short yaw_ctr;
    short pitch_ctr;
    short roll_ctr;
    short acx_forward;
    short acy_lateral;
    short acz_up;
    unsigned char throttle_percentage;
    int flight_cmd_mode;     //11:USER_MODE; 22: landing_cmd; 33: no ctr
    int landing_cmd;       //0x10：就地降落; 0x28：立即关车
};
extern XY_CMD_MODE xy_cmd_mode;

