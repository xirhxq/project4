#ifndef GROUNDSTATION_H
#define GROUNDSTATION_H

#include "object.h"
#include "protocol.h"
#include "fifo.h"

typedef void (*protocol_parsed_ok_func_t)(unsigned char *buff,int length);

#define USR_MODE_CTR_CMD            0XB2//User mode controls the frame
#define FLIIGHT_PARAM_CMD           0XC5//Flight control flight parameters frame flight parameters 2	0xC4	·É¿Ø·ÉÐÐ²ÎÊýÖ¡	·ÉÐÐ²ÎÊý
#define NEW_CTR_CMD                 0XD6  //202107: add auto_landing and propeller_armed function to user mode  
#define FUSE_CMD            0X07
#define LOAD_CMD            0X09

typedef enum
{
    GS_IDLE=0x0,
    GS_RECV_POD=0x01,//ÊÕµ½µõ²ÕÐÅÏ¢
    GS_RECV_YX_POWER=0x02,//ÊÕµ½ÒýÐÅÐÅÏ¢
    GS_RECV_CONTROL_MODE=0x04,//ÊÕµ½¿ØÖÆÄ£Ê½ÐÅÏ¢
}GS_State;

typedef struct
{
    /*1.		Ë®Æ½¿ØÖÆÄ£Ê½	1	UINT08	0x00£º·ÇÓÃ»§¿ØÖÆÄ£Ê½
                                        0x01£ºÎ»ÖÃ¿ØÖÆÄ£Ê½
                                        0x02£ºËÙ¶È¿ØÖÆÄ£Ê½
                                        0x03£º×ËÌ¬¿ØÖÆÄ£Ê½
    */
    unsigned char horizonal_control_mode;
    /*
    1.		´¹Ïò¿ØÖÆÄ£Ê½	1	UINT08	0x00£º·ÇÓÃ»§¿ØÖÆÄ£Ê½
                                    0x01£ºÎ»ÖÃ¿ØÖÆÄ£Ê½
                                    0x02£ºËÙ¶È¿ØÖÆÄ£Ê½
                                    0x03£ºÍÆÁ¦¿ØÖÆÄ£Ê½
    */
    unsigned char vertical_control_mode;
    /*
    1.		º½Ïò¿ØÖÆ	1	UINT08	0x00£º·ÇÓÃ»§¿ØÖÆÄ£Ê½
                                0x01£ºº½Ïò¿ØÖÆ
                                0x02£ºº½Ïò²»ÒªÇó
    */
    unsigned char heading_control_mode;
    /*
            Î»ÖÃÖ¸Áî[X_c,Y_c,Z_c]	12	FP32	Ã×(m)
    */
    float position_x_ctr;
    float position_y_ctr;
    float position_z_ctr;
    /*
    1.		¶«ÏòËÙ¶È¿ØÖÆ	2	INT16	10^-1m/s
    */
    short   ve_ctr;
    /*
    1.		±±ÏòËÙ¶È¿ØÖÆ	2	INT16	10^-1m/s
    */
    short   vn_ctr;
    /*
    1.		ÌìËÙ¿ØÖÆ	2	INT16	10^-1m/s
    */
    short   vu_ctr;
    /*
    1.		º½Ïò½Ç¿ØÖÆ	2	INT16	10^-3rad
    */
    short   yaw_ctr;
    /*
    1.		¸©Ñö½Ç¿ØÖÆ	2	INT16	10^-3rad
    */
    short   pitch_ctr;
    /*
    1.		¹ö×ª½Ç¿ØÖÆ	2	INT16	10^-3rad
    */
    short   roll_ctr;   
	short acx_forward;
	short acy_lateral;
	short acz_up;

    /*
    1.		ÓÍÃÅ°Ù·Ö±È	1	UINT08	·Ö±æÂÊ:10^-2   £¨100¶ÔÓ¦×î´óÓÍÃÅ£©
    */
    unsigned char throttle_percentage;

}control_infor_out_t;

typedef struct
{
    /*1.		µ±Ç°Ë®Æ½¿ØÖÆÄ£Ê½	1	UINT08	0x00£º·ÇÓÃ»§¿ØÖÆÄ£Ê½
                                            0x01£ºÎ»ÖÃ¿ØÖÆÄ£Ê½
                                            0x02£ºËÙ¶È¿ØÖÆÄ£Ê½
                                            0x03£º×ËÌ¬¿ØÖÆÄ£Ê½
    */
    unsigned char cur_horizonal_control_mode;
    /*
    1.		µ±Ç°´¹Ïò¿ØÖÆÄ£Ê½	1	UINT08	0x00£º·ÇÓÃ»§¿ØÖÆÄ£Ê½
                                        0x01£ºÎ»ÖÃ¿ØÖÆÄ£Ê½
                                        0x02£ºËÙ¶È¿ØÖÆÄ£Ê½
                                        0x03£ºÍÆÁ¦¿ØÖÆÄ£Ê½
    */
    unsigned char cur_vertical_control_mode;
    /*
    1.		µ±Ç°º½Ïò¿ØÖÆ	1	UINT08	0x00£º·ÇÓÃ»§¿ØÖÆÄ£Ê½
                                    0x01£ºº½Ïò¿ØÖÆ
                                    0x02£ºº½Ïò²»ÒªÇó
    */
    unsigned char cur_heading_control_mode;
    /*
     1.		ÎÞÈË»ú¾­¶È	4	INT32	·Ö±æÂÊ:10^-6¡ã
    */
    int   lon;
    /*
    1.		ÎÞÈË»úÎ³¶È	4	INT32	·Ö±æÂÊ:10^-6¡ã
    */
    int   lat;
    /*
    1.		ÎÞÈË»úÎÀµ¼¸ß¶È	2	UINT16	·Ö±æÂÊ:10^-2m
    */
    int   alt_satelite;
    /*
    1.		ÎÞÈË»úÆøÑ¹¸ß¶È	2	UINT16	·Ö±æÂÊ:10^-2m
    */
    int   barometer_satelite;
    /*
    1.		ÎÞÈË»ú¶«ÏòËÙ¶È	2	INT16	10^-1m/s
    */
    short   ve;
    /*
    1.		ÎÞÈË»ú±±ÏòËÙ¶È	2	INT16	10^-1m/s
    */
    short   vn;
    /*
    1.		ÎÞÈË»úÌìËÙ	2	INT16	10^-1m/s
    */
    short   vu;
    /*
            Î»ÖÃÖ¸Áî[X_c,Y_c,Z_c]	12	FP32	Ã×(m)
    */
    float position_x_cur;
    float position_y_cur;
    float position_z_cur;
    /*
    1.		ÎÞÈË»úº½Ïò½Ç	2	INT16	10^-3rad
    */
    short   yaw;
    /*
    1.		ÎÞÈË»ú¸©Ñö½Ç	2	INT16	10^-3rad
    */
    short   pitch;
    /*
    1.		ÎÞÈË»ú¹ö×ª½Ç	2	INT16	10^-3rad
    */
    short   roll;
    /*
    1.		ÓÍÃÅ°Ù·Ö±È	1	UINT08	·Ö±æÂÊ:10^-2   £¨100¶ÔÓ¦×î´óÓÍÃÅ£©
    */
    unsigned char throttle_percentage;
    /*
    1.		¼ÓËÙ¶ÈX	2	INT16	·Ö±æÂÊ:10^-2 m/s^2
    */
    short   ax;
    /*
    1.		¼ÓËÙ¶ÈY	2	INT16	·Ö±æÂÊ:10^-2 m/s^2
    */
    short   ay;
    /*
    1.		¼ÓËÙ¶ÈZ	2	INT16	·Ö±æÂÊ:10^-2 m/s^2
    */
    short   az;
    /*
    1.		ÍÓÂÝX	2	INT16	·Ö±æÂÊ:10^-2¡ã
    */
    short   gx;
    /*
    1.		ÍÓÂÝY	2	INT16	·Ö±æÂÊ:10^-2¡ã
    */
    short   gy;
    /*
    1.		ÍÓÂÝZ	2	INT16	·Ö±æÂÊ:10^-2¡ã
    */
    short   gz;
    /*
    1.		ÎÞÈË»ú×´Ì¬	2	UINT16	ÎÞÈË»ú×´Ì¬×Ö½Ú
    */
    unsigned char  uav_status;

}control_infor_in_t;

/* add by wg
 * data:2021-07-12
 * add new protocol*/
#define NEWCTRL_TXLENGTH (11+1)
#define NEWCTRL_RXLENGTH (11+1)
typedef struct {
    /*±ÈÀýÏµÊý£º 1
      ÃèÊöÐÅÏ¢£º 1.		Ö¡Í·	2	UINT08	0xEA¡¢0x9B*/
    unsigned char head1;
    /*±ÈÀýÏµÊý£º 1
      ÃèÊöÐÅÏ¢£º 1.		Ö¡Í·	2	UINT08	0xEA¡¢0x9B*/
    unsigned char head2;
    /*±ÈÀýÏµÊý£º 1
      ÃèÊöÐÅÏ¢£º 1.		Ö¡ID	1	UINT08	0xC4*/
    unsigned char ID;
    /*±ÈÀýÏµÊý£º 1
      ÃèÊöÐÅÏ¢£º 1.		Ö¡¼ÆÊý	1	UINT08	0-255Ñ­»·ÀÛ¼Ó*/
    unsigned char frame_count;
    /*±ÈÀýÏµÊý£º 1
      ÃèÊöÐÅÏ¢£º 1.		Ö¡³¤¶È	1	UINT08	126*/
    unsigned char length;
    /*±ÈÀýÏµÊý£º 1
      ÃèÊöÐÅÏ¢£º 0xf3*/
    unsigned char main_cmd = 243;
    /*±ÈÀýÏµÊý£º 1
      ÃèÊöÐÅÏ¢£º 0x10:¾ÍµØ½µÂä£¬0x28Á¢¿Ì¹Ø³µ*/
    unsigned char sub_command;
    /*±ÈÀýÏµÊý£º 1
      ÃèÊöÐÅÏ¢£º des*/
    unsigned short param1;
    /*±ÈÀýÏµÊý£º 1
      ÃèÊöÐÅÏ¢£º des*/
    unsigned short param2;
    /*±ÈÀýÏµÊý£º 1
      ÃèÊöÐÅÏ¢£º 12. 2 Ð£ÑéºÍ SUM£¨0~31£© Ç° 32 Î»ºÍ£¬ È¡ºóÁ½Î»*/
    unsigned char check;
}newctrl_infor_out_t;
//==============================================================

class GroundStation:public Object
{
public:

    unsigned int    gs_state;
    GroundStation();
    Protocol *gs_tx2_protocl_up;
    Protocol *gs_tx2_protocl_douwn;
    /* add by wg
     * data:2021-07-12
     * add new protocol*/
    Protocol *newctrl_to_xy_protocl;
    //==============================================================

    Protocol *gs_pod_protocl;
    int read_data(unsigned char *buf,unsigned int size) override;
    int write_data(const unsigned char *buf,unsigned int size) override;
    int child_signal(class Object *)override;
    int unpack_process();
    int pack_process();
    int pack_process_N2();
    Fifo podbuf;
    Buff    fuze_power_buf;
    Buff    control_infor_buff;
    Buff    control_infor_out_buff;
    Buff    newctrl_infor_out_buff;
    control_infor_in_t  control_infor_in;
    control_infor_out_t control_infor_out;
    newctrl_infor_out_t newctrl_infor_out;
	
private:
    Fifo fifo;
    unsigned char buff[1024];


};

struct XY_DATA
{
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
    
};
extern XY_DATA xy_data;

#endif // GROUNDSTATION_H


struct XY_CMD_MODE
{
    unsigned char horizonal_control_mode;
    unsigned char vertical_control_mode;
    unsigned char heading_control_mode;
    float position_x_ctr;
    float position_y_ctr;
    float position_z_ctr;
    short   ve_ctr;
    short   vn_ctr;
    short   vu_ctr;
    short   yaw_ctr;
    short   pitch_ctr;
    short   roll_ctr;   
    short acx_forward;
    short acy_lateral;
    short acz_up;
    unsigned char throttle_percentage;
    int flight_cmd_mode;     //11:USER_MODE; 22: landing_cmd; 33: no ctr
    int landing_cmd;       //0x10：就地降落; 0x28：立即关车
};
extern XY_CMD_MODE xy_cmd_mode;

