#ifndef TXIANG_UTOOLS_H_
#define TXIANG_UTOOLS_H_
#define FLASH_USER_DATA_ADDR    0x30080//要+4
#define PAREMETER_COUNT 12
#define PAREMETER_GROUP_COUNT 4
#include "common.h"
typedef struct {
    float kp,ki,kd;
    float B0;//KI+KP+KD
    float B1;//KP+2KD
    float B2;//KD
    float err_pre1;
    float err_pre2;
    float out;//增量后out
    float out_last;//记录
    float upper;
    float lower;
}PID_CONTROLER_F;
typedef struct {
    float old;
    float mid;
    float new;
    float ave_out;
}float_ave;
void init();
void PID_init_F(PID_CONTROLER_F* pic,float out0,float ki,float kp,float kd,float upper,float lower);
void PID_step_F(PID_CONTROLER_F* pic,float err);

void write_int_to_flash(uint32_t* value_arr,uint32_t address,uint32_t count);
void read_int_from_flash(uint32_t* value_arr,uint32_t address,uint32_t count);
void write_float_to_flash(float** value_arr,uint32_t address,uint32_t count);
void read_float_from_flash(float** value_arr,uint32_t address,uint32_t count);

void float_ave_init(float_ave * fa);
float float_ave_step(float_ave * fa,float datain);
#endif
