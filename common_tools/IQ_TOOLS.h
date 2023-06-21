#ifndef IQ_TOOLS_H_
#define IQ_TOOLS_H_
#include "./IQmath/IQmathLib.h"
#define fix_IQ _IQ20
#define fix_sin _IQ20sin
#define fix_cos _IQ20cos
#define fix_mpy _IQ20mpy
#define fix_div _IQ20div
#define fix_sqrt _IQ20sqrt
#define QUEUE_SIZE 50
#define AVE_SIZE 10
typedef struct {
    _iq20 kp,ki,kd;
    _iq20 B0;//KI+KP+KD
    _iq20 B1;//KP+2KD
    _iq20 B2;//KD
    _iq20 err_pre1;
    _iq20 err_pre2;
    _iq20 out;//增量后out
    _iq20 out_last;//记录
    _iq20 upper;
    _iq20 lower;
}PID_CONTROLER_Q;

typedef struct {
    _iq15 kp,ki,kd;
    _iq15 B0;//KI+KP+KD
    _iq15 B1;//KP+2KD
    _iq15 B2;//KD
    _iq15 err_pre1;
    _iq15 err_pre2;
    _iq15 out;//增量后out
    _iq15 out_last;//记录
    _iq15 upper;
    _iq15 lower;
}PID_CONTROLER_IQ15;
typedef struct{
    _iq20 a;
    _iq20 b;
    _iq20 c;
}ABC;

typedef struct{
   _iq20 alpha;
   _iq20 beta;
}Alpha_Beta;

typedef struct{
   _iq20 d;
    _iq20 q;
}D_Q;
typedef struct 
{
    _iq20 sin_theta;
    _iq20 cos_theta;
}Theta;

typedef struct 
{
    _iq20 data[QUEUE_SIZE];
    int head;
    int tail;
}queue;
typedef struct 
{
    _iq20 data[AVE_SIZE];
    int pointer;
}ave_queue;
typedef struct 
{
    _iq20 y_out;
    _iq20 a;//a=2*pi*fc/fs fc截止频率 fs 采样频率
    _iq20 b;
}IQ_filter;
typedef struct
{
    _iq20 w1;
    _iq20 w2;
    _iq20 w3;
}Wave;
void PID_init_Q(PID_CONTROLER_Q* pic,_iq20 kp,_iq20 ki,_iq20 kd,_iq20 upper,_iq20 lower);
void PID_init_IQ15(PID_CONTROLER_IQ15* pic,_iq15 kp,_iq15 ki,_iq15 kd,_iq15 upper,_iq15 lower);
void PID_Q_step(PID_CONTROLER_Q* pic,_iq20 err);
void PID_IQ15_step(PID_CONTROLER_IQ15* pic,_iq15 err);
void change_pid_par(PID_CONTROLER_Q* pic,_iq20 kp,_iq20 ki,_iq20 kd);
void abc2alpha_beta(ABC * abc,Alpha_Beta * alpha_beta);
void alpha_beta2dq(Alpha_Beta * I_alpha_beta,Alpha_Beta * U_alpha_beta,Theta * theta,D_Q * dq);
void dq2alpha_beta(D_Q *dq,Theta * theta,Alpha_Beta * alpha_beta);
void alpha_beta2wave(Alpha_Beta * alpha_beta,Wave * wave);
void queue_push(queue* q,_iq20* x);
_iq20 queue_pop(queue * q);
void queue_clear(queue * q);
void ave_queue_push(ave_queue* q,_iq20 x);
void phase_adjust(Alpha_Beta * data_alpha_beta,_iq20 sin_theta,_iq20 cos_theta);
void pid_out_init(PID_CONTROLER_Q* pic);
void IQ_filter_init(IQ_filter * filter,_iq20 a);
_iq20 IQ_filter_step(IQ_filter * filter,_iq20 x);
_iq20 ave_queue_sum(ave_queue * q);
extern _iq20 U_alpha_beta_L;
extern _iq20 I_alpha_beta_L;
extern _iq20 DQ_alpha_beta_L;
extern _iq20 sqrt_3;
extern _iq20 sqrt_2;
extern _iq20 sqrt_2_3;
#endif
