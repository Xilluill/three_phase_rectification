#include "IQ_TOOLS.h"
_iq20 U_alpha_beta_L=0;
_iq20 I_alpha_beta_L=0;
_iq20 DQ_alpha_beta_L=0;
_iq20 sqrt_2=1482910;//sqrt(2)*2^20
_iq20 sqrt_3=1816186;//sqrt(3)*2^20
_iq20 sqrt_2_3=699050;//(2/3)*2^20
void PID_init_Q(PID_CONTROLER_Q* pic,_iq20 kp,_iq20 ki,_iq20 kd,_iq20 upper,_iq20 lower)
{
    //_IQ20
    pic->kp = kp;
    pic->ki = ki;
    pic->kd = kd;
    pic->out_last=0;
    pic ->B0=ki+kp;
    pic->B1 = kp+2*kd;
    //pic->B1 = kp;
    pic->B2 = kd;
    pic->upper = upper;
    pic->lower = lower;
    pic->err_pre1 = 0;
    pic->err_pre2 = 0;
    pic->out = 0;
}
void PID_init_IQ15(PID_CONTROLER_IQ15* pic,_iq15 kp,_iq15 ki,_iq15 kd,_iq15 upper,_iq15 lower)
{
    //_IQ15
    pic->kp = kp;
    pic->ki = ki;
    pic->kd = kd;
    pic->out_last=0;
    pic ->B0=ki+kp;
    //pic->B1 = kp+2*kd;
    pic->B1 = kp;
    pic->B2 = 0;
    pic->upper = upper;
    pic->lower = lower;
    pic->err_pre1 = 0;
    pic->err_pre2 = 0;
    pic->out = 0;
}
void PID_Q_step(PID_CONTROLER_Q *pic, _iq20 err)
{
    //pic->out = pic->out_last + pic->B0 * err + pic->B2 * pic->err_pre2 - pic->B1 * pic->err_pre1;
    //pic->out = pic->out_last + fix_mpy(pic->B0,err) + fix_mpy(pic->B2,pic->err_pre2) - fix_mpy(pic->B1,pic->err_pre1);
   // pic->out = pic->out_last + fix_mpy(pic->kp,err)- fix_mpy(pic->kp,pic->err_pre1);

    //if(_IQabs(err)<_IQ20(0.01))err=0;
    pic->out = pic->out_last + fix_mpy(pic->B0,err)- fix_mpy(pic->B1,pic->err_pre1);
    pic->out = (pic->out > pic->upper) ? pic->upper : ((pic->out < pic->lower) ? pic->lower : pic->out);
    //pic->err_pre2 = pic->err_pre1;
    pic->err_pre1 = err;
    pic->out_last = pic->out;
}
void PID_IQ15_step(PID_CONTROLER_IQ15* pic,_iq15 err)
{
    //pic->out = pic->out_last + pic->B0 * err + pic->B2 * pic->err_pre2 - pic->B1 * pic->err_pre1;
    //pic->out = pic->out_last + fix_mpy(pic->B0,err) + fix_mpy(pic->B2,pic->err_pre2) - fix_mpy(pic->B1,pic->err_pre1);
   // pic->out = pic->out_last + fix_mpy(pic->kp,err)- fix_mpy(pic->kp,pic->err_pre1);
    //if(_IQabs(err)<_IQ15(0.05))err=0;
    pic->out = pic->out_last + _IQ15mpy(pic->B0,err)- _IQ15mpy(pic->B1,pic->err_pre1);
    pic->out = (pic->out > pic->upper) ? pic->upper : ((pic->out < pic->lower) ? pic->lower : pic->out);
    //pic->err_pre2 = pic->err_pre1;
    pic->err_pre1 = err;
    pic->out_last = pic->out;
}
void abc2alpha_beta(ABC * abc,Alpha_Beta * alpha_beta)
{
    alpha_beta->alpha = fix_mpy(sqrt_2_3,abc->a-_IQdiv2(abc->b+abc->c));
    alpha_beta->beta = _IQ20div(abc->b-abc->c,sqrt_3);
}
void alpha_beta2dq(Alpha_Beta * I_alpha_beta,Alpha_Beta * U_alpha_beta,Theta * theta,D_Q * dq)
{
    U_alpha_beta_L = fix_sqrt(fix_mpy(U_alpha_beta->alpha,U_alpha_beta->alpha)+fix_mpy(U_alpha_beta->beta,U_alpha_beta->beta));
    I_alpha_beta_L = fix_sqrt(fix_mpy(I_alpha_beta->alpha,I_alpha_beta->alpha)+fix_mpy(I_alpha_beta->beta,I_alpha_beta->beta));
    theta->sin_theta = fix_div(U_alpha_beta->beta,U_alpha_beta_L);
    theta->cos_theta = fix_div(U_alpha_beta->alpha,U_alpha_beta_L);
    dq->q = -fix_mpy(I_alpha_beta->alpha,theta->sin_theta)+fix_mpy(I_alpha_beta->beta,theta->cos_theta);
    dq->d = fix_mpy(I_alpha_beta->alpha,theta->cos_theta)+fix_mpy(I_alpha_beta->beta,theta->sin_theta);
}

void dq2alpha_beta(D_Q *dq,Theta * theta,Alpha_Beta * alpha_beta)
{  
    DQ_alpha_beta_L=fix_sqrt(fix_mpy(dq->d,dq->d)+fix_mpy(dq->q,dq->q));
    alpha_beta->beta = fix_mpy(dq->d,theta->sin_theta) +fix_mpy(dq->q,theta->cos_theta);
    alpha_beta->alpha = fix_mpy(dq->d,theta->cos_theta) - fix_mpy(dq->q,theta->sin_theta);
}

void queue_push(queue* q,_iq20* x)
{
    *(q->data+q->tail)=*x;
    q->tail=(q->tail+1)%QUEUE_SIZE;
}
_iq20 queue_pop(queue* q)
{
    _iq20 x=*(q->data+q->head);
    q->head=(q->head+1)%QUEUE_SIZE;
    return x;
}
void queue_clear(queue* q)
{
    q->head=0;
    q->tail=0;
    _iq20 i=0;
    for(i=0;i<QUEUE_SIZE;i++)
    {
        *(q->data+i)=0;
    }
}
void ave_queue_push(ave_queue* q,_iq20 x)
{
    q->data[q->pointer]=x;
    q->pointer=(q->pointer+1)%AVE_SIZE;
}
_iq20 ave_queue_sum(ave_queue * q)
{
    _iq20 sum=0;
    int i=0;
    for(i=0;i<AVE_SIZE;i++)
    {
        sum+=q->data[i];
    }
    return _IQ20div(sum,_IQ20(AVE_SIZE));
}
void phase_adjust(Alpha_Beta * data_alpha_beta,_iq20 sin_theta,_iq20 cos_theta)
{
    data_alpha_beta->alpha = fix_mpy(data_alpha_beta->alpha,cos_theta) - fix_mpy(data_alpha_beta->beta,sin_theta);
    data_alpha_beta->beta = fix_mpy(data_alpha_beta->beta,cos_theta) + fix_mpy(data_alpha_beta->beta,sin_theta);
}
void change_pid_par(PID_CONTROLER_Q* pic,_iq20 kp,_iq20 ki,_iq20 kd)
{
    pic->kp = kp;
    pic->ki = ki;
        pic->kd = kd;
        pic ->B0=ki+kp;
        //pic->B1 = kp+2*kd;
        pic->B1 = kp;
        pic->B2 = 0;
}
void pid_out_init(PID_CONTROLER_Q* pic)
{
    pic->out_last=0;
    pic->err_pre1 = 0;
       pic->err_pre2 = 0;
       pic->out = 0;
}
void IQ_filter_init(IQ_filter * filter,_iq20 a)
{
    filter->a = a;
    filter->b = _IQ20(1)-a;
    filter->y_out = 0;
}
_iq20 IQ_filter_step(IQ_filter * filter,_iq20 x)
{
    filter->y_out = fix_mpy(filter->b,filter->y_out)+fix_mpy(filter->a,x);
    return filter->y_out;
}
void alpha_beta2wave(Alpha_Beta * alpha_beta,Wave * wave)
{
    wave->w1 = alpha_beta->alpha;
    wave->w2 = _IQdiv2(-alpha_beta->alpha + fix_mpy(sqrt_3,alpha_beta->beta));
    wave->w3 = _IQdiv2(-alpha_beta->alpha - fix_mpy(sqrt_3,alpha_beta->beta));
    // int k = fix_div(fix_IQ(512),Udc);
    // wave->w1 = fix_mpy(k,Udc - fix_mpy(fix_IQ(2.0),alpha_beta->alpha));
    // wave->w2 = fix_mpy(k,Udc + alpha_beta->alpha - fix_mpy(fix_IQ(1.732051),alpha_beta->beta));
    // wave->w3 = fix_mpy(k,Udc + alpha_beta->alpha + fix_mpy(fix_IQ(1.732051),alpha_beta->beta));
}

