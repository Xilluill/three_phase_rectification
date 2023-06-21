/*//1
三相交流电子负载 dq逆变换法
*/
#include "common_tools/LCD12864_rom_enable.h"
#include "common_tools/common.h"
#include "common_tools/bus_fpga.h"
#include "common_tools/ADS8688_TIVA_FPGA.h"
#include "common_tools/Txiang_utools.h"
#include "common_tools/blue.h"
#include "common_tools/IQ_TOOLS.h"
#define pi 3.1415926
int mode_flag = 0, test_flag = 0; // 0待机发安全波，1开始控制
int LC_flag = 0, I_inverse_flag = 1;
void *p;
int time_flag = 0;
float ubus_set = 36, empty_1 = 0, empty_2 = 0; // 输出电压有效,值保持36v,电流3.6a过流保护
float ua_k = 25.3229, ua_b = 0.0329;           // 28.1956
float ub_k = 25.5654, ub_b = -0.0151;          // 28.1956
float uc_k = 25.7142, uc_b = 0.009;            // 28.1956
float ia_k = 1.5354, ia_b = 0.0042;            // 1.7 -0.05
float ib_k = 1.5396, ib_b = -0.0008;           // 1.7 -0.05
float ic_k = 1.5398, ic_b = -0.0009;           // 1.7 -0.05
float ua_rms, ub_rms, uc_rms, ia_rms, ib_rms, ic_rms;
// 采集数据 延迟1/4个周期 0.005s 采样率20k 0.00005s
int ch0, ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8;
ave_queue udc_mean;
_iq20 udc, u1_rms;
_iq20 w1_final, w2_final, w3_final;
_iq20 U_a, U_b, U_c, I_a, I_b, I_c;
_iq20 cons_rest_div, cons_ad_adj, cons_1, cons_2;
_iq20 cons_10_24, cons_dc_k, cons_div, cons_512, cons_511, cons_64;
_iq20 cons_5_12, cons_w_L, cons_iqref, cons_idref, cons_pf_tan, cons_dc_b;
_iq20 cons_ua_k, cons_ub_k, cons_uc_k, cons_ia_k, cons_ib_k, cons_ic_k;
_iq20 cons_ua_b, cons_ub_b, cons_uc_b, cons_ia_b, cons_ib_b, cons_ic_b;
_iq15 cons_65535, cons_bus_ref, cons_i_set;
// 坐标变换
ABC U_abc;
ABC I_abc;
Alpha_Beta U_alpha_beta;
Alpha_Beta I_alpha_beta;
Alpha_Beta U_out_alpha_beta;
D_Q U_in_U;
D_Q I_in_U;
D_Q U_out_dq;
Theta theta;
Wave wave;
IQ_filter U_ad;
IQ_filter I_ad;
// pid参数
int pid_flag = 0;
PID_CONTROLER_Q d_loop; // 需要控制有效值，通过调制比来调整
PID_CONTROLER_Q q_loop;
PID_CONTROLER_Q i_loop;
PID_CONTROLER_IQ15 buck_loop;
float dq_kp = 4, dq_ki = 0.03, dq_kd = 0;
float buck_kp = 1, buck_ki = 0.1, buck_kd = 0;
float i_kp = 1, i_ki = 0.03, i_kd = 0;
// 其他参数
float adj = 5000, ad_adj = 1;
float u_rms = 0.0, i_rms = 0.0;
float alpha_set = 2.8, pf_set = 1;
float phase_ad = 0;
float fai = 0;
// 参数显示与存储
float *paremeter[12] = {&alpha_set, &pf_set, &ad_adj,
                        &dq_kp, &dq_ki, &dq_kd,
                        &i_kp, &i_ki, &i_kd,
                        &ubus_set, &adj, &fai};
int paremeter_group_flag = 0, paremeter_choose_flag = 0;
int paremeter_flag = 0;
void paremeter_group_add();
void paremeter_group_sub();
void paremeter_choose_add();
void paremeter_choose_sub();
void paremeter_add();
void paremeter_sub();
void paremeter_display();
void paremeter_to_int();
extern void PID_Q_init(PID_CONTROLER_Q *pic, _iq20 kp, _iq20 ki, _iq20 kd, _iq20 upper, _iq20 lower);
// 函数段
void doADC();
void init();
void ReadKey();
void Display();
void Timer0IntHandler(void); // 10hz
void Timer1AIntHandler(void);
// void Timer1IntHandler(void); // 10
char keydat = 0, keydat_last = 0;
unsigned int a = 0, a_last = 0;
int key_flag = 0;
void KeyToControl();
void rest();
void PID();
void change_IQ_par();
void *model_func_pointer[2] = {rest, PID};
// 测量取平均相关函数
void ave_init();
int main(void)

{
    FPULazyStackingEnable();
    FPUEnable();
    cons_1 = _IQ20(1); // 常数全都提前计算好
    cons_2 = _IQ20(2);
    cons_10_24 = _IQ20(10.24);
    cons_5_12 = _IQ20(5.12);
    cons_dc_k = _IQ20(109.57);
    cons_dc_b = _IQ20(1 - 0.219);
    // cons_w_L=_IQ20(0.387*pi);
    cons_pf_tan = _IQ20div((_IQ20(1) - _IQ20(pf_set) * _IQ20(pf_set)), _IQ20(pf_set));
    // cons_iqref=fix_mpy(_IQ20(alpha_set),fix_sqrt(_IQ20(1)-fix_mpy(_IQ20(pf_set),_IQ20(pf_set))));
    // cons_idref=fix_mpy(_IQ20(alpha_set),_IQ20(pf_set));
    cons_65535 = _IQ15(65535);
    cons_64 = _IQ20(64);
    cons_bus_ref = _IQ15(ubus_set);
    cons_512 = _IQ20(512);
    cons_511 = _IQ20(511);
    // 拟合系数存储
    cons_ua_k = _IQ20(ua_k);
    cons_ub_k = _IQ20(ub_k);
    cons_uc_k = _IQ20(uc_k);
    cons_ia_k = _IQ20(ia_k);
    cons_ib_k = _IQ20(ib_k);
    cons_ic_k = _IQ20(ic_k);
    cons_ua_b = _IQ20(ua_b);
    cons_ub_b = _IQ20(ub_b);
    cons_uc_b = _IQ20(uc_b);
    cons_ia_b = _IQ20(ia_b);
    cons_ib_b = _IQ20(ib_b);
    cons_ic_b = _IQ20(ic_b);

    cons_div = _IQ20(700);
    cons_rest_div = _IQ20(700);
    cons_ad_adj = _IQ20(ad_adj);

    cons_i_set = _IQ20(alpha_set);
    // blue1(&u_rms);blue2(&i_rms);
    init();
    SysCtlDelay(SysCtlClockGet() / 35);
    IOWR(CSWR, 3, 1999);
    // IOWR(CSWR,4,duty);
    PID_init_IQ15(&buck_loop, _IQ15(buck_kp), _IQ15(buck_ki), _IQ15(buck_kd), _IQ15(5000), _IQ15(100)); // pointer,out0,ki kp kd,up,low
    PID_init_Q(&d_loop, _IQ20(dq_kp), _IQ20(dq_ki), _IQ20(dq_kd), _IQ20(1000), _IQ20(-1000));
    PID_init_Q(&i_loop, _IQ20(i_kp), _IQ20(i_ki), _IQ20(i_kd), _IQ20(1000), _IQ20(-1000));
    PID_init_Q(&q_loop, _IQ20(dq_kp), _IQ20(dq_ki), _IQ20(dq_kd), _IQ20(1000), _IQ20(-1000));
    IQ_filter_init(&U_ad, _IQ20(2 * pi * 500 / (QUEUE_SIZE * 200)));
    IQ_filter_init(&I_ad, _IQ20(2 * pi * 500 / (QUEUE_SIZE * 200))); // a=2*pi*fc/fs fc截止频率 fs 采样频率 fc设置为200hz fs设置为5000hz
    blue1(&ic_rms);
    // blue2(&i3_rms);
    // blue3(&fhz);
    while (1)
    {
        Display();
        ReadKey();
        // IOWR(CSWR,0,(int)(2.56*fhz));
        if (a_last != a)
            KeyToControl();
        //  keydat=IORD(CS0,0);5
        // doADC();
    }
    return 0;
}

//------------------------------------------------------------------------------------//

void KeyToControl()
{
    switch (a)
    {
    case 1:
        mode_flag = 0;
        pid_out_init(&d_loop);
        pid_out_init(&q_loop);
        break;
    case 2:
        mode_flag = 1; // 手动控制状态
        break;
    case 3:
        test_flag = 1 - test_flag;
        break;
    case 4:
        break;
    case 5:
        break;
    case 6:
        break;
    case 7:

        break;
    case 8:
        break;
    case 9:
        paremeter_group_add();
        break;
    case 10:
        paremeter_group_sub();
        break;
    case 11:
        break;
    case 12:
        break;
    case 13:
        paremeter_choose_add();
        break;
    case 14:
        paremeter_choose_sub();
        break;
    case 15:
        paremeter_add();
        break;
    case 16:
        paremeter_sub();
        break;
    default:
        break;
    }
}
//------------------------------------------------------------------------------------//
void Timer0IntHandler(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    p = model_func_pointer[mode_flag];
    (*(unsigned int (*)(void))p)();
}
// void Timer1AIntHandler(void)
// {
//     TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
//     //u_rms=getVoltageRMS(CS2, 6, 12);
//     //if(uart1_flag)sentdata();
//     //u_rms=U_alpha_beta_L/1048576.0f;
//     //u_rms=u_rms*13.6895+0.055;
//     //i_rms=I_alpha_beta_L/1048576.0f;
//     //PID_IQ15_step(&buck_loop, cons_bus_ref-_IQ20toIQ(udc));
//     IOWR(CSWR,3,1999);
//     //if(udc==0)udc=_IQ20(1.0);
// }

void Display()
{
    
     DispString5x8(1, 1, "mode:", 1);DispNumber5x8(1, 24, mode_flag, 1, 1);
      DispString5x8(2, 1, "U:", 1);DispFloat5x8(2, 24,U_alpha_beta_L/741455.2f, 1, 2, 4);//DispFloat5x8(2, 72, udc/1048576.0, 1, 2, 4);
      DispString5x8(3, 1, "I:", 1);DispFloat5x8(3, 24,I_alpha_beta_L/741455.2f, 1, 2, 4);
      DispString5x8(4, 1, "idq:", 1);DispFloat5x8(4, 24, I_in_U.d/1048576.0f,1, 2, 4);DispFloat5x8(4, 72,I_in_U.q/1048576.0f, 1, 2, 4);
//    DispFloat5x8(2, 1, ua_rms,1,2,4);DispFloat5x8(2, 60,ia_rms, 1, 2, 4);//DispFloat5x8(2, 72, udc/1048576.0, 1, 2, 4);
//    DispFloat5x8(3, 1, ub_rms,1,2,4);DispFloat5x8(3, 60,ib_rms, 1, 2, 4);
//    DispFloat5x8(4, 1, uc_rms,1,2,4);DispFloat5x8(4, 60,ic_rms, 1, 2, 4);
      //DispNumber5x8(5, 1,512+(w1_final>>20), 4, 1);DispNumber5x8(5, 30,512+(w2_final>>20), 4, 1);DispNumber5x8(5,60,512+(w3_final>>20), 4, 1);
    //DispString5x8(5, 1, "adj:", 1);DispNumber5x8(5, 24,(int)adj, 4, 1);
    //DispString5x8(5, 1, "vsi:", 1);DispFloat5x8(5, 24, adj,1, 4, 1);//U_out_alpha_beta.alpha
    DispString5x8(6, 1, "keyd:", 1);DispNumber5x8(6, 30, keydat, 3, 1);DispString5x8(6, 54, "a:", 1);DispNumber5x8(6, 66, a, 3, 1);
    DispString5x8(7, 1, "pio:", 1);DispFloat5x8(7, 24, U_out_dq.d/1048576.0f,1, 4, 2);DispFloat5x8(7, 72,U_out_dq.q/1048576.0f, 1, 4, 2);
    paremeter_display();
    //    DispNumber5x8(1,1,ch0,5,1);
    //    DispNumber5x8(2,1,ch1,5,1);
    //    DispNumber5x8(3,1,ch2,5,1);
    //    DispNumber5x8(4,1,ch3,5,1);
    //    DispNumber5x8(5,1,ch4,5,1);
    //    DispNumber5x8(6,1,ch5,5,1);
    //    DispNumber5x8(7,1,ch6,5,1);
    //    DispNumber5x8(8,1,ch7,5,1);
}
void init()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= GPIO_PIN_0;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= GPIO_PIN_7;

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    lcd12864_init();
    DispClear();
    initialize_uart();
    BUS_Init();

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC | TIMER_CFG_A_PERIODIC);

    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() / 5000 - 1); // 10KHz

    IntEnable(INT_TIMER0A);

    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0IntHandler);
    TimerEnable(TIMER0_BASE, TIMER_A);

    // SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

    // TimerConfigure(TIMER1_BASE,TIMER_CFG_PERIODIC | TIMER_CFG_A_PERIODIC);

    // TimerLoadSet(TIMER1_BASE,TIMER_A,SysCtlClockGet()/10-1);    //10Hz

    // IntEnable(INT_TIMER1A);

    // TimerIntEnable(TIMER1_BASE,TIMER_TIMA_TIMEOUT);

    // TimerIntRegister(TIMER1_BASE,TIMER_A,Timer1AIntHandler);
    // TimerEnable(TIMER1_BASE,TIMER_A);

    IntMasterEnable();
}
void doADC()
{
    //    ua_rms=getVoltageRMS(CS3, 3, 12)*ua_k+ua_b;
    //    ub_rms=getVoltageRMS(CS2, 3, 12)*ub_k+ub_b;
    //    uc_rms=getVoltageRMS(CS2, 6, 12)* uc_k + uc_b;
    //    ia_rms=getVoltageRMS(CS3, 0, 12)* ia_k + ia_b;
    //    ib_rms=getVoltageRMS(CS2, 0, 12)* ib_k+ib_b;;
    //    ic_rms=getVoltageRMS(CS2, 9, 12)* ic_k + ic_b;
    ch0 = IORD(CS1, 0); // ib
    ch1 = IORD(CS1, 1); // ub

    ch2 = IORD(CS1, 2); // uc
    ch3 = IORD(CS1, 3); // ic
    // ch4=IORD(CS1,4);//ic
    // ch5=IORD(CS1,5);//udc

    ch6 = IORD(CS1, 6); // ia
    ch7 = IORD(CS1, 7); // ua

    //     ave_queue_push(&udc_mean,_IQ20mpy(cons_dc_k,_IQtoIQ20(_IQ15div(_IQ15(ch1-32768),cons_65535)))+cons_dc_b);
    //     udc=ave_queue_sum(&udc_mean);
    //  U_abc.a =_IQ20mpy(cons_10_24,_IQtoIQ20(_IQ15div(_IQ15(ch7-32768),cons_65535)));//采样 因为上限原因用iq15存读取的值 再转换成浮点数
    //  I_abc.a =_IQ20mpy(cons_10_24,_IQtoIQ20(_IQ15div(_IQ15(ch6-32768),cons_65535)));
    U_abc.a = _IQtoIQ20(_IQ15div(_IQ15(ch7 - 32768), cons_64)); // 采样 因为上限原因用iq15存读取的值 再转换成浮点数
    I_abc.a = _IQtoIQ20(_IQ15div(_IQ15(ch6 - 32768), cons_64));
    //  U_abc.b =_IQ20mpy(cons_10_24,_IQtoIQ20(_IQ15div(_IQ15(ch1-32768),cons_65535)));
    //  I_abc.b =_IQ20mpy(cons_10_24,_IQtoIQ20(_IQ15div(_IQ15(ch0-32768),cons_65535)));
    U_abc.b = _IQtoIQ20(_IQ15div(_IQ15(ch1 - 32768), cons_64));
    I_abc.c = _IQtoIQ20(_IQ15div(_IQ15(ch0 - 32768), cons_64));

    //  U_abc.c =_IQ20mpy(cons_10_24,_IQtoIQ20(_IQ15div(_IQ15(ch2-32768),cons_65535)));
    //  I_abc.c =_IQ20mpy(cons_10_24,_IQtoIQ20(_IQ15div(_IQ15(ch3-32768),cons_65535)));
    U_abc.c = _IQtoIQ20(_IQ15div(_IQ15(ch2 - 32768), cons_64));
    I_abc.c = _IQtoIQ20(_IQ15div(_IQ15(ch3 - 32768), cons_64));
    if (!test_flag)
    {
        U_abc.a = _IQ20mpy(cons_ua_k, U_abc.a) + cons_ua_b; // 拟合
        U_abc.b = _IQ20mpy(cons_ub_k, U_abc.b) + cons_ub_b;
        U_abc.c = _IQ20mpy(cons_uc_k, U_abc.c) + cons_uc_b;

        I_abc.a = -_IQ20mpy(cons_ia_k, I_abc.a) - cons_ia_b;
        I_abc.b = -_IQ20mpy(cons_ib_k, I_abc.b) - cons_ib_b;
        I_abc.c = -_IQ20mpy(cons_ic_k, I_abc.c) - cons_ic_b;
        // U_now=IQ_filter_step(&U_ad,U_now);//一阶滤波
        // I_now=IQ_filter_step(&I_ad,I_now);
    }
    //     U_abc.a=U_a;
    //     U_abc.b=U_b;
    //     U_abc.c=U_c;
    //     I_abc.a=I_a;
    //     I_abc.b=I_b;
    //     I_abc.c=I_c;//三相坐标系
    abc2alpha_beta(&U_abc, &U_alpha_beta); // 三相坐标系转换成αβ坐标系
    abc2alpha_beta(&I_abc, &I_alpha_beta);
    alpha_beta2dq(&I_alpha_beta, &U_alpha_beta, &theta, &I_in_U); // * I_alpha_beta,Alpha_Beta * U_alpha_beta,Theta * theta,D_Q * dq)
    U_in_U.d = U_alpha_beta_L;
    U_in_U.q = 0;
}
void rest()
{
    //     PID_init_Q(&vsi_loop,_IQ20(vsi_kp), _IQ20(vsi_ki), _IQ20(vsi_kd), 5000, 0); // pointer,out0,ki kp kd,up,low
    //     PID_init_Q(&d_loop,_IQ20(d_kp), _IQ20(d_ki), _IQ20(d_kd), 30, 0);
    //     PID_init_Q(&q_loop,q_kp, _IQ20(q_ki),_IQ20(q_kd),30, 0);
    doADC();
    if (time_flag < 10000)
    {
        time_flag++;
        return;
    }
    //  w1_final=_IQ20div(U_abc.a,U_alpha_beta_L);
    //  w2_final=_IQ20div(U_abc.b,U_alpha_beta_L);
    //  w3_final=_IQ20div(U_abc.c,U_alpha_beta_L);
    //   w1_final=w1_final>cons_1?cons_1:(w1_final<-cons_1?-cons_1:w1_final);
    //   w2_final=w2_final>cons_1?cons_1:(w2_final<-cons_1?-cons_1:w2_final);
    //   w3_final=w3_final>cons_1?cons_1:(w3_final<-cons_1?-cons_1:w3_final);
    //  w1_final=fix_mpy(w1_final,cons_511);
    //  w2_final=fix_mpy(w2_final,cons_511);
    //  w3_final=fix_mpy(w3_final,cons_511);
    //  IOWR(CSWR,0,512+(w1_final>>20));
    //  IOWR(CSWR,1,512+(w2_final>>20));
    //  IOWR(CSWR,2,512+(w3_final>>20));
}
void PID()
{
    doADC();
    //    PID_Q_step(&i_loop,cons_bus_ref-udc);//I_in_U.d
    //    cons_idref=i_loop.out;
    //    cons_iqref=_IQ20mpy(cons_idref,cons_pf_tan);
    PID_Q_step(&d_loop, -cons_i_set + I_in_U.d); // I_in_U.d
    PID_Q_step(&q_loop, I_in_U.q);
    U_out_dq.d = U_in_U.d - d_loop.out; //-fix_mpy(I_in_U.q,cons_w_L);//-d_loop.out//OUT_alpha -512-511 dq轴在一个圆内
    U_out_dq.q = U_in_U.q - q_loop.out; //+fix_mpy(I_in_U.d,cons_w_L);
                                        //    U_out_dq.d=U_out_dq.d>dq_up?dq_up:(U_out_dq.d<dq_low?dq_low:U_out_dq.d);
                                        //    U_out_dq.q=U_out_dq.q>dq_up?dq_up:(U_out_dq.q<dq_low?dq_low:U_out_dq.q);
    dq2alpha_beta(&U_out_dq, &theta, &U_out_alpha_beta);
    alpha_beta2wave(&U_out_alpha_beta, &wave);
    w1_final = _IQ20div(wave.w1, cons_rest_div);
    w2_final = _IQ20div(wave.w2, cons_rest_div);
    w3_final = _IQ20div(wave.w3, cons_rest_div);
    w1_final = w1_final > cons_1 ? cons_1 : (w1_final < -cons_1 ? -cons_1 : w1_final);
    w2_final = w2_final > cons_1 ? cons_1 : (w2_final < -cons_1 ? -cons_1 : w2_final);
    w3_final = w3_final > cons_1 ? cons_1 : (w3_final < -cons_1 ? -cons_1 : w3_final);
    w1_final = fix_mpy(w1_final, cons_511);
    w2_final = fix_mpy(w2_final, cons_511);
    w3_final = fix_mpy(w3_final, cons_511);
    IOWR(CSWR, 0, 512 + (w1_final >> 20));
    IOWR(CSWR, 1, 512 + (w2_final >> 20));
    IOWR(CSWR, 2, 512 + (w3_final >> 20));
}

void paremeter_group_add()
{
    paremeter_group_flag = (paremeter_group_flag + 1) % PAREMETER_GROUP_COUNT;
    paremeter_choose_flag = 0;
    paremeter_flag = paremeter_group_flag * 3 + paremeter_choose_flag;
    DispString5x8(8, 1, "      ", 1);
    DispString5x8(8, 30, "      ", 1);
}
void paremeter_group_sub()
{
    paremeter_group_flag = (paremeter_group_flag + PAREMETER_GROUP_COUNT - 1) % PAREMETER_GROUP_COUNT;
    paremeter_choose_flag = 0;
    paremeter_flag = paremeter_group_flag * 3 + paremeter_choose_flag;
    DispString5x8(8, 1, "      ", 1);
    DispString5x8(8, 30, "      ", 1);
}
void paremeter_choose_add()
{
    paremeter_choose_flag = (paremeter_choose_flag + 1) % 3;
    paremeter_flag = paremeter_group_flag * 3 + paremeter_choose_flag;
    DispString5x8(8, 1, "      ", 1);
    DispString5x8(8, 30, "      ", 1);
}
void paremeter_choose_sub()
{
    paremeter_choose_flag = (paremeter_choose_flag + 2) % 3;
    paremeter_flag = paremeter_group_flag * 3 + paremeter_choose_flag;
    DispString5x8(8, 1, "      ", 1);
    DispString5x8(8, 30, "      ", 1);
}

void paremeter_add()
{
    switch (paremeter_group_flag)
    {
    case 0:
        // if(paremeter_choose_flag==2)*paremeter[paremeter_flag]+=100;
        // else
        *paremeter[paremeter_flag] += 0.1;
        break;
    case 1:
        //*paremeter[paremeter_flag]+=0.1;
        if (paremeter_choose_flag == 0)
            *paremeter[paremeter_flag] += 1;
        else if (paremeter_choose_flag == 1)
            *paremeter[paremeter_flag] += 0.001;
        break;
    case 2:
        if (paremeter_choose_flag == 0)
            *paremeter[paremeter_flag] += 1;
        else if (paremeter_choose_flag == 1)
            *paremeter[paremeter_flag] += 0.001;
        break;
    case 3:
        if (paremeter_choose_flag == 0)
            *paremeter[paremeter_flag] += 1;
        else if (paremeter_choose_flag == 1)
            *paremeter[paremeter_flag] += 100;
        else if (paremeter_choose_flag == 2)
            *paremeter[paremeter_flag] += 10;
        break;
    default:
        break;
    }
    change_IQ_par();
    //  float* paremeter[7]
    // write_float_to_flash(paremeter,FLASH_USER_DATA_ADDR,9);
    //    paremeter_to_int();
    //    write_int_to_flash(paremeter_int,FLASH_USER_DATA_ADDR,7);
}
void paremeter_sub()
{
    switch (paremeter_group_flag)
    {
    case 0:
        // if(paremeter_choose_flag==2)*paremeter[paremeter_flag]-=100;
        // else
        *paremeter[paremeter_flag] -= 0.1;
        break;
    case 1:
        if (paremeter_choose_flag == 0)
            *paremeter[paremeter_flag] -= 1;
        else if (paremeter_choose_flag == 1)
            *paremeter[paremeter_flag] -= 0.001;
        break;
    case 2:
        if (paremeter_choose_flag == 0)
            *paremeter[paremeter_flag] -= 1;
        else if (paremeter_choose_flag == 1)
            *paremeter[paremeter_flag] -= 0.001;
        break;
    case 3:
        if (paremeter_choose_flag == 0)
            *paremeter[paremeter_flag] -= 1;
        else if (paremeter_choose_flag == 1)
            *paremeter[paremeter_flag] -= 100;
        else if (paremeter_choose_flag == 2)
            *paremeter[paremeter_flag] -= 10;
        break;
    default:
        break;
    }
    change_IQ_par();
    // write_float_to_flash(paremeter,FLASH_USER_DATA_ADDR,9);
    //   paremeter_to_int();
    //   write_int_to_flash(paremeter_int,FLASH_USER_DATA_ADDR,7);
}
// float* paremeter[15] = {&uabc_set,&i_set,&k1, &fhz,&adj,&b1, &u1_kp,&u1_ki,&u1_kd, &u2_kp,&u2_ki,&u2_kd, &u3_kp,&u3_ki,&u3_kd};
void paremeter_display()
{
    switch (paremeter_flag)
    { // DispString5x8(4, 1, "u1:", 1);DispFloat5x8(4, 30, u1, 1, 2, 4);
    case 0:
        DispString5x8(8, 1, "Aset:", 1);
        DispFloat5x8(8, 30, alpha_set, 1, 2, 2);
        break;
    case 1:
        DispString5x8(8, 1, "pfset:", 1);
        DispFloat5x8(8, 30, pf_set, 1, 2, 2);
        break;
    case 2:
        DispString5x8(8, 1, "adj:", 1);
        DispFloat5x8(8, 30, ad_adj, 1, 2, 2);
        break;
    case 3:
        DispString5x8(8, 1, "d_kp:", 1);
        DispFloat5x8(8, 30, dq_kp, 1, 1, 4);
        break;
    case 4:
        DispString5x8(8, 1, "d_ki:", 1);
        DispFloat5x8(8, 30, dq_ki, 1, 1, 4);
        break;
    case 5:
        DispString5x8(8, 1, "d_kd:", 1);
        DispFloat5x8(8, 30, dq_kd, 1, 2, 1);
        break;
    case 6:
        DispString5x8(8, 1, "i_kp:", 1);
        DispFloat5x8(8, 30, i_kp, 1, 1, 4);
        break;
    case 7:
        DispString5x8(8, 1, "i_ki:", 1);
        DispFloat5x8(8, 30, i_ki, 1, 1, 4);
        break;
    case 8:
        DispString5x8(8, 1, "i_kd:", 1);
        DispFloat5x8(8, 30, i_kd, 1, 2, 1);
        break;
    case 9:
        DispString5x8(8, 1, "ubus:", 1);
        DispFloat5x8(8, 30, ubus_set, 1, 2, 1);
        break;
    case 10:
        DispString5x8(8, 1, "adj:", 1);
        DispFloat5x8(8, 30, adj, 1, 4, 1);
        break;
    case 11:
        DispString5x8(8, 1, "jd:", 1);
        DispFloat5x8(8, 30, fai, 1, 3, 1);
        break;
    default:
        break;
    }
}
void change_IQ_par() // 按键之后一些常数需要更新
{
    cons_pf_tan = _IQ20div((_IQ20(1) - _IQ20(pf_set) * _IQ20(pf_set)), _IQ20(pf_set));

    change_pid_par(&d_loop, _IQ20(dq_kp), _IQ20(dq_ki), _IQ20(dq_kd));
    change_pid_par(&q_loop, _IQ20(dq_kp), _IQ20(dq_ki), _IQ20(dq_kd));
    change_pid_par(&i_loop, _IQ20(i_kp), _IQ20(i_ki), _IQ20(i_kd));
    cons_bus_ref = _IQ15(ubus_set);
    cons_i_set = _IQ20(alpha_set);
}
void ReadKey()
{
    a_last = a;
    keydat = IORD(CS0, 0);
    SysCtlDelay(SysCtlClockGet() / 35);
    if (keydat = IORD(CS0, 0))
        ;
    else
        keydat = 0;
    switch (keydat)
    {
    case 0x11:
        a = 16;
        break;
    case 0x21:
        a = 15;
        break;
    case 0x41:
        a = 14;
        break;
    case 0x81:
        a = 13;
        break;
    case 0x12:
        a = 12;
        break;
    case 0x22:
        a = 11;
        break;
    case 0x42:
        a = 10;
        break;
    case 0x82:
        a = 9;
        break;
    case 0x14:
        a = 8;
        break;
    case 0x24:
        a = 7;
        break;
    case 0x44:
        a = 6;
        break;
    case 0x84:
        a = 5;
        break;
    case 0x18:
        a = 4;
        break;
    case 0x28:
        a = 3;
        break;
    case 0x48:
        a = 2;
        break;
    case 136:
        a = 1;
        break;
    case 0:
        a = 0;
        break;
    default:
        break;
    }
}
