/*
 * blue.c
 *
 *  Created on: 2019年6月29日
 *      Author: SkingWei
 */
#include "stdlib.h"
#include "blue.h"
#include"LCD12864_rom_enable.h" //不用当注释
volatile float  *monitorData1;   //V
volatile float  *monitorData2;   //I
volatile float  *monitorData3;   //V
volatile float  *monitorData4;   //I
int L;
int debug_count=0;
int uart1_flag=0;
char charget='*';
char buffer1[512]="";
char buffer2[512]="";
char buffer3[512]="";
char buffer4[512]="";

unsigned long num_pow(unsigned char m,unsigned char n)
{
    unsigned long result=1;
    while(n--)result*=m;
    return result;
}

//浮点数转换成字符串,参数：生成的数组str,待转换的浮点数floa,整数长度为len1,小数长度为len2
void Float2Str(unsigned char str[],double floa,unsigned char len1,unsigned char len2)
{
    unsigned char i=0,j=0;
    unsigned int zhengshu = floa;
    unsigned int xiaoshu = (floa-zhengshu)*num_pow(10,len2);
    while(len1--)
    {
        str[len1] = zhengshu%10+0x30;
        zhengshu = zhengshu/10;
        i++;
    }
    str[i] = '.';
    while(len2--)
    {
        str[i+len2+1] = xiaoshu%10+0x30;
        xiaoshu = xiaoshu/10;
        j++;
    }
    str[i+j+1] = '\0';
}
void UART1IntHandler(void);
void initialize_uart()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
     //使能GPIO外设
     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
//     GPIO模式配置 PA0--RX PA1--TX
     GPIOPinConfigure(GPIO_PB0_U1RX);
     GPIOPinConfigure(GPIO_PB1_U1TX);

     //GPIO的UART模式配置
     GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
     //UART协议配置 波特率115200 8位 1停止位  无校验位

     UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200,
                 (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));
     //UART禁用FIFO 默认FIFO Level为4/8 寄存器满8字节后产生中断
     //禁用后接收1位就产生中断
//     UARTFIFODisable(UART0_BASE);
     UARTFIFODisable(UART1_BASE);
     //使能UART0中断
//     IntEnable(INT_UART0);
     IntEnable(INT_UART1);
     //使能UART0接收中断
     UARTIntEnable(UART1_BASE,UART_INT_RX);
     //UART中断地址注册
     UARTIntRegister(UART1_BASE,UART1IntHandler);
     //全局中断使能
     IntMasterEnable();
}

/*****************绑定要监视的数据***************************/
void blue1(volatile float  *sendData1){
    monitorData1 = sendData1;
}
void blue2(volatile float  *sendData2){
    monitorData2 = sendData2;
}
void blue3(volatile float  *sendData3){
    monitorData3= sendData3;
}
void blue4(volatile float  *sendData4){
    monitorData4 = sendData4;
}
void printCharFVT(unsigned char sendChar) {
    UARTCharPut(UART1_BASE, sendChar);
}
void printStringFVT(char *sendString) {

    //UARTIntDisable(UART1_BASE, UART_INT_RX | UART_INT_RT);      //暂时关闭中断
    while ((*sendString) != '\0')
    {
        UARTCharPut(UART1_BASE, *sendString);
        sendString++;
    }
    //UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);       //重新开始接收
}
void UARTStringPut(uint32_t ui32Base, unsigned char *p,unsigned char length)
{
    while(length--)
    {
        while(UARTBusy(ui32Base));
        UARTCharPut(ui32Base,*p++);
    }
}
/**************发送数据***************************/

void sentdata()
{
    Float2Str(buffer1,*monitorData1,2,6);
    printStringFVT(buffer1);
    printCharFVT('\n');

//    Float2Str(buffer2,*monitorData2,2,3);
//    printStringFVT(buffer2);
//    printCharFVT(' ');

    // Float2Str(buffer3,*monitorData3,3,0);
    // printStringFVT(buffer3);
    // printCharFVT(' ');
//    Float2Str(buffer1,*monitorData1,2,4);
//    printCharFVT('a')
//     printStringFVT(buffer1);

}

/***************接收字符串*************************/
//char *tempStr;
void getStringFVT() {

    charget=ROM_UARTCharGet(UART1_BASE);
    //接收并发送
//    printCharFVT(charget);
//    程序处理
    switch(charget)
    {
    case '0': DispChar5x8(1,1,charget,1);break;
    case '1': DispChar5x8(1,1,charget,1);break;
    case '2': DispChar5x8(1,1,charget,1);break;
    case '3': DispChar5x8(1,1,charget,1);break;
    case '4': DispChar5x8(1,1,charget,1);break;
    case '5': DispChar5x8(1,1,charget,1);break;
    case '6': DispChar5x8(1,1,charget,1);break;
    case '7': DispChar5x8(1,1,charget,1);break;
    case '8': DispChar5x8(1,1,charget,1);break;
    case '9': DispChar5x8(1,1,charget,1);break;
    case 'A': DispChar5x8(1,1,charget,1);break;
    case 'B': DispChar5x8(1,1,charget,1);break;
    case 'C': DispChar5x8(1,1,charget,1);break;
    case 'D': DispChar5x8(1,1,charget,1);break;
    case 'E': DispChar5x8(1,1,charget,1);break;
    case 'F': DispChar5x8(1,1,charget,1);break;
    default: DispString5x8(1,1,"w",1);break;

    }



}

//UART0中断函数
void UART1IntHandler(void)
{
    //获取中断标志 原始中断状态 不屏蔽中断标志
    //  uint32_t flag = UARTIntStatus(UART0_BASE,0);
    //获取中断标志 原始中断状态 屏蔽中断标志
//    UARTIntClear( UART1_BASE , UART_INT_OE );
//    UARTIntClear( UART1_BASE , UART_INT_RX );
//    UARTRxErrorClear( UART1_BASE );
    uint32_t flag = UARTIntStatus(UART1_BASE,1);
    //清除中断标志
    UARTIntClear(UART1_BASE,flag);
    if(flag&UART_INT_RX)
        //UARTCharsAvail()判断FIFO是否还有数据
        while(UARTCharsAvail(UART1_BASE))
        {
            UARTCharGetNonBlocking(UART1_BASE);
            sentdata();
           // UARTCharPutNonBlocking(UART1_BASE, UARTCharGetNonBlocking(UART1_BASE)); //echo character
            //getStringFVT();         //接收字符串
        }
}
