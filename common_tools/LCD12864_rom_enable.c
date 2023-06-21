/*
 * LCD12864_rom_enable.c
 *
 *  Created on: 2018��8��14��
 *      Author: rebeater
 */
// x 1~8,y 1~128
#include"LCD12864_rom_enable.h"



const unsigned long long int numtab[] = { 1, 10, 100, 1000, 10000, 100000,
        1000000, 10000000, 100000000 };
const uint8_t linetab[8]={0,0x80,0xc0,0xe0,0xf0,0xf8,0xfc,0xfe};

//#include"pic.h"
/*
 * write command
 */
void lcd_write_com(uint8_t com)
{
    uint8_t i;
    LCD12864_CLR_CS;//CS=0;
    LCD12864_CLR_RS;//RS=0
//    __delay_cycles(10);
    for(i=0;i<8;i++)
    {
        LCD12864_CLR_SCLK;//SCLK=0;
//       __delay_cycles(10);
       if(com&0x80)
           LCD12864_SET_SDA;//SDA=1;
       else
           LCD12864_CLR_SDA;//SDA=0;
//       __delay_cycles(10);
       LCD12864_SET_SCLK;//SCLK=1;
//       __delay_cycles(10);
       com=com<<1;
    }
    LCD12864_SET_CS;//CS=1;
//    __delay_cycles(10);
}
/*
 * write data
 */
void lcd_write_data(uint8_t data)
{
    uint8_t i;
    LCD12864_CLR_CS;//CS=0;
    LCD12864_SET_RS;//RS=1;
//    __delay_cycles(10);
    for(i=0;i<8;i++)
    {
        LCD12864_CLR_SCLK;//SCLK=0;
//       __delay_cycles(10);
       if(data&0x80)
           LCD12864_SET_SDA;//SDA=1;
       else
           LCD12864_CLR_SDA;//SDA=0;
//       __delay_cycles(10);
       LCD12864_SET_SCLK;//SCLK=1;
//       __delay_cycles(10);
       data=data<<1;
    }
    LCD12864_SET_CS;//CS=1;
//    __delay_cycles(10);
}
/*
 * lcd12864 initialize
 */
void lcd12864_init(void)
{
    //ʹ������
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    //GPIO init
    ROM_GPIOPinTypeGPIOOutput(LCD12864_SCLK_PORT,LCD12864_SCLK_PIN);
    ROM_GPIOPinTypeGPIOOutput(LCD12864_SDA_PORT,LCD12864_SDA_PIN);
    ROM_GPIOPinTypeGPIOOutput(LCD12864_RS_PORT,LCD12864_RS_PIN);
    ROM_GPIOPinTypeGPIOOutput(LCD12864_CS_PORT,LCD12864_CS_PIN);
    ROM_GPIOPinTypeGPIOOutput(LCD12864_RST_PORT,LCD12864_RST_PIN);

    ROM_GPIOPinTypeGPIOInput(LCD12864_ROM_OUT_PORT,LCD12864_ROM_OUT_PIN);

    ROM_GPIOPadConfigSet(LCD12864_ROM_OUT_PORT, LCD12864_ROM_OUT_PIN,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    ROM_GPIOPinTypeGPIOOutput(LCD12864_ROM_IN_PORT,LCD12864_ROM_IN_PIN);
    ROM_GPIOPinTypeGPIOOutput(LCD12864_ROM_SCK_PORT,LCD12864_ROM_SCK_PIN);
    ROM_GPIOPinTypeGPIOOutput(LCD12864_ROM_CS_PORT,LCD12864_ROM_CS_PIN);

    LCD12864_CLR_RST;//RST=0;//�͵�ƽ��λ
    __delay_cycles(100);
    LCD12864_SET_RST;//RST=1;//�͵�ƽ��λ
    __delay_cycles(100);
    lcd_write_com(0xe2);
    __delay_cycles(100);
    lcd_write_com(0x2c);
    __delay_cycles(100);
    lcd_write_com(0x2e);
    __delay_cycles(100);
    lcd_write_com(0x2f);


    lcd_write_com(0x23); //�ֵ��Աȶȣ������÷�Χ0x20��0x27
    lcd_write_com(0x81); //΢���Աȶ�
    lcd_write_com(0x28); //΢���Աȶȵ�ֵ�������÷�Χ0x00��0x3f
    lcd_write_com(0xa2); //1/9 ƫѹ�ȣ�bias��
    lcd_write_com(0xc8); //��ɨ��˳�򣺴��ϵ���
    lcd_write_com(0xa0); //��ɨ��˳�򣺴�����
    lcd_write_com(0x40); //��ʼ�У���һ�п�ʼ
    lcd_write_com(0xaf); //����ʾ
//    DispClear();
}

static void lcd_address(uint8_t page,uint8_t column)
{
    column=column-0x01;
    lcd_write_com(0xb0+page-1); //����ҳ��ַ��ÿ8 ��Ϊһҳ��ȫ����64 �У����ֳ�8 ҳ
    lcd_write_com(0x10+(column>>4&0x0f)); //�����е�ַ�ĸ�4 λ
    lcd_write_com(column&0x0f); //�����е�ַ�ĵ�4 λ

}

//ȫ������
void DispClear()
{
    unsigned char i,j;
    for(i=0;i<9;i++)
    {
        lcd_write_com(0xb0+i);
        lcd_write_com(0x10);
        lcd_write_com(0x00);
        for(j=0;j<132;j++)
        {
          lcd_write_data(0x00);
        }
    }
}
//�ֲ�����
void DispClear_line(uint8_t x,uint8_t y,uint8_t len)//x 1~8,y 1~128 len+y<128
{
   unsigned char j;
   lcd_address(x,y);
   for(j=y;j<y+len;j++)
   {
       lcd_write_data(0x00);
   }
}
/*
 * display picture size:128*64
 */
void Display_128x64(uint8_t dp[1024])
{
    uint8_t i,j;
    for(j=0;j<8;j++)
    {
        lcd_address(j+1,1);
        for (i=0;i<128;i++)
        {
            lcd_write_data(*dp); //д���ݵ�LCD,ÿд��һ��8 λ�����ݺ��е�ַ�Զ���1
            dp++;
        }
    }
}
//��ʾ16x16 ����ͼ�񡢺��֡���Ƨ�ֻ�16x16 ���������ͼ��
void display_graphic_16x16(uint8_t page,uint8_t column,uint8_t *dp,uint8_t style)
{
    uint8_t i,j;

    for(j=0;j<2;j++)
    {
        lcd_address(page+j,column);
        for (i=0;i<16;i++)
        {
            if(style)
                lcd_write_data(*dp);
            else
                lcd_write_data(~(*dp));
            dp++;
        }
    }
}

//��ʾ8x16 ����ͼ��ASCII, ��8x16 ����������ַ�������ͼ��
void display_graphic_8x16(uint8_t page,uint8_t column,uint8_t *dp,uint8_t style)
{
    uint8_t i,j;
    for(j=0;j<2;j++)
    {
        lcd_address(page+j,column);
        for (i=0;i<8;i++)
        {
            if(style)
                lcd_write_data(*dp);
            else
                lcd_write_data(~(*dp));
            dp++;
        }
    }
}
//��ʾ5X8 ����ͼ��ASCII, ��5x8 ����������ַ�������ͼ��
void display_graphic_5x8(uint8_t page,uint8_t column,uint8_t *dp,uint8_t style)
{
    uint8_t i;
    lcd_address(page,column);
    for (i=0;i<6;i++)
    {
        if(style)
            lcd_write_data(*dp);
        else
            lcd_write_data(~(*dp));
        dp++;
    }
}
// ����ָ��ֿ�IC
static void write_command_to_ROM(uint8_t com)
{
    uint8_t i;
    for(i=0;i<8;i++)
    {
        LCD12864_CLR_ROM_SCK;
       // __delay_cycles(10);
        if(com&0x80)
            LCD12864_SET_ROM_IN;
        else
            LCD12864_CLR_ROM_IN;
        com=com<<1;
        LCD12864_SET_ROM_SCK;

       // __delay_cycles(10);
    }
}
//��IC�ж�ȡ���ֻ����ַ����� 1byte
static uint8_t get_data_form_ROM()
{
    uint8_t i;
   uint8_t data=0x00;
   for(i=0;i<8;i++)
   {
       LCD12864_CLR_ROM_SCK;
    //   __delay_cycles(1000);
       data=data<<1;
       if(LCD12864_GET_BIT_ROM_OUT)
           data|=0x01;
       LCD12864_SET_ROM_SCK;
     //  __delay_cycles(1000);
   }
   return data;
}
//��ָ����ַ��������д��Һ����ָ����page,column)������
void get_and_write_16x16(uint32_t font_addr,uint8_t page,uint8_t column,bool style)
{
    uint8_t i,j,disp_data;
    LCD12864_CLR_ROM_CS;
    write_command_to_ROM(0x03);
    write_command_to_ROM((font_addr&0xff0000)>>16);//��ַ�߰�λ
    write_command_to_ROM((font_addr&0xff00)>>8);//��ַ��8λ
    write_command_to_ROM(font_addr&0xff);//��8λ
    for(j=0;j<2;j++)
    {
        lcd_address(page+j,column);
        for(i=0;i<16;i++)
        {
            disp_data=get_data_form_ROM();
            lcd_write_data(style?disp_data:(~disp_data));
        }
    }
    LCD12864_SET_ROM_CS;
}
//��ָ����ַ��������д��Һ����ָ����page,column)������
void get_and_write_8x16(uint32_t font_addr,uint8_t page,uint8_t column,bool style)
{
    uint8_t i,j,disp_data;
    LCD12864_CLR_ROM_CS;
    write_command_to_ROM(0x03);
    write_command_to_ROM((font_addr&0xff0000)>>16);//��ַ�߰�λ
    write_command_to_ROM((font_addr&0xff00)>>8);//��ַ��8λ
    write_command_to_ROM(font_addr&0xff);//��8λ
    for(j=0;j<2;j++)
    {
        lcd_address(page+j,column);
        for(i=0;i<8;i++)
        {
            disp_data=get_data_form_ROM();
            lcd_write_data(style?disp_data:(~disp_data));
        }
    }
    LCD12864_SET_ROM_CS;
}
//��ָ����ַ��������д��Һ����ָ����page,column)������
void get_and_write_5x8(uint32_t font_addr,uint8_t page,uint8_t column,bool style)
{
    uint8_t i,disp_data;
    LCD12864_CLR_ROM_CS;
    write_command_to_ROM(0x03);
    write_command_to_ROM((font_addr&0xff0000)>>16);//��ַ�߰�λ
    write_command_to_ROM((font_addr&0xff00)>>8);//��ַ��8λ
    write_command_to_ROM(font_addr&0xff);//��8λ
    lcd_address(page,column);
    for(i=0;i<5;i++)
    {
        disp_data=get_data_form_ROM();
        lcd_write_data(style?disp_data:(~disp_data));
    }

    LCD12864_SET_ROM_CS;
}

void Disp_GB2312_string_16x16(uint8_t page,uint8_t column,uint8_t *text,bool style)
{
    uint8_t i=0;
    uint32_t font_addr;
    while(text[i]>0x00)
    {
        if(((text[i]>=0xb0) &&(text[i]<=0xf7))&&(text[i+1]>=0xa1))
        {
            //������壨GB2312�������ھ���Ѷ�ֿ� IC �еĵ�ַ�����¹�ʽ�����㣺
            //Address = ((MSB - 0xB0) * 94 + (LSB - 0xA1)+ 846)*32+ BaseAdd;BaseAdd=0
            //���ڵ��� 8 λ��Ƭ���г˷�������⣬���Է�����ȡ��ַ
            font_addr = (text[i]- 0xb0)*94;
            font_addr += (text[i+1]-0xa1)+846;
            font_addr = (uint32_t)(font_addr*32);
            get_and_write_16x16(font_addr,page,column,style); //��ָ����ַ��������д��Һ����ָ����page,column)������
            i+=2;
            column+=16;
        }
        else if(((text[i]>=0xa1) &&(text[i]<=0xa3))&&(text[i+1]>=0xa1))
        {
        //������壨GB2312��15x16 ����ַ��ھ���Ѷ�ֿ� IC �еĵ�ַ�����¹�ʽ�����㣺
            //Address = ((MSB - 0xa1) * 94 + (LSB - 0xA1))*32+ BaseAdd;BaseAdd=0
            //���ڵ��� 8 λ��Ƭ���г˷�������⣬���Է�����ȡ��ַ
            font_addr = (text[i]- 0xa1)*94;
            font_addr += (text[i+1]-0xa1);
            font_addr = (uint32_t)(font_addr*32);
            get_and_write_16x16(font_addr,page,column, style); //��ָ����ַ��������д��Һ����ָ����page,column)������
            i+=2;
            column+=16;
        }
        else if((text[i]>=0x20) &&(text[i]<=0x7e))
        {
            font_addr = (text[i]- 0x20);
            font_addr = (unsigned long)(font_addr*16);
            font_addr = (unsigned long)(font_addr+0x3cf80);
            get_and_write_8x16(font_addr,page,column,style); //��ָ����ַ��������д��Һ����ָ����page,column)������
            i+=1; column+=8;
        }
        else
            i++;
    }
}
/*
 * ��ʾ�����ַ�
 */
void DispChar5x8(uint8_t x,uint8_t y,char ch,bool style)
{
    uint32_t font_addr;
    if(ch>=0x20&&ch<=0x7e)
    {
        font_addr=(ch-0x20);
        font_addr=font_addr<<3;
        font_addr=font_addr+0x3bfc0;

        get_and_write_5x8(font_addr,x,y,style);
    }

}
/*
 * ��ʾ�ַ���
 */
void DispString5x8(uint8_t x,uint8_t y,char *str,bool style)
{
    uint8_t i=0;
    uint32_t font_addr;
    while(str[i]>0x00)
    {
        if(str[i]>=0x20&&str[i]<=0x7e)
        {
            font_addr=(str[i]-0x20);
            font_addr=font_addr<<3;
            font_addr=font_addr+0x3bfc0;

            get_and_write_5x8(font_addr,x,y,style);

            i++;
            y+=6;
        }
    }
}
/*
 * ��ʾ�ⲿ�����ַ�
 */
void DispExtern16x16(uint8_t x,uint8_t y,uint8_t *str,bool style)
{
    display_graphic_16x16(x,y,str,style);
}

/*
 * ��ʾ���� ����
 */
void DispNumber5x8(uint8_t page,uint8_t column,uint32_t data,uint8_t len,bool style)
{
    uint8_t i,j=0;
    uint8_t ch;
    uint32_t font_addr;
     uint8_t num[10]={0};//�����ʾ2^32��10λ��
     while(j<len)
     {
         num[j]=data%10+0x30;
         data=data/10;
         j++;
     }
     if(len==0)
     {
      ch=0+0x30;
      if(ch>=0x20&&ch<=0x7e)
        {
           font_addr=(ch-0x20);
           font_addr=font_addr<<3;
           font_addr=font_addr+0x3bfc0;//0x3b7c0ϸ�� 3bfc0�ֵ�

           get_and_write_5x8(font_addr,page,column,style);
       }
     }
     for(i=0;i<len;i++)
     {
         ch=num[len-1-i];//+0x30;
         font_addr=(ch-0x20);
         font_addr=font_addr<<3;
         font_addr=font_addr+0x3bfc0;//0x3b7c0ϸ�� 3bfc0�ֵ�

         get_and_write_5x8(font_addr,page,column+6*i,style);

     }
}
void DispNumber8x16(uint8_t page,uint8_t column,uint32_t data,uint8_t len,bool style)
{
    uint8_t j=0,i;
    uint8_t ch;
    uint32_t font_addr;
     uint8_t num[10]={0};//�����ʾ2^32��10λ��
     while(j<len)
     {
         num[len]=data%10+0x30;
         data=data/10;
         j++;
     }
     if(len==0)
     {
      ch=0+0x30;
      font_addr = (ch- 0x20);
      font_addr = (unsigned long)(font_addr*16);
      font_addr = (unsigned long)(font_addr+0x3b7c0);//0x3b7c0ϸ�� 3bfc0�ֵ�
      get_and_write_8x16(font_addr,page,column,style);
     }
     for(i=0;i<len;i++)
     {
         ch=num[len-1-i];//+0x30;��������ΪTimesNewRoman
         font_addr = (ch- 0x20);
         font_addr = (unsigned long)(font_addr*16);
         font_addr = (unsigned long)(font_addr+0x3b7c0);//0x3b7c0ϸ�� 3bfc0�ֵ�
         get_and_write_8x16(font_addr,page,column+8*i,style);

     }
}
/*
 * ��ʾ���� ������
 */
/***************************************************************************************************��ʾfloat*/
void DispFloat8x16(uint8_t page,uint8_t column,float dat,bool style,uint8_t len1,uint8_t len2 )
{
    int dat1,dat2;
//    uint8_t len1=0;
    uint32_t font_addr;
    uint8_t ch;
    uint8_t i;
    uint8_t num[10]={0};//�����ʾ2^32��10λ��
    dat1=(int)dat;
//    uint32_t data=dat1;
//    while(data>0)//�����������ֳ���
//    {
//       // num[len1]=data%10+0x30;
//        data=data/10;
//        len1++;
//    }
    if(len1==0)
    {
        len1++;
        ch=0+0x30;
        font_addr = (ch- 0x20);
        font_addr = (unsigned long)(font_addr*16);
        font_addr = (unsigned long)(font_addr+0x3b7c0);//0x3b7c0ϸ�� 3bfc0�ֵ�
        get_and_write_8x16(font_addr,page,column,style);
    }
    else
    {
        DispNumber8x16(page,column,dat1,style,len1);
    }
    dat2=(int)(((dat-dat1)*numtab[len2])+0.5);//�޸�bug,����0.014��ʾΪ0.013
    Disp_GB2312_string_16x16(page,column+8*len1,".",style);//С����

    uint8_t len=0;
       while(len<=len2)
       {
           num[len]=dat2%10+0x30;
           dat2=dat2/10;
           len++;
       }
       for(i=0;i<len2;i++)
       {
           ch=num[len2-1-i];//+0x30;��������ΪTimesNewRoman
            font_addr = (ch- 0x20);
            font_addr = (unsigned long)(font_addr*16);
            font_addr = (unsigned long)(font_addr+0x3b7c0);//0x3b7c0ϸ�� 3bfc0�ֵ�
            get_and_write_8x16(font_addr,page,column+4+8*(i+len1),style);//4��С�����λ��
       }
  //  DispString8x16At(x,y+8*len1,".",style);
  //  DispNumber8x16(x,y+8*(len1+1),dat2,style);
}
void DispFloat5x8(uint8_t page,uint8_t column,float dat,bool style,uint8_t len1,uint8_t len2)
{
    int dat1,dat2,lower_flag=0;
    if(dat<0)
        {
            lower_flag=1;
            dat=-dat;
        }
//    uint8_t len1=0;
    uint32_t font_addr;
    uint8_t ch;
    uint8_t i;
    uint8_t num[10]={0};//�����ʾ2^32��10λ��
    dat1=(int)dat;
//    while(data>0)//�����������ֳ���
//    {
//       // num[len1]=data%10+0x30;
//        data=data/10;
//        len1++;
//    }
    dat2=(int)(((dat-dat1)*numtab[len2])+0.5);//�޸�bug�е�bug,����1.99(5)��ʾΪ1.00(0)
    if(dat2>=numtab[len2])
        dat1++;
    if(len1==0)
    {//0x3b7c0ϸ�� 3bfc0�ֵ�
        len1++;
        ch=0+0x30;
        font_addr=(ch-0x20);
        font_addr=font_addr<<3;
        font_addr=font_addr+0x3bfc0;
        get_and_write_5x8(font_addr,page,column,style);
    }
    else
    {
        if(lower_flag)DispString5x8(page,column,"-",style);
        else DispString5x8(page,column,"+",style);
        DispNumber5x8(page,column+6,dat1,len1,style);
    }
//    dat2=(int)(((dat-dat1)*numtab[len2])+0.5);//�޸�bug,����0.014��ʾΪ0.013
    DispString5x8(page,column+6*len1+6,".",style);//С����

    uint8_t len=0;
       while(len<=len2)
       {
           num[len]=dat2%10+0x30;
           dat2=dat2/10;
           len++;
       }
       for(i=0;i<len2;i++)
       {
           ch=num[len2-1-i];//+0x30;��������ΪTimesNewRoman
           font_addr=(ch-0x20);
           font_addr=font_addr<<3;
           font_addr=font_addr+0x3bfc0;
           get_and_write_5x8(font_addr,page,column+4+6*(i+len1+1),style);
       }
}
/*
 * ������ͼ
 */
void display_vertical_line( uint8_t y,uint8_t len,bool style)//style=1,��״ͼ������ ��״ͼ
{

    uint8_t column=0;
  //  lcd_address(1,1);

    uint8_t full=len/8;
    uint8_t unfull=len%8;
    for(column=0;column<8;column++)
    {
        if(column<(7-full))
        {
             lcd_address(column+1,y);
             lcd_write_data(0x00);
        }
        else if(column==(7-full))
            {
               lcd_address(column+1,y);
               if(style)
               lcd_write_data(linetab[unfull]);
               else
                lcd_write_data(0x01<<(7-unfull));
             }
        else
            {
               lcd_address(column+1,y);
              if(style)
                  {
                      lcd_write_data(0xff);
                  }
              else{
                  lcd_write_data(0x00);
              }
            }
    }
}

void DispFloat(int row,int col,float data,int len1,int len2)
{
    int dat1 = (int)data;
    data = data-dat1;
    int len2_tmp = len2;
    while(len2_tmp>0)
    {
        data = data*10;
        len2_tmp--;
    }
    int dat2 = (int)data;
    DispNumber5x8(row,col,dat1,len1,1);
    DispString5x8(row,col+len1*6,".",1);
    DispNumber5x8(row,col+(len1+1)*6,dat2,len2,1);
}
