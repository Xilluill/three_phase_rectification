/*
 * bus_fpga.c
 *
 *  Created on: 2019��6��29��
 *      Author: SkingWei
 */
#include"bus_fpga.h"
void BUS_Init()
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_1|GPIO_PIN_2);
    CLR_WR;
    CLR_RD;
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, 0x0f);
    ROM_GPIOPinWrite(GPIO_PORTE_BASE, 0x0f, 0x0f);

    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTH_BASE, 0x0f);
    ROM_GPIOPinWrite(GPIO_PORTH_BASE, 0x0f, 0x0f);

}

void IOWR(unsigned char BASE,unsigned char OFFSET,unsigned int DATA)
{
  unsigned char data_l=DATA; //111 1101 0000ת��Ϊʮ����Ϊ2000
  unsigned char data_h=DATA>>8;//
  ROM_GPIOPinWrite(GPIO_PORTE_BASE, 0x0f, BASE);
  ROM_GPIOPinWrite(GPIO_PORTH_BASE, 0x0f, OFFSET);
  
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, 0xff);
  ROM_GPIOPinWrite(GPIO_PORTF_BASE, 0xff, data_l);

  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, 0xff);
  ROM_GPIOPinWrite(GPIO_PORTD_BASE, 0xff, data_h);
  SET_WR;
  CLR_WR;
}

unsigned int IORD(unsigned char BASE,unsigned char OFFSET)
{
  unsigned int rddat_h8,rddat_l,rddat15_11,rddat10,rddat9_2,rddat;
  ROM_GPIOPinWrite(GPIO_PORTE_BASE, 0x0f, BASE);
  ROM_GPIOPinWrite(GPIO_PORTH_BASE, 0x0f, OFFSET);
  SET_RD;
  ROM_GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, 0xff);
  ROM_GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, 0xff);
  rddat_h8=ROM_GPIOPinRead(GPIO_PORTD_BASE, 0xff);
  rddat_l=ROM_GPIOPinRead(GPIO_PORTF_BASE, 0xff);
//  rddat15_11=rddat_h8>>3;
//  rddat10=(rddat>>1)%2;
//  rddat9_2=rddat_l;
//  rddat=(rddat15_11<<9)+(rddat10<<8)+rddat9_2;
  CLR_RD;
  return (rddat_h8<<8)+rddat_l;
};



