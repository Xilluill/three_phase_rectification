/*
 * bus_fpga.h
 *
 *  Created on: 2019年5月24日
 *      Author: 91375
 */
#ifndef TARGET_IS_TM4C123_RA1
#define TARGET_IS_TM4C123_RA1
#endif
#include <stdint.h>
#include <stdbool.h>


#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"

#include "driverlib/rom.h"

#ifndef BUS_FPGA_H_
#define BUS_FPGA_H_


#define SET_WR      ROM_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_1, GPIO_PIN_1)
#define CLR_WR      ROM_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_1, 0)
#define SET_RD      ROM_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_2, GPIO_PIN_2)
#define CLR_RD      ROM_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_2, 0)


//一共支持15个模块。
#define CS0     0X00
#define CS1     0X01
#define CS2     0X02
#define CS3     0X03
#define CS4     0X04
#define CS5     0X05
#define CS6     0X06
#define CS7     0X07
#define CS8     0X08
#define CS9     0X09
#define CS10    0X0A
#define CS11    0X0B
#define CS12    0X0C
#define CS13    0X0D
#define CSWR    0X0E
#define CS_ADDRH16_REG  0X0F
void BUS_Init();
void IOWR(unsigned char BASE,unsigned char OFFSET,unsigned int DATA);
unsigned int IORD(unsigned char BASE,unsigned char OFFSET);

#endif

