/*
 * blue.h
 *
 *  Created on: 2019��6��29��
 *      Author: SkingWei
 */
#include "stdlib.h"
#include "common.h"
#ifndef SRC_BLUE_H_
#define SRC_BLUE_H_
//��ʼ������
void initialize_uart();
//��Ҫ���͵��ַ���
void blue1(volatile float *sendData1);
void blue2(volatile float *sendData1);
void blue3(volatile float *sendData1);
void blue4(volatile float *sendData1);
//����һ���ַ���
void printCharFVT(unsigned char sendChar);
void printStringFVT(char *sendString);
void f_char_printf(float Xangle);
void UARTStringPut(uint32_t ui32Base, unsigned char *p,unsigned char length);
void sentdata();
extern int uart1_flag;


#endif /* SRC_BLUE_H_ */
