#ifndef ADS8688_TIVA_FPGA_H_
#define ADS8688_TIVA_FPGA_H_

#define MEASURING_RANGE     5.12    // ���̣�-MEASURING_RANGE ~ MEASURING_RANGE��MEASURING_RANGE����Ϊ2.56��5.12��10.24

double getVoltage(unsigned char BASE,unsigned char OFFSET);                                             // ��õ�ѹ��˲ʱֵ
double getVoltageRMS(unsigned char BASE, unsigned char OFFSET, unsigned char SAMPLE_NUM_OFFSET);        // ��õ�ѹ����Чֵ
double getVoltageMean(unsigned char BASE, unsigned char OFFSET, unsigned char CNT_MEAN_OFFSET);         // ��õ�ѹ��ƽ��ֵ


#endif
