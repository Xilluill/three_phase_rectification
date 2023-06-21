/*
 * @ ADS8688
 * @ 读取瞬时值、有效值、平均值
 * @ Yuyang
 * @ 2021.07.11
 */
#include "ADS8688_TIVA_FPGA.h"
#include "bus_fpga.h"
#include "common.h"

/*
 * 获得电压的瞬时值
 * 输入：BASE片选地址，OFFSET片内地址
 * 输出：电压的瞬时值
 */
double getVoltage(unsigned char BASE, unsigned char OFFSET)
{
    double proportion = MEASURING_RANGE * 2 / 65536;
    unsigned int volt16bit = 0;
    double voltage = 0;
    volt16bit = IORD(BASE, OFFSET);

    if (volt16bit > 0x8000)
        voltage = proportion * (volt16bit - 0x8000);
    else
        voltage = -proportion * (0x8000 - volt16bit);
    return voltage;

}

/*
 * 获得电压的有效值
 * 输入：BASE片选地址，OFFSET片内首地址，SAMPLE_NUM_OFFSET采样次数片内地址
 * 输出：电压的有效值
 */
double getVoltageRMS(unsigned char BASE, unsigned char OFFSET, unsigned char SAMPLE_NUM_OFFSET)
{
    double proportion = MEASURING_RANGE * 2 / 65536;
    long long int Power[3] = { 0 };
    double voltage_rms = 0;
    // 读取有效值数据，48位数据，分三次读取
    Power[2] = IORD(BASE, OFFSET);
    Power[1] = IORD(BASE, OFFSET + 1);
    Power[0] = IORD(BASE, OFFSET + 2);
    int sample_num = IORD(BASE, SAMPLE_NUM_OFFSET);

    Power[2] = Power[2] << 32;
    Power[1] = Power[1] << 16;
    voltage_rms = (sqrt(1.0*(Power[0] + Power[1] + Power[2]) / sample_num)) * proportion;     // 计算有效值

    return voltage_rms;

}

/*
 * 获得电压的均值
 * 输入：BASE片选地址，OFFSET片内首地址，CNT_MEAN_OFFSET采样次数片内地址
 * 输出：电压的平均值
 */
double getVoltageMean(unsigned char BASE, unsigned char OFFSET, unsigned char CNT_MEAN_OFFSET)
{
    double proportion = MEASURING_RANGE * 2 / 65536;
    long long int Mean[3] = { 0 };
    double voltage_mean = 0;
    // 读取平均值数据，48位数据，分三次读取
    Mean[2] = IORD(BASE, OFFSET);
    Mean[1] = IORD(BASE, OFFSET + 1);
    Mean[0] = IORD(BASE, OFFSET + 2);
    int cntMean = IORD(BASE, CNT_MEAN_OFFSET);

    Mean[2] = Mean[2] << 32;
    Mean[1] = Mean[1] << 16;
    voltage_mean = 1.0 * (Mean[0] + Mean[1] + Mean[2]) / cntMean  * proportion;     // 计算均值
    return voltage_mean;

}









