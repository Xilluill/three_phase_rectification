/*
 * common.h
 *
 *  Created on: 2019��6��16��
 *      Author: Skingwei
 */
//����tiva �⺯��
#ifndef COMMON_H_
#define COMMON_H_

#ifndef TARGET_IS_BLIZZARD_RA1
#define TARGET_IS_BLIZZARD_RA1
#endif

#ifndef  PART_TM4C123GH6PZ
#define  PART_TM4C123GH6PZ
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "math.h"
#include "string.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/adc.h"
#include "driverlib/sysctl.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/ssi.h"
#include "utils/uartstdio.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/fpu.h"
#include "driverlib/interrupt.h"
#include "driverlib/flash.h"
#include "IQmath/IQmathLib.h"

/********************��ʱ*******************************/
#define delay_ns(x) SysCtlDelay(x * (SysCtlClockGet() / 80000000));   //37.5ns  80MHz
#define delay_us(x) SysCtlDelay(x * (SysCtlClockGet() / 3000000));
#define delay_ms(x) SysCtlDelay(x * (SysCtlClockGet() / 3000));


#endif /* COMMON_H_ */
