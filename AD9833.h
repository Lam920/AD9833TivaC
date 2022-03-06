/*
 * AD9833.h
 *
 *  Created on: Dec 30, 2021
 *      Author: Admin
 */

#ifndef AD9833_AD9833_H_
#define AD9833_AD9833_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/can.h"
#include "driverlib/eeprom.h"
//#include "driverlib/i2c.h
#include "driverlib/lcd.h"
#include "driverlib/mpu.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"
#include "driverlib/adc.h"
#include "driverlib/interrupt.h"
#include "driverlib/qei.h"
#include "driverlib/fpu.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "inc/hw_ints.h"

#include "driverlib/ssi.h"

#define FMCLK 25000000        // Master Clock On AD9833


#define AD9833PORT						GPIO_PORTD_BASE
#define AD9833SS						GPIO_PIN_1


enum WaveType{SIN, SQR, TRI}; // Wave Selection Enum

// ------------------ Functions  ---------------------
void SPI_init(void);
void AD9833_SetWave(uint16_t Wave);
void AD9833_selectFSY1();// Sets Output Wave Type
void AD9833_deselectFSY1();
void AD9833_SetWaveData(float Frequency,float Phase);    // Sets Wave Frequency & Phase
void AD9833_Init(uint16_t Wave,float FRQ,float Phase);   // Initializing AD9833
#endif /* AD9833_AD9833_H_ */
