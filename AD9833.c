/*
 * AD9833.c
 *
 *  Created on: Dec 30, 2021
 *      Author: Lam Bui 
 * 		Modify from STM32 library
 */

#include "AD9833.h"
// ------------------- Variables ----------------
uint16_t FRQLW = 0;    // MSB of Frequency Tuning Word
uint16_t FRQHW = 0;    // LSB of Frequency Tuning Word
uint32_t  phaseVal=0;  // Phase Tuning Value
uint8_t WKNOWN=0;      // Flag Variable
// -------------------------------- Functions --------------------------------

// ------------------------------------------------ Software SPI Function
void SPI_init(void)
	{
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);
	    SysCtlDelay(10);
	    GPIOPinConfigure(GPIO_PD0_SSI3CLK);
	    GPIOPinConfigure(GPIO_PD2_SSI3RX);
	    GPIOPinConfigure(GPIO_PD3_SSI3TX);

	    GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_0);

	    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1);
	    //GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_6);



	    GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);

	    GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_3 | GPIO_PIN_0, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);
	    GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);
	    //GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);
	    SysCtlDelay(10);
	   // SSIConfigSetExpClk(SSI3_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 1000000, 8);
	    SSIConfigSetExpClk(SSI3_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_2, SSI_MODE_MASTER, 20000000, 8);
	    SSIEnable(SSI3_BASE);

			    //HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);	//CS OFF
		}
void AD9833_selectFSY1()
{
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0);
}


void AD9833_deselectFSY1()
{
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1);
}
void writeSPI(uint16_t word) {
	AD9833_selectFSY1();
	//SPI_ENABLE();
	SSIDataPut(SSI3_BASE, (word>>8)&0xFF);
	while(SSIBusy(SSI3_BASE)){}
	SSIDataPut(SSI3_BASE, (word&0xFF));
	while(SSIBusy(SSI3_BASE)){}
	AD9833_deselectFSY1();

}

// ------------------------------------------------ Sets Output Wave Type
void AD9833_SetWave(uint16_t Wave){
	  switch(Wave){
	  case 0:
	    AD9833_selectFSY1();
		writeSPI(0x2000); // Value for Sinusoidal Wave
	    AD9833_deselectFSY1();
		WKNOWN=0;
		break;
	  case 1:
		AD9833_selectFSY1();
		writeSPI(0x2028); // Value for Square Wave
		AD9833_deselectFSY1();
		WKNOWN=1;
		break;
	  case 2:
		AD9833_selectFSY1();
		writeSPI(0x2002); // Value for Triangle Wave
		AD9833_deselectFSY1();
		WKNOWN=2;
		break;
	  default:
		break;
	  }
}

// ------------------------------------------------ Sets Wave Frequency & Phase (In Degree) In PHASE0 & FREQ0 Registers
void AD9833_SetWaveData(float Frequency,float Phase){
	SysCtlDelay(2000);
	 // ---------- Tuning Word for Phase ( 0 - 360 Degree )
	 if(Phase<0)Phase=0; // Changing Phase Value to Positive
	 if(Phase>360)Phase=360; // Maximum value For Phase (In Degree)
	 phaseVal  = ((int)(Phase*(4096/360)))|0XC000;  // 4096/360 = 11.37 change per Degree for Register And using 0xC000 which is Phase 0 Register Address

	 // ---------- Tuning word for Frequency
	long freq=0;
	freq=(int)(((Frequency*pow(2,28))/FMCLK)+1); // Tuning Word
	FRQHW=(int)((freq & 0xFFFC000) >> 14); // FREQ MSB
	FRQLW=(int)(freq & 0x3FFF);  // FREQ LSB
	FRQLW |= 0x4000;
	FRQHW |= 0x4000;
	 // ------------------------------------------------ Writing DATA

	AD9833_deselectFSY1();
	AD9833_selectFSY1();
	writeSPI(0x2100); // enable 16bit words and set reset bit
	writeSPI(FRQLW);
	writeSPI(FRQHW);
	writeSPI(phaseVal);
	writeSPI(0x2000); // clear reset bit
	SysCtlDelay(2000);
		
	AD9833_deselectFSY1();
	AD9833_SetWave(WKNOWN);
	SysCtlDelay(2000);
	return;
}

// ------------------------------------------------ Initializing AD9833
void AD9833_Init(uint16_t WaveType,float FRQ,float Phase){
//	HAL_GPIO_WritePin(AD9833PORT,AD9833DATA,GPIO_PIN_SET); // Set All SPI pings to High
//	HAL_GPIO_WritePin(AD9833PORT,AD9833SCK,GPIO_PIN_SET);  // Set All SPI pings to High
//	HAL_GPIO_WritePin(AD9833PORT,AD9833SS,GPIO_PIN_SET);   // Set All SPI pings to High
	AD9833_SetWave(WaveType);                              // Type Of Wave
	AD9833_SetWaveData(FRQ,Phase);                         // Frequency & Phase Set
	return;
}

