/*
  ******************************************************************************
  HIER BITTE NAMEN + MATRIKELNUMMER EINTRAGEN
	Andreas Bruestle, 11905168
	Oliver Woehrer, 11907563

	
	In diesem Projekt gilt:
	*=============================================================================
  *        SYSCLK(Hz)                             | 64 000 000
  *-----------------------------------------------------------------------------
  *        AHB Prescaler                          | 1
  *-----------------------------------------------------------------------------
  *        APB2 Prescaler                         | 1
  *-----------------------------------------------------------------------------
  *        APB1 Prescaler                         | 2
  *=============================================================================
  ******************************************************************************
*/
#include "stm32f30x_conf.h"
//Global Constants:
#define RES_BUFFER_LENGTH 20
#define TIM_PRE_SCALER 199
#define TIME_SLOTS 2052

//Image Constants:
//#define OFFSET 0
unsigned int OFFSET = 0;

#define RED_EVEN 0x00
#define RED_ODD 0x01
#define GREEN_EVEN 0x02
#define GREEN_ODD 0x03
#define BLUE_EVEN 0x04
#define BLUE_ODD 0x05

#define ROW0 (0*6)
#define ROW1 (1*6)
#define ROW2 (2*6)
#define ROW3 (3*6)
#define ROW4 (4*6)
#define ROW5 (5*6)
#define ROW6 (6*6)
#define ROW7 (7*6)
#define ROW8 (8*6)
#define ROW9 (9*6)
#define ROW10 (10*6)
#define ROW11 (11*6)
#define ROW12 (12*6)
#define ROW13 (13*6)
#define ROW14 (14*6)
#define ROW15 (15*6)
#define ROW16 (16*6)
#define ROW17 (17*6)
#define ROW18 (18*6)
#define ROW19 (19*6)
#define ROW20 (20*6)
#define ROW21 (21*6)
#define ROW22 (22*6)
#define ROW23 (23*6)
#define ROW24 (24*6)
#define ROW25 (25*6)
#define ROW26 (26*6)
#define ROW27 (27*6)
#define ROW28 (28*6)
#define ROW29 (29*6)

unsigned int step;
int oldADCRaw = 0;


void RCC_Config(void){
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE); 				// port A: NSS, UART
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE); 				// port B: SPI
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE); 				// port C: ADC
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);  				// enable TIM2 (from APB1 clock)
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); 				// enable USART2 (from APB1 clock)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE); 				// enable SPI (from APB2 clock)
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 				// enable TIM3 (from APB2 clock)
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE); 				// enable ADC12 (from AHB clock)
	RCC_ADCCLKConfig(RCC_ADC12PLLCLK_OFF); 								// use AHB clock signal for converting
}

void GPIO_Config(void){
	GPIO_InitTypeDef GPIO_InitStructure;

	//<<< GPIO (Strobe) >>>
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;						// pin mode "output"
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;						// output type "push-pull"
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;  							// OR operator to config both pins
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;					// no pull up/down resistor needed
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;					// standart, speed does not matter that much
	GPIO_Init(GPIOB, &GPIO_InitStructure); 								// initialize port B

	//<<< GPIO (UART) >>>
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;						// pin mode "alternative function"
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;						// output type "push-pull"
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; 				// OR operator to config both pins
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;					// no pull up/down resistor needed
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;					// standart, speed does not matter that much
	GPIO_Init(GPIOA, &GPIO_InitStructure); 								// initialize port A
	
	// Connect Pins to Alternate Function:
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_7); 				// -> Datasheet P.42, Table 14. "Alternate  Functions"
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_7); 				// -> AF Num. 7 (PA2=USART2_TX, PA3=USART_RX)

	//<<< GPIO (SPIO) >>>
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;						// pin mode "alternative function"
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;						// output type "push-pull"
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5; // OR operator to config both pins
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;						// no pull up/down resistor needed
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;					// standart, speed does not matter that much
	GPIO_Init(GPIOB, &GPIO_InitStructure);								// initialize port A
	
	// Initialize GPIO A:
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;  						// OR operator to config both pins
	GPIO_Init(GPIOA, &GPIO_InitStructure); 								// initialize port A
	
	// SPI Pins to Alternate Function:
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_5); 				// -> Datasheet P.42, Table 14. "Alternate  Functions"
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_5); 				// -> AF Num. 5 (PB3=SCLK, PB4=MISO, PB5=MOSI)
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_5);

	//<<< GPIO (ADC) >>>
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;						// pin mode "analog"
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;						// output type "push-pull"
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;  							// potentiometer on PC0
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;					// no pull up/down resistor needed
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;					// standart, speed does not matter that much
	GPIO_Init(GPIOC, &GPIO_InitStructure); 	 						 	// initialize port C


	// TIM3:
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;						// pin mode "alternate function"
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;						// output type "push-pull"
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;  							// 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;					// no pull up/down resistor needed
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;					// standart, speed does not matter that much
	GPIO_Init(GPIOB, &GPIO_InitStructure); 								// initialize port B
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_2); // alternate function
}

void TIM_Config(void){
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

	// Initialize Timer 2:
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
	TIM_TimeBaseInitStructure.TIM_Prescaler = 63999; // 64 MHz clock devided by 7999+1 -> 1kHz timer frequency
	TIM_TimeBaseInitStructure.TIM_Period = 999; // 999+1 counts -> 1 sec
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); // enable Interrupt on update

	// Initialize Timer 3:
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
	TIM_TimeBaseInitStructure.TIM_Prescaler = TIM_PRE_SCALER; // 64 MHz clock devided by 399+1 -> 160kHz timer frequency
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
  
	TIM_Cmd(TIM3, ENABLE); // enable counter
	TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE); // Enable the CC2 Interrupt Request 
}

void USART_Config(void){
	// Initialize USART:
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 9600;                                   	// Baudrate = 9600
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     // 8 bits
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                          // 1 stop bit
	USART_InitStructure.USART_Parity = USART_Parity_No;                             // no parity bit
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // no Hardware Flow Control
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;                	// init both RX and TX
	USART_Init(USART2, &USART_InitStructure);
	
  	// Enable USART:
	USART_Cmd(USART2, ENABLE);
	
	// Enable "Receive Not Empty" (RXNE) Interrupt:
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}

void SPI_Config(void){
		// Initialize SPI:
	SPI_InitTypeDef  SPI_InitStructure;
	SPI_I2S_DeInit(SPI1);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = 8;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;												// master mode
  SPI_Init(SPI1, &SPI_InitStructure);
	
	
	SPI_SSOutputCmd(SPI1, ENABLE);
	//SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, ENABLE); // enable the Tx buffer empty interrupt
	SPI_Cmd(SPI1, ENABLE);
}

void ADC_Config(void){
	ADC_CommonInitTypeDef    ADC_CommonInitStructure; // for all/both ADC peripherals
	ADC_InitTypeDef          ADC_InitStructure; // for individual converter
			
	// Initialize Common ADC:
	ADC_CommonStructInit(&ADC_CommonInitStructure); // fill default values of struct
	ADC_CommonInitStructure.ADC_Clock = ADC_Clock_SynClkModeDiv1; // use AHB clock speed devided by 1	
	ADC_CommonInit(ADC1, &ADC_CommonInitStructure);
		
	// Initialize ADC 1:
 	ADC_StructInit(&ADC_InitStructure); // fill default values of struct
	ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable; // set continous mode
	ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Enable; // set overrun mode to get latest result
	ADC_Init(ADC1, &ADC_InitStructure);

	// Configure Pin for ADC1 Input:
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 1, ADC_SampleTime_61Cycles5); // ADC1, Channel 6 for pin PC0, channel to be first in list, 61.5 cycles
	ADC_RegularChannelSequencerLengthConfig(ADC1,1); // set channel sequence length to one, because single convertion
    
	// Set Up Voltage Regulator and wait:
	ADC_VoltageRegulatorCmd(ADC1, ENABLE);
	
	// Enable Timer 2 and wait for Voltage Regulator Weak Up:
	TIM_Cmd(TIM2, ENABLE);
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update); // clear flag after starting timer
	while(TIM_GetFlagStatus(TIM2,TIM_IT_Update) == 0) {}; // wait for flag
	TIM_Cmd(TIM2, DISABLE); // disable again
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);	
	NVIC_ClearPendingIRQ(TIM2_IRQn);

	// Calibration Calls:
	ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1) != RESET );
	uint32_t calibration_value = ADC_GetCalibrationValue(ADC1); // original: calibration_value = ADC_GetCalibrationValue(ADC1);
	ADC_Cmd(ADC1, ENABLE);
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY));
	
	// Start Continous Conversion:
	ADC_StartConversion(ADC1);
}

void NVIC_Config(void){
	NVIC_InitTypeDef NVIC_InitStructure;

	// Set Number of Bits fot Priority Groups:
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	// UART2:
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;				// -> Reference Manual P.195, Table 35: "Vector Table"
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					// enable channel for interrupt
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; // lowest priority (highest value)
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			// set sub-priority to zero
	NVIC_Init(&NVIC_InitStructure);

	// TIM2:
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
  
	// SPI1:
	NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// TIM3:
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

void UARTPrint( char string[], int xvalue){
	unsigned int i = 0;
	while(string[i] != '\0') {
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE)!=SET) {}; // wait for possible transmission to complete
		if(string[i] == 'X') {
			USART_SendData(USART2, xvalue);
		} else {
			USART_SendData(USART2, string[i]);
		}
		i++;
	}
}

void setEngineSpeed(int speed) {
	GPIO_ResetBits(GPIOB, GPIO_Pin_1);
	SPI_SendData8(SPI1, 0x70|speed); // ((speed-0x36)<<2)
}

void setDemoMode(int mode) {
	GPIO_ResetBits(GPIOB, GPIO_Pin_1);
	SPI_SendData8(SPI1, 0x90|mode); //(recvd-0x30)
}

void setLeds(char leds1, char leds2, char leds3, char row) {
	GPIO_ResetBits(GPIOB, GPIO_Pin_1);
	SPI_SendData8(SPI1, leds1);
	SPI_SendData8(SPI1, leds2);
	SPI_SendData8(SPI1, leds3);
	SPI_SendData8(SPI1, row);
}

void displaySchweden(void){
	//row 0
	if (step == OFFSET+ROW4+RED_EVEN) setLeds(0xC0,0xC7,0xC0,0x10|RED_EVEN);
	else if (step == OFFSET+ROW6+RED_ODD) setLeds(0xC0,0xC7,0xC0,0x10|RED_ODD);
	else if (step == OFFSET+ROW8+GREEN_EVEN) setLeds(0xC0, 0xC7, 0xC0, 0x10|GREEN_EVEN);
	else if (step == OFFSET+ROW10+GREEN_ODD) setLeds(0xC0,0xC7,0xC0,0x10|GREEN_ODD);
	else if (step == OFFSET+ROW12+BLUE_EVEN) setLeds(0xFF, 0xF8, 0xFF, 0x10|BLUE_EVEN);
	else if (step == OFFSET+ROW14+BLUE_ODD) setLeds(0xFF,0xF8,0xFF,0x10|BLUE_ODD);
	
	//row 1
	else if (step == OFFSET+ROW5+RED_EVEN) setLeds(0xC0,0xC7,0xC0,0x10|RED_EVEN);
	else if (step == OFFSET+ROW7+RED_ODD) setLeds(0xC0,0xC7,0xC0,0x10|RED_ODD);
	else if (step == OFFSET+ROW9+GREEN_EVEN) setLeds(0xC0, 0xC7, 0xC0, 0x10|GREEN_EVEN);
	else if (step == OFFSET+ROW11+GREEN_ODD) setLeds(0xC0,0xC7,0xC0,0x10|GREEN_ODD);
	else if (step == OFFSET+ROW13+BLUE_EVEN) setLeds(0xFF, 0xF8, 0xFF, 0x10|BLUE_EVEN);
	else if (step == OFFSET+ROW15+BLUE_ODD) setLeds(0xFF,0xF8,0xFF,0x10|BLUE_ODD);
	
	//row 2
	else if (step == OFFSET+ROW6+RED_EVEN) setLeds(0xC0,0xC7,0xC0,0x10|RED_EVEN);
	else if (step == OFFSET+ROW8+RED_ODD) setLeds(0xC0,0xC7,0xC0,0x10|RED_ODD);
	else if (step == OFFSET+ROW10+GREEN_EVEN) setLeds(0xC0, 0xC7, 0xC0, 0x10|GREEN_EVEN);
	else if (step == OFFSET+ROW12+GREEN_ODD) setLeds(0xC0,0xC7,0xC0,0x10|GREEN_ODD);
	else if (step == OFFSET+ROW14+BLUE_EVEN) setLeds(0xFF, 0xF8, 0xFF, 0x10|BLUE_EVEN);
	else if (step == OFFSET+ROW16+BLUE_ODD) setLeds(0xFF,0xF8,0xFF,0x10|BLUE_ODD);
	
	//row 3
	else if (step == OFFSET+ROW7+RED_EVEN) setLeds(0xC0,0xC7,0xC0,0x10|RED_EVEN);
	else if (step == OFFSET+ROW9+RED_ODD) setLeds(0xC0,0xC7,0xC0,0x10|RED_ODD);
	else if (step == OFFSET+ROW11+GREEN_EVEN) setLeds(0xC0, 0xC7, 0xC0, 0x10|GREEN_EVEN);
	else if (step == OFFSET+ROW13+GREEN_ODD) setLeds(0xC0,0xC7,0xC0,0x10|GREEN_ODD);
	else if (step == OFFSET+ROW15+BLUE_EVEN) setLeds(0xFF, 0xF8, 0xFF, 0x10|BLUE_EVEN);
	else if (step == OFFSET+ROW17+BLUE_ODD) setLeds(0xFF,0xF8,0xFF,0x10|BLUE_ODD);
	
	//row 4
	else if (step == OFFSET+ROW8+RED_EVEN) setLeds(0xC0,0xC7,0xC0,0x10|RED_EVEN);
	else if (step == OFFSET+ROW10+RED_ODD) setLeds(0xC0,0xC7,0xC0,0x10|RED_ODD);
	else if (step == OFFSET+ROW12+GREEN_EVEN) setLeds(0xC0, 0xC7, 0xC0, 0x10|GREEN_EVEN);
	else if (step == OFFSET+ROW14+GREEN_ODD) setLeds(0xC0,0xC7,0xC0,0x10|GREEN_ODD);
	else if (step == OFFSET+ROW16+BLUE_EVEN) setLeds(0xFF, 0xF8, 0xFF, 0x10|BLUE_EVEN);
	else if (step == OFFSET+ROW18+BLUE_ODD) setLeds(0xFF,0xF8,0xFF,0x10|BLUE_ODD);
	
	//row 5 (yellow)
	else if (step == OFFSET+ROW9+RED_EVEN) setLeds(0xFF,0xFF,0xFF,0x10|RED_EVEN);
	else if (step == OFFSET+ROW11+RED_ODD) setLeds(0xFF,0xFF,0xFF,0x10|RED_ODD);
	else if (step == OFFSET+ROW13+GREEN_EVEN) setLeds(0xFF, 0xFF, 0xFF, 0x10|GREEN_EVEN);
	else if (step == OFFSET+ROW15+GREEN_ODD) setLeds(0xFF,0xFF,0xFF,0x10|GREEN_ODD);
	else if (step == OFFSET+ROW17+BLUE_EVEN) setLeds(0xC0, 0xC0, 0xC0, 0x10|BLUE_EVEN); 
	else if (step == OFFSET+ROW19+BLUE_ODD) setLeds(0xC0,0xC0,0xC0,0x10|BLUE_ODD);
	
	//row 6 (yellow)
	else if (step == OFFSET+ROW10+RED_EVEN) setLeds(0xFF,0xFF,0xFF,0x10|RED_EVEN);
	else if (step == OFFSET+ROW12+RED_ODD) setLeds(0xFF,0xFF,0xFF,0x10|RED_ODD);
	else if (step == OFFSET+ROW14+GREEN_EVEN) setLeds(0xFF, 0xFF, 0xFF, 0x10|GREEN_EVEN);
	else if (step == OFFSET+ROW16+GREEN_ODD) setLeds(0xFF,0xFF,0xFF,0x10|GREEN_ODD);
	else if (step == OFFSET+ROW18+BLUE_EVEN) setLeds(0xC0, 0xC0, 0xC0, 0x10|BLUE_EVEN); 
	else if (step == OFFSET+ROW20+BLUE_ODD) setLeds(0xC0,0xC0,0xC0,0x10|BLUE_ODD);
	
	//row 7 (yellow)
	else if (step == OFFSET+ROW11+RED_EVEN) setLeds(0xFF,0xFF,0xFF,0x10|RED_EVEN);
	else if (step == OFFSET+ROW13+RED_ODD) setLeds(0xFF,0xFF,0xFF,0x10|RED_ODD);
	else if (step == OFFSET+ROW15+GREEN_EVEN) setLeds(0xFF, 0xFF, 0xFF, 0x10|GREEN_EVEN);
	else if (step == OFFSET+ROW17+GREEN_ODD) setLeds(0xFF,0xFF,0xFF,0x10|GREEN_ODD);
	else if (step == OFFSET+ROW19+BLUE_EVEN) setLeds(0xC0, 0xC0, 0xC0, 0x10|BLUE_EVEN); 
	else if (step == OFFSET+ROW21+BLUE_ODD) setLeds(0xC0,0xC0,0xC0,0x10|BLUE_ODD);
	
	//row 8
	else if (step == OFFSET+ROW12+RED_EVEN) setLeds(0xC0,0xC7,0xC0,0x10|RED_EVEN);
	else if (step == OFFSET+ROW14+RED_ODD) setLeds(0xC0,0xC7,0xC0,0x10|RED_ODD);
	else if (step == OFFSET+ROW16+GREEN_EVEN) setLeds(0xC0, 0xC7, 0xC0, 0x10|GREEN_EVEN);
	else if (step == OFFSET+ROW18+GREEN_ODD) setLeds(0xC0,0xC7,0xC0,0x10|GREEN_ODD);
	else if (step == OFFSET+ROW20+BLUE_EVEN) setLeds(0xFF, 0xF8, 0xFF, 0x10|BLUE_EVEN);
	else if (step == OFFSET+ROW22+BLUE_ODD) setLeds(0xFF,0xF8,0xFF,0x10|BLUE_ODD);
	
	//row 9
	else if (step == OFFSET+ROW13+RED_EVEN) setLeds(0xC0,0xC7,0xC0,0x10|RED_EVEN);
	else if (step == OFFSET+ROW15+RED_ODD) setLeds(0xC0,0xC7,0xC0,0x10|RED_ODD);
	else if (step == OFFSET+ROW17+GREEN_EVEN) setLeds(0xC0, 0xC7, 0xC0, 0x10|GREEN_EVEN);
	else if (step == OFFSET+ROW19+GREEN_ODD) setLeds(0xC0,0xC7,0xC0,0x10|GREEN_ODD);
	else if (step == OFFSET+ROW21+BLUE_EVEN) setLeds(0xFF, 0xF8, 0xFF, 0x10|BLUE_EVEN);
	else if (step == OFFSET+ROW23+BLUE_ODD) setLeds(0xFF,0xF8,0xFF,0x10|BLUE_ODD);
	
	//row 10
	else if (step == OFFSET+ROW14+RED_EVEN) setLeds(0xC0,0xC7,0xC0,0x10|RED_EVEN);
	else if (step == OFFSET+ROW16+RED_ODD) setLeds(0xC0,0xC7,0xC0,0x10|RED_ODD);
	else if (step == OFFSET+ROW18+GREEN_EVEN) setLeds(0xC0, 0xC7, 0xC0, 0x10|GREEN_EVEN);
	else if (step == OFFSET+ROW20+GREEN_ODD) setLeds(0xC0,0xC7,0xC0,0x10|GREEN_ODD);
	else if (step == OFFSET+ROW22+BLUE_EVEN) setLeds(0xFF, 0xF8, 0xFF, 0x10|BLUE_EVEN);
	else if (step == OFFSET+ROW24+BLUE_ODD) setLeds(0xFF,0xF8,0xFF,0x10|BLUE_ODD);
	
	//row 11
	else if (step == OFFSET+ROW15+RED_EVEN) setLeds(0xC0,0xC7,0xC0,0x10|RED_EVEN);
	else if (step == OFFSET+ROW17+RED_ODD) setLeds(0xC0,0xC7,0xC0,0x10|RED_ODD);
	else if (step == OFFSET+ROW19+GREEN_EVEN) setLeds(0xC0, 0xC7, 0xC0, 0x10|GREEN_EVEN);
	else if (step == OFFSET+ROW21+GREEN_ODD) setLeds(0xC0,0xC7,0xC0,0x10|GREEN_ODD);
	else if (step == OFFSET+ROW23+BLUE_EVEN) setLeds(0xFF, 0xF8, 0xFF, 0x10|BLUE_EVEN);
	else if (step == OFFSET+ROW25+BLUE_ODD) setLeds(0xFF,0xF8,0xFF,0x10|BLUE_ODD);
	
}

void displayAustria(void){
	//row 0 (red)
	if (step == OFFSET+ROW0+RED_EVEN) setLeds(0xFF,0xFF,0xFF,0x10|RED_EVEN);
	else if (step == OFFSET+ROW2+RED_ODD) setLeds(0xFF,0xFF,0xFF,0x10|RED_ODD);
		
	//row 1 (red)
	else if (step == OFFSET+ROW1+RED_EVEN) setLeds(0xFF, 0xFF, 0xFF, 0x10|RED_EVEN);
	else if (step == OFFSET+ROW3+RED_ODD) setLeds(0xFF,0xFF,0xFF,0x10|RED_ODD);
	
	//row 2 (red)
	else if (step == OFFSET+ROW2+RED_EVEN) setLeds(0xFF, 0xFF, 0xFF, 0x10|RED_EVEN);
	else if (step == OFFSET+ROW4+RED_ODD) setLeds(0xFF,0xFF,0xFF,0x10|RED_ODD);
	
	//row 3 (red)
	else if (step == OFFSET+ROW3+RED_EVEN) setLeds(0xFF, 0xFF, 0xFF, 0x10|RED_EVEN);
	else if (step == OFFSET+ROW5+RED_ODD) setLeds(0xFF,0xFF,0xFF,0x10|RED_ODD);
	
	//row 4 (white)
	else if (step == OFFSET+ROW4+RED_EVEN) setLeds(0xFF, 0xFF, 0xFF, 0x10|RED_EVEN);
	else if (step == OFFSET+ROW6+RED_ODD) setLeds(0xFF,0xFF,0xFF,0x10|RED_ODD);
	else if (step == OFFSET+ROW8+GREEN_EVEN) setLeds(0xFF, 0xFF, 0xFF, 0x10|GREEN_EVEN);
	else if (step == OFFSET+ROW10+GREEN_ODD) setLeds(0xFF,0xFF,0xFF,0x10|GREEN_ODD);
	else if (step == OFFSET+ROW12+BLUE_EVEN) setLeds(0xFF, 0xFF, 0xFF, 0x10|BLUE_EVEN);
	else if (step == OFFSET+ROW14+BLUE_ODD) setLeds(0xFF,0xFF,0xFF,0x10|BLUE_ODD);
	
	//row 5 (white)
	else if (step == OFFSET+ROW5+RED_EVEN) setLeds(0xFF, 0xFF, 0xFF, 0x10|RED_EVEN);
	else if (step == OFFSET+ROW7+RED_ODD) setLeds(0xFF,0xFF,0xFF,0x10|RED_ODD);
	else if (step == OFFSET+ROW9+GREEN_EVEN) setLeds(0xFF, 0xFF, 0xFF, 0x10|GREEN_EVEN);
	else if (step == OFFSET+ROW11+GREEN_ODD) setLeds(0xFF,0xFF,0xFF,0x10|GREEN_ODD);
	else if (step == OFFSET+ROW13+BLUE_EVEN) setLeds(0xFF, 0xFF, 0xFF, 0x10|BLUE_EVEN);
	else if (step == OFFSET+ROW15+BLUE_ODD) setLeds(0xFF,0xFF,0xFF,0x10|BLUE_ODD);
	
	//row 6 (white)
	else if (step == OFFSET+ROW6+RED_EVEN) setLeds(0xFF, 0xFF, 0xFF, 0x10|RED_EVEN);
	else if (step == OFFSET+ROW8+RED_ODD) setLeds(0xFF,0xFF,0xFF,0x10|RED_ODD);
	else if (step == OFFSET+ROW10+GREEN_EVEN) setLeds(0xFF, 0xFF, 0xFF, 0x10|GREEN_EVEN);
	else if (step == OFFSET+ROW12+GREEN_ODD) setLeds(0xFF,0xFF,0xFF,0x10|GREEN_ODD);
	else if (step == OFFSET+ROW14+BLUE_EVEN) setLeds(0xFF, 0xFF, 0xFF, 0x10|BLUE_EVEN);
	else if (step == OFFSET+ROW16+BLUE_ODD) setLeds(0xFF,0xFF,0xFF,0x10|BLUE_ODD);
	
	//row 7 (white)
	else if (step == OFFSET+ROW7+RED_EVEN) setLeds(0xFF, 0xFF, 0xFF, 0x10|RED_EVEN);
	else if (step == OFFSET+ROW9+RED_ODD) setLeds(0xFF,0xFF,0xFF,0x10|RED_ODD);
	else if (step == OFFSET+ROW11+GREEN_EVEN) setLeds(0xFF, 0xFF, 0xFF, 0x10|GREEN_EVEN);
	else if (step == OFFSET+ROW13+GREEN_ODD) setLeds(0xFF,0xFF,0xFF,0x10|GREEN_ODD);
	else if (step == OFFSET+ROW15+BLUE_EVEN) setLeds(0xFF, 0xFF, 0xFF, 0x10|BLUE_EVEN);
	else if (step == OFFSET+ROW17+BLUE_ODD) setLeds(0xFF,0xFF,0xFF,0x10|BLUE_ODD);
	
	//row 8 (red)
	else if (step == OFFSET+ROW8+RED_EVEN) setLeds(0xFF, 0xFF, 0xFF, 0x10|RED_EVEN);
	else if (step == OFFSET+ROW10+RED_ODD) setLeds(0xFF,0xFF,0xFF,0x10|RED_ODD);
	
	//row 9 (red)
	else if (step == OFFSET+ROW9+RED_EVEN) setLeds(0xFF, 0xFF, 0xFF, 0x10|RED_EVEN);
	else if (step == OFFSET+ROW11+RED_ODD) setLeds(0xFF,0xFF,0xFF,0x10|RED_ODD);
	
	//row 10 (red)
	else if (step == OFFSET+ROW10+RED_EVEN) setLeds(0xFF, 0xFF, 0xFF, 0x10|RED_EVEN);
	else if (step == OFFSET+ROW12+RED_ODD) setLeds(0xFF,0xFF,0xFF,0x10|RED_ODD);
	
	//row 11 (red)
	else if (step == OFFSET+ROW11+RED_EVEN) setLeds(0xFF, 0xFF, 0xFF, 0x10|RED_EVEN);
	else if (step == OFFSET+ROW13+RED_ODD) setLeds(0xFF,0xFF,0xFF,0x10|RED_ODD);
}

void displayCzech(void) {
	//row 0
	if (step == OFFSET+ROW4+RED_EVEN) setLeds(0xFF,0xFF,0xFE,0x10|RED_EVEN);
	else if (step == OFFSET+ROW6+RED_ODD) setLeds(0xFF,0xFF,0xFE,0x10|RED_ODD);
	else if (step == OFFSET+ROW8+GREEN_EVEN) setLeds(0xFF, 0xFF, 0xFE, 0x10|GREEN_EVEN);
	else if (step == OFFSET+ROW10+GREEN_ODD) setLeds(0xFF,0xFF,0xFE,0x10|GREEN_ODD);
	else if (step == OFFSET+ROW12+BLUE_EVEN) setLeds(0xFF, 0xFF, 0xFF, 0x10|BLUE_EVEN);
	else if (step == OFFSET+ROW14+BLUE_ODD) setLeds(0xFF,0xFF,0xFF,0x10|BLUE_ODD);
	
	//row 1
	else if (step == OFFSET+ROW5+RED_EVEN) setLeds(0xFF,0xFF,0xFC,0x10|RED_EVEN);
	else if (step == OFFSET+ROW7+RED_ODD) setLeds(0xFF,0xFF,0xFC,0x10|RED_ODD);
	else if (step == OFFSET+ROW9+GREEN_EVEN) setLeds(0xFF, 0xFF, 0xFC, 0x10|GREEN_EVEN);
	else if (step == OFFSET+ROW11+GREEN_ODD) setLeds(0xFF,0xFF,0xFC,0x10|GREEN_ODD);
	else if (step == OFFSET+ROW13+BLUE_EVEN) setLeds(0xFF, 0xFF, 0xFF, 0x10|BLUE_EVEN);
	else if (step == OFFSET+ROW15+BLUE_ODD) setLeds(0xFF,0xFF,0xFF,0x10|BLUE_ODD);
	
	//row 3
	else if (step == OFFSET+ROW6+RED_EVEN) setLeds(0xFF,0xFF,0xF8,0x10|RED_EVEN);
	else if (step == OFFSET+ROW8+RED_ODD) setLeds(0xFF,0xFF,0xF8,0x10|RED_ODD);
	else if (step == OFFSET+ROW10+GREEN_EVEN) setLeds(0xFF, 0xFF, 0xF8, 0x10|GREEN_EVEN);
	else if (step == OFFSET+ROW12+GREEN_ODD) setLeds(0xFF,0xFF,0xF8,0x10|GREEN_ODD);
	else if (step == OFFSET+ROW14+BLUE_EVEN) setLeds(0xFF, 0xFF, 0xFF, 0x10|BLUE_EVEN);
	else if (step == OFFSET+ROW16+BLUE_ODD) setLeds(0xFF,0xFF,0xFF,0x10|BLUE_ODD);
	
	//row 4
	else if (step == OFFSET+ROW7+RED_EVEN) setLeds(0xFF,0xFF,0xF0,0x10|RED_EVEN);
	else if (step == OFFSET+ROW9+RED_ODD) setLeds(0xFF,0xFF,0xF0,0x10|RED_ODD);
	else if (step == OFFSET+ROW11+GREEN_EVEN) setLeds(0xFF, 0xFF, 0xF0, 0x10|GREEN_EVEN);
	else if (step == OFFSET+ROW13+GREEN_ODD) setLeds(0xFF,0xFF,0xF0,0x10|GREEN_ODD);
	else if (step == OFFSET+ROW15+BLUE_EVEN) setLeds(0xFF, 0xFF, 0xFF, 0x10|BLUE_EVEN);
	else if (step == OFFSET+ROW17+BLUE_ODD) setLeds(0xFF,0xFF,0xFF,0x10|BLUE_ODD);
	
	//row 5
	else if (step == OFFSET+ROW7+RED_EVEN) setLeds(0xFF,0xFF,0xE0,0x10|RED_EVEN);
	else if (step == OFFSET+ROW9+RED_ODD) setLeds(0xFF,0xFF,0xE0,0x10|RED_ODD);
	else if (step == OFFSET+ROW11+GREEN_EVEN) setLeds(0xFF, 0xFF, 0xE0, 0x10|GREEN_EVEN);
	else if (step == OFFSET+ROW13+GREEN_ODD) setLeds(0xFF,0xFF,0xE0,0x10|GREEN_ODD);
	else if (step == OFFSET+ROW15+BLUE_EVEN) setLeds(0xFF, 0xFF, 0xFF, 0x10|BLUE_EVEN);
	else if (step == OFFSET+ROW17+BLUE_ODD) setLeds(0xFF,0xFF,0xFF,0x10|BLUE_ODD);
	
	//row 6
	else if (step == OFFSET+ROW8+RED_EVEN) setLeds(0xFF,0xFF,0xE0,0x10|RED_EVEN);
	else if (step == OFFSET+ROW10+RED_ODD) setLeds(0xFF,0xFF,0xE0,0x10|RED_ODD);
	else if (step == OFFSET+ROW12+GREEN_EVEN) setLeds(0xC0, 0xC0, 0xC0, 0x10|GREEN_EVEN);
	else if (step == OFFSET+ROW14+GREEN_ODD) setLeds(0xC0,0xC0,0xC0,0x10|GREEN_ODD);
	else if (step == OFFSET+ROW16+BLUE_EVEN) setLeds(0xC0, 0xC0, 0xDF, 0x10|BLUE_EVEN);
	else if (step == OFFSET+ROW18+BLUE_ODD) setLeds(0xC0,0xC0,0xDF,0x10|BLUE_ODD);
	
	//row 7
	else if (step == OFFSET+ROW9+RED_EVEN) setLeds(0xFF,0xFF,0xF0,0x10|RED_EVEN);
	else if (step == OFFSET+ROW11+RED_ODD) setLeds(0xFF,0xFF,0xF0,0x10|RED_ODD);
	else if (step == OFFSET+ROW13+GREEN_EVEN) setLeds(0xC0, 0xC0, 0xC0, 0x10|GREEN_EVEN);
	else if (step == OFFSET+ROW15+GREEN_ODD) setLeds(0xC0,0xC0,0xC0,0x10|GREEN_ODD);
	else if (step == OFFSET+ROW17+BLUE_EVEN) setLeds(0xC0, 0xC0, 0xCF, 0x10|BLUE_EVEN);
	else if (step == OFFSET+ROW19+BLUE_ODD) setLeds(0xC0,0xC0,0xCF,0x10|BLUE_ODD);
	
	//row 7
	else if (step == OFFSET+ROW10+RED_EVEN) setLeds(0xFF,0xFF,0xF8,0x10|RED_EVEN);
	else if (step == OFFSET+ROW12+RED_ODD) setLeds(0xFF,0xFF,0xF8,0x10|RED_ODD);
	else if (step == OFFSET+ROW14+GREEN_EVEN) setLeds(0xC0, 0xC0, 0xC0, 0x10|GREEN_EVEN);
	else if (step == OFFSET+ROW16+GREEN_ODD) setLeds(0xC0,0xC0,0xC0,0x10|GREEN_ODD);
	else if (step == OFFSET+ROW18+BLUE_EVEN) setLeds(0xC0, 0xC0, 0xC7, 0x10|BLUE_EVEN);
	else if (step == OFFSET+ROW20+BLUE_ODD) setLeds(0xC0,0xC0,0xC7,0x10|BLUE_ODD);
	
	//row 8
	else if (step == OFFSET+ROW11+RED_EVEN) setLeds(0xFF,0xFF,0xFC,0x10|RED_EVEN);
	else if (step == OFFSET+ROW13+RED_ODD) setLeds(0xFF,0xFF,0xFC,0x10|RED_ODD);
	else if (step == OFFSET+ROW15+GREEN_EVEN) setLeds(0xC0, 0xC0, 0xC0, 0x10|GREEN_EVEN);
	else if (step == OFFSET+ROW17+GREEN_ODD) setLeds(0xC0,0xC0,0xC0,0x10|GREEN_ODD);
	else if (step == OFFSET+ROW19+BLUE_EVEN) setLeds(0xC0, 0xC0, 0xC3, 0x10|BLUE_EVEN);
	else if (step == OFFSET+ROW21+BLUE_ODD) setLeds(0xC0,0xC0,0xC3,0x10|BLUE_ODD);
	
	//row 9
	else if (step == OFFSET+ROW12+RED_EVEN) setLeds(0xFF,0xFF,0xFE,0x10|RED_EVEN);
	else if (step == OFFSET+ROW14+RED_ODD) setLeds(0xFF,0xFF,0xFE,0x10|RED_ODD);
	else if (step == OFFSET+ROW16+GREEN_EVEN) setLeds(0xC0, 0xC0, 0xC0, 0x10|GREEN_EVEN);
	else if (step == OFFSET+ROW18+GREEN_ODD) setLeds(0xC0,0xC0,0xC0,0x10|GREEN_ODD);
	else if (step == OFFSET+ROW20+BLUE_EVEN) setLeds(0xC0, 0xC0, 0xC1, 0x10|BLUE_EVEN);
	else if (step == OFFSET+ROW22+BLUE_ODD) setLeds(0xC0,0xC0,0xC1,0x10|BLUE_ODD);
	
	//row 10
	else if (step == OFFSET+ROW12+RED_EVEN) setLeds(0xFF,0xFF,0xFF,0x10|RED_EVEN);
	else if (step == OFFSET+ROW14+RED_ODD) setLeds(0xFF,0xFF,0xFF,0x10|RED_ODD);
	else if (step == OFFSET+ROW16+GREEN_EVEN) setLeds(0xC0, 0xC0, 0xC0, 0x10|GREEN_EVEN);
	else if (step == OFFSET+ROW18+GREEN_ODD) setLeds(0xC0,0xC0,0xC0,0x10|GREEN_ODD);
	else if (step == OFFSET+ROW20+BLUE_EVEN) setLeds(0xC0, 0xC0, 0xC0, 0x10|BLUE_EVEN);
	else if (step == OFFSET+ROW22+BLUE_ODD) setLeds(0xC0,0xC0,0xC0,0x10|BLUE_ODD);
	
}

int main(void) {
	//<<< Config >>>
	RCC_Config();
	GPIO_Config();

	TIM_Config();
	USART_Config();
	SPI_Config();
	ADC_Config();
	
	NVIC_Config();
	
	GPIO_ResetBits(GPIOB, GPIO_Pin_1);
	__nop();
	GPIO_SetBits(GPIOB, GPIO_Pin_1);
	
	// Update Timer 2 Prescaler to match Timer 3:
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
	TIM_TimeBaseInitStructure.TIM_Prescaler = 0; // 64 MHz clock devided by 7999+1 -> 1kHz timer frequency
	//TIM_TimeBaseInitStructure.TIM_Period = 999; // 999+1 counts -> 1 sec
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); // enable Interrupt on update
	TIM_Cmd(TIM2, ENABLE);
	
	while(1) {
		
	}
}

void USART2_IRQHandler() {
	int recvd = USART_ReceiveData(USART2);
	if('a' == recvd) {
		int adcRaw = ADC_GetConversionValue(ADC1);
    double adcValue = ((double)adcRaw)/(4095) * 3.3; // 12 bit = 4096 steps, Vdd=3.3V
		int adcChar = (int) adcValue;
		UARTPrint("ADC Value XV\r\n", 0x30|adcChar);
			
	} else if('6' <= recvd && recvd <= '9') {
		UARTPrint("Engine Speed: X\r\n", recvd); // send serial response
		setEngineSpeed((recvd-0x36)<<2); // Send SPI Command to set speed
	} else if('0' <= recvd && recvd <= '5') {
		UARTPrint("Mode X selected\r\n",recvd); // send serial response
		setDemoMode(recvd-0x30); // Send SPI Command to set mode
	} else {
		UARTPrint("Invalid Input: X\r\n",recvd); // send serial response
	}
}

void TIM2_IRQHandler() {
	GPIO_SetBits(GPIOB, GPIO_Pin_1);
	step = (step+1) % 342; // step is reset every cycle
	
	//displaySchweden();
	displayAustria();
	//displayCzech();
	
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update); // clear pending bit manually
}

void SPI1_IRQHandler() {
	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, DISABLE);
	SPI_I2S_ClearFlag(SPI1, SPI_I2S_FLAG_TXE);
	//UARTPrint("SPI Sent!\r\n",0x30);
	//GPIO_SetBits(GPIOB, GPIO_Pin_1);
}

void TIM3_IRQHandler() {
	TIM_SetCounter(TIM3, 0);
	TIM_SetCounter(TIM2, 0);
	step = 0;
	
	int adcRaw = ADC_GetConversionValue(ADC1);
	OFFSET = (adcRaw*120) / 4096;
	
	
	int cntPerRound = TIM_GetCapture3(TIM3);
	int stepSize = (cntPerRound*(TIM_PRE_SCALER+1)) / TIME_SLOTS;
	
	TIM_SetAutoreload(TIM2, stepSize);
	TIM_GenerateEvent(TIM2,TIM_EventSource_Update);
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update); // clear pending bit manually
}

