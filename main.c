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

#define RES_BUFFER_LENGTH (20)

void GPIO_Config(void){
	GPIO_InitTypeDef GPIO_InitStructure;

	//<<< GPIO (Strobe) >>>
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;						// pin mode "output"
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;						// output type "push-pull"
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;  							// OR operator to config both pins
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;					// no pull up/down resistor needed
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;					// standart, speed does not matter that much
	GPIO_Init(GPIOB, &GPIO_InitStructure); 								// initialize port B

	//<<< GPIO (Optical Sensor) >>>
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;						// pin mode "alternate function"
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;						// output type "push-pull"
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;  							// 
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
	GPIO_InitStr>ucture.GPIO_Mode = GPIO_Mode_AF;						// pin mode "alternative function"
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;						// output type "push-pull"
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5; // OR operator to config both pins
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;						// no pull up/down resistor needed
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;					// standart, speed does not matter that much
	GPIO_Init(GPIOB, &GPIO_InitStructure);								// initialize port A
	
	// Initialize GPIO A:
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;  						// OR operator to config both pins
	GPIO_Init(GPIOA, &GPIO_InitStructure); 								// initialize port A
	
	// Connect Pins to Alternate Function:
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


	//<<< GPIO (TIM1) >>>
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_6);
}

void RCC_Config(void){
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE); 				// port A: NSS, UART
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE); 				// port B: SPI
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE); 				// port C: ADC
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);  				// enable TIM2 (from APB1 clock)
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); 				// enable USART2 (from APB1 clock)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE); 				// enable SPI (from APB2 clock)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); 				// enable TIM1 (from APB2 clock)
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE); 				// enable ADC12 (from AHB clock)
	RCC_ADCCLKConfig(RCC_ADC12PLLCLK_OFF); 								// use AHB clock signal for converting
}

void NVIC_Config(void){
	NVIC_InitTypeDef NVIC_InitStructure;

	// Set Number of Bits fot Priority Groups:
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	//<<< NVIC (UART2) >>>
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;				// -> Reference Manual P.195, Table 35: "Vector Table"
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					// enable channel for interrupt
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;	// set priority to 3 (lowest)
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			// set sub-priority to zero
	NVIC_Init(&NVIC_InitStructure);

	//<<< NVIC (TIM2) >>>
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
  
	//<<< NVIC (SPI1) >>>
	NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//<<< NVIC (TIM1) >>>
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

void TIM_Config(void){
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

	// Initialize Timer 2:
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
	TIM_TimeBaseInitStructure.TIM_Prescaler = 63999; // 64 MHz clock devided by 7999+1 -> 1kHz timer frequency
	TIM_TimeBaseInitStructure.TIM_Period = 999; // 9999+1 counts -> 10 sec
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
	
	// Enable Timer 2:
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	// Enable Timer 2 and wait one cycle:
	TIM_Cmd(TIM2, ENABLE);
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update); // clear flag after starting timer
	while(TIM_GetFlagStatus(TIM2,TIM_IT_Update) == 0) {}; // wait for flag
	TIM_Cmd(TIM2, DISABLE); // disable again
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);	
	NVIC_ClearPendingIRQ(TIM2_IRQn);

	// Init Timer 1:
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;
	TIM_ICInit(TIM2, &TIM_ICInitStructure);
  
	/* TIM enable counter */
	TIM_Cmd(TIM2, ENABLE);

	/* Enable the CC2 Interrupt Request */
	TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
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
	uint32_t calibration_value = 0;
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

	// Calibration Calls:
	ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1) != RESET );
	calibration_value = ADC_GetCalibrationValue(ADC1); // original: calibration_value = ADC_GetCalibrationValue(ADC1);
	ADC_Cmd(ADC1, ENABLE);
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY));
	
	// Start Continous Conversion:
	ADC_StartConversion(ADC1);
}

void uart_print( char string[], int xvalue, int adc){
	unsigned int i = 0;
	while(string[i] != '\0') {
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE)!=SET) {}; // wait for possible transmission to complete
		if(string[i] == 'X') {
			if (adc == 1){
				USART_SendData(USART2, 0x30 | xvalue);
			} else {
				USART_SendData(USART2, xvalue);
			}

		} else {
			USART_SendData(USART2, string[i]);
		}
		i++;
	}
}

int main(void) {
	//<<< Config >>>
	GPIO_Config();
	RCC_Config();
	NVIC_Config();

	TIM_Config();
	USART_Config();
	SPI_Config();
	ADC_Config();
	
	while(1)
	{}

}

void USART2_IRQHandler() {
	int recvd = USART_ReceiveData(USART2);
	if('a' == recvd) {
		int adcRaw = ADC_GetConversionValue(ADC1);
    	double adcValue = ((double)adcRaw)/(4095) * 3.3; // 12 bit = 4096 steps, Vdd=3.3V
		int adcChar = (int) adcValue;

		//Send Serial Response:
		uart_print("ADC Value XV\r\n",adcChar,1);
			
	} else if('l' == recvd) {
		//Send Serial Response:
		uart_print("Led data: X\r\n", recvd, 0);

		for(unsigned int i=0; i<6; i++) {
			GPIO_ResetBits(GPIOB, GPIO_Pin_1);
			SPI_SendData8(SPI1, 0xFF);
			SPI_SendData8(SPI1, 0xFF);
			SPI_SendData8(SPI1, 0xFF);
			SPI_SendData8(SPI1, 0x10 | i);
			SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, ENABLE); // enable the Tx buffer empty interrupt
			while (SPI_GetTransmissionFIFOStatus(SPI1) != SPI_TransmissionFIFOStatus_Empty) {}; // Waiting until TX FIFO is empty
			while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET) {}; // Wait busy flag
		}	
	} else if('6' <= recvd && recvd <= '9') {
		//Send Serial Response:
		uart_print("Engine Speed: X\r\n", recvd, 0);
		
		//Send SPI Command:
		GPIO_ResetBits(GPIOB, GPIO_Pin_1);
    	SPI_SendData8(SPI1, 0x70 | ((recvd-0x36)<<2) );
		SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, ENABLE); // enable the Tx buffer empty interrupt
		while (SPI_GetTransmissionFIFOStatus(SPI1) != SPI_TransmissionFIFOStatus_Empty) {}; // Waiting until TX FIFO is empty
    	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET) {}; // Wait busy flag
		//SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, DISABLE);
		
	} else if('0' <= recvd && recvd <= '5') {
		//Send Serial Response:
		uart_print("Mode X selected\r\n",recvd,0);
		
		//Send SPI Command:
		GPIO_ResetBits(GPIOB, GPIO_Pin_1);
    	SPI_SendData8(SPI1, 0x90 | (recvd-0x30));
		SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, ENABLE); // enable the Tx buffer empty interrupt
		while (SPI_GetTransmissionFIFOStatus(SPI1) != SPI_TransmissionFIFOStatus_Empty) {}; // Waiting until TX FIFO is empty
    	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET) {}; // Wait busy flag
		//SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, DISABLE);
	} else {
		//Send Serial Response:
		uart_print("Invalid Input: X\r\n",recvd,0);
	}
}

void TIM2_IRQHandler() {
	/* send adc value in loop:
	int adcRaw = ADC_GetConversionValue(ADC1);
	double adcValue = ((double)adcRaw)/(4095) * 3.3; // 12 bit = 4096 steps, Vdd=3.3V

	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE)!=SET) {};
	int adcChar = (int) adcValue;
	USART_SendData(USART2, 0x30 | adcChar);
	*/
	
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update); // clear pending bit manually
}

void SPI1_IRQHandler() {
	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, DISABLE);
	SPI_I2S_ClearFlag(SPI1, SPI_I2S_FLAG_TXE);
	
	const char resBuffer[RES_BUFFER_LENGTH] = "SPI Sent!\r\n"; // max length: RES_BUFFER_LENGTH!
	unsigned int i = 0;
	while(resBuffer[i] != '\0' && i<RES_BUFFER_LENGTH ) {
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE)!=SET) {}; // wait for possible transmission to complete
		USART_SendData(USART2, resBuffer[i]);
		i++;
	}
	GPIO_SetBits(GPIOB, GPIO_Pin_1);

	
}

