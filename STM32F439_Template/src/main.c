
/********************************************
*			STM32F439 Main (C Startup File)  			*
*			Developed for the STM32								*
*			Author: Rodrigo Collioni RIbeiro			*
*							Anuraj Verma									*
*			Source File														*
*     LAST CHANGE: RODRIGO 29/05/2026 17:31	*
********************************************/


#include <stdint.h>
#include "boardSupport.h"
#include "stm32f439xx.h"
#include "main.h"

// *****************************
// DEFINES
// *****************************
#define a_BITMASK 0b00100000 				// LIGHT BITMASK
#define b_BITMASK 0b00010000				// HEATER BITMASK
#define c_BITMASK 0b00001000				// COOLER BITMASK
#define d_BITMASK 0b00000100				// FAN BITMASK
#define a_OFFSET 0x05 							// LIGHT OFFSET
#define b_OFFSET 0x04								// HEATER OFFSET
#define c_OFFSET 0x03								// COOLER OFFSET
#define d_OFFSET 0x02								// FAN OFFSET
#define TX_FIXED_BITS 0b01000001		// TRANSMITTER FIXED BITS (0b 1 0 _ _ _ _ 0 1)
#define MIN_TEMP_RANGE -3000				// TEMPERATURE IN DEGREES CELSIUS (POTENTIOMETER LOWER THRESHOLD)
#define MAX_TEMP_RANGE 5500					// TEMPERATURE IN DEGREES CELSIUS (POTENTIOMETER UPPER THRESHOLD)
#define MIN_TEMP_RANGE_AUTO 2200		// TEMPERATURE IN DEGREES CELSIUS (HISTERESES LOWER THRESHOLD)
#define MAX_TEMP_RANGE_AUTO 2400		// TEMPERATURE IN DEGREES CELSIUS (HISTERESES UPPER THRESHOLD)
#define MIN_TEMP_RANGE_MANUAL 1500	// TEMPERATURE IN DEGREES CELSIUS (IGNORES USART LOWER THRESHOLD)
#define MAX_TEMP_RANGE_MANUAL 3000	// TEMPERATURE IN DEGREES CELSIUS (IGNORES USART UPPER THRESHOLD)
#define TEMPERATURE_DRIFT 50				// TEMPERATURE IN DEGREES CELSIUS (CURRENT DRIFT IN POTENTIOMETER)
#define ON 0x01											// ON = 1
#define OFF 0x00										// OFF = 0
#define HEADER 0x26									// '&' (0x26)
#define DELIMITER 0x7E 							// '~' (0x7E)
#define CR 0x0D											// '\r'(0x0D)
#define LF 0x0A											// '\n'(0x0A)
#define KEY_TIMER_PSC       8399		// PRESCALER
#define KEY_TIMER_1MS_ARR   90			// BASE ARR (1ms)
#define DEBOUNCE_10MS       10			// KEY DEBOUNCE COUNTER (10ms)
#define LOCKOUT_2S		      2000		// KEY LOCKOUT COUNTER (2s)
#define PRIORITY_1S					1000		// USART LIGHT PRIORITY/LOCKOUT COUNTER (1s)
#define FAN_10S							10000		// FAN LOCKOUT COUNTER( 10s)
#define TEMPERATURE_10S			10000		// HEATER/COOLER LOCKOUT COUNTER (10s)
#define TRANSMIT_4S					4000		// TRANSMIT LOCKOUT COUNTER (4s)

// *****************************
// PROTOTYPES
// *****************************
// Configurations
void configureRCC();																												
void configureGPIO();																												
void configureTimer2(void);                 															 	
void configureUSART3();																											
void configureADC(void); 																										
// Scheduler/Counters
void Scheduler(uint16_t*,uint16_t*,uint16_t*,uint16_t*,uint16_t*,uint16_t*,uint16_t*,uint16_t*);	
// Rx
int16_t getSerialCommand();																									
void DecodeSerialCommand(uint8_t*, uint8_t*, uint8_t*, uint8_t*, uint8_t);	
// Tx
uint8_t buildStatusByte(uint8_t, uint8_t, uint8_t, uint8_t);								
void uartSendByte(uint8_t);																									
void sendTempASCII(int16_t, char*);																					
void TransmitControlMessage(char*,uint8_t);																	
// LEDs
void ChangeLEDs (uint8_t, uint8_t, uint8_t, uint8_t);												
void setLEDOff(uint8_t);																										
void setLEDOn(uint8_t);																											
// ADC
uint16_t readADC(void); 																										
int16_t ADCtoTemperature(uint16_t adcValue);																
// LIGHT
uint8_t checkLightIntensity(void);          															 	
void LightControl(uint8_t*,uint8_t*,uint16_t*, uint16_t*, uint16_t*);				
// FAN
void FanControl(uint8_t*,uint8_t*,uint16_t*,uint16_t*,uint16_t*,uint8_t*);	
// HEATER/COOLER
void TemperatureControl(uint8_t*, uint8_t*, int16_t, uint8_t*,uint16_t*,uint8_t*); 

//******************************************************************************//
// Author: Rodrigo
// Function: main
// Input : NONE
// Return : NONE
// Description : Entry point into the application.
// *****************************************************************************//
int main(void)
{	
	// *****************************
	// CONFIGURATIONS:
	// *****************************
	// Bring up the GPIO for the power regulators.
	boardSupport_init();
	
	// Configures RCC
	configureRCC();
	
	// Configures GPIO
	configureGPIO();
	
	// Configures TIM2
	configureTimer2();
	
	// Configures USART3
	configureUSART3();
	
	// Configure ADC
	configureADC();

	// *****************************
	// INITIALIZE VARIABLES
	// *****************************
	// USART Rx
	int16_t SerialCommand = 0;
	
	// Outputs / USART Tx
	int8_t StatusByte 		= 0;
	uint8_t LightOutput   = 0;
	uint8_t HeaterOutput  = 0;
	uint8_t CoolerOutput  = 0;
	uint8_t FanOutput     = 0;
	
	// Temperature
	// Both temperature readings must be initiated with the same reading
	int16_t TemperatureReading = ADCtoTemperature(readADC());
	char TemperatureText[6]= {0,0,0,0,0,0};
	char* pointerTemperature = &TemperatureText[0];
	
	// Control Bits
	uint8_t USARTTemperature = 0;			// 1 if Command received through USART3
	uint8_t USARTLight = 0;						// 1 if Command received through USART3
	uint8_t USARTFan = 0;							// 1 if Command received through USART3
	uint8_t AutoControlDisabled = 0;	// 1 if AUTO CONTROL DISABLED
	
	// Scheduler Counters
	uint16_t lightDebounceCount = 0;
	uint16_t lightLockoutCount = 0;
	uint16_t USARTPriorityCount = 0;
	uint16_t fanDebounceCount = 0;
	uint16_t fanLockoutCount = 0;
	uint16_t fan10sLockoutCount = 0;
	uint16_t TemperatureUSARTCount = 0;
	uint16_t ControlMessageCount = 0;
	// *****************************
	// MAIN LOOP
	// *****************************
  while (1)
  {
		// SCHEDULER/COUNTERS
		// Handles all counter updates
		Scheduler(&lightDebounceCount,&lightLockoutCount,&USARTPriorityCount,
							&fanDebounceCount,&fanLockoutCount,&fan10sLockoutCount,
							&TemperatureUSARTCount,&ControlMessageCount);
		
		// Read temperature from potentiometer in this while loop :)
		TemperatureReading = ADCtoTemperature(readADC());
		
		// USART3 Rx
		// Reads USART3 for Serial Commands
		SerialCommand = getSerialCommand();
		// Only accept USART commands if they are inside the temperature range (30.00 > Command > 15.00)
		if((TemperatureReading < MAX_TEMP_RANGE_MANUAL) && (TemperatureReading > MIN_TEMP_RANGE_MANUAL))
		{
			// If a Serial Command was received, decode it
			if (SerialCommand != -1)
			{
				DecodeSerialCommand(&LightOutput, &HeaterOutput, &CoolerOutput, &FanOutput, (uint8_t)SerialCommand);
				USARTTemperature = 1;
				USARTLight = 1;
				USARTFan = 1;
				AutoControlDisabled = 0;
			}
		}
		// HEATER, COOLER CONTROL
		// PROBLEM: ADC Current fluctuates, therefore the temperature changes
		TemperatureControl(&HeaterOutput, &CoolerOutput, TemperatureReading, &USARTTemperature, &TemperatureUSARTCount, &AutoControlDisabled);		
		// LIGHT CONTROL
		// SW4 (LIGHT), SW2 (LIGHT INTENSITY)
		LightControl(&LightOutput, &USARTLight, &lightDebounceCount, &lightLockoutCount, &USARTPriorityCount);
		
		// FAN CONTROL
		// SW5(FAN)
		FanControl(&FanOutput,&USARTFan,&fanDebounceCount,&fanLockoutCount,&fan10sLockoutCount,&AutoControlDisabled);	
		

		// USART3 Tx
		// Transmits control message every 4 seconds
		if(ControlMessageCount == 0)
		{
			// Transforms ADC value into a 6-digit ASCII matrix
			sendTempASCII(TemperatureReading, pointerTemperature);
			// Build the Status Byte
			StatusByte = buildStatusByte(LightOutput, HeaterOutput, CoolerOutput, FanOutput);
			// Transmits Control Message (&~(TEMPERATURE IN ASCII)~(STATUS BYTE)
			TransmitControlMessage(pointerTemperature, StatusByte);
			// Restarts the 4s counter
			ControlMessageCount = TRANSMIT_4S;
		}
		
		// ChangeLEDs based on the outputs
		ChangeLEDs(LightOutput, HeaterOutput, CoolerOutput, FanOutput);
	}
} 

//******************************************************************************//
// Author: Rodrigo
// Function: configureRCC
// Input : NONE
// Return : NONE
// Description : Configures RCC for I/O (GPIO, TIM2, UART3, ADC)
// *****************************************************************************//
void configureRCC()
{
	// Enables the RCC for GPIO A, B and F
	RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOFEN_Pos) | (1 << RCC_AHB1ENR_GPIOBEN_Pos) |
											(1 << RCC_AHB1ENR_GPIOAEN_Pos);
	
	// Enables the RCC for TIM2
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	
	// Enables the RCC for UART3	
	RCC->APB1ENR |= (RCC_APB1ENR_USART3EN);
	
	// Enables the RCC for ADC3
	RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;
	
	// Resets the peripherals
	// GPIO A, B and F
	RCC->AHB1RSTR |= (1 << RCC_AHB1RSTR_GPIOFRST_Pos) | (1 << RCC_AHB1RSTR_GPIOBRST_Pos) |
											(1 << RCC_AHB1RSTR_GPIOARST_Pos);
	// TIM2
	RCC->APB1RSTR |= RCC_APB1RSTR_TIM2RST;
	
	// USART3	
	RCC->APB1RSTR |= (RCC_APB1RSTR_USART3RST);
	
	// ADC
	RCC->APB2RSTR |= RCC_APB2RSTR_ADCRST;
	
	// Wait 2 cycles
	__ASM("NOP");	__ASM("NOP");
	
	// Release Reset for peripherals
	// GPIO A, B and F
	RCC->AHB1RSTR &= ~((1 << RCC_AHB1RSTR_GPIOFRST_Pos) | (1 << RCC_AHB1RSTR_GPIOBRST_Pos) |
											(1 << RCC_AHB1RSTR_GPIOARST_Pos));
	
	// TIM2
	RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM2RST;
	
	// USART3
	RCC->APB1RSTR &= ~(RCC_APB1RSTR_USART3RST);
	
	// ADC
	RCC->APB2RSTR &= ~(RCC_APB2RSTR_ADCRST);
	
	// Wait 2 cycles
	__ASM("NOP");	__ASM("NOP");
	return;
}



//******************************************************************************//
// Author: Rodrigo
// Function: configureGPIO
// Input : NONE
// Return : NONE
// Description : Configures GPIO for I/O
// *****************************************************************************//
void configureGPIO()
{
	//PORT F ->both PF10 and PF8 configured.
	GPIOF->MODER &= ~((0x03 << GPIO_MODER_MODE10_Pos) | (0x03 << GPIO_MODER_MODE8_Pos));
	GPIOF->MODER |=  (0x03 << GPIO_MODER_MODE10_Pos);  // PF10 the analogue input
	GPIOF->MODER |=  (0x01 << GPIO_MODER_MODE8_Pos);   // PF8 -> the output
	GPIOF->OTYPER &= ~(0x01 << GPIO_OTYPER_OT8_Pos);
	GPIOF->OSPEEDR &= ~((0x03 << GPIO_OSPEEDR_OSPEED10_Pos) | (0x03 << GPIO_OSPEEDR_OSPEED8_Pos));
	GPIOF->PUPDR &= ~((0x03 << GPIO_PUPDR_PUPD10_Pos) | (0x03 << GPIO_PUPDR_PUPD8_Pos));
	
	//PORT B -> PB8 (OUTPUT), PB1 (OUTPUT), PB0 (INPUT)
	GPIOB->MODER |= (0x01 << GPIO_MODER_MODE8_Pos) | (0x01 << GPIO_MODER_MODE1_Pos) | (0x00 << GPIO_MODER_MODE0_Pos);
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT0);
  GPIOB->OTYPER |= (0x01 << GPIO_OTYPER_OT8_Pos) | (0x01 << GPIO_OTYPER_OT1_Pos);
	GPIOB->OSPEEDR &= ~((0x03 << GPIO_OSPEEDR_OSPEED8_Pos)|(0x03 << GPIO_OSPEEDR_OSPEED1_Pos)|(0x03 << GPIO_OSPEEDR_OSPEED0_Pos));
	GPIOB->PUPDR |= (0x01 << GPIO_PUPDR_PUPD8_Pos)|(0x01 << GPIO_PUPDR_PUPD1_Pos)|(0x01 << GPIO_PUPDR_PUPD0_Pos);
	
	//PORT A -> PA10 (INPUT), PA9 (OUTPUT), PA8 (INPUT)
  GPIOA->MODER |= (0x00 << GPIO_MODER_MODE10_Pos) | (0x01 << GPIO_MODER_MODE9_Pos) | (0x00 << GPIO_MODER_MODE8_Pos);
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT10 | GPIO_OTYPER_OT8);
  GPIOA->OTYPER |= (0x01 << GPIO_OTYPER_OT9_Pos);
	GPIOA->OSPEEDR &= ~((0x03 << GPIO_OSPEEDR_OSPEED10_Pos)|(0x03 << GPIO_OSPEEDR_OSPEED9_Pos)|(0x03 << GPIO_OSPEEDR_OSPEED8_Pos)|(0x03 << GPIO_OSPEEDR_OSPEED3_Pos));
	GPIOA->PUPDR |= (0x01 << GPIO_PUPDR_PUPD10_Pos)|(0x01 << GPIO_PUPDR_PUPD9_Pos)|(0x01 << GPIO_PUPDR_PUPD8_Pos);
	
	// Sets all LEDs ON
		for(uint8_t i=0;i<8;i++)
	{
		setLEDOn(i);
	}
	return;
}
//******************************************************************************//
// Author: Anuraj
// Function: configureTimer2
// Input : NONE
// Return : NONE
// Description : Configures TIM2 for 1ms cycles
// *****************************************************************************//
void configureTimer2(void)
{
	// TIM2 gives a 1ms tick for switch timing
	TIM2->CR1 &= ~TIM_CR1_CEN;                 // Stops the timer

	TIM2->PSC &= ~(TIM_PSC_PSC_Msk);           // Clear prescaler
	TIM2->PSC = KEY_TIMER_PSC;                 // 10kHz timer clock

	TIM2->ARR &= ~(TIM_ARR_ARR_Msk);           // Clear ARR
	TIM2->ARR = KEY_TIMER_1MS_ARR;             // 1ms tick

	TIM2->CNT = 0;                             // Reset count
	TIM2->SR &= ~TIM_SR_UIF;                   // Clear flag

	TIM2->CR1 &= ~TIM_CR1_OPM;                 // Continuous mode
	TIM2->CR1 |= TIM_CR1_CEN;                  // Start timer
	return;
}
//******************************************************************************//
// Author: Rodrigo
// Function: configureUSART3
// Input : NONE
// Return : NONE
// Description : Configure USART3, PB11 = Rx, PB10 = Tx
// *****************************************************************************//
void configureUSART3()
{
	// Configure GPIOB MODER register
	GPIOB->MODER &= ~(GPIO_MODER_MODE11_Msk | GPIO_MODER_MODE10_Msk);
	GPIOB->MODER |= (0x02 << GPIO_MODER_MODE11_Pos) | (0x02 << GPIO_MODER_MODE10_Pos);
	
	// Setup the Alternate Function - A7
	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL11_Msk | GPIO_AFRH_AFSEL10_Msk);
	GPIOB->AFR[1] |= (0x07 << GPIO_AFRH_AFSEL11_Pos) | (0x07 << GPIO_AFRH_AFSEL10_Pos);
	
	// Turn on 16 time over sampling
	USART3->CR1 &= ~(USART_CR1_OVER8);
	
	// Set the USART Baud Rate - Clear out the register first
	USART3->BRR &= 0xFFFF0000;
	
	// Now set the baud rate (57600)
	// USARTDIV = f_CK/(8*(2-OVER8)*BAUDRATE)
	// The code uses 16 oversampling, so OVER8=0
	// USARTDIV = 42*10^6/ (16*57600) = 45.573  
	// Mantissa = 0x2D (45d) 
	// Fractional = 0x09 (0.573 x 16 rounded to the closest integer)
	USART3->BRR = (0x2D << USART_BRR_DIV_Mantissa_Pos | 0x09 << USART_BRR_DIV_Fraction_Pos); 
	
	// Set the number of bits per transfer (9-bits)
	USART3->CR1 |=(0x01 << USART_CR1_M_Pos);  		// 9-bit frame = 8 data bits + 1 parity bit
	
	// Set the number of stop bits
	USART3->CR2 &= ~(USART_CR2_STOP_Msk);
	USART3->CR2 |= (0x00 << USART_CR2_STOP_Pos);	// 1 stop bit
	
	// Odd parity
  USART3->CR1 |= (0x01 << USART_CR1_PCE_Pos); 	// PCE = 1, Bit 10
  USART3->CR1 |= (0x01 << USART_CR1_PS_Pos);		// odd parity, bit 9 (wrote 1)
	
	// Select mode of operation (async - no clock)
	USART3->CR2 &= ~(USART_CR2_CLKEN | USART_CR2_CPOL | USART_CR2_CPHA);
	
	// Disable hardware flow control
	USART3->CR3 &= ~(USART_CR3_CTSE | USART_CR3_RTSE);
	
	// Now that the configuration is complete, enable the USART, transmitter and receive sections
	USART3->CR1 |= (USART_CR1_TE | USART_CR1_UE | USART_CR1_RE);
	return;
}

//******************************************************************************//
// Author: Anuraj
// Function: configureADC
// Input : NONE
// Return : NONE
// Description : Configures ADC
// *****************************************************************************//
void configureADC(void)
{
	// ADC common prescaler
	ADC->CCR &= ~ADC_CCR_ADCPRE;
	ADC->CCR |= (0x01 << ADC_CCR_ADCPRE_Pos);  // PCLK2 / 4

	// Clear ADC3 control registers
	ADC3->CR1 = 0x00000000;
	ADC3->CR2 = 0x00000000;

	// 12-bit resolution, single conversion
	ADC3->CR1 &= ~ADC_CR1_RES;
	ADC3->CR2 &= ~ADC_CR2_CONT;

	// Right alignment
	ADC3->CR2 &= ~ADC_CR2_ALIGN;

	// One conversion in the regular sequence
	ADC3->SQR1 = 0x00000000;
	ADC3->SQR2 = 0x00000000;

	// PF10 is ADC3 channel 8
	ADC3->SQR3 = 8;

	// Sampling time for channel 8
	ADC3->SMPR2 &= ~ADC_SMPR2_SMP8_Msk;
	ADC3->SMPR2 |= (0x06 << ADC_SMPR2_SMP8_Pos);

	// Turn ADC3 on
	ADC3->CR2 |= ADC_CR2_ADON;
	return;
}

//******************************************************************************//
// Author: Rodrigo
// Function: Scheduler
// Input : *lightDebounceCount, *lightLockoutCount, *USARTPriorityCount,
//				 *fanDebounceCount, *fanLockoutCount, *fan10sLockoutCount,
//				 *TemperatureUSARTCount, *ControlMessageCount
// Return : NONE
// Description : Keeps track of every counter
// *****************************************************************************//
void Scheduler(uint16_t *lightDebounceCount,uint16_t *lightLockoutCount,uint16_t *USARTPriorityCount,
							 uint16_t *fanDebounceCount, uint16_t *fanLockoutCount, uint16_t *fan10sLockoutCount,
							 uint16_t *TemperatureUSARTCount, uint16_t *ControlMessageCount)
{
	uint16_t ticks = 0;

	while (TIM2->SR & TIM_SR_UIF)
	{
		TIM2->SR &= ~TIM_SR_UIF; // Clear each tick
		ticks++;
	}

	if (ticks == 0) return; // Nothing to do

	// Decrement each counter by ticks, floor at 0
	if (*lightDebounceCount > ticks)       *lightDebounceCount -= ticks;
	else                                   *lightDebounceCount = 0;

	if (*lightLockoutCount > ticks)        *lightLockoutCount -= ticks;
	else                                   *lightLockoutCount = 0;

	if (*USARTPriorityCount > ticks)       *USARTPriorityCount -= ticks;
	else                                   *USARTPriorityCount = 0;

	if (*fanDebounceCount > ticks)         
		*fanDebounceCount -= ticks;
	else                                   *fanDebounceCount = 0;

	if (*fanLockoutCount > ticks)          *fanLockoutCount -= ticks;
	else                                   *fanLockoutCount = 0;

	if (*fan10sLockoutCount > ticks)       *fan10sLockoutCount -= ticks;
	else                                   *fan10sLockoutCount = 0;

	if (*TemperatureUSARTCount > ticks)    *TemperatureUSARTCount -= ticks;
	else                                   *TemperatureUSARTCount = 0;

	if (*ControlMessageCount > ticks)      *ControlMessageCount -= ticks;
	else                                   *ControlMessageCount = 0;
}

//******************************************************************************//
// Author: Rodrigo
// Function: getSerialCommand
// Input : NONE
// Return : -1 (No Update), Serial Command (If Detected)
// Description : Checks the UART to see if new serial value has been received,
//							 pools 2 characters, checks to see if its a valid Serial Command
//							 on an ENTER
// *****************************************************************************//
int16_t getSerialCommand()
{
	// 1 0 LIGHT HEATING COOLER FAN 00
	// POSSIBLE SERIAL COMMANDS
	// &@ - ALL OFF --------------------- @ = 0x40
	// &D - FAN ON ---------------------- D = 0x44
	// &H - COOLER ON ------------------- H = 0x48
	// &L - FAN / COOLER ON ------------- L = 0x4C
	// &P - HEATER ON ------------------- P = 0x50
	// &T - HEATER / FAN ON ------------- T = 0x54
	// &X - HEATER / COOLER ON ---------- X = 0x58 // INVALID HEATER AND COOLER CANNOT BE ON AT THE SAME TIME
	// &\ - HEATER / COOLER / FAN ON ---- \ = 0x5C // INVALID HEATER AND COOLER CANNOT BE ON AT THE SAME TIME
	// &` - LIGHT ON -------------------- ` = 0x60
	// &d - LIGHT / FAN ON -------------- d = 0x64
	// &h - LIGHT / COOLER ON ----------- h = 0x68
	// &l - LIGHT / COOLER / FAN ON ----- l = 0x6C
	// &p - LIGHT / HEATER ON ----------- p = 0x70
	// &t - LIGHT / HEATER / FAN ON ----- t = 0x74
	// &x - LIGHT / HEATER / Cooler ON -- x = 0x78 // INVALID HEATER AND COOLER CANNOT BE ON AT THE SAME TIME
	// &| - ALL ON ---------------------- | = 0x7C // INVALID HEATER AND COOLER CANNOT BE ON AT THE SAME TIME
	
	static uint8_t buffer[2]= {0, 0};	// 2 char
	static uint8_t i = 0;							// Buffer index
	static uint8_t overflow = 0;  		// Overflow flag
	int16_t receivedCharacter = -1;		// Return variable 
	
	
	// Check to see whether a character has been received
  if (!(USART3->SR & USART_SR_RXNE))
  {
		// If not, return -1
		receivedCharacter = -1;
		return receivedCharacter;				
	}
	
	// Check for parity error before reading
	if (USART3->SR & USART_SR_PE)
	{
    (void) USART3->DR;  // Must read DR to clear the PE flag, discard the byte
    receivedCharacter = -1;
    return receivedCharacter;
	}
	
	// Get the character from the data register
	receivedCharacter = ((USART3->DR)&(0xFF));
	
	if(receivedCharacter == LF)
	{
		receivedCharacter = -1;
		return receivedCharacter;
	}
		
	// End of line detected
	if (receivedCharacter == CR)
	{
		receivedCharacter = -1; // No valid input (Standard Response)

		// Checks for valid buffer
		if 	(	!overflow 	&&														// Checks if it isn't overflow
					i == 2 		&&															// Checks if the index is in correct position (1)
				 (buffer[0] == HEADER))											// Checks if first byte is valid  (HEADER/&/0x26)
    {
			// Switch case based on second byte
			switch (buffer[1])
			{				
				case '@': receivedCharacter = buffer[1]; break;
				case 'D': receivedCharacter = buffer[1]; break;
				case 'H': receivedCharacter = buffer[1]; break;
				case 'L': receivedCharacter = buffer[1]; break;
				case 'P': receivedCharacter = buffer[1]; break;
				case 'T': receivedCharacter = buffer[1]; break;
				case '`': receivedCharacter = buffer[1]; break;
				case 'd': receivedCharacter = buffer[1]; break;
				case 'h': receivedCharacter = buffer[1]; break;
				case 'l': receivedCharacter = buffer[1]; break;
				case 'p': receivedCharacter = buffer[1]; break;
				case 't': receivedCharacter = buffer[1]; break;
				default: break; 
				// Cases 'X','\','x' and '|', fall through default,
				// as they are considered invalid due to setting Heater and Cooler ON at the same time
			}		
    }
		i = 0; 									// Resets buffer
		overflow = 0;  					// Clear for next input
		buffer[0] = 0;					// Clears buffer
		buffer[1] = 0;
		return receivedCharacter;	// Returns serial value 
  }


	// Valid buffer position
	if ((i < 2) && (receivedCharacter != LF) && (receivedCharacter != CR))
	{
		buffer[i] = receivedCharacter;	// Saves digit into buffer
		i++;
	}
	else if((receivedCharacter != LF) && (receivedCharacter != CR))
	{
		overflow = 1;  	// Overflow detected, marks input to be ignored
		i = 0;					// Sets index to the beginning 
	}

  receivedCharacter = -1; // No valid input
	return receivedCharacter; // Returns no valid input
}

//******************************************************************************//
// Author: Rodrigo
// Function: DecodeSerialCommand() - Decodes the Serial Command
// Input : *LightOutput(light),*HeaterOutput(heater),*CoolerOutput(cooler),
//				 *FanOutput(fan), Code(SerialCommand)
// Return : NONE
// Description : Decodes the Serial Command and loads the bits into the Outputs
// *****************************************************************************//
void DecodeSerialCommand(uint8_t* light, uint8_t* heater, uint8_t* cooler, uint8_t* fan, uint8_t Code)
{
	// decoded bits: 0 1 a b c d 0 0
	// a = LightOutput
	// b = HeaterOutput
	// c = CoolerOutput
	// d = FanOutput
	// Masks them normalizes the individual control bits
	*light   = (Code & a_BITMASK) >> a_OFFSET;
	*heater  = (Code & b_BITMASK) >> b_OFFSET;
	*cooler  = (Code & c_BITMASK) >> c_OFFSET;
	*fan     = (Code & d_BITMASK) >> d_OFFSET;
}

//******************************************************************************//
// Author: Rodrigo
// Function: buildStatusByte()
// Input : LightOutput, HeaterOutput, CoolerOutput, FanOutput 
// Return : status(StatusByte)
// Description : Builds the Status Byte based on the outputs
// *****************************************************************************//
uint8_t buildStatusByte(uint8_t light, uint8_t heater, uint8_t Cooler, uint8_t fan)
{
		// encoded bits: 0 1 a b c d 0 1
		// a = LightOutput
		// b = HeaterOutput
		// c = CoolerOutput
		// d = FanOutput
		// fixed bits: 0 1 _ _ _ _ 0 1
	
		// POSSIBLE STATUS BYTES
		// ALL OFF ----------------------	A = 0x41
		// FAN ON -----------------------	E = 0x45
		// COOLER ON --------------------	I = 0x49
		// FAN / COOLER ON --------------	M = 0x4D
		// HEATER ON --------------------	Q = 0x51
		// HEATER / FAN ON -------------- U = 0x55
		// LIGHT ON ---------------------	a = 0x61
		// LIGHT / FAN ON --------------- e = 0x65
		// LIGHT / COOLER ON ------------	i = 0x69
		// LIGHT / COOLER / FAN ON ------	m = 0x6D
		// LIGHT / HEATER ON ------------	q = 0x71
		// LIGHT / HEATER / FAN ON ------	u = 0x75
	
	
    uint8_t status = TX_FIXED_BITS;          
    status |= (light   & 0x01) << a_OFFSET;
    status |= (heater  & 0x01) << b_OFFSET;
    status |= (Cooler  & 0x01) << c_OFFSET;
    status |= (fan     & 0x01) << d_OFFSET;
    return status;
}

//******************************************************************************//
// Author: Anuraj
// Function: uartSendByte()
// Input : txData (unsigned 8-bit value)
// Return : NONE
// Description : Sends a byte through USART3 Tx
// *****************************************************************************//
void uartSendByte(uint8_t txData)
{
	// Waits for Transmitter to be ready
  while ((USART3->SR & USART_SR_TXE) == 0)
  {
	}

  USART3->DR = txData; // Sends the packets
	return;
}
//******************************************************************************//
// Author: Anuraj
// Function: sendTempASCII()
// Input : TemperatureReading(tempCenti), TemperatureText(tempText)
// Return : NONE
// Description : Sends the received value through the USART3 transmitter
// *****************************************************************************//
void sendTempASCII(int16_t tempCenti, char* tempText)
{
    int16_t tempAbs = 0;

		// Setting the ranges
    if (tempCenti > MAX_TEMP_RANGE) //55.00 degrees Celsius
    {
        tempCenti = MAX_TEMP_RANGE;
    }
    else if (tempCenti < MIN_TEMP_RANGE) //-30.00 degrees Celsius
    {
        tempCenti = MIN_TEMP_RANGE;
    }
		
		// If else statement for the - or the + sign.
		if (tempCenti < 0)
		{
				*tempText = '-';
				tempAbs = (int16_t)(-tempCenti);  // negate to make positive
		}
		else
		{
				*tempText = '+';
				tempAbs = tempCenti;              // already positive
		}
		
		// Loads the rest of the individual temperature digits into buffer
		tempText++;
    *tempText = (char)('0' + ((tempAbs / 1000) % 10));
		tempText++;
    *tempText = (char)('0' + ((tempAbs / 100) % 10));
		tempText++;
    *tempText = '.';
		tempText++;
    *tempText = (char)('0' + ((tempAbs / 10) % 10));
		tempText++;
    *tempText = (char)('0' + (tempAbs % 10));
		
		return;
}

//******************************************************************************//
// Author: Rodrigo
// Function: uartSendByte()
// Input : *TemperatureText, StatusByte
// Return : NONE
// Description : Sends control message
// *****************************************************************************//
void TransmitControlMessage(char* tempText, uint8_t StatusByte)
{
			uartSendByte((uint8_t)HEADER);   		// header '&'
			uartSendByte((uint8_t)DELIMITER);   // first tilde
			for (int i = 0; i < 6; i++)					// Sends the 6 bytes of temperature reading
			{
				uartSendByte((uint8_t)*tempText);
				tempText++;
			}
			uartSendByte((uint8_t)DELIMITER);   // second tilde
			uartSendByte((uint8_t)StatusByte);	// your abcd bits
			uartSendByte((uint8_t)CR);					// Sends CR
			uartSendByte((uint8_t)LF);					// Sends LF
}

//******************************************************************************//
// Author: Rodrigo
// Function: ChangeLEDs()
// Input : LightOutput,HeaterOutput, CoolerOutput, FanOutput
// Return : NONE
// Description : Changes the LEDs based on Outputs
// 							 LED 2 - LIGHT
// 							 LED 7 - HEATER
// 							 LED 6 - COOLER
// 							 LED 5 - FAN
// *****************************************************************************//
void ChangeLEDs (uint8_t LightLED, uint8_t HeaterLED, uint8_t CoolerLED, uint8_t FanLED)
{
	switch(LightLED)
	{
		case 0: setLEDOff(2);	break;
		case 1: setLEDOn(2);	break;
		default:	break;
	}
	
	switch(HeaterLED)
	{
		case 0: setLEDOff(7);	break;
		case 1: setLEDOn(7);	break;
		default:	break;
	}
	
	switch(CoolerLED)
	{
		case 0: setLEDOff(6);	break;
		case 1: setLEDOn(6);	break;
		default:	break;
	}
	
	switch(FanLED)
	{
		case 0: setLEDOff(5);	break;
		case 1: setLEDOn(5);	break;
		default:	break;
	}
	return;
}

//******************************************************************************//
// Author: Rodrigo
// Function: setLEDOff()
// Input : ledNumber
// Return : NONE
// Description : Sets LED as OFF
// *****************************************************************************//
void setLEDOff(uint8_t ledNumber)
{
	switch(ledNumber)
	{
		case 0:
		{
			GPIOA->BSRR |= (0x01 << GPIO_BSRR_BS3_Pos);
			break;
		}
			case 1:
		{
			GPIOA->BSRR |= (0x01<< GPIO_BSRR_BS8_Pos);
			break;
		}
			case 2:
		{
			GPIOA->BSRR |= (0x01<< GPIO_BSRR_BS9_Pos);
			break;
		}
			case 3:
		{
			GPIOA->BSRR |= (0x01<< GPIO_BSRR_BS10_Pos);
			break;
		}
			case 4:
		{
			GPIOB->BSRR |= (0x01<< GPIO_BSRR_BS0_Pos);
			break;
		}
			case 5:
		{
			GPIOB->BSRR |= (0x01<< GPIO_BSRR_BS1_Pos);
			break;
		}
			case 6:
		{
			GPIOB->BSRR |= (0x01<< GPIO_BSRR_BS8_Pos);
			break;
		}
			case 7:
		{
			GPIOF->BSRR |= (0x01<< GPIO_BSRR_BS8_Pos);
			break;
		}

			default:
				break;
	}
	return;
}

//******************************************************************************//
// Author: Rodrigo
// Function: setLEDOn()
// Input : ledNumber
// Return : NONE
// Description : Sets LED as OFF
// *****************************************************************************//
void setLEDOn(uint8_t ledNumber)
{
		switch(ledNumber)
	{
		case 0:
		{
			GPIOA->BSRR |= (0x01<< GPIO_BSRR_BR3_Pos);
			break;
		}
			case 1:
		{
			GPIOA->BSRR |= (0x01<< GPIO_BSRR_BR8_Pos);
			break;
		}
			case 2:
		{
			GPIOA->BSRR |= (0x01<< GPIO_BSRR_BR9_Pos);
			break;
		}
			case 3:
		{
			GPIOA->BSRR |= (0x01<< GPIO_BSRR_BR10_Pos);
			break;
		}
			case 4:
		{
			GPIOB->BSRR |= (0x01<< GPIO_BSRR_BR0_Pos);
			break;
		}
			case 5:
		{
			GPIOB->BSRR |= (0x01<< GPIO_BSRR_BR1_Pos);
			break;
		}
			case 6:
		{
			GPIOB->BSRR |= (0x01<< GPIO_BSRR_BR8_Pos);
			break;
		}
			case 7:
		{
			GPIOF->BSRR |= (0x01<< GPIO_BSRR_BR8_Pos);
			break;
		}

			default:
				break;
	}
	return;
}

//******************************************************************************//
// Author: Anuraj
// Function: readADC
// Input : NONE
// Return : adcValue
// Description : Reads the ADC, controlled by a potentiometer in this case
// *****************************************************************************//
uint16_t readADC(void)
{
	uint16_t adcValue = 0;
	// Start conversion
	ADC3->CR2 |= ADC_CR2_SWSTART;

	// Wait until conversion is complete
	while ((ADC3->SR & ADC_SR_EOC) == 0)
	{
	}
	adcValue = (uint16_t)(ADC3->DR & 0x0FFF);
	return adcValue;
}

//******************************************************************************//
// Author: Anuraj
// Function: readADC
// Input : adcValue
// Return : temperature (TemperatureReading)
// Description : Converts the ADC value reading into degrees Celsius
// *****************************************************************************//
int16_t ADCtoTemperature(uint16_t adcValue)
{
	int32_t temperature = 0;
	// 0 ADC = +55.00C, 4095 ADC = -30.00C
	temperature = 5500 - (((int32_t)adcValue * 8500) / 4095);
	return (int16_t)temperature;
}

//******************************************************************************//
// Author: Anuraj
// Function: checkLightIntensity
// Input : NONE
// Return : 1 if room is already lit
// Description : Checks SW2 (Light Intensity Input)
// *****************************************************************************//
uint8_t checkLightIntensity(void)
{
	// PA8 is active low
	if ((GPIOA->IDR & (0x01 << 8)) == 0)
	{
		// Room is lit, while SW2 pressed
		return 1;
	}
	return 0;
}

//******************************************************************************//
// Author: Anuraj
// Function: LightControl
// Input : LightOutput
// Return : NONE
// Description : Reads SW4 (LIGHT SWITCH),
//							 operates as a toggle, if SW2 (Light Intensity Input) is pressed,
//							 LIGHT cannot be turned ON, only OFF
// *****************************************************************************//
void LightControl(uint8_t *light, uint8_t *USARTLight, uint16_t *lightDebounceCount,uint16_t *lightLockoutCount,uint16_t *USARTPriorityCount)
{
	static uint8_t lightWasPressed = 0; 
	static uint8_t lightDebouncing = 0;
	uint8_t lightLow = 0;
	uint8_t roomLit = 0;

	// Read active low switches
	if ((GPIOA->IDR & (0x01 << 10)) == 0)
	{
		lightLow = 1;
	}

	// Checks (SW2 Light Sensitivity Switch)
	roomLit = checkLightIntensity();
	
	// Triggers once upon receiving USART Command
	if(*USARTLight == 1)
	{
		// For 1s the USART Command will be upheld over SWITCH Commands
		*USARTPriorityCount = PRIORITY_1S;
		*USARTLight = 0;
	}
	
	// Waits for 1s of USART priority to end
	if(*USARTPriorityCount == 0)
	{
		// Light switch handling
		if (*lightLockoutCount == 0)
		{
			if (lightLow == 1)
			{
				// Start debounce counter
				if ((lightWasPressed == 0) && (lightDebouncing == 0))
				{
					lightDebouncing = 1;
					*lightDebounceCount = DEBOUNCE_10MS;
				}

				// Accept press after 10ms debounce
				if ((lightDebouncing == 1) && (*lightDebounceCount == 0))
				{
					lightDebouncing = 0;
					lightWasPressed = 1;
				}
			}
			// Only acts on a rising edge (key stopped being pressed)
			else
			{
				// Register on release
				if (lightWasPressed == 1)
				{
					lightWasPressed = 0;
					*lightLockoutCount = LOCKOUT_2S;

					// If the light is ON turn it OFF
					if (*light == ON)
					{
						*light = OFF;
					}
					// If the light is OFF
					else
					{
						// Check if the Light Sensitivity Key is pressed
						// Only turn light ON if not
						if (roomLit == 0)
						{
							*light = ON;
						}
					}
				}

				lightDebouncing = 0;
				*lightDebounceCount = 0;
			}
		}
	}
}

//******************************************************************************//
// Author: Anuraj
// Function: FanControl
// Input : *FanOutput(fan), *USARTFan, *fanDebounceCount, *fanLockoutCount
//				 *fan10sLockoutCount, *AutoControlDisabled
// Return : NONE
// Description : Reads SW5 (FAN SWITCH), if in AUTO CONTROL, always ON
//							 can be turned off for 10s by both USART (Priotity) and SWITCH,
//							 these 10s can be OVERWRITTEN by a new USART Command,
//							 if AUTO CONTROL is DISABLED, operates as a toggle
// *****************************************************************************//
void FanControl(uint8_t *fan, uint8_t *USARTFan, uint16_t *fanDebounceCount, uint16_t *fanLockoutCount, uint16_t *fan10sLockoutCount, uint8_t *AutoControlDisabled)
{
	static uint8_t fanWasPressed = 0;
	static uint8_t fanDebouncing = 0;
	static uint8_t SwitchFan = 0;
	uint8_t fanLow = 0;

	if ((GPIOB->IDR & (0x01 << 0)) == 0)
	{
		fanLow = 1;
	}

	// Triggers once upon receiving USART Command
	// USART Commands will override SWITCH Commands
	if((*USARTFan == 1) || (SwitchFan == 1))
	{
		*fan10sLockoutCount = FAN_10S;
		*USARTFan = 0;
		SwitchFan = 0;
	}

	// Waits for 10s lockout to end
	if(*fan10sLockoutCount == 0)
	{
		// Fan switch handling
		if (*fanLockoutCount == 0)
		{
			if (fanLow == 1)
			{
				if ((fanWasPressed == 0) && (fanDebouncing == 0))
				{
					fanDebouncing = 1;
					*fanDebounceCount = DEBOUNCE_10MS;
				}

				if ((fanDebouncing == 1) && (*fanDebounceCount == 0))
				{
					fanDebouncing = 0;
					fanWasPressed = 1;
				}
			}
			else
			{
				if (fanWasPressed == 1)
				{
					fanWasPressed = 0;
					*fanLockoutCount = LOCKOUT_2S;

					if((*fan == ON) && (*AutoControlDisabled == 0))
					{
						*fan = OFF;
						SwitchFan = 1;
					}
					if((*fan == ON) && (*AutoControlDisabled == 1))
					{
						*fan = OFF;
					}
					if((*fan == OFF) && (*AutoControlDisabled == 1))
					{
						*fan = ON;
					}
				}

				fanDebouncing = 0;
				*fanDebounceCount = 0;
			}
		}

		// AUTO CONTROL: FAN ALWAYS ON
		// Only reached if fan10sLockoutCount == 0 and no switch activity overrode it
		if ((*AutoControlDisabled == 0) && (*USARTFan == 0) && (SwitchFan == 0))
		{
			*fan = ON;
		}
	}
	return;
}

//******************************************************************************//
// Author: Rodrigo
// Function: TemperatureControl
// Input : *HeaterOutput, *CoolerOutput, TemperatureReading, PreviousTemperature,
//				 *USARTTemperature, *TemperatureUSARTCount, *AutoControlDisabled
// Return : NONE
// Description : Automatically controls the HEATER and COOLER OUTPUT,
//							 can be overwritten for 10s by a USART Command,
//							 if the temperature changes during these 10s, DISABLE AUTO CONTROL
// *****************************************************************************//
void TemperatureControl(uint8_t* heater, uint8_t* cooler, int16_t temperature, uint8_t *USARTTemperature, uint16_t *TemperatureUSARTCount, uint8_t *AutoControlDisabled)
{
	static int16_t baselineTemperature = 0;
	static uint8_t baselineSet = 0;

	// Triggers once upon receiving USART Command
	if(*USARTTemperature == 1)
	{
		*TemperatureUSARTCount = TEMPERATURE_10S;
		*USARTTemperature = 0;
		// Snapshot the temperature at the moment USART command arrives
		baselineTemperature = temperature;
		baselineSet = 1;
	}

	// During the 10s USART window, check cumulative drift from baseline
	if((*TemperatureUSARTCount > 0) && (baselineSet == 1))
	{
		int16_t drift = temperature - baselineTemperature;
		if(drift < 0) drift = -drift; // Absolute value

		// If potentiometer has moved more than TEMPERATURE_DRIFT degrees from baseline, disable auto control
		if(drift >= TEMPERATURE_DRIFT)
		{
			*AutoControlDisabled = 1;
			baselineSet = 0;
		}
	}

	// Reset baseline once lockout expires naturally (no pot movement detected)
	if(*TemperatureUSARTCount == 0)
	{
		baselineSet = 0;
	}

	// Automatic Control (Sensor based)
	// Ignored for 10s if USART Command received
	if((*TemperatureUSARTCount == 0) && (*AutoControlDisabled == 0))
	{
		if (temperature < MIN_TEMP_RANGE_AUTO)
		{
			*heater = ON;
			*cooler = OFF;
		}
		else if (temperature > MAX_TEMP_RANGE_AUTO)
		{
			*heater = OFF;
			*cooler = ON;
		}
		else
		{
			*heater = OFF;
			*cooler = OFF;
		}
	}
	return;
}