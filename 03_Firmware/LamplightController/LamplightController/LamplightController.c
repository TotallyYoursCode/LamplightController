/*
 * LamplightController.c
 *
 * Created: 28.08.2015 15:19:03
 *  Author: Belyaev Dmitry
 *
 * 2 groups, 4 PWM Chennels each.
 * RS-485, Modbus RTU
 *
 *
 * Fuses: 0xFC 0xD9 0xFF
 */ 

// -1- Includes ---------------------------------------------------------------//
#include "config.h"


// -2- Definitions ------------------------------------------------------------//

// -3- Types ------------------------------------------------------------------//

// -4- Variables --------------------------------------------------------------//
volatile uint8_t tic;
// -5- Prototypes -------------------------------------------------------------//

// -6- Implementations --------------------------------------------------------//

//=============================================================================//
//=============================================================================//
uint8_t PcntToUint8(uint16_t val)
{
	if (val > 100) val = 100;
	double temp = 2.55D;
	temp = temp * val;
	return lroundf(temp);
}
//=============================================================================//
//=============================================================================//
ISR(TIMER0_OVF_vect)
{
	tic++;
}
//=============================================================================//
//=============================================================================//
static void SystemUpTimeRoutine(void)
{
	if (tic >= 225)
	{
		tic = 0;
		
		REGISTERS[UPTIME_LSB]++;
		if (REGISTERS[UPTIME_LSB] == 0xffff)
		{
			REGISTERS[UPTIME_LSB] = 0;
			REGISTERS[UPTIME_MSB]++;
		}
	}
}
//=============================================================================//
//=============================================================================//
static void PortInit(void)								// Input-Output Ports Initialization
{
	DDRB  = 0b00001011; PORTB = 0x03;
	DDRC  = 0b00000000; PORTC = 0x1F;
	DDRD  = 0b01101100; PORTD = 0x00;
}
//=============================================================================//
//=============================================================================//
static void PWMInit(void)								// PWM Outputs Initialization
{
	// Timer 0
	TCNT0 = 0;
	TCCR0A = 0b10100001;
	TCCR0B = 0b00000100; // f/256
	TIMSK0 |= (1<<TOIE0);
	// Timer 2
	TCNT2 = 130;
	TCCR2A = 0b10100001;
	TCCR2B = 0b00000110; // f/256
	
	// 14.745600 Mhz / 510 / 64 = 452 Hz
	// 14.745600 Mhz / 510 / 256 = 113 Hz    
}
//=============================================================================//
//=============================================================================//
static void DipSwitchRead(void)						// Опрос DIP-переключателей
{
	REGISTER_42101 = DEVICE_ADDRESS_DEFAULT + (((!GPIO_A0_read())<<0) | ((!GPIO_A1_read())<<1) | ((!GPIO_A2_read())<<2) | ((!GPIO_A3_read())<<3));
	REGISTER_42102 = GPIO_BR_read() ? BAUD_RATE_DEFAULT : 1152;
}
//=============================================================================//
//=============================================================================//
static void InitDevices(void)						// Инициализация устройств
{
	wdt_enable(WDTO_500MS);							// Инициализация сторожевого таймера 0.5 sec
	cli();											// Запретить глобально ВСЕ прерывания
	PortInit();										// Input-Output Ports Initialization
	PWMInit();										// PWM Outputs Initialization
	DipSwitchRead();								// Опрос DIP-переключателей
	ModbusInit();
	//-----------------
	EnableInterrupts();								// Global enable interrupts
	//-----------------
}
//=============================================================================//
//=============================================================================//
int main(void)
{
	InitDevices();
	for(;;)
	{
		wdt_reset();
		
		switch(REGISTERS[GROUP_SELECT])
		{
			case 0:
				GPIO_OE1_set();
				GPIO_OE2_set();
			break;
			case 1:
				GPIO_OE1_clr();
				GPIO_OE2_set();
			break;
			case 2:
				GPIO_OE1_set();
				GPIO_OE2_clr();
			break;
			case 3:
				GPIO_OE1_clr();
				GPIO_OE2_clr();
			break;
			default:
			break;
		}	
		 PWM1 = PcntToUint8(REGISTERS[PWM1PCNT]);
		 PWM2 = PcntToUint8(REGISTERS[PWM2PCNT]);
		 PWM3 = PcntToUint8(REGISTERS[PWM3PCNT]);
		 PWM4 = PcntToUint8(REGISTERS[PWM4PCNT]);

		SystemUpTimeRoutine();
	}
}
//=============================================================================//
//=============================================================================//
