#ifndef CONFIG_H_
#define CONFIG_H_

#define F_CPU 14745600UL
//----------------------------------------------------------------------------------
#include <avr/io.h>
//#include <avr/eeprom.h>
//#include <stdint.h>
#include <stdbool.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
//#include <string.h>
//#include <stdio.h>
//#include <stdlib.h>

#include "sys/my_macroses.h"
#include "sys/uart.h"
#include "nwk/modbus_rtu.h"

// -1- Connection ---------------------------------------------------------------//
//		Name					PORT,PIN

// PORT B
GPIO_PIN(OE1,					B, 0);	// Output Enable Group 1
GPIO_PIN(OE2,					B, 1);	// Output Enable Group 2
GPIO_PIN(PWMO1,					B, 3);	// PWM Output 1

GPIO_PIN(A0,					C, 0);	// Address 0
GPIO_PIN(A1,					C, 1);	// Address 1
GPIO_PIN(A2,					C, 2);	// Address 2
GPIO_PIN(A3,					C, 3);	// Address 3
GPIO_PIN(BR,					C, 4);	// Baud Rate Select

GPIO_PIN(PWMO4,					D, 3);	// PWM Output 4
GPIO_PIN(PWMO3,					D, 5);	// PWM Output 3
GPIO_PIN(PWMO2,					D, 6);	// PWM Output 2


//------------------------------DEFAULT_SETTINGS------------------------------------
#define BAUD_RATE_DEFAULT					96		// * 100
#define DEVICE_ADDRESS_DEFAULT				1

#define PWM1	OCR2A
#define PWM2	OCR0A
#define PWM3	OCR0B
#define PWM4	OCR2B

// Template

/* brief ***** implementation
*/

// -1- Includes ---------------------------------------------------------------//

// -2- Definitions ------------------------------------------------------------//

// -3- Types ------------------------------------------------------------------//

// -4- Variables --------------------------------------------------------------//

// -5- Prototypes -------------------------------------------------------------//

// -6- Implementations --------------------------------------------------------//

//=============================================================================//
//=============================================================================//





#endif /* CONFIG_H_ */