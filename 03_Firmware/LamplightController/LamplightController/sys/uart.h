/* brief ATmega128 UART with FIFO buffer interface
*/

#ifndef UART_H_
#define UART_H_

// -1- Includes ---------------------------------------------------------------//

// -2- Definitions ------------------------------------------------------------//
#define HAL_UART0_IO_SIZE				0xff	// размер массива для приёма и передачи данных UART0

#define		START_TIMER1	TCCR1B |= (0<<CS12)|(1<<CS11)|(0<<CS10)				// clk/8
#define		STOP_TIMER1		TCCR1B &= ~((1<<CS12)|(1<<CS11)|(1<<CS10))			// Остановка таймера 1

GPIO_PIN(STROBE,				D, 2);			// RS-485 Strobe

// -3- Types ------------------------------------------------------------------//
typedef struct
{
	volatile bool		TXComplete;
	uint16_t			size;
	volatile uint16_t	bytes;
	volatile uint16_t	index;
	uint8_t				*data;
} IOBuffer_t;

// -4- Variables --------------------------------------------------------------//
IOBuffer_t UART0IO;
// -5- Prototypes -------------------------------------------------------------//
void HAL_Uart0BufferInit(void);


// -6- Implementations --------------------------------------------------------//
//=============================================================================//
//=============================================================================//
INLINE void TIM16_WriteTCNT1 (uint16_t val)
{
	ATOMIC_SECTION_ENTER
	TCNT1 = val;
	START_TIMER1;
	ATOMIC_SECTION_LEAVE;
}
//=============================================================================//
//=============================================================================//
#endif /* UART_H_ */