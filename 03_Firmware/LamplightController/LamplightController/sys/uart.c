/* brief ATmega UART with FIFO buffer implementation

*/


// -1- Includes ---------------------------------------------------------------//
#include "config.h"

// -2- Definitions ------------------------------------------------------------//

// -3- Types ------------------------------------------------------------------//

// -4- Variables --------------------------------------------------------------//
// UART0
IOBuffer_t UART0IO;
static uint8_t UART0_Data[HAL_UART0_IO_SIZE+1];

// -5- Prototypes -------------------------------------------------------------//

// -6- Implementations --------------------------------------------------------//
//=============================================================================//
//=============================================================================//
static void timer1_init(void)								// Инициализация Timer1[16bit] Normal Mode
{
	TCCR1A = 0b00000000;	// COM1A1 COM1A0 COM1B1 COM1B0 - - WGM11 WGM10
	TCCR1B = 0b00000000;	// ICNC1  ICES1     –    WGM13  WGM12  CS12   CS11  CS10   Fcpu/8 prescaler
	TCCR1C = 0b00000000;	// FOC1A   FOC1B  FOC1C    –      –      –      –     –
	TCNT1H = 0b00000000;	// Timer/Counter1 High Register
	TCNT1L = 0b00000000;	// Timer/Counter1 Low Register
	OCR1AH = 0b00000000;
	OCR1AL = 0b00000000;	// Output Compare Register 1 A
	OCR1BH = 0b00000000;
	OCR1BL = 0b00000000;	// Output Compare Register 1 B
	ICR1H  = 0b00000000;
	ICR1L  = 0b00000000;	// ICR1H and ICR1L – Input Capture Register 1

	TIMSK1 |= (1<<TOIE1);	// Timer/Counter1, Overflow Interrupt Enable
}
//=============================================================================//
//=============================================================================//
static void USART0_Init(uint16_t baudrate)                        // Функция инициализации USART0 (RS232)
{
	uint16_t ubrr = (F_CPU/baudrate/100/16)-1; // UBRR Contents of the UBRRH and UBRRL Registers, (0 - 4095)
	UBRR0H = (ubrr >> 8) & 0xff;
	UBRR0L = (ubrr & 0xff);
	UCSR0B = (1<<RXCIE0)|(0<<TXCIE0)|(0<<UDRIE0)|(1<<RXEN0)|(0<<TXEN0)|(0<<UCSZ02);
	// Asynchronous Operation Set frame format: 8data, 1stop bit, Parity none
	UCSR0C = (0<<UMSEL01)|(0<<UMSEL00)|(0<<UPM01)|(0<<UPM00)|(0<<USBS0)|(1<<UCSZ01)|(1<<UCSZ00);	// USART Control and Status Register C – UCSRnC
}
//=============================================================================//
//=============================================================================//
void HAL_Uart0BufferInit(void)
{
	/*
	For interrupt driven USART operation, the global interrupt flag should be cleared 
	(and interrupts globally disabled) when doing the initialization.
	*/
	
	UART0IO.data = UART0_Data;
	UART0IO.size = HAL_UART0_IO_SIZE;
	UART0IO.bytes = 0;
	UART0IO.index = 0;
	UART0IO.TXComplete = true;
	
	// RS-485
	GPIO_STROBE_out();
	GPIO_STROBE_clr();
	
	timer1_init();
	USART0_Init(MB_SLAVE_0.baud_rate);
}
//=============================================================================//
//=============================================================================//
ISR(USART_RX_vect)									// Прерывание по факту получения нового байта USART0 (RS232)
{
	uint8_t status, byte;
	STOP_TIMER1;
	status = UCSR0A;
	byte = UDR0;
	// Frame Error; Data OverRun; Parity Error
	if (status & ((1 << FE0) | (1 << DOR0) | (1 << UPE0)))
	{
		// Frame Error
		while ( UCSR0A & (1<<RXC0) ) byte = UDR0; // Flushing the Receive Buffer
		UART0IO.bytes = 0;
		UART0IO.index = 0;
	}
	else
	{
		// Frame Ok
		if (UART0IO.index >= UART0IO.size)
		{
			// Packet is too long
			UART0IO.bytes = 0;
			UART0IO.index = 0;
		}
		else
		{
			UART0IO.data[UART0IO.index++] = byte;
			TIM16_WriteTCNT1 (MB_SLAVE_0.timer_value);
		}
	}
}
//=============================================================================//
//=============================================================================//
ISR(TIMER1_OVF_vect)								// Прерывание по переполнению Таймер1 (16bit) Modbus
{
	clrbit(UCSR0B,RXEN0);	// Отключить приёмник
	clrbit(UCSR0B,RXCIE0);	// Отключить прерывание по RX
	STOP_TIMER1;
	UART0IO.bytes = UART0IO.index;
	UART0IO.index = 0;
	ModbusSlaveRequestParse(&MB_SLAVE_0, &UART0IO); // Функция обработки принятого пакета SLAVE; // Разборка полученных данных
	// Передача первого байта
	if (UART0IO.bytes)		// Если пакет был не наш, то bytes = 0
	{
		UART0IO.TXComplete = false;
		
		setbit(UCSR0B,TXEN0);		// Включить передатчик
		GPIO_STROBE_set();			// Установить "STROBE" в 1 - Включить передатчик, отключить приёмник
		
		setbit(UCSR0A,TXC0);		// Note that the TXC flag must be cleared before each transmission (before UDR is written) if it is used for this purpose.
		
		UDR0 = UART0IO.data[UART0IO.index++];
		UART0IO.bytes--;
		
		if (UART0IO.bytes)
		{
			// Есть ещё байты на передачу
			setbit(UCSR0B,UDRIE0);	// Включить прерывание USART Data Register Empty Interrupt для оставшихся байт
		}
		else
		{
			// Был всего 1 байт на передачу
			setbit(UCSR0B,TXCIE0);	// Включить прерывание USART Transmit Complete Interrupt
		}
	}
	else // Нечего передавать. Слушаем дальше
	{
		setbit(UCSR0B,RXEN0);	// Включить приёмник
		setbit(UCSR0A,RXC0);	// flag must be cleared
		setbit(UCSR0B,RXCIE0);	// Включить прерывание по RX
	}
}
//=============================================================================//
//=============================================================================//
ISR(USART_UDRE_vect)
{
	if (UART0IO.bytes)
	{
		setbit(UCSR0A,TXC0); // Note that the TXC flag must be cleared before each transmission (before UDR is written) if it is used for this purpose.
		UDR0 = UART0IO.data[UART0IO.index++];
		UART0IO.bytes--;
	}
	else // Больше нет очереди на передачу
	{
		clrbit(UCSR0B,UDRIE0);
		setbit(UCSR0B,TXCIE0);
	}
}
//=============================================================================//
//=============================================================================//
ISR(USART_TX_vect)				// Прерывание по факту передачи последнего байта USART0 (RS232) TX Complete Interrupt
{
	// Для RS 485 необходимо дождаться окончания передачи последнего байта, и только затем отключить передатчик и включить приёмник
	GPIO_STROBE_clr();	// сбросить "STROBE1" в 0 - Отключить передатчик, включить приёмник
	clrbit(UCSR0B,TXCIE0);
	clrbit(UCSR0B,TXEN0);		// Отключить передатчик
	
	UART0IO.TXComplete = true;
	UART0IO.index = 0;
	
	setbit(UCSR0B,RXEN0);		// Включить приёмник
	setbit(UCSR0B,RXCIE0);		// Включить прерывание по RX
}	
//=============================================================================//
//=============================================================================//