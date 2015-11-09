/* brief Modbus RTU protocol
 http://masters.donntu.edu.ua/2004/fema/kovalenko/library/art7.html RS-485 для чайников
*/

#ifndef MODBUS_RTU_H_
#define MODBUS_RTU_H_

// -1- Includes ---------------------------------------------------------------//

// -2- Definitions ------------------------------------------------------------//

// -3- Types ------------------------------------------------------------------//
enum
{
	GROUP_SELECT,		// [W]	
	PWM1PCNT,			// [W]	
	PWM2PCNT,			// [W]
	PWM3PCNT,			// [W]
	PWM4PCNT,			// [W]
	UPTIME_LSB,			// [R}
	UPTIME_MSB,			// {R]
	
//----------------------
//----------------------	
	TOTAL_REGISTERS,	// Общее количество регистров
};

typedef enum MB_DataOrder_t
{
	DATA_ORDER_1_2_3_4,
	DATA_ORDER_2_1_4_3,
	DATA_ORDER_4_3_2_1,
	DATA_ORDER_3_4_1_2,
} MB_DataOrder_t;

typedef enum MB_DataType_t
{
	uint16,
	uint32,
	float32,
} MB_DataType_t;

typedef struct
{
	uint8_t uart_no;
	uint16_t baud_rate;
	uint8_t slave_address;
	uint16_t timer_value; // silence_interval
} Modbus_Settings_t;

// -4- Variables --------------------------------------------------------------//
//SYS_Timer_t MB_master_response_timeout;
uint16_t REGISTERS[TOTAL_REGISTERS];
uint8_t REGISTER_42101;		// Address
uint16_t REGISTER_42102;	// Baud rate/100
// UART0
Modbus_Settings_t MB_SLAVE_0;
// Flags
volatile bool ChangeNetworkSettingsFLAG;
// -5- Prototypes -------------------------------------------------------------//
void ModbusInit(void);
void ModbusSlaveRequestParse(Modbus_Settings_t *settings, IOBuffer_t *UARTIO);   // Функция обработки принятого пакета SLAVE
// -6- Implementations --------------------------------------------------------//


#endif /* MODBUS_RTU_H_ */