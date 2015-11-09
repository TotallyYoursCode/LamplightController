/* brief Modbus RTU protocol implementation
 http://masters.donntu.edu.ua/2004/fema/kovalenko/library/art7.html RS-485 для чайников
*/

// -1- Includes ---------------------------------------------------------------//
#include "config.h"

// -2- Definitions ------------------------------------------------------------//

// -3- Types ------------------------------------------------------------------//

// -4- Variables --------------------------------------------------------------//
// Timers
//SYS_Timer_t MB_master_response_timeout;
// Register's Map
uint16_t REGISTERS[TOTAL_REGISTERS];
uint8_t REGISTER_42101;		// Регистр для задания адреса устройства
uint16_t REGISTER_42102;	// Регистр для задания скорости связи (baud/100)

// UART0
Modbus_Settings_t MB_SLAVE_0;
// Flags
volatile bool ChangeNetworkSettingsFLAG = false;
// Modbus CRC16 Table (High & Low Bytes)
static const uint8_t auchCRCHi[] PROGMEM = {
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
	0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
	0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
	0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
	0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
	0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40};

static const uint8_t auchCRCLo[] PROGMEM = {
	0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
	0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
	0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
	0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
	0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
	0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
	0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
	0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
	0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
	0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
	0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
	0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
	0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
	0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
	0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
	0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
	0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
0x40};

// -5- Prototypes -------------------------------------------------------------//

// -6- Implementations --------------------------------------------------------//

//=============================================================================//
//=============================================================================//
static uint16_t ModbusSilenceInterval (uint16_t baud_rate)
{
	// В протоколе Modbus RTU сообщение начинает восприниматься как новое после
	// паузы (тишины) на шине длительностью не менее 3,5 символов (14 бит),
	// т.е. величина паузы в секундах зависит от скорости передачи.
	// 3.5*11/9600=0,00401041(6) т.е. более 4 миллисекунд
	// Игнорировать сообщение, если возник интервал тишины в 1.5 символа
	// 1.5*11/9600=0,00171875 т.е. более 1 миллисекунд
	// Для > 19200 1.75 mSec и 0.75 mSec
	double temp;
	uint16_t result;
	temp = (baud_rate<=192) ? 0xffff-(F_CPU/baud_rate/100/8*3.5*11) : 0xffff-(F_CPU/8/1000*1.75);
	result = ceil(temp);
	return result;
}
//=============================================================================//
//=============================================================================//
void ModbusInit(void)
{
	/*
	For interrupt driven USART operation, the global interrupt flag should be cleared 
	(and interrupts globally disabled) when doing the initialization.
	*/
	ATOMIC_SECTION_ENTER
	// UART0 MB SLAVE
	MB_SLAVE_0.uart_no = 0;
	MB_SLAVE_0.slave_address = REGISTER_42101;
	MB_SLAVE_0.baud_rate = REGISTER_42102;
	MB_SLAVE_0.timer_value = ModbusSilenceInterval(MB_SLAVE_0.baud_rate);
	HAL_Uart0BufferInit();
	ATOMIC_SECTION_LEAVE;
}
//=============================================================================//
//=============================================================================//
static uint16_t CRC16Calculation (uint8_t *source_array, uint16_t data_length)
{
	uint8_t uchCRCHi = 0xff;   					// High byte of CRC initialized
	uint8_t uchCRCLo = 0xff;   					// Low byte of CRC initialized
	uint8_t uIndex;           					// It will index into CRC lookup table

	while (data_length--)         						// Pass through message buffer
	{
		uIndex = uchCRCLo ^ *(source_array++);  		// Calculate the CRC
		uchCRCLo = uchCRCHi ^ pgm_read_byte_near(&(auchCRCHi[uIndex]));
		uchCRCHi = pgm_read_byte_near(&(auchCRCLo[uIndex]));
	}
	return (uchCRCHi << 8 | uchCRCLo);
}
//=============================================================================//
//=============================================================================//
static void ModbusPacketError (IOBuffer_t *UARTIO, uint8_t function_code, uint8_t exception_code, uint8_t slave_address) // Функция формирования пакета об ошибоке
{
	uint16_t crc_hi_lo;
	
	UARTIO->data[0] = slave_address;						// Device Address
	UARTIO->data[1] = function_code + 0x80;					// Error_code = Function code + 0x80
	UARTIO->data[2] = exception_code;						// Exception Code (01 or 02 or 03 or 04)
	crc_hi_lo = CRC16Calculation (UARTIO->data, 3);			// CRC16 Calculation
	UARTIO->data[3] = crc_hi_lo & 0xff;						// CRC16 Low
	UARTIO->data[4] = crc_hi_lo >> 8;						// CRC16 High
	
	UARTIO->bytes = 5;
	
	// Коды ошибок "exception_code"
	// 01 - Принятый код функции не может быть обработан на подчинённом устройстве;
	// 02 - Адрес данных, указанных в запросе, не доступен данному подчинённому;
	// 03 - Величина, содержащаяся в поле данных запроса, является недопустимой величиной для подчинённого;
	// 04 - Невосстанавливаемая ошибка имела место, пока подчинённый пытался выполнить затребованное действие;
	// 05 - Подчинённый принял запрос и обрабатывает его, но это требует времени. Чтобы Master не зафиксировал тайм-аут;
	// 06 - Slave занят обработкой команды. Master должен повторить сообщение позже;
	// 07 - Slave не может выполнить программную функцию, принятую в запросе. Этот код возвращается для
	//		неудачного программного запроса использующего функции с номером 13 или 14. Master должен
	//		запросить диагностическую информацию от подчинённого;
	// 08 - Подчинённый пытается читать расширенную память, но обнаружил ошибку паритета.
	//		Обычно в таких случаях требуется ремонт.
}
//=============================================================================//
//=============================================================================//
void ModbusSlaveRequestParse(Modbus_Settings_t *settings, IOBuffer_t *UARTIO)   // Функция обработки принятого пакета SLAVE
{
	uint8_t BytesReceived=0, function_code, quantity_of_data_bytes=0, i=0;
	uint16_t start_register, quantity_of_registers = 0, CRC16_from_packet, CRC16_to_send, data_byte = 0;
	BytesReceived = UARTIO->bytes;
	UARTIO->bytes = 0;
	if (BytesReceived < 3) return; // Явно не Modbus пакет
	if (UARTIO->data[0] != settings->slave_address) return; // Если запрос адресован не этому устройству -> ВЫХОД без ответа

	// При передаче 16 бит контрольной суммы CRC в сообщении, сначала передается младший байт, затем старший.
	CRC16_from_packet = UARTIO->data[BytesReceived-1];	// CRC16 from packet (MSB)
	CRC16_from_packet <<= 8;
	CRC16_from_packet |= UARTIO->data[BytesReceived-2];	// CRC16 from packet (LSB)
	
	if (CRC16_from_packet != CRC16Calculation (UARTIO->data, BytesReceived-2)) return;// Если прикреплённое CRC и рассчитанное не совпадают -> ВЫХОД
	
	function_code = UARTIO->data[1];					// Function code
	
	start_register = UARTIO->data[2];					// Start address   High (MSB)
	start_register <<= 8;
	start_register |= UARTIO->data[3];					// Start address   Low  (LSB)
	
	switch (function_code)								// Обработка кода функции
	{
		case 3:											// -------[MODBUS COMMAND <3>: Read Holding Registers 0x03]-------
		{
			quantity_of_registers = UARTIO->data[4];	// Количество регистров ст. байт
			quantity_of_registers <<= 8;
			quantity_of_registers |= UARTIO->data[5];	// Количество регистров мл. байт
			
			if (((0x0001<=quantity_of_registers)&&(quantity_of_registers<=0x007D)))
			{
				if (((start_register <= TOTAL_REGISTERS-1)&&((start_register+quantity_of_registers-1)<= TOTAL_REGISTERS-1))||(((start_register == 42101)||(start_register == 42102))&&(quantity_of_registers <= 2)))
				{
					// Request Processing
					if (start_register == 42101)
					{
						if (quantity_of_registers == 2)
						{
							UARTIO->data[3] = 0;							// Data Byte High (MSB)
							UARTIO->data[4] = REGISTER_42101;				// Data Byte Low  (LSB)
							quantity_of_data_bytes +=2;
							UARTIO->data[5] = REGISTER_42102 >> 8;			// Data Byte High (MSB)
							UARTIO->data[6] = REGISTER_42102 & 0xff;		// Data Byte Low  (LSB)
							quantity_of_data_bytes +=2;
						}
						else
						{
							UARTIO->data[3] = 0;							// Data Byte High (MSB)
							UARTIO->data[4] = REGISTER_42101;				// Data Byte Low  (LSB)
							quantity_of_data_bytes +=2;
						}
					}
					else
					{
						if (start_register == 42102)
						{
							UARTIO->data[3] = REGISTER_42102 >> 8;			// Data Byte High (MSB)
							UARTIO->data[4] = REGISTER_42102 & 0xff;		// Data Byte Low  (LSB)
							quantity_of_data_bytes +=2;
						}
						else
						{
							for (i=0; i<quantity_of_registers; i++)									// Считываем и записываем указанное количество регистров
							{
								UARTIO->data[3+(2*i)] = REGISTERS[start_register+i] >> 8;				// Data Byte High (MSB)
								UARTIO->data[4+(2*i)] = REGISTERS[start_register+i] & 0xff;				// Data Byte Low  (LSB)
								quantity_of_data_bytes +=2;
							}
						}
					}
				
					UARTIO->data[0] = settings->slave_address;									// 0 byte - Device Address
					UARTIO->data[1] = function_code;											// 1 byte - Function Code
					UARTIO->data[2] = quantity_of_data_bytes;									// 2 byte - Byte Count
					CRC16_to_send = CRC16Calculation (UARTIO->data, quantity_of_data_bytes+3);	// CRC16 Calculation
					UARTIO->data[quantity_of_data_bytes+3] = CRC16_to_send & 0xff;				// CRC16 Low
					UARTIO->data[quantity_of_data_bytes+4] = CRC16_to_send >> 8;				// CRC16 High
					UARTIO->bytes = quantity_of_data_bytes + 5;
				}
				else // ERROR: Starting Address == NOK !AND! Starting Address + Quantity of Registers == NOK
				{
					// 2 - Slave threw exception "Illegal data address"
					ModbusPacketError (UARTIO, function_code, 2, settings->slave_address);		// Функция формирования пакета для ошибки
				}
			}
			else // ERROR: This equation doesn't executable: "0x0001 <=[Quantity of Register]<= 0x007D
			{
				// 3 - Register Address NOK
				ModbusPacketError (UARTIO, function_code, 3, settings->slave_address);							// Функция формирования пакета для ошибки
			}
		}
		break;
		case 6:										// -------[MODBUS COMMAND <06>: Write Single Register 0x06]----------
		{
			if ((start_register <= TOTAL_REGISTERS-1)||(start_register==42101)||(start_register==42102)||(start_register==49990))
			{
				data_byte = UARTIO->data[4];			// Data Byte   High (MSB)
				data_byte <<= 8;
				data_byte |= UARTIO->data[5];			// Data Byte   Low  (LSB)
				
				if (start_register <= TOTAL_REGISTERS-1) REGISTERS[start_register] = data_byte; // Адрес регистра входит в рабочий диапазон регистров устройства.
				else // Адрес регистра вне рабочего диапазона регистров устройства. [Адрес, скорость, пароль]
				{
					switch (start_register)
					{
						case 42101: // Address
							REGISTER_42101 = data_byte & 0xff;					// Address (1 byte)
						break;
						case 42102: // Baud rate /100
							REGISTER_42102 = data_byte;							// Baud rate /100 (2 bytes)
						break;
						case 49990: // Settings apply. !!!!!Execution operation code!!!!! "1234" (1 byte)
							if (data_byte == 1234) ChangeNetworkSettingsFLAG = true; // Установить флаг разрешения принятия новых сетевых настроек (скорость, адрес)
						break;
					}
				}
				// Формирование пакета для ответа
				UARTIO->data[0] = settings->slave_address;				// 0 byte - Device Address
				UARTIO->data[1] = 6;									// 1 byte - Function Code
				UARTIO->data[2] = start_register >> 8;					// Register Address High
				UARTIO->data[3] = start_register & 0xff;				// Register Address Low
				UARTIO->data[4] = data_byte >> 8;						// Register Address High
				UARTIO->data[5] = data_byte & 0xff;						// Register Address Low
				CRC16_to_send = CRC16Calculation(UARTIO->data,6);		// CRC16 Calculation
				UARTIO->data[6] = CRC16_to_send & 0xff;					// CRC16 Low
				UARTIO->data[7] = CRC16_to_send >> 8;					// CRC16 High
				UARTIO->bytes = 8;
			}
			else // ERROR: Register Address NOK
			{
				// 2 - Register Address NOK
				ModbusPacketError (UARTIO, function_code, 2, settings->slave_address);	// Функция формирования пакета для ошибки
			}
		}
		break;
		case 16:									// -------[MODBUS COMMAND <16>: Write Multiple Registers 0x10]-------
		{
			quantity_of_registers = UARTIO->data[4];	// Количество регистров ст. байт
			quantity_of_registers <<= 8;
			quantity_of_registers |= UARTIO->data[5];	// Количество регистров мл. байт
			
			quantity_of_data_bytes = UARTIO->data[6];	// Количество байт данных
			if (((0x1<=quantity_of_registers)&&(quantity_of_registers<=0x7B))&&(quantity_of_data_bytes==quantity_of_registers*2))
			{
				if ((start_register <= TOTAL_REGISTERS-1)&&((start_register+quantity_of_registers-1)<= TOTAL_REGISTERS-1))
				{
					for (i=0; i<quantity_of_registers; i++)			// Считываем указанное количество регистров
					{
						REGISTERS[start_register+i] = UARTIO->data[7+(2*i)];	// Data Byte High (MSB)
						REGISTERS[start_register+i] <<= 8;
						REGISTERS[start_register+i] |= UARTIO->data[8+(2*i)];	// Data Byte Low  (LSB)
					}
					// Формирование пакета для ответа
					UARTIO->data[0] = settings->slave_address;			// 0 byte - Device Address
					UARTIO->data[1] = 16;								// 1 byte - Function Code
					UARTIO->data[2] = start_register >> 8;				// Register Address High
					UARTIO->data[3] = start_register & 0xff;			// Register Address Low
					UARTIO->data[4] = quantity_of_registers >> 8;		// Register Address High
					UARTIO->data[5] = quantity_of_registers & 0xff;		// Register Address Low
					CRC16_to_send = CRC16Calculation (UARTIO->data,6);	// CRC16 Calculation
					UARTIO->data[6] = CRC16_to_send & 0xff;				// CRC16 Low
					UARTIO->data[7] = CRC16_to_send >> 8;				// CRC16 High
					UARTIO->bytes = 8;
				}
				else // ERROR: Starting Address == NOK !AND! Starting Address + Quantity of Registers == NOK
				{
					// 2 - Slave threw exception "Illegal data address"
					ModbusPacketError (UARTIO, function_code, 2, settings->slave_address);// Функция формирования пакета об ошибке
				}
			}
			else // ERROR: This equation doesn't executable: "0x0001 <=[Quantity of Register]<= 0x007B !AND! Byte Count == Quantity of Registers *2"
			{
				// 3 - Register Address NOK
				ModbusPacketError (UARTIO, function_code, 3, settings->slave_address);	// Функция формирования пакета об ошибке
			}
			break;
		}
		default:									// -------[MODBUS COMMAND ERROR: Modbus Command doesn't supported]-------
			// 1 - Function code doesn't supported
			ModbusPacketError (UARTIO, function_code, 1, settings->slave_address);		// Функция формирования пакета об ошибке
		break;
	}
}
//=============================================================================//
//=============================================================================//