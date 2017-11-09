/**
 * @file 	plantModbus.h
 * @version     0.1
 * @date        2017May3
 * @author 	pjc

 *
 * @description
 *  Helpers for plant lighting and control using Modbus
 *
 * Using arduino modbus implementation @
 * https://github.com/smarmengol/Modbus-Master-Slave-for-Arduino
*/


#include <hydroModbusCommon.h>


// specific read holding registers to unit
//#define FUTURE CUSTOM_HR_START_READ

// Because of 485 traffic requests have been made so that holding registers addressing are contiguous
// the concept of a common memory map and an area for custom has been abandoned
// and addressing will  be retrofited

#define B1R1_1A_LT_005_MB 11
#define B1R1_1A_ZSC_005_MB B1R1_1A_LT_005_MB + 1 
#define B1R1_1A_LT_006_MB  B1R1_1A_ZSC_005_MB  + 1
#define B1R1_1A_FT_003_MB B1R1_1A_LT_006_MB   + 1
#define B1R1_1A_FT_004_MB B1R1_1A_FT_003_MB   + 1
#define HEARTBEAT 		  B1R1_1A_FT_004_MB   + 1




// specific write holding registers to unit
#define FUTURES CUSTOM_HR_START_WRITE


// 
// write analogs/sp specific to units
//  HOLDING_REGISTER_WRITE_OFFSET + LAST #DEFINE IN THE LIST ON TOP.
//  IF YOU ADD MORE ENSURE THE CHANGE IS MADE HERE 



#define MODBUS_REG_COUNT HOLDING_REGISTER_WRITE_OFFSET + CUSTOM_HR_START_READ + 1
uint16_t modbusRegisters[MODBUS_REG_COUNT];

#define MB_SPEED 			    19200
#define MB_SLAVE_ID				24
#define MB_SERIAL_PORT			0
#define MB_MAX485_PIN			6  // set to zero for RS-232



Modbus slave(MB_SLAVE_ID, MB_SERIAL_PORT,MB_MAX485_PIN); 


