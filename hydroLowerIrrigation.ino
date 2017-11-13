


/**
*  @file    hydroLowerIrrigation.ino
*  @author  peter c
*  @date    11/08/2017
*  @version 0.1
*
*
*  @section DESCRIPTION
*  Lower Irrigation control
** @section HISTORY
** 2017ONov8 - created
*/
#include <HardwareSerial.h>


#include <Streaming.h>

#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>



#include <DA_Analoginput.h>
#include <DA_Discreteinput.h>
#include <DA_DiscreteOutput.h>

#include <DA_NonBlockingDelay.h>
#include <flowmeter.h>

#include "unitModbus.h"
// comment out to  include terminal processing for debugging
// #define PROCESS_TERMINAL
//#define TRACE_FLOW_SENSOR
// #define TRACE_1WIRE
// #define TRACE_ANALOGS
// #define TRACE_DISCRETES
 //#define TRACE_MODBUS
// comment out to disable modbus
#define PROCESS_MODBUS
// refresh intervals
#define POLL_CYCLE_SECONDS 2 // sonar and 1-wire refresh rate





DA_DiscreteOutput B1R1_1A_XY_003 = DA_DiscreteOutput(11, LOW); // return pump
DA_DiscreteOutput B1R1_1A_XY_004 = DA_DiscreteOutput(10, LOW); // condensate pump


DA_AnalogInput B1R1_1A_LT_005 = DA_AnalogInput(A1, 0.0, 1023.); // min max
DA_AnalogInput B1R1_1A_LT_006 = DA_AnalogInput(A2, 0.0, 1023.); // min max

DA_DiscreteInput B1R1_1A_ZSC_005 = DA_DiscreteInput(4, DA_DiscreteInput::None, true);

//Flow meter 
#define B1R1_1A_FT_003_PIN  2
#define B1R1_1A_FT_004_PIN  3
//#define FT_004_PIN  D4
#define FLOW_CALC_PERIOD_SECONDS 1 // flow rate calc period

#define ENABLE_FLOW3_SENSOR_INTERRUPTS attachInterrupt(digitalPinToInterrupt(B1R1_1A_FT_003_PIN), onB1R1_1A_FT_003_PulseIn, RISING)
#define DISABLE_FLOW3_SENSOR_INTERRUPTS detachInterrupt(digitalPinToInterrupt(B1R1_1A_FT_003_PIN))
#define ENABLE_FLOW4_SENSOR_INTERRUPTS attachInterrupt(digitalPinToInterrupt(B1R1_1A_FT_004_PIN), onB1R1_1A_FT_004_PulseIn, RISING)
#define DISABLE_FLOW4_SENSOR_INTERRUPTS detachInterrupt(digitalPinToInterrupt(B1R1_1A_FT_004_PIN))

FlowMeter B1R1_1A_FT_004(B1R1_1A_FT_004_PIN, FLOW_CALC_PERIOD_SECONDS); // interrupt pin, calculation period in seconds
FlowMeter B1R1_1A_FT_003(B1R1_1A_FT_003_PIN, FLOW_CALC_PERIOD_SECONDS);

// poll I/O every 2 seconds
DA_NonBlockingDelay pollTimer = DA_NonBlockingDelay( POLL_CYCLE_SECONDS*1000, &doOnPoll);
DA_NonBlockingDelay flowRateTimer = DA_NonBlockingDelay( FLOW_CALC_PERIOD_SECONDS*1000, &doOnCalcFlowRate);


// HEARTBEAT
unsigned int heartBeat = 0;



#ifdef PROCESS_TERMINAL
HardwareSerial *tracePort = & Serial;
#endif

void onB1R1_1A_FT_004_PulseIn()
{
  B1R1_1A_FT_004.handleFlowDetection();
}

void onB1R1_1A_FT_003_PulseIn()
{
  B1R1_1A_FT_003.handleFlowDetection();
}



void setup()
{

#ifdef PROCESS_TERMINAL
  tracePort->begin(9600);
#endif

#ifdef PROCESS_MODBUS
  slave.begin(MB_SPEED);
#endif

  randomSeed(analogRead(3));

  ENABLE_FLOW4_SENSOR_INTERRUPTS;
  ENABLE_FLOW3_SENSOR_INTERRUPTS;
}

void loop()
{

#ifdef PROCESS_MODBUS
  refreshModbusRegisters();
  slave.poll(modbusRegisters, MODBUS_REG_COUNT);
  processModbusCommands();
#endif
pollTimer.refresh();
  flowRateTimer.refresh();
  //*tracePort << "looping" << endl;

}

void processFlowMeter4()
{
    DISABLE_FLOW4_SENSOR_INTERRUPTS;
  B1R1_1A_FT_004.end();

  #ifdef TRACE_FLOW_SENSOR
  *tracePort << "FLOW 4:";
    B1R1_1A_FT_004.serialize( tracePort, true);
   *tracePort << "getCurrentPulses()" << B1R1_1A_FT_004.getCurrentPulses() << endl; 
  #endif

  B1R1_1A_FT_004.begin();
  ENABLE_FLOW4_SENSOR_INTERRUPTS;
}

void processFlowMeter3()
{
    DISABLE_FLOW3_SENSOR_INTERRUPTS;
  B1R1_1A_FT_003.end();
  
  #ifdef TRACE_FLOW_SENSOR
  *tracePort << "FLOW 3:";
    B1R1_1A_FT_003.serialize( tracePort, true);
   *tracePort << "getCurrentPulses()" << B1R1_1A_FT_003.getCurrentPulses() << endl;


  #endif

  B1R1_1A_FT_003.begin();
  ENABLE_FLOW3_SENSOR_INTERRUPTS;
}

void doOnCalcFlowRate()
{
processFlowMeter4();
processFlowMeter3();


  // resetTotalizers();
}

// update sonar and 1-wire DHT-22 readings
void doOnPoll()
{

  doReadAnalogs();
  B1R1_1A_ZSC_005.refresh();


  heartBeat++;

}



void doReadAnalogs()
{
  B1R1_1A_LT_005.refresh();
  B1R1_1A_LT_006.refresh();



#ifdef TRACE_3NALOGS
  B1R1_1A_LT_005.serialize(tracePort, true);
  B1R1_1A_LT_006.serialize(tracePort, true);

 
#endif

}



// 
/*
** Modbus related functions
*/

#ifdef PROCESS_MODBUS
void refreshModbusRegisters()
{



  modbusRegisters[B1R1_1A_LT_005_MB] = B1R1_1A_LT_005.getRawSample();
  modbusRegisters[B1R1_1A_LT_006_MB] = B1R1_1A_LT_006.getRawSample();

  modbusRegisters[B1R1_1A_FT_003_MB] = B1R1_1A_FT_003.getCurrentPulses();
  modbusRegisters[B1R1_1A_FT_004_MB] = B1R1_1A_FT_004.getCurrentPulses();  
  modbusRegisters[HEARTBEAT] = heartBeat;

   modbusRegisters[ B1R1_1A_ZSC_005_MB ] = B1R1_1A_ZSC_005.getSample();
}


bool getModbusCoilValue(unsigned short startAddress, unsigned short bitPos)
{
  // *tracePort << "reading at " << startAddress << " bit offset " << bitPos << "value=" << bitRead(modbusRegisters[startAddress + (int)(bitPos / 16)], bitPos % 16 ) << endl;
  return(bitRead(modbusRegisters[startAddress + (int) (bitPos / 16)], bitPos % 16));
}

void writeModbusCoil(unsigned short startAddress, unsigned short bitPos, bool value)
{
  bitWrite(modbusRegisters[startAddress + (int) (bitPos / 16)], bitPos % 16, value);
}

void checkAndActivateDO(unsigned int bitOffset, DA_DiscreteOutput * aDO)
{
  // look for a change from 0 to 1
  if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, bitOffset))
  {
    aDO->activate();

  #ifdef TRACE_MODBUS
    *tracePort << "Activate DO:";
    aDO->serialize(tracePort, true);
    LED.activate();
  #endif

   // writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, bitOffset, false); // clear the bit
  }
}

void checkAndResetDO(unsigned int bitOffset, DA_DiscreteOutput * aDO)
{
  // look for a change from 0 to 1
  if (!getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, bitOffset))
  {
    aDO->reset();

  #ifdef TRACE_MODBUS
    *tracePort << "Reset DO:";
    aDO->serialize(tracePort, true);
    LED.reset();
  #endif

   // writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, bitOffset, false); // clear the bit
  }
}

void processValveCommands()
{
  checkAndActivateDO(VALVE1_OPEN_CLOSE, & B1R1_1A_XY_003);
  checkAndResetDO(VALVE1_OPEN_CLOSE, & B1R1_1A_XY_003);

  checkAndActivateDO(VALVE2_OPEN_CLOSE, & B1R1_1A_XY_004);
  checkAndResetDO(VALVE2_OPEN_CLOSE, & B1R1_1A_XY_004);

}

void processModbusCommands()
{
  processValveCommands();
}

#endif
