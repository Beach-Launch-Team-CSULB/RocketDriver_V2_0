// RocketDriver V2.0 Propulsion Control and Data Acquisition - Embedded System Node Program
// Originally by Dan Morgan and Mat Arnold
// For Renegade, Beach Launch Team, Dan Morgan + Brandon Summers' personal machinations and more
//
//
// -------------------------------------------------------------
// Use top level define conditional to determine which system the code is operating
// Maintain definition header sets for a given propulsion system and update here accordingly
#define PASABANG

//----- Pasafire BangBang Static Fire Stand/Vehicle -----
#ifdef PASABANG
//#include "ValveDefinitionsPasaBang.h"
#include "PyroDefinitionsPasaBang.h"
#include "AutoSequenceDefinitionsPasaBang.h"
#include "SensorDefinitionsPasaBang.h"
#include "TankPressControllerDefinitionsPasaBang.h"
#include "EngineControllerDefinitionsPasaBang.h"
#include "ControlFunctionsPasaBang.h"
#include "ALARASensorControllerDefinitionsPasaBang.h"
#endif

//----- Renegade Static Fire Stand -----
#ifdef RENEGADESF
#include "ValveDefinitionsRenegadeSF.h"
#include "PyroDefinitionsRenegadeSF.h"
#include "AutoSequenceDefinitionsRenegadeSF.h"
#include "SensorDefinitionsRenegadeSF.h"
#include "TankPressControllerDefinitionsRenegadeSF.h"
#include "EngineControllerDefinitionsRenegadeSF.h"
#include "ControlFunctionsRenegadeSF.h"
#include "ALARASensorControllerDefinitionsRenegadeSF.h"
#endif

//----- BabyShark -----
#ifdef BABYSHARK
#include "ValveDefinitionsRenegadeBabyShark.h"
#include "PyroDefinitionsRenegadeBabyShark.h"
#include "AutoSequenceDefinitionsRenegadeBabyShark.h"
#include "SensorDefinitionsRenegadeBabyShark.h"
#include "TankPressControllerDefinitionsBabyShark.h"
#include "EngineControllerDefinitionsBabyShark.h"
#include "ControlFunctionsRenegadeBabyShark.h"
#include "ALARASensorControllerDefinitionsBabyShark.h"
#endif
// -------------------------------------------------------------

// ----- "COTS" includes ----- //
#include <Arduino.h>
#include <EEPROM.h>
#include <ADC.h>
#include <ADC_util.h>
#include <FlexCAN.h>
#include <kinetis_flexcan.h>
#include <WireKinetis.h>
#include <Wire.h>
#include "Adafruit_MCP9808.h"
#include <InternalTemperature.h>
#include <array>
#include <string>
#include <list>
#include <unordered_map>
using std::string;
#include <IntervalTimer.h>


#include "ALARAUtilityFunctions.h"
#include "ToMillisTimeTracker.h"
#include "CANRead.h"
#include "CANWrite.h"
#include "OperationFunctionTemplates.h"
#include "ALARApinDefines.h"
#include "fluidSystemSimulation.h"

//Trying to figure out RTC stuff with these libs
#include <TimeLib.h>
#include <DS1307RTC.h>

#define PROPULSIONSYSNODEIDPRESET 8;     //NOT in use normally, for testing with the address IO register inactive

// Timer for setting main loop debugging print rate
elapsedMillis mainLoopTestingTimer;
elapsedMillis ezModeControllerTimer;
elapsedMillis commandExecuteTimer;

//For use in doing serial inputs as CAN commands for testing
uint8_t fakeCANmsg; //CAN2.0 byte array, first 4 bytes are ID field for full extended ID compatibility
uint8_t fakeCanIterator = 0;

bool mainloopprints = true;

bool localNodeResetFlag = false; //flag to trigger register reset from commanded reset over CAN
bool abortHaltFlag; //creates halt flag that is a backup override of state machine, am I currently using it?

///// NODE DECLARATION /////
//default sets to max nodeID intentionally to be bogus until otherwise set
ALARASN thisALARA;
uint8_t ALARAnodeID = 3;                      // ALARA hardware node address
uint8_t ALARAnodeIDfromEEPROM;            //nodeID read out of EEPROM
uint32_t ALARAnodeIDfromEEPROM_errorFlag;            //nodeID read out of EEPROM
bool nodeIDdeterminefromEEPROM;           //boolean flag for if startup is to run the nodeID detect read
uint32_t nodeIDdeterminefromEEPROM_errorFlag;
uint8_t PropulsionSysNodeID = 8;              //engine node = 2, prop node = 3, Pasafire node = 8
uint8_t PropulsionSysNodeIDfromEEPROM;    //PropulsionSysNodeID read out of EEPROM
uint32_t PropulsionSysNodeIDfromEEPROM_errorFlag;    //PropulsionSysNodeID read out of EEPROM

const uint8_t configVerificationKey = 166; //upgrade to a map later

///// WATCHDOG SYSTEM /////
elapsedMillis propulsionControlWatchdog;                  // Watchdog timer that must be reset by ground control over bus to prevent an autovent
uint32_t propulsionControlWatchdogVentTime = 120000;   // 120 seconds in millis gives two minutes to reestablish control before autovent, DISABLE IN FLIGHT

/* ///// ADC /////
ADC* adc = new ADC();
 */
///// LED /////
elapsedMillis sinceLED;

///// CAN /////
CAN_message_t message;
CAN_message_t rxmsg;
CAN_message_t extended;
bool CANSensorReportConverted = false;
bool NewCommandMessage{false};
bool NewConfigMessage{false};

const int CAN2busSpeed = 500000; // CAN2.0 baudrate - do not set above 500000 for full distance run bunker to pad

bool startup{true}; // bool for storing if this is the first loop on startup, ESSENTIAL FOR STATE MACHINE OPERATION (maybe not anymore?)

uint32_t loopCount {0};// for debugging

// Set the global command, and global state
Command currentCommand{command_NOCOMMAND}; 
VehicleState currentVehicleState{VehicleState::passive};
VehicleState priorVehicleState;
MissionState currentMissionState(MissionState::passive);
MissionState priorMissionState;

commandMSG currentCommandMSG{};
configMSG currentConfigMSG{};

uint32_t vehicleStateAddressfromEEPROM_errorFlag;
uint32_t missionStateAddressfromEEPROM_errorFlag;

//AutoSequence stuff for main
int64_t currentCountdownForMain;

////// Set EEPROM addresses
// Change these up occasionally to reduce write cycle wear on the same bytes
// I could use EEPROM itself to store current start byte of my data and automate iterating this. Good idea for future upgrade.
uint16_t vehicleStateAddress1{4};
uint16_t vehicleStateAddress2{5};
uint16_t vehicleStateAddress3{6};
uint16_t missionStateAddress1{7};
uint16_t missionStateAddress2{8};
uint16_t missionStateAddress3{9};
uint16_t PropulsionSysNodeIDAddress1{16};
uint16_t PropulsionSysNodeIDAddress2{17};
uint16_t PropulsionSysNodeIDAddress3{18};
uint16_t nodeIDDetermineAddress1{19};
uint16_t nodeIDDetermineAddress2{20};
uint16_t nodeIDDetermineAddress3{21};
uint16_t NodeIDAddress1{22};
uint16_t NodeIDAddress2{23};
uint16_t NodeIDAddress3{24};

///// Temp Sensor for TC Cold Junction /////        ----- Move into sensor classes (if this is even retained)
//Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();
//int roundedtemp;

//-------------------------------------------------------//
void setup() {
  startup = true;   // Necessary to set startup to true for the code loop so it does one startup loop for the state machine before entering regular loop behavior

  // ----- MUX Setups for ALARA -----
  // Board Addressing MUX
  MUXSetup(true, ALARA_DIGITAL_ADDRESS_1, ALARA_DIGITAL_ADDRESS_2, ALARA_DIGITAL_ADDRESS_3, ALARA_DIGITAL_ADDRESS_4);
  
  // MOVE NODEDETECTSHITHERE!!!
  // Check map for ALARASN configutation
  lookupALARASNmap(thisALARA, ALARAnodeID);

  // NOR Flash CS pin MUX
  MUXSetup(false, ALARA_NOR_S0, ALARA_NOR_S1, ALARA_NOR_S2);
///// ----- Insert a board rev check to pin defines here, if it fails disable our GPIO? ------ //

  // -----Read Last State off eeprom and update -----
  currentVehicleState = static_cast<VehicleState>(tripleEEPROMread(vehicleStateAddress1, vehicleStateAddress2, vehicleStateAddress3, vehicleStateAddressfromEEPROM_errorFlag));
  currentMissionState = static_cast<MissionState>(tripleEEPROMread(missionStateAddress1, missionStateAddress2, missionStateAddress3, missionStateAddressfromEEPROM_errorFlag));
  
  PropulsionSysNodeIDfromEEPROM = tripleEEPROMread(PropulsionSysNodeIDAddress1, PropulsionSysNodeIDAddress2, PropulsionSysNodeIDAddress3, PropulsionSysNodeIDfromEEPROM_errorFlag);
  nodeIDdeterminefromEEPROM = tripleEEPROMread(nodeIDDetermineAddress1, nodeIDDetermineAddress2, nodeIDDetermineAddress3, nodeIDdeterminefromEEPROM_errorFlag);
  startupStateCheck(currentVehicleState, currentCommand);

  // ----- Run the Node ID Detection Function -----
  //PropulsionSysNodeID = NodeIDDetect(nodeID, nodeIDdeterminefromEEPROM, nodeIDfromEEPROM); // - OVERHAUL WITH NEW FUNCTION AND SYSTEM
  //PropulsionSysNodeID = PROPULSIONSYSNODEIDPRESET;       //For manually assigning NodeID isntead of the address read, make sure to comment out for operational use
  //PropulsionSysNodeID = thisALARA.propulsionSysNodeID;
  // Write 0 to byte for nodeIDDetermineAddress after reading it after a reset
  //tripleEEPROMwrite(0, nodeIDDetermineAddress1, nodeIDDetermineAddress2, nodeIDDetermineAddress3);
  //CHEATER OVERRIDE!!!!!
  //PropulsionSysNodeID = 8;

  // -----Initialize CAN-----
  vectorCommandBufferInitialize(32);  //number of vector elements memory is reserved for
  vectorConfigBufferInitialize(32); //number of vector elements memory is reserved for
  // CAN0 - FlexCAN 2.0 bus
  Can0.begin(CAN2busSpeed);

  // -----Initialize ADCs-----
  MCUADCSetup();

  // -----Run Valve PropulsionSysNodeID Check-----
  ValveNodeIDCheck(valveArray, PropulsionSysNodeID);

  // -----Run Valve PropulsionSysNodeID Check-----
  PyroNodeIDCheck(pyroArray, PropulsionSysNodeID);

  // -----Run Sensor PropulsionSysNodeID Check-----
  SensorNodeIDCheck(sensorArray, PropulsionSysNodeID);

  // -----Run Valve Setup-----
  valveSetUp(valveArray);

  // -----Run Valve Setup-----
  pyroSetUp(pyroArray);

  // -----Run Valve Setup-----
  engineControllerSetup(engineControllerArray);

  // -----Run Valve Setup-----
  tankPressControllerSetup(tankPressControllerArray);

  // -----Run AutoSequence Setup-----
  autoSequenceSetUp(autoSequenceArray);
  
  // -----Run Sensor Setup -----
  sensorSetUp(sensorArray);
  // ----- Set Controller Dependent Sensor Settings -----
  controllerSensorSetup(valveArray, pyroArray, autoSequenceArray, sensorArray, tankPressControllerArray, engineControllerArray);


  // pin setup
  // NEEDS LOTS OF UPDATES FOR ALARA V2
  pinMode(LED_BUILTIN, OUTPUT);
  

  Serial.begin(9600); // Value is arbitrary on Teensy, it will initialize at the MCU dictate baud rate regardless what you feed this

}

void loop() 
{
  //Display the node number with serial print statement start of each loop
  //Serial.print("PropulsionSysNodeID: ");
  //Serial.println(PropulsionSysNodeID);

///// Custom function for tracking miliseconds and seconds level system time for timestamping /////
myTimeTrackingFunction();
/* Serial.print(second());
Serial.print(" : ");
Serial.println(timeSubSecondsMicros); */

  // --- Read CAN bus and update current command ---
  if(CANread(Can0, configVerificationKey, NewConfigMessage, currentCommand, currentConfigMSG, PropulsionSysNodeID) && !startup) // do not execute on the first loop
  {
    Serial.print("CAN Message Recieved: ");
    Serial.println(currentCommand); //currently only does the command not any message
  }
  if(SerialAsCANread())
  {
    Serial.println("Command Entered");
  }
  
  //while (Serial.available())
  //{
/*   {
    Serial.print("available");
    Serial.print(Serial.available());
    Serial.println();
  for (fakeCanIterator; fakeCanIterator < Serial.available(); fakeCanIterator++)
  {
    fakeCANmsg[fakeCanIterator] = Serial.read();
    Serial.print("i");
    Serial.println(fakeCanIterator);
  }
  }
  //else {fakeCanIterator = 0;}
  

    Serial.print("fakeCANmsg");
    Serial.println();
  for (size_t i = 0; i < 12; i++)
  {
    Serial.print(fakeCANmsg[i]);
    Serial.print(" : ");
  }
    Serial.println(); */

    //Serial.print(Serial.read());
    //Serial.println();
    //fakeCANmsg = Serial.read();

      //if(fakeCANmsg[0]  < command_SIZE) //enter 0 inter serial to trigger command read
      //{
          //add in code here to prompt for command code and update current command from this
          //Serial.println("Enter Command Byte");
          //CurrentCommand = Serial.read();
              
              //if(fakeCANmsg < command_SIZE)                                           // this checks if the message at that location in the buffer could be a valid command
              //{
                  //currentCommand = static_cast<Command>(fakeCANmsg);
                  //NewCommandMessage = true;
              //}
          //Serial.println("Command Entered");
        //}
    //mainLoopTestingTimer = 0;
    //}

///// ------ MESSAGE PROCESSING BLOCK ----- /////  
  //pull next command message from buffer, if there is one
  if (!NewCommandMessage)
  {
  NewCommandMessage = readRemoveVectorBuffer(currentCommandMSG);
  currentCommand = currentCommandMSG.commandEnum;
  }

  //process command
  //if (commandExecuteTimer >= 2000)
  //{
  commandExecute(currentVehicleState, priorVehicleState, currentCommand, NewCommandMessage, autoSequenceArray, sensorArray, tankPressControllerArray, engineControllerArray);
  //}
  //pull next config message from buffer, if there is one
  NewConfigMessage = readRemoveVectorBuffer(currentConfigMSG);
  //process config message
  configMSGread(currentConfigMSG, NewConfigMessage, valveArray, pyroArray, sensorArray, autoSequenceArray, tankPressControllerArray, engineControllerArray, waterGoesVroom);
///// ------------------------------------ /////  

  if (ezModeControllerTimer >= 5) // 5 = 200Hz controller rate
  {
  // -----Process Commands Here-----
  vehicleStateMachine(currentVehicleState, priorVehicleState, currentCommand, autoSequenceArray, sensorArray, tankPressControllerArray, engineControllerArray, abortHaltFlag);
  controllerDataSync(valveArray, pyroArray, autoSequenceArray, sensorArray, tankPressControllerArray, engineControllerArray);
  autoSequenceTasks(autoSequenceArray, PropulsionSysNodeID);
  tankPressControllerTasks(tankPressControllerArray, PropulsionSysNodeID, IgnitionAutoSequence);
  engineControllerTasks(engineControllerArray, PropulsionSysNodeID, IgnitionAutoSequence);
  //autoSequenceTasks(autoSequenceArray, PropulsionSysNodeID);
  controllerDeviceSync(currentVehicleState, priorVehicleState, currentCommand, valveArray, pyroArray, autoSequenceArray, sensorArray, tankPressControllerArray, engineControllerArray, abortHaltFlag);
  //fluid sim stuff
  if (currentVehicleState == VehicleState::passive)
  {
    waterGoesVroom.resetSim();
  }

  waterGoesVroom.fluidSystemUpdate();
  //Serial.println(waterGoesVroom.FuelTank.CurrPressure/6895);
  waterGoesVroom.FuelTank.SetValveStates(FuelBang.getState(),FuelMV.getState(),FuelVent.getState());
  waterGoesVroom.LoxTank.SetValveStates(LoxBang.getState(),LoxMV.getState(),LoxVent.getState());
  
  ezModeControllerTimer = 0;
  }

  // -----Advance needed controller system tasks (tank press controllers, ignition autosequence, . ..) ----- //
  // -----Advance needed propulsion system tasks (valve, pyro, sensors, . ..) ----- //
  cli(); // disables interrupts to ensure complete propulsion output state is driven
  valveTasks(valveArray, PropulsionSysNodeID);
  pyroTasks(pyroArray, PropulsionSysNodeID);
  sei(); // reenables interrupts after propulsion output state set is completed
  //sensorTasks(sensorArray, adc, rocketDriverSeconds, rocketDriverMicros, PropulsionSysNodeID);
  sensorTasks(sensorArray, rocketDriverSeconds, rocketDriverMicros, PropulsionSysNodeID);

  // -----Update States on EEPROM -----
  //change to only write if something new to write!!! Make wrapper function that checks for new info?
  //tripleEEPROMwrite(static_cast<uint8_t>(currentVehicleState), vehicleStateAddress1, vehicleStateAddress2, vehicleStateAddress3);

/*   // CAN State Report and Sensor data send Functions
  CAN2PropSystemStateReport(Can0, currentVehicleState, currentCommand, valveArray, pyroArray, abortHaltFlag, PropulsionSysNodeID);
  CAN2AutosequenceTimerReport(Can0, autoSequenceArray, abortHaltFlag, PropulsionSysNodeID);
  CAN2SensorArraySend(Can0, sensorArray, PropulsionSysNodeID, CANSensorReportConverted);
 */
  // Reset function to reboot Teensy with internal reset register
  // Need to figure out how to rework using this feature with reworked ID system
  TeensyInternalReset(localNodeResetFlag, nodeIDDetermineAddress1, nodeIDDetermineAddress2, nodeIDDetermineAddress3);

  if (mainLoopTestingTimer >= 500)
  {

  if (mainloopprints)
  {
  //Main Loop state and command print statements - for testing only - TEMPORARY BULLSHIT
  Serial.print("prop node ID : ");
  Serial.print(PropulsionSysNodeID);
  Serial.print(" currentVehicleState :");
  Serial.println(static_cast<uint8_t>(currentVehicleState));
  Serial.print(" currentCommand :");
  Serial.println(currentCommand);

vectorBufferPrintout();

  Serial.print(" currentConfigMSG :");
  Serial.print(" targetID :");
  Serial.print(currentConfigMSG.TargetObjectID);
  Serial.print(" settingID:");
  Serial.print(currentConfigMSG.ObjectSettingID);
  Serial.print(" float:");
  Serial.print(currentConfigMSG.floatValue);
  Serial.print(" uint8:");
  Serial.print(currentConfigMSG.uint8Value);

  Serial.println();
  Serial.print(waterGoesVroom.TimeDelta, 10);
  Serial.print(" : ");
  Serial.print(waterGoesVroom.FuelTank.CurrPressure/6895, 10);
  Serial.print(" : ");
  Serial.print(waterGoesVroom.LoxTank.CurrPressure/6895, 10);
  Serial.print(" : ");
  Serial.print(waterGoesVroom.HiPressTank.CurrPressure/6895, 10);
  Serial.println(" fluid sim update ran");

    for(auto tankPressController : tankPressControllerArray)
    {
            Serial.print( ": TankControllerState: ");
            Serial.print(static_cast<uint8_t>(tankPressController->getState()));
            Serial.println(": ");
            Serial.print(static_cast<uint8_t>(tankPressController->getPrimaryPressValveState()));
            Serial.print(": ");
            Serial.print(static_cast<uint8_t>(tankPressController->getPressLineVentState()));
            Serial.print(": ");
            Serial.print(static_cast<uint8_t>(tankPressController->getTankVentState()));
            if (tankPressController->getIsBang())
            {
            Serial.print(": Target");
            Serial.print(tankPressController->getTargetValue(),10);
            Serial.print(": K_p");
            Serial.print(tankPressController->getKp(),10);
            Serial.print(": K_i");
            Serial.print(tankPressController->getKi(),10);
            Serial.print(": K_d");            
            Serial.print(tankPressController->getKd(),10);
            Serial.print(": e_p");
            Serial.print(tankPressController->getPfunc(),10);
            Serial.print(": e_i");
            Serial.print(tankPressController->getIfunc(),10);
            Serial.print(": e_d");            
            Serial.print(tankPressController->getDfunc(),10);
            Serial.print(": PID result");
            Serial.println(tankPressController->getPIDoutput(),10);
            }

    }    
    for(auto engineController : engineControllerArray)
    {
            Serial.print( ": EngineControllerState: ");
            Serial.print(static_cast<uint8_t>(engineController->getState()));
            Serial.println(": ");
            Serial.print(static_cast<uint8_t>(engineController->getPilotMVFuelValveState()));
            Serial.print(": ");
            Serial.print(static_cast<uint8_t>(engineController->getPilotMVLoxValveState()));
            Serial.print(": ");
            Serial.print(static_cast<uint8_t>(engineController->getIgniter1State()));
            Serial.print(": ");
            Serial.print(static_cast<uint8_t>(engineController->getIgniter2State()));
            Serial.println(": ");

            for (auto i = engineController->throttleProgram.begin(); i != engineController->throttleProgram.end(); ++i)
            {
                Serial.print(" throttle program point: ");
                Serial.print(" time: ");
                Serial.print(i->autoSequenceTimeValue);
                Serial.print(" Pc: ");
                Serial.println(i->targetPcValue);
            }

    }
    
    for(auto valve : valveArray)
    {
            //Serial.print("ValveNodeID: ");
            //Serial.print(static_cast<uint8_t>(valve->getValveNodeID()));
            //Serial.print("ValveID: ");
            //Serial.print(static_cast<uint8_t>(valve->getValveID()));
        
        if (valve->getValveNodeID() == PropulsionSysNodeID)
        {
            Serial.print(" ValveID: ");
            Serial.print(static_cast<uint8_t>(valve->getValveID()));
            Serial.print( ": ValveState: ");
            Serial.print(static_cast<uint8_t>(valve->getState()));
            Serial.println(": ");
            Serial.print( ": ValveType: ");
            Serial.print(static_cast<uint8_t>(valve->getValveType()));
            Serial.println(": ");
        }
    }
    for(auto pyro : pyroArray)
    {
        
            //Serial.print("PyroNodeID: ");
            //Serial.print(static_cast<uint8_t>(pyro->getPyroNodeID()));
            //Serial.print("PyroID: ");
            //Serial.print(static_cast<uint8_t>(pyro->getPyroID()));
        if (pyro->getPyroNodeID() == PropulsionSysNodeID)
        {
            Serial.print(" PyroID: ");
            Serial.print(static_cast<uint8_t>(pyro->getPyroID()));
            Serial.print( ": PyroState: ");
            Serial.print(static_cast<uint8_t>(pyro->getState()));
            Serial.println(": ");
        }
    }

  }
    for(auto sensor : sensorArray)
    {
        if (sensor->getSensorNodeID() == PropulsionSysNodeID)
        {
        sensor->setState(SensorState::Slow);
        
/*             Serial.print("SensorID: ");
            Serial.print(static_cast<uint8_t>(sensor->getSensorID()));
            Serial.print( ": converted: ");
            Serial.print(static_cast<float>(sensor->getCurrentConvertedValue()));
            Serial.print( ": EMA: ");
            Serial.print(sensor->getEMAConvertedValue(),10);
            Serial.print( ": I: ");
            Serial.print(sensor->getIntegralSum(),10);
            if (sensor->getEnableLinearRegressionCalc())
            {
            Serial.print( ": D: ");
            Serial.print(sensor->getLinRegSlope(),10);
            }
            Serial.println(": ");
 */
        }
    
    }

  Serial.print("Current Autosequence Time: ");
  Serial.println(IgnitionAutoSequence.getCurrentCountdown());

  mainLoopTestingTimer = 0; //resets timer to zero each time the loop prints
  //Serial.print("EEPROM Node ID Read :");
  //Serial.println(EEPROM.read(nodeIDAddress));
  }

// Resets the startup bool, DO NOT REMOVE
startup = false;
  
  //Serial.println("main loop ran");

    //ALARASN& thisALARA = ALARASNmap[ALARAnodeID];
/*     Serial.print("prop system nodeID: ");
    Serial.println(thisALARA.propulsionSysNodeID);
    Serial.print("board rev: ");
    Serial.println(static_cast<uint8_t>(thisALARA.boardRev)); */


}