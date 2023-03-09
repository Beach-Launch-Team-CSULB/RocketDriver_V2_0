// RocketDriver V2.0 Propulsion Control and Data Acquisition - Embedded System Node Program
// Originally by Dan Morgan and Mat Arnold
// For Renegade, Beach Launch Team, Dan Morgan + Brandon Summers' personal machinations and more
//
//
// -------------------------------------------------------------
// Use top level define conditional to determine which system the code is operating
// Maintain definition header sets for a given propulsion system
#include "ControlFunctions.h"
#include "ValveDefinitions.h"
#include "PyroDefinitions.h"
#include "AutoSequenceDefinitions.h"
#include "SensorDefinitions.h"
#include "TankPressControllerDefinitions.h"
#include "EngineControllerDefinitions.h"
#include "ControlFunctions.h"
#include "ALARASensorControllerDefinitions.h"
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
#include <SD.h>

// ----- Other in Code Classes ----
#include "ALARABoardControllerClass.h"
#include "ALARABoardControllerDefinitions.h"
//#include "ToMillisTimeTracker.h"
#include "CANRead.h"
#include "CANWrite.h"
#include "OperationFunctionTemplates.h"
#include "ALARApinDefines.h"
#include "fluidSystemSimulation.h"
#include "FlexCAN3Controller.h"
#include "SerialUSBController.h"
#include "extendedIO/extendedIO.h"
#include "ms5607/ms5607.h"
//Trying to figure out RTC stuff with these libs
#include <TimeLib.h>
#include <DS1307RTC.h>
#include <ADC.h>

//#define PROPULSIONSYSNODEIDPRESET 2;     //NOT in use normally, for testing with the address IO register inactive

///// ADC /////
ADC* adc = new ADC();
// All ALARA boards need to be set to REF_1V2
ADC_REFERENCE ref0 = ADC_REFERENCE::REF_1V2;
ADC_REFERENCE ref1 = ADC_REFERENCE::REF_1V2;
uint8_t averages0 = 4;
uint8_t averages1 = 4;

MS5607 ALARAbaro;

uint32_t rocketDriverSeconds;
uint32_t rocketDriverMicros;


// Timer for setting main loop debugging print rate
elapsedMillis mainLoopTestingTimer;
elapsedMillis ezModeControllerTimer;
elapsedMillis commandExecuteTimer;
elapsedMillis shittyCANTimer;
elapsedMillis crashTimer;

//For use in doing serial inputs as CAN commands for testing
uint8_t fakeCANmsg; //CAN2.0 byte array, first 4 bytes are ID field for full extended ID compatibility
uint8_t fakeCanIterator = 0;

bool localNodeResetFlag = false; //flag to trigger register reset from commanded reset over CAN
bool abortHaltFlag; //creates halt flag that is a backup override of state machine, am I currently using it?
bool outputOverride = true; // initializes as true to block outputs until changed

///// NODE DECLARATION /////
//default sets to max nodeID intentionally to be bogus until otherwise set
ALARASN thisALARA;
uint8_t ALARAnodeID;                      // ALARA hardware node address 2 for engine 3 for prop
uint8_t ALARAnodeIDfromFlash;            //nodeID read out of EEPROM
uint32_t ALARAnodeIDfromFlash_errorFlag;            //nodeID read out of EEPROM
bool nodeIDdeterminefromFlash;           //boolean flag for if startup is to run the nodeID detect read
uint32_t nodeIDdeterminefromFlash_errorFlag;
//uint8_t ALARAnodeIDfromEEPROM;            //nodeID read out of EEPROM
//uint32_t ALARAnodeIDfromEEPROM_errorFlag;            //nodeID read out of EEPROM
//bool nodeIDdeterminefromEEPROM;           //boolean flag for if startup is to run the nodeID detect read
//uint32_t nodeIDdeterminefromEEPROM_errorFlag;
//uint8_t PropulsionSysNodeID = PROPULSIONSYSNODEIDPRESET;              //engine node = 2, prop node = 3, Pasafire node = 8
//uint8_t PropulsionSysNodeIDfromEEPROM;    //PropulsionSysNodeID read out of EEPROM
//uint32_t PropulsionSysNodeIDfromEEPROM_errorFlag;    //PropulsionSysNodeID read out of EEPROM
//uint32_t vehicleStatefromEEPROM_errorFlag;
//uint32_t missionStatefromEEPROM_errorFlag;

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
CAN_stats_t Can0stats;
bool CANSensorReportConverted = false;
bool NewCommandMessage{false};
bool NewConfigMessage{false};

CAN_filter_t wtf;

FlexCan3Controller Can2msgController;
SerialUSBController SerialUSBdataController;

// SD on board data logger stuff
File onBoardLog;
std::string logString = "";
std::string fileLogName = "test.txt";

const int CAN2busSpeed = 500000; // CAN2.0 baudrate - do not set above 500000 for full distance run bunker to pad

bool startup{true}; // bool for storing if this is the first loop on startup, ESSENTIAL FOR STATE MACHINE OPERATION (maybe not anymore?)

uint32_t loopCount {0};// for debugging

// Set the global command, and global state
Command currentCommand{command_NOCOMMAND}; 
VehicleState currentVehicleState{VehicleState::passive};
VehicleState priorVehicleState{VehicleState::passive};
MissionState currentMissionState{MissionState::passive};
MissionState priorMissionState{MissionState::passive};
// SET "staticTest = true" FOR GROUND TESTING, "false" FOR FLIGHT!!!!!
bool staticTest = true;


commandMSG currentCommandMSG{};
configMSG currentConfigMSG{};

//uint32_t vehicleStateAddressfromEEPROM_errorFlag;
//uint32_t missionStateAddressfromEEPROM_errorFlag;

//AutoSequence stuff for main
int64_t currentCountdownForMain;

////// Set EEPROM addresses
uint16_t vehicleStateAddress1{1};
uint16_t vehicleStateAddress2{2};
uint16_t vehicleStateAddress3{3};
uint16_t missionStateAddress1{4};
uint16_t missionStateAddress2{5};
uint16_t missionStateAddress3{6};
uint16_t PropulsionSysNodeIDAddress1{7};
uint16_t PropulsionSysNodeIDAddress2{8};
uint16_t PropulsionSysNodeIDAddress3{9};
uint16_t nodeIDDetermineAddress1{10};
uint16_t nodeIDDetermineAddress2{11};
uint16_t nodeIDDetermineAddress3{12};
uint16_t nodeIDAddress1{13};
uint16_t nodeIDAddress2{14};
uint16_t nodeIDAddress3{15};

////// Set Flash Vars
bool reFormateFlash = false;

uint8_t PropulsionSysNodeID;              //engine node = 2, prop node = 3, Pasafire node = 8
uint8_t PropulsionSysNodeIDfromFlash;    //PropulsionSysNodeID read out of EEPROM
uint32_t PropulsionSysNodeIDfromFlash_errorFlag;    //PropulsionSysNodeID read out of EEPROM
uint32_t vehicleStatefromFlash_errorFlag;
uint32_t missionStatefromFlash_errorFlag;

uint32_t vehicleStateAddressfromFlash_errorFlag;
uint32_t missionStateAddressfromFlash_errorFlag;

std::string vehicleStateFlashFileName1 = "vehicleState1.txt";
std::string vehicleStateFlashFileName2 = "vehicleState2.txt";
std::string vehicleStateFlashFileName3 = "vehicleState3.txt";
std::string missionStateFlashFileName1 = "missionState1.txt";
std::string missionStateFlashFileName2 = "missionState2.txt";
std::string missionStateFlashFileName3 = "missionState3.txt";
std::string propSysNodeIDFlashFileName1 = "propSysNodeID1.txt";
std::string propSysNodeIDFlashFileName2 = "propSysNodeID2.txt";
std::string propSysNodeIDFlashFileName3 = "propSysNodeID3.txt";
std::string alaraNodeIDFlashFileName1 = "alaraNodeID1.txt";
std::string alaraNodeIDFlashFileName2 = "alaraNodeID2.txt";
std::string alaraNodeIDFlashFileName3 = "alaraNodeID3.txt";


//-------------------------------------------------------//
void setup() {
  //delay(10000);
  startup = true;   // Necessary to set startup to true for the code loop so it does one startup loop for the state machine before entering regular loop behavior
  Serial.begin(9600); // Value is arbitrary on Teensy, it will initialize at the MCU dictate baud rate regardless what you feed this
  Wire.begin();
  SPI.begin();
  
  #ifdef ALARAV2_1
  // ----- MUX Setups for ALARA -----
  // Board Addressing MUX
  MUXSetup(true, ALARA_DIGITAL_ADDRESS_1, ALARA_DIGITAL_ADDRESS_2, ALARA_DIGITAL_ADDRESS_3, ALARA_DIGITAL_ADDRESS_4);
  
  // MOVE NODEDETECTSHITHERE!!!
  // Check map for ALARASN configutation
  //tripleFlashwrite(2, alaraNodeIDFlashFileName1, alaraNodeIDFlashFileName2, alaraNodeIDFlashFileName3, 0);
  ALARAnodeID = tripleFlashread(alaraNodeIDFlashFileName1, alaraNodeIDFlashFileName2, alaraNodeIDFlashFileName3, ALARAnodeIDfromFlash_errorFlag, 0);
  lookupALARASNmap(thisALARA, ALARAnodeID);


  // NOR Flash CS pin MUX
  MUXSetup(false, ALARA_NOR_S0, ALARA_NOR_S1, ALARA_NOR_S2);
  flashSetup(reFormateFlash);
  #endif
///// ----- Insert a board rev check to pin defines here, if it fails disable our GPIO? ------ //

  // -----Read Last State off eeprom and update -----
  //tripleFlashwrite(2, vehicleStateFlashFileName1, vehicleStateFlashFileName2, vehicleStateFlashFileName3, 0);
  //tripleFlashwrite(1, missionStateFlashFileName1, missionStateFlashFileName2, missionStateFlashFileName3, 0);
  //currentVehicleState = static_cast<VehicleState>(tripleEEPROMread(vehicleStateAddress1, vehicleStateAddress2, vehicleStateAddress3, vehicleStateAddressfromEEPROM_errorFlag));
  //currentMissionState = static_cast<MissionState>(tripleEEPROMread(missionStateAddress1, missionStateAddress2, missionStateAddress3, missionStateAddressfromEEPROM_errorFlag));
  currentVehicleState = static_cast<VehicleState>(tripleFlashread(vehicleStateFlashFileName1, vehicleStateFlashFileName2, vehicleStateFlashFileName3, vehicleStateAddressfromFlash_errorFlag, 0));
  currentMissionState = static_cast<MissionState>(tripleFlashread(missionStateFlashFileName1, missionStateFlashFileName2, missionStateFlashFileName3, missionStateAddressfromFlash_errorFlag, 0));
  // Only write to EEPROM the node ID if manual ID define is present at top of Main
  //#ifdef PROPULSIONSYSNODEIDPRESET
  //tripleEEPROMwrite(static_cast<uint8_t>(2), PropulsionSysNodeIDAddress1, PropulsionSysNodeIDAddress2, PropulsionSysNodeIDAddress3);
  //tripleFlashwrite(static_cast<uint8_t>(2), propSysNodeIDFlashFileName1, propSysNodeIDFlashFileName2, propSysNodeIDFlashFileName3, 0);
  //#endif
  //PropulsionSysNodeIDfromEEPROM = tripleEEPROMread(PropulsionSysNodeIDAddress1, PropulsionSysNodeIDAddress2, PropulsionSysNodeIDAddress3, PropulsionSysNodeIDfromEEPROM_errorFlag);
  //PropulsionSysNodeID =  tripleEEPROMread(PropulsionSysNodeIDAddress1, PropulsionSysNodeIDAddress2, PropulsionSysNodeIDAddress3, PropulsionSysNodeIDfromEEPROM_errorFlag);
  //Serial.println(tripleFlashread(propSysNodeIDFlashFileName1, propSysNodeIDFlashFileName2, propSysNodeIDFlashFileName3, PropulsionSysNodeIDfromFlash_errorFlag, 0));
  PropulsionSysNodeID = tripleFlashread(propSysNodeIDFlashFileName1, propSysNodeIDFlashFileName2, propSysNodeIDFlashFileName3, PropulsionSysNodeIDfromFlash_errorFlag, 0);
  //nodeIDdeterminefromEEPROM = tripleEEPROMread(nodeIDDetermineAddress1, nodeIDDetermineAddress2, nodeIDDetermineAddress3, nodeIDdeterminefromEEPROM_errorFlag);
  startupStateCheck(currentVehicleState, currentCommand);
  // Set NewCommandMessage true so the command from startupStateCheck gets read by commandExecute
  NewCommandMessage = true;

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
  // Quick and dirty way to setup unique ADC use case on the LC/TC Teensy node
  #ifdef TEENSY3_X
    // Setting alt I2C pins because I used default I2C pins
    Wire.setSDA(8);
    Wire.setSCL(7);
    ref0 = ADC_REFERENCE::REF_3V3;
    ref1 = ADC_REFERENCE::REF_1V2;
    averages0 = 32;
    averages1 = 32;
  #endif
  MCUADCSetup(*adc, ref0, ref1, averages0, averages1);

  // -----Run Valve PropulsionSysNodeID Check-----
  ValveNodeIDCheck(valveArray, PropulsionSysNodeID);

  // -----Run Valve PropulsionSysNodeID Check-----
  PyroNodeIDCheck(pyroArray, PropulsionSysNodeID);

  // -----Run Sensor PropulsionSysNodeID Check-----
  SensorNodeIDCheck(sensorArray, PropulsionSysNodeID);
  SensorNodeIDCheck(HPsensorArray, PropulsionSysNodeID);
  //SensorNodeIDCheck(TCsensorArray, PropulsionSysNodeID);

  // -----Run Valve Setup-----
  valveSetUp(valveArray, ALARA_HP_Array);

  // -----Run Valve Setup-----
  pyroSetUp(pyroArray, ALARA_HP_Array);

  // -----Run Valve Setup-----
  engineControllerSetup(engineControllerArray);

  // -----Run Valve Setup-----
  tankPressControllerSetup(tankPressControllerArray);

  // -----Run AutoSequence Setup-----
  autoSequenceSetUp(autoSequenceArray);
  
  // -----Run Sensor Setup -----
  sensorSetUp(sensorArray);
  sensorSetUp(HPsensorArray);
  sensorSetUp(TCsensorArray);
  #ifdef TEENSY3_X
  coldJunctionRenegade.begin();
  #endif

  // ----- Set Controller Dependent Sensor Settings -----
  controllerSensorSetup(valveArray, pyroArray, autoSequenceArray, sensorArray, tankPressControllerArray, engineControllerArray);

  #ifdef ALARAV2_1
  pinModeExtended(ALARA_DIGITAL_ADDRESS_OE, OUTPUT);
  //ALARAbaro.init(ALARA_BPS_CSN,OSR_1024); //dont know why this breaks SPI for flash need to ask summers - Ivan
  boardController.begin();
  #endif
  
  SerialUSBdataController.setPropStatusPrints(true);
  SerialUSBdataController.setPropCSVStreamPrints(false);
  Can0.startStats();
  Can0.setTxBufferSize(64);
}

void loop()
{

// Lazy "SensorTasks" for the RTD sensor
 if (coldJunctionRenegade.getSensorNodeID() == PropulsionSysNodeID)
 {
   coldJunctionRenegade.read();
 }

//fakesensorShit(rocketDriverSeconds, rocketDriverMicros, &myTimeTrackingFunction);

///// Custom function for tracking miliseconds and seconds level system time for timestamping /////
// let it run each loop in addition to when called to keep it synced up
 myTimeTrackingFunction(rocketDriverSeconds, rocketDriverMicros);

  // --- Read CAN bus and update current command ---
  if(CANread(Can0, configVerificationKey, NewConfigMessage, currentCommand, currentConfigMSG, PropulsionSysNodeID) && !startup) // do not execute on the first loop
  {
    //Serial.print("CAN Message Recieved: ");
    //Serial.println(currentCommand); //currently only does the command not any message
  }
  if(SerialAsCANread())
  {
    //Serial.println("Command Entered");
  }
  //Serial.println("Do I get to live?");
  

///// ------ MESSAGE PROCESSING BLOCK ----- /////  
  //pull next command message from buffer, if there is one
  if (!NewCommandMessage)
  {
  NewCommandMessage = readRemoveVectorBuffer(currentCommandMSG);
  currentCommand = currentCommandMSG.commandEnum;
  }
  //Serial.println("Do I get past new command message?");
  //process command
  //if (commandExecuteTimer >= 2000)
  //{
  commandExecute(currentVehicleState, priorVehicleState, currentMissionState, priorMissionState, currentCommand, NewCommandMessage, autoSequenceArray, sensorArray, tankPressControllerArray, engineControllerArray);
  //Serial.println("Do I get past command execute?");
  //}
  //pull next config message from buffer, if there is one
  NewConfigMessage = readRemoveVectorBuffer(currentConfigMSG);
  //Serial.println("Do I get past new config msg?");
  //process config message
  configMSGread(currentConfigMSG, NewConfigMessage, valveArray, pyroArray, sensorArray, autoSequenceArray, tankPressControllerArray, engineControllerArray, waterGoesVroom);
  //Serial.println("Do I get past config msg read?");
///// ------------------------------------ /////  

  if (ezModeControllerTimer >= 20) // 5 = 200Hz controller rate, 20 = 50Hz rate
  {
  //Serial.println("Do I get into ezModeControllerTimer?");
  // -----Process Commands Here-----
  vehicleStateMachine(currentVehicleState, priorVehicleState, currentCommand, boardController, autoSequenceArray, sensorArray, tankPressControllerArray, engineControllerArray, waterGoesVroom, abortHaltFlag, outputOverride);
  //Serial.println("Do I get past vehicleStateMachine?");
  missionStateMachine(currentVehicleState, priorVehicleState, currentMissionState, priorMissionState, boardController, autoSequenceArray, staticTest, abortHaltFlag);
  //Serial.println("Do I get past misionStateMachine?");
  #ifdef ALARAV2_1
  controllerDataSync(valveArray, pyroArray, autoSequenceArray, sensorArray, tankPressControllerArray, engineControllerArray);
  #endif
  //Serial.println("Do I get past controllerDataSync?");
  autoSequenceTasks(autoSequenceArray, PropulsionSysNodeID);
  //Serial.println("Do I get past autoSequenceTasks?");
  tankPressControllerTasks(tankPressControllerArray, PropulsionSysNodeID, IgnitionAutoSequence);
  //Serial.println("Do I get past tankPressControllerTasks?");
  engineControllerTasks(engineControllerArray, PropulsionSysNodeID, IgnitionAutoSequence);
  //Serial.println("Do I get past engineControllerTasks?");
  controllerDeviceSync(currentVehicleState, priorVehicleState, currentCommand, valveArray, pyroArray, autoSequenceArray, sensorArray, tankPressControllerArray, engineControllerArray, waterGoesVroom, abortHaltFlag);
  //Serial.println("Do I get past controllerDeviceSync?");
  //fluid sim run
  waterGoesVroom.fluidSystemUpdate();
  //Serial.println("Is this Pizza's fault?");
  ezModeControllerTimer = 0;

//ALARAbaro.update();
//ALARAbaro.print_all();
  }

  // -----Advance needed controller system tasks (tank press controllers, ignition autosequence, . ..) ----- //
  // -----Advance needed propulsion system tasks (valve, pyro, sensors, . ..) ----- //
  cli(); // disables interrupts to ensure complete propulsion output state is driven
  valveTasks(valveArray, PropulsionSysNodeID, outputOverride, *autoSequenceArray.at(0));
  pyroTasks(pyroArray, PropulsionSysNodeID, outputOverride, *autoSequenceArray.at(0));
  //MUST KEEP HP OVERRIDE AFTER VALVE/PYRO TASKS
  #ifdef ALARAV2_1
  ALARAHPOverride(ALARA_HP_Array, outputOverride);
  #endif
  sei(); // reenables interrupts after propulsion output state set is completed
  //Serial.println("Do I get past valveTasks,PyroTasks, and HPOverride?");
  sensorTasks(sensorArray, *adc, PropulsionSysNodeID, rocketDriverSeconds, rocketDriverMicros);
  //Serial.println("Do I get past sensorTasks?");
  ALARAHPsensorTasks(HPsensorArray, *adc, PropulsionSysNodeID, rocketDriverSeconds, rocketDriverMicros, outputOverride);
  //Serial.println("Do I get past HPsensorTasks?");
  TCsensorTasks(TCsensorArray, *adc, PropulsionSysNodeID, rocketDriverSeconds, rocketDriverMicros);
  //Serial.println("Do I get past HPsensorTasks?");
  // -----Update States on EEPROM -----
  // ONLY write if something new to write!!! Don't spam EEMPROM it will kill the memory bytes physically if overused
  //if ((static_cast<uint8_t>(currentVehicleState)) != (tripleEEPROMread(vehicleStateAddress1, vehicleStateAddress2, vehicleStateAddress3, vehicleStatefromEEPROM_errorFlag)))
  //{
  //tripleEEPROMwrite(static_cast<uint8_t>(currentVehicleState), vehicleStateAddress1, vehicleStateAddress2, vehicleStateAddress3);
  //}
  //if ((static_cast<uint8_t>(currentMissionState)) != (tripleEEPROMread(missionStateAddress1, missionStateAddress2, missionStateAddress3, missionStatefromEEPROM_errorFlag)))
  //{
  //tripleEEPROMwrite(static_cast<uint8_t>(currentMissionState), missionStateAddress1, missionStateAddress2, missionStateAddress3);
/*   Serial.println("Does current vs prior MISSION state EEPROM protect work as expected? ");
  Serial.print(" priorMissionState : ");
  Serial.print(static_cast<uint8_t>(priorMissionState));
  Serial.print(" currentMissionState : ");
  Serial.println(static_cast<uint8_t>(currentMissionState)); */
  //}
  //tripleFlashwrite(static_cast<uint8_t>(1), vehicleStateFlashFileName1, vehicleStateFlashFileName2, vehicleStateFlashFileName3, 0);
  //tripleFlashwrite(static_cast<uint8_t>(0), missionStateFlashFileName1, missionStateFlashFileName2, missionStateFlashFileName3, 0);
  if ((static_cast<uint8_t>(currentVehicleState)) != (tripleFlashread(vehicleStateFlashFileName1, vehicleStateFlashFileName2, vehicleStateFlashFileName3, vehicleStatefromFlash_errorFlag, 0)))
  { 
    //Serial.println("===Vehical State Update===");
    //Serial.print("Current Vehical State: ");
    //Serial.println(static_cast<uint8_t>(currentVehicleState));
    //Serial.print("File Vehical State: ");
    //Serial.println(tripleFlashread(vehicleStateFlashFileName1, vehicleStateFlashFileName2, vehicleStateFlashFileName3, vehicleStatefromFlash_errorFlag, 0));

    tripleFlashwrite(static_cast<uint8_t>(currentVehicleState), vehicleStateFlashFileName1, vehicleStateFlashFileName2, vehicleStateFlashFileName3, 0);

    //Serial.print("Updated File Vehical State: ");
    //Serial.println(tripleFlashread(vehicleStateFlashFileName1, vehicleStateFlashFileName2, vehicleStateFlashFileName3, vehicleStatefromFlash_errorFlag, 0));
  }
  if ((static_cast<uint8_t>(currentMissionState)) != (tripleFlashread(missionStateFlashFileName1, missionStateFlashFileName2, missionStateFlashFileName3, missionStatefromFlash_errorFlag, 0)))
  {
    /*Serial.println("===Mission State Update===");
    Serial.print("Current Mission State: ");
    Serial.println(static_cast<uint8_t>(currentMissionState));
    Serial.print("File Mission State: ");
    Serial.println(tripleFlashread(missionStateFlashFileName1, missionStateFlashFileName2, missionStateFlashFileName3, missionStatefromFlash_errorFlag, 0));
    */

    tripleFlashwrite(static_cast<uint8_t>(currentMissionState), missionStateFlashFileName1, missionStateFlashFileName2, missionStateFlashFileName3, 0);

    //Serial.print("Updated File Mission State: ");
    //Serial.println(tripleFlashread(missionStateFlashFileName1, missionStateFlashFileName2, missionStateFlashFileName3, missionStatefromFlash_errorFlag, 0));

  }
  
  // Reset function to reboot Teensy with internal reset register
  // Need to figure out how to rework using this feature with reworked ID system
  //TeensyInternalReset(localNodeResetFlag, nodeIDDetermineAddress1, nodeIDDetermineAddress2, nodeIDDetermineAddress3);


///// ----- All outgoing CAN2 messages managed here ----- /////
// Run every loop
if (shittyCANTimer >= 250)
{
  Can2msgController.setExternalStateChange(true); //cheater force for quasi messages, AM I USING THIS AT ALL RIGHT NOW???
  shittyCANTimer = 0;
}
  
  Can2msgController.controllerTasks(Can0, currentVehicleState, currentMissionState, currentCommand, engineControllerArray, tankPressControllerArray, valveArray, pyroArray, sensorArray, HPsensorArray, autoSequenceArray, waterGoesVroom, PropulsionSysNodeID);
  /*Serial.println("Do I get past Can2 controllerTasks?");
  Can0stats = Can0.getStats();
  Serial.print("Can0stats.ringRxMax? ");
  Serial.println(Can0stats.ringRxMax);
  Serial.print("Can0stats.ringTxHighWater? ");
  Serial.println(Can0stats.ringTxMax);*/

///// ----- Serial Print Functions ----- /////
  if (mainLoopTestingTimer >= 250)
  {
  SerialUSBdataController.propulsionNodeStatusPrints(currentVehicleState, priorVehicleState, currentMissionState, priorMissionState, currentCommand, currentCommandMSG, currentConfigMSG, autoSequenceArray, engineControllerArray, waterGoesVroom, tankPressControllerArray, valveArray, pyroArray, sensorArray, HPsensorArray, PropulsionSysNodeID);
  SerialUSBdataController.propulsionNodeCSVStreamPrints(currentVehicleState, priorVehicleState, currentMissionState, priorMissionState, currentCommand, currentCommandMSG, currentConfigMSG, autoSequenceArray, engineControllerArray, waterGoesVroom, tankPressControllerArray, valveArray, pyroArray, sensorArray, PropulsionSysNodeID);
  mainLoopTestingTimer = 0; //resets timer to zero each time the loop prints
  Serial.print(" Crash Timer Millis: ");
  Serial.println(crashTimer);
  }

///// ----- SD card writing ----- /////
 if(!SD.begin(BUILTIN_SDCARD)) 
 {
    Serial.println("SD Failed");
 } 
 else 
 {
  onBoardLog = SD.open(fileLogName.c_str(), FILE_WRITE);
    
    if(onBoardLog) 
    {
      //time stamp (ms)
      onBoardLog.printf("Time (ms): %lu | ", (unsigned long) millis());

      //state update
      onBoardLog.printf("State: %u | ", (uint8_t) currentVehicleState);

      //---valve states---

      //high press valves
      onBoardLog.printf("HP: %u | ", (uint8_t) valveArray[0]->getState());
      onBoardLog.printf("HV: %u | ", (uint8_t) valveArray[1]->getState());

      //lox valves
      onBoardLog.printf("LDR: %u | ", (uint8_t) valveArray[5]->getState());
      onBoardLog.printf("LDV: %u | ", (uint8_t) valveArray[6]->getState());
      onBoardLog.printf("LV: %u | ", (uint8_t) valveArray[4]->getState());
      onBoardLog.printf("LMV: %u | ", (uint8_t) valveArray[2]->getState());

      //fuel valves
      onBoardLog.printf("FDR: %u | ", (uint8_t) valveArray[8]->getState());
      onBoardLog.printf("FDV: %u | ", (uint8_t) valveArray[9]->getState());
      onBoardLog.printf("FV: %u | ", (uint8_t) valveArray[7]->getState());
      onBoardLog.printf("FMV: %u | ", (uint8_t) valveArray[3]->getState());

      //igniters
      onBoardLog.printf("IG1: %u | ", (uint8_t) pyroArray[0]->getState());
      onBoardLog.printf("IG2: %u | ", (uint8_t) pyroArray[1]->getState());

      //---PT Data (PSI)---

      //Hish Side PTs
      onBoardLog.printf("PTHighLoxSide (psi): %.1f | ", (float) sensorArray[16]->getCurrentConvertedValue());
      onBoardLog.printf("PTHighFuelSide (psi): %.1f | ", (float) sensorArray[15]->getCurrentConvertedValue());

      //Lox Tank PTs
      onBoardLog.printf("PTLoxTank1 (psi): %.1f | ", (float) sensorArray[13]->getCurrentConvertedValue());
      onBoardLog.printf("PTLoxTank2 (psi): %.1f | ", (float) sensorArray[14]->getCurrentConvertedValue());

      //Fuel Tank PTs
      onBoardLog.printf("PTFuelTank1 (psi): %.1f | ", (float) sensorArray[11]->getCurrentConvertedValue());
      onBoardLog.printf("PTFuelTank2 (psi): %.1f | ", (float) sensorArray[12]->getCurrentConvertedValue());

      //Dome Reg PTs
      onBoardLog.printf("PTLoxDome (psi): %.1f | ", (float) sensorArray[10]->getCurrentConvertedValue());
      onBoardLog.printf("PTFuelDome (psi): %.1f | ", (float) sensorArray[9]->getCurrentConvertedValue());

      //Fuel Engine PTs
      onBoardLog.printf("PTFuelInlet (psi): %.1f | ", (float) sensorArray[5]->getCurrentConvertedValue());
      onBoardLog.printf("PTFuelInjector (psi): %.1f | ", (float) sensorArray[6]->getCurrentConvertedValue());

      //Lox Engine PTs
      onBoardLog.printf("PTLoxInlet (psi): %.1f | ", (float) sensorArray[7]->getCurrentConvertedValue());

      //Main Pneumatics PT
      onBoardLog.printf("PTMainPneumatics (psi): %.1f | ", (float) sensorArray[8]->getCurrentConvertedValue());
      
      //Chamber PTs
      onBoardLog.printf("PTChamber1 (psi): %.1f | ", (float) sensorArray[4]->getCurrentConvertedValue());
      onBoardLog.printf("PTChamber2 (psi): %u | ", (float) sensorArray[3]->getCurrentConvertedValue());

      //onBoardLog.printf("PTChamber2 (raw): %u | ", (uint32_t) sensorArray[3]->getCurrentRawValue()); //grabing raw value

      onBoardLog.println("");
      onBoardLog.close();
    }
 }


// Resets the startup bool, DO NOT REMOVE
startup = false;

}