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

#include "ALARAUtilityFunctions.h"
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

#define PROPULSIONSYSNODEIDPRESET 8;     //NOT in use normally, for testing with the address IO register inactive

///// ADC /////
ADC* adc = new ADC();

MS5607 ALARAbaro;

uint32_t rocketDriverSeconds;
uint32_t rocketDriverMicros;


// Timer for setting main loop debugging print rate
elapsedMillis mainLoopTestingTimer;
elapsedMillis ezModeControllerTimer;
elapsedMillis commandExecuteTimer;
elapsedMillis shittyCANTimer;

//For use in doing serial inputs as CAN commands for testing
uint8_t fakeCANmsg; //CAN2.0 byte array, first 4 bytes are ID field for full extended ID compatibility
uint8_t fakeCanIterator = 0;

bool localNodeResetFlag = false; //flag to trigger register reset from commanded reset over CAN
bool abortHaltFlag; //creates halt flag that is a backup override of state machine, am I currently using it?
bool outputOverride = true; // initializes as true to block outputs until changed

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

CAN_filter_t wtf;

FlexCan3Controller Can2msgController;
SerialUSBController SerialUSBdataController;

const int CAN2busSpeed = 500000; // CAN2.0 baudrate - do not set above 500000 for full distance run bunker to pad

bool startup{true}; // bool for storing if this is the first loop on startup, ESSENTIAL FOR STATE MACHINE OPERATION (maybe not anymore?)

uint32_t loopCount {0};// for debugging

// Set the global command, and global state
Command currentCommand{command_NOCOMMAND}; 
VehicleState currentVehicleState{VehicleState::passive};
VehicleState priorVehicleState;
MissionState currentMissionState(MissionState::passive);
MissionState priorMissionState;
// SET "staticTest = true" FOR GROUND TESTING, "false" FOR FLIGHT!!!!!
bool staticTest = true;


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
uint16_t nodeIDAddress1{22};
uint16_t nodeIDAddress2{23};
uint16_t nodeIDAddress3{24};

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
  MCUADCSetup(*adc);

  // -----Run Valve PropulsionSysNodeID Check-----
  ValveNodeIDCheck(valveArray, PropulsionSysNodeID);

  // -----Run Valve PropulsionSysNodeID Check-----
  PyroNodeIDCheck(pyroArray, PropulsionSysNodeID);

  // -----Run Sensor PropulsionSysNodeID Check-----
  SensorNodeIDCheck(sensorArray, PropulsionSysNodeID);
  SensorNodeIDCheck(HPsensorArray, PropulsionSysNodeID);

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


  //sensorSetUp(sensorArray, rocketDriverSeconds, rocketDriverMicros, &myTimeTrackingFunction);
  // ----- Set Controller Dependent Sensor Settings -----
  controllerSensorSetup(valveArray, pyroArray, autoSequenceArray, sensorArray, tankPressControllerArray, engineControllerArray);


  Serial.begin(9600); // Value is arbitrary on Teensy, it will initialize at the MCU dictate baud rate regardless what you feed this

  Wire.begin();
  SPI.begin();
  ALARAbaro.init(ALARA_BPS_CSN,OSR_1024);
  boardController.begin();

  SerialUSBdataController.setPropStatusPrints(true);
  SerialUSBdataController.setPropCSVStreamPrints(false);
  pinModeExtended(ALARA_DIGITAL_ADDRESS_OE, OUTPUT);
}

void loop() 
{

//fakesensorShit(rocketDriverSeconds, rocketDriverMicros, &myTimeTrackingFunction);

  //Display the node number with serial print statement start of each loop
  //Serial.print("PropulsionSysNodeID: ");
  //Serial.println(PropulsionSysNodeID);

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
  Serial.println("Do I get to live?");
  
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
  Serial.println("Do I get past new command message?");
  //process command
  //if (commandExecuteTimer >= 2000)
  //{
  commandExecute(currentVehicleState, priorVehicleState, currentMissionState, priorMissionState, currentCommand, NewCommandMessage, autoSequenceArray, sensorArray, tankPressControllerArray, engineControllerArray);
  Serial.println("Do I get past command execute?");
  //}
  //pull next config message from buffer, if there is one
  NewConfigMessage = readRemoveVectorBuffer(currentConfigMSG);
  Serial.println("Do I get past new config msg?");
  //process config message
  configMSGread(currentConfigMSG, NewConfigMessage, valveArray, pyroArray, sensorArray, autoSequenceArray, tankPressControllerArray, engineControllerArray, waterGoesVroom);
  Serial.println("Do I get past config msg read?");
///// ------------------------------------ /////  

  if (ezModeControllerTimer >= 50) // 5 = 200Hz controller rate
  {
  Serial.println("Do I get into ezModeControllerTimer?");
  // -----Process Commands Here-----
  vehicleStateMachine(currentVehicleState, priorVehicleState, currentCommand, boardController, autoSequenceArray, sensorArray, tankPressControllerArray, engineControllerArray, waterGoesVroom, abortHaltFlag, outputOverride);
  Serial.println("Do I get past vehicleStateMachine?");
  missionStateMachine(currentVehicleState, priorVehicleState, currentMissionState, priorMissionState, boardController, autoSequenceArray, staticTest, abortHaltFlag);
  Serial.println("Do I get past misionStateMachine?");
  controllerDataSync(valveArray, pyroArray, autoSequenceArray, sensorArray, tankPressControllerArray, engineControllerArray);
  Serial.println("Do I get past controllerDataSync?");
  autoSequenceTasks(autoSequenceArray, PropulsionSysNodeID);
  Serial.println("Do I get past autoSequenceTasks?");
  tankPressControllerTasks(tankPressControllerArray, PropulsionSysNodeID, IgnitionAutoSequence);
  Serial.println("Do I get past tankPressControllerTasks?");
  engineControllerTasks(engineControllerArray, PropulsionSysNodeID, IgnitionAutoSequence);
  Serial.println("Do I get past engineControllerTasks?");
  //autoSequenceTasks(autoSequenceArray, PropulsionSysNodeID);
  controllerDeviceSync(currentVehicleState, priorVehicleState, currentCommand, valveArray, pyroArray, autoSequenceArray, sensorArray, tankPressControllerArray, engineControllerArray, waterGoesVroom, abortHaltFlag);
  Serial.println("Do I get past controllerDeviceSync?");
  //fluid sim run
  waterGoesVroom.fluidSystemUpdate();
  Serial.println("Is this Pizza's fault?");
  ezModeControllerTimer = 0;

//ALARAbaro.update();
//ALARAbaro.print_all();
  }

  // -----Advance needed controller system tasks (tank press controllers, ignition autosequence, . ..) ----- //
  // -----Advance needed propulsion system tasks (valve, pyro, sensors, . ..) ----- //
  cli(); // disables interrupts to ensure complete propulsion output state is driven
  valveTasks(valveArray, PropulsionSysNodeID, outputOverride, *autoSequenceArray.at(0));
  pyroTasks(pyroArray, PropulsionSysNodeID, outputOverride, *autoSequenceArray.at(0));
  ALARAHPOverride(ALARA_HP_Array, outputOverride);
  sei(); // reenables interrupts after propulsion output state set is completed
  sensorTasks(sensorArray, *adc, PropulsionSysNodeID, rocketDriverSeconds, rocketDriverMicros);
  ALARAHPsensorTasks(HPsensorArray, *adc, PropulsionSysNodeID, rocketDriverSeconds, rocketDriverMicros);

  // -----Update States on EEPROM -----
  //change to only write if something new to write!!! Make wrapper function that checks for new info?
  //tripleEEPROMwrite(static_cast<uint8_t>(currentVehicleState), vehicleStateAddress1, vehicleStateAddress2, vehicleStateAddress3);

  // Reset function to reboot Teensy with internal reset register
  // Need to figure out how to rework using this feature with reworked ID system
  //TeensyInternalReset(localNodeResetFlag, nodeIDDetermineAddress1, nodeIDDetermineAddress2, nodeIDDetermineAddress3);


///// ----- All outgoing CAN2 messages managed here ----- /////
// Run every loop
if (shittyCANTimer >= 1000)
{
  Can2msgController.setExternalStateChange(true); //cheater force for quasi messages
  shittyCANTimer = 0;
}
  
  Can2msgController.controllerTasks(Can0, currentVehicleState, currentMissionState, currentCommand, engineControllerArray, tankPressControllerArray, valveArray, pyroArray, sensorArray, HPsensorArray, autoSequenceArray, waterGoesVroom, PropulsionSysNodeID);
  
///// ----- Serial Print Functions ----- /////
  if (mainLoopTestingTimer >= 250)
  {
  
/*     for(auto sensor : HPsensorArray)
    {
        if (sensor->getSensorNodeID() == PropulsionSysNodeID)
        {
        sensor->setState(SensorState::Fast);
         
            Serial.print("SensorID: ");
            Serial.print(static_cast<uint8_t>(sensor->getSensorID()));
            //Serial.print( ": new converted bool: ");
            //Serial.print( ": new raw bool: ");
            //Serial.print(sensor->getNewSensorValueCheckCAN());
            //Serial.print(sensor->getNewSensorConversionCheck());
            Serial.print( ": raw value: ");
            Serial.print(sensor->getCurrentRawValue());
            Serial.print( ": converted: ");
            Serial.print(static_cast<float>(sensor->getCurrentConvertedValue()));
            Serial.print( ": EMA: ");
            Serial.print(sensor->getEMAConvertedValue(),10);
            Serial.println(": ");

        }
    
    }
 */  
  
  SerialUSBdataController.propulsionNodeStatusPrints(currentVehicleState, priorVehicleState, currentMissionState, priorMissionState, currentCommand, currentCommandMSG, currentConfigMSG, autoSequenceArray, engineControllerArray, waterGoesVroom, tankPressControllerArray, valveArray, pyroArray, sensorArray, HPsensorArray, PropulsionSysNodeID);
  SerialUSBdataController.propulsionNodeCSVStreamPrints(currentVehicleState, priorVehicleState, currentMissionState, priorMissionState, currentCommand, currentCommandMSG, currentConfigMSG, autoSequenceArray, engineControllerArray, waterGoesVroom, tankPressControllerArray, valveArray, pyroArray, sensorArray, PropulsionSysNodeID);
  mainLoopTestingTimer = 0; //resets timer to zero each time the loop prints
  }

// Resets the startup bool, DO NOT REMOVE
startup = false;

/* digitalWriteExtended(ALARA_DIGITAL_ADDRESS_OE,HIGH);

uint8_t NodeIDAddressRead;
  for (size_t i = 0; i < 8; i++)
  {
    NodeIDAddressRead = (readOnlyMUX(i, ALARA_DIGITAL_ADDRESS_1, ALARA_DIGITAL_ADDRESS_2, ALARA_DIGITAL_ADDRESS_3, ALARA_DIGITAL_ADDRESS_4) << (i+1))-1;
  Serial.print("nodeID Read Inside Loop: ");
  Serial.println(NodeIDAddressRead);
  }
  Serial.print("nodeID Read: ");
  Serial.println(NodeIDAddressRead);
 */
}