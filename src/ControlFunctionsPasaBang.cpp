#include "ControlFunctionsPasaBang.h"

// -------------------------------------------------------------
// CONFIRM All DEFINES MATCH DEVICE DEFINITIONS
// valveArray position defines for pointers
#define HiPressVent_ArrayPointer 0
#define LoxMV_ArrayPointer 1
#define FuelMV_ArrayPointer 2
#define LoxVent_ArrayPointer 3
#define LoxBang_ArrayPointer 4
#define FuelVent_ArrayPointer 5
#define FuelBang_ArrayPointer 6

// pyroArray position defines for pointers
#define EngineIgniter1_ArrayPointer 0
#define EngineIgniter2_ArrayPointer 1

// autosequenceArray position defines for pointers
#define IgnitionAutoSequence 0

// sensorArray position defines for pointers

#define ChamberPT1_ArrayPointer 0
#define FuelLinePT_ArrayPointer 1
#define LoxLinePT_ArrayPointer 2
#define FuelTank1PT_ArrayPointer 3
#define FuelTank2PT_ArrayPointer 4
#define LoxTank1PT_ArrayPointer 5
#define LoxTank2PT_ArrayPointer 6
#define HiPressPT_ArrayPointer 7

#define FakeChamberPT1_ArrayPointer 8
#define FakeFuelLinePT_ArrayPointer 9
#define FakeLoxLinePT_ArrayPointer 10
#define FakeFuelTankPT_ArrayPointer 11
#define FakeLoxTankPT_ArrayPointer 12
#define FakeHiPressPT_ArrayPointer 13

// actuator position defines for pointers
#define Engine1TVC_Y_ArrayPointer 0
#define Engine1TVC_Z_ArrayPointer 1

// tank press controller defines for pointers
#define HighPressTankController_ArrayPointer 0
#define LoxTankController_ArrayPointer 1
#define FuelTankController_ArrayPointer 2

// engine controller defines for pointers
#define Engine1Controller_ArrayPointer 0

// -------------------------------------------------------------

void startupStateCheck(const VehicleState& currentState, Command& currentCommand)
{
    switch (currentState)
    {
    case VehicleState::passive:
        currentCommand = command_passive;
        break;
    case VehicleState::test:
        currentCommand = command_test;
        break;
    case VehicleState::HiPressArm:
        currentCommand = command_HiPressArm;
        break;
    case VehicleState::HiPressPressurized:
        currentCommand = command_HiPressPressurized;
        break;
    case VehicleState::TankPressArm:
        currentCommand = command_TankPressArm;
        break;
    case VehicleState::TankPressPressurized:
        currentCommand = commend_TankPressPressurized;
        break;
    case VehicleState::fireArmed:
        currentCommand = command_fireArm;
        break;
    case VehicleState::fire: // if we powercycle mid fire, we just vent (maybe shouldn't always be true with multinode systems)
        currentCommand = command_abort;
        break;
    case VehicleState::abort:
        currentCommand = command_abort;
        break;
    case VehicleState::vent:
        currentCommand = command_vent;
        break;
    default:
        break;
    }
}


void vehicleStateMachine(VehicleState& currentState, VehicleState& priorState, Command& currentCommand, const std::array<AutoSequence*, NUM_AUTOSEQUENCES>& autoSequenceArray, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const std::array<TankPressController*, NUM_TANKPRESSCONTROLLERS>& tankPressControllerArray, const std::array<EngineController*, NUM_ENGINECONTROLLERS>& engineControllerArray, bool & haltFlag)
{
    switch (currentCommand)
    {
        case command_debug:
            currentState = VehicleState::debug;
            break;
        case command_passive:
            autoSequenceArray.at(0)->setState(AutoSequenceState::Hold);
            tankPressControllerArray.at(LoxTankController_ArrayPointer)->setK_i(0);  //turms K_i back off
            tankPressControllerArray.at(FuelTankController_ArrayPointer)->setK_i(0); //turms K_i back off
            tankPressControllerArray.at(HighPressTankController_ArrayPointer)->setState(TankPressControllerState::Passive);
            tankPressControllerArray.at(LoxTankController_ArrayPointer)->setState(TankPressControllerState::Passive);
            tankPressControllerArray.at(FuelTankController_ArrayPointer)->setState(TankPressControllerState::Passive);
            engineControllerArray.at(Engine1Controller_ArrayPointer)->setState(EngineControllerState::Passive);
            currentState = VehicleState::passive;
            haltFlag = false;
            break;
        case command_test:
            if(currentState == VehicleState::passive)
            {
            tankPressControllerArray.at(HighPressTankController_ArrayPointer)->setState(TankPressControllerState::TestPassthrough);
            tankPressControllerArray.at(LoxTankController_ArrayPointer)->setState(TankPressControllerState::TestPassthrough);
            tankPressControllerArray.at(FuelTankController_ArrayPointer)->setState(TankPressControllerState::TestPassthrough);
            engineControllerArray.at(Engine1Controller_ArrayPointer)->setState(EngineControllerState::TestPassthrough);
            currentState = VehicleState::test;
            }
            break;
        case command_EnterOffNominal:
            priorState = currentState; //for remembering the state the system was in when entering Off Nominal
            tankPressControllerArray.at(HighPressTankController_ArrayPointer)->setState(TankPressControllerState::OffNominalPassthrough);
            tankPressControllerArray.at(LoxTankController_ArrayPointer)->setState(TankPressControllerState::OffNominalPassthrough);
            tankPressControllerArray.at(FuelTankController_ArrayPointer)->setState(TankPressControllerState::OffNominalPassthrough);
            engineControllerArray.at(Engine1Controller_ArrayPointer)->setState(EngineControllerState::OffNominalPassthrough);
            currentState = VehicleState::offNominal;
            break;            
        case command_ExitOffNominal:
            if(currentState == VehicleState::offNominal)
            {
            tankPressControllerArray.at(HighPressTankController_ArrayPointer)->setState(tankPressControllerArray.at(HighPressTankController_ArrayPointer)->getPriorState());
            tankPressControllerArray.at(LoxTankController_ArrayPointer)->setState(tankPressControllerArray.at(LoxTankController_ArrayPointer)->getPriorState());
            tankPressControllerArray.at(FuelTankController_ArrayPointer)->setState(tankPressControllerArray.at(FuelTankController_ArrayPointer)->getPriorState());
            engineControllerArray.at(Engine1Controller_ArrayPointer)->setState(engineControllerArray.at(Engine1Controller_ArrayPointer)->getPriorState());
            currentState = priorState;              //IS THIS STILL TRUE???? - Beware, this currently doesn't function fully as desired. Will leave ValveEnables all on and not actually enter the prior command
            }
            break;
        case command_abort:
            haltFlag = true;
            currentState = VehicleState::abort;
            tankPressControllerArray.at(HighPressTankController_ArrayPointer)->setState(TankPressControllerState::Abort);
            tankPressControllerArray.at(LoxTankController_ArrayPointer)->setState(TankPressControllerState::Abort);
            tankPressControllerArray.at(FuelTankController_ArrayPointer)->setState(TankPressControllerState::Abort);
            engineControllerArray.at(Engine1Controller_ArrayPointer)->setState(EngineControllerState::Shutdown);
            autoSequenceArray.at(0)->setState(AutoSequenceState::Hold);
            break;
        case command_vent:
            autoSequenceArray.at(0)->setState(AutoSequenceState::Hold);
            tankPressControllerArray.at(HighPressTankController_ArrayPointer)->setState(TankPressControllerState::Passive);
            tankPressControllerArray.at(LoxTankController_ArrayPointer)->setState(TankPressControllerState::Vent);
            tankPressControllerArray.at(FuelTankController_ArrayPointer)->setState(TankPressControllerState::Vent);
            engineControllerArray.at(Engine1Controller_ArrayPointer)->setState(EngineControllerState::Passive);
            currentState = VehicleState::vent;
            break;
// Fire Sequence commands will only be executed from the proper state
        case command_HiPressArm:
            if(currentState == VehicleState::passive)
            {
            autoSequenceArray.at(0)->setState(AutoSequenceState::Standby);
            tankPressControllerArray.at(HighPressTankController_ArrayPointer)->setState(TankPressControllerState::Armed);
            tankPressControllerArray.at(LoxTankController_ArrayPointer)->setState(TankPressControllerState::Passive);
            tankPressControllerArray.at(FuelTankController_ArrayPointer)->setState(TankPressControllerState::Passive);
            engineControllerArray.at(Engine1Controller_ArrayPointer)->setState(EngineControllerState::Passive);
            currentState = VehicleState::HiPressArm;
            }
            break;
        case command_HiPressPressurized:
            if(currentState == VehicleState::HiPressArm || currentState == VehicleState::TankPressArm) //added second conditional to allow entry backwards in a "disarm" state change
            {
            autoSequenceArray.at(0)->setState(AutoSequenceState::Standby);
            tankPressControllerArray.at(HighPressTankController_ArrayPointer)->setState(TankPressControllerState::RegPressActive);
            tankPressControllerArray.at(LoxTankController_ArrayPointer)->setState(TankPressControllerState::Passive);
            tankPressControllerArray.at(FuelTankController_ArrayPointer)->setState(TankPressControllerState::Passive);
            engineControllerArray.at(Engine1Controller_ArrayPointer)->setState(EngineControllerState::Passive);
            currentState = VehicleState::HiPressPressurized;
            }
            break;
        case command_TankPressArm:
            if(currentState == VehicleState::HiPressPressurized)
            {
            autoSequenceArray.at(0)->setState(AutoSequenceState::Standby);
            tankPressControllerArray.at(HighPressTankController_ArrayPointer)->setState(TankPressControllerState::RegPressActive);
            tankPressControllerArray.at(LoxTankController_ArrayPointer)->setState(TankPressControllerState::Armed);
            tankPressControllerArray.at(FuelTankController_ArrayPointer)->setState(TankPressControllerState::Armed);
            engineControllerArray.at(Engine1Controller_ArrayPointer)->setState(EngineControllerState::Passive);
            currentState = VehicleState::TankPressArm;
            }
            break;
        case commend_TankPressPressurized:
            if(currentState == VehicleState::TankPressArm)
            {
            autoSequenceArray.at(0)->setState(AutoSequenceState::Standby);
            tankPressControllerArray.at(HighPressTankController_ArrayPointer)->setState(TankPressControllerState::RegPressActive);
            tankPressControllerArray.at(LoxTankController_ArrayPointer)->setState(TankPressControllerState::BangBangActive);
            tankPressControllerArray.at(FuelTankController_ArrayPointer)->setState(TankPressControllerState::BangBangActive);
            engineControllerArray.at(Engine1Controller_ArrayPointer)->setState(EngineControllerState::Passive);
            currentState = VehicleState::TankPressPressurized;
            }
            break;
        case command_fireArm:
            if(currentState == VehicleState::TankPressPressurized)
            {
            autoSequenceArray.at(0)->setState(AutoSequenceState::Standby);
            tankPressControllerArray.at(HighPressTankController_ArrayPointer)->setState(TankPressControllerState::RegPressActive);
            tankPressControllerArray.at(LoxTankController_ArrayPointer)->setState(TankPressControllerState::BangBangActive);
            tankPressControllerArray.at(FuelTankController_ArrayPointer)->setState(TankPressControllerState::BangBangActive);
            engineControllerArray.at(Engine1Controller_ArrayPointer)->setState(EngineControllerState::Passive);
            currentState = VehicleState::fireArmed;
            }
            break;
        case command_fire:
            if(currentState == VehicleState::fireArmed)
            {
            autoSequenceArray.at(0)->setState(AutoSequenceState::RunCommanded);            
            tankPressControllerArray.at(LoxTankController_ArrayPointer)->setK_i();  //turms K_i back on
            tankPressControllerArray.at(FuelTankController_ArrayPointer)->setK_i(); //turms K_i back on
            tankPressControllerArray.at(HighPressTankController_ArrayPointer)->setState(TankPressControllerState::RegPressActive);
            tankPressControllerArray.at(LoxTankController_ArrayPointer)->setState(TankPressControllerState::BangBangActive);
            tankPressControllerArray.at(LoxTankController_ArrayPointer)->resetIntegralCalc(true);
            tankPressControllerArray.at(FuelTankController_ArrayPointer)->setState(TankPressControllerState::BangBangActive);
            tankPressControllerArray.at(FuelTankController_ArrayPointer)->resetIntegralCalc(true);
            engineControllerArray.at(Engine1Controller_ArrayPointer)->setState(EngineControllerState::FiringAutosequence);
            currentState = VehicleState::fire;
            }
            break;
        case command_closeHiPress:
            if(currentState == VehicleState::test || currentState == VehicleState::offNominal)
            {
                tankPressControllerArray.at(HighPressTankController_ArrayPointer)->testSetPrimaryPressValveState(ValveState::CloseCommanded);
            }
            break;
        case command_openHiPress:
             if(currentState == VehicleState::test || currentState == VehicleState::offNominal)
            {
                tankPressControllerArray.at(HighPressTankController_ArrayPointer)->testSetPrimaryPressValveState(ValveState::OpenCommanded);
            }
            break;
        case command_closeHiPressVent:
            if(currentState == VehicleState::test || currentState == VehicleState::offNominal)
            {
                tankPressControllerArray.at(HighPressTankController_ArrayPointer)->testSetPressLineVentState(ValveState::CloseCommanded);
            }
            break;
        case command_openHiPressVent:
             if(currentState == VehicleState::test || currentState == VehicleState::offNominal)
            {
                tankPressControllerArray.at(HighPressTankController_ArrayPointer)->testSetPressLineVentState(ValveState::OpenCommanded);
            }
            break;
        case command_closeLoxVent:
            if(currentState == VehicleState::test || currentState == VehicleState::offNominal)
            {
                tankPressControllerArray.at(LoxTankController_ArrayPointer)->testSetTankVentState(ValveState::CloseCommanded);
            }
            break;
        case command_openLoxVent:
             if(currentState == VehicleState::test || currentState == VehicleState::offNominal)
            {
                tankPressControllerArray.at(LoxTankController_ArrayPointer)->testSetTankVentState(ValveState::OpenCommanded);
            }
            break;
        case command_closeLoxPressValve:
            if(currentState == VehicleState::test || currentState == VehicleState::offNominal)
            {
                tankPressControllerArray.at(LoxTankController_ArrayPointer)->testSetPrimaryPressValveState(ValveState::CloseCommanded);
            }           
            break;
        case command_openLoxPressValve:
             if(currentState == VehicleState::test || currentState == VehicleState::offNominal)
            {
                tankPressControllerArray.at(LoxTankController_ArrayPointer)->testSetPrimaryPressValveState(ValveState::OpenCommanded);
            }            
            break; 
        case command_closeLoxPressLineVent:
            if(currentState == VehicleState::test || currentState == VehicleState::offNominal)
            {
                tankPressControllerArray.at(LoxTankController_ArrayPointer)->testSetPressLineVentState(ValveState::CloseCommanded);
            }
            break;
        case command_openLoxPressLineVent:
             if(currentState == VehicleState::test || currentState == VehicleState::offNominal)
            {
                tankPressControllerArray.at(LoxTankController_ArrayPointer)->testSetPressLineVentState(ValveState::OpenCommanded);
            }
            break; 
        case command_closeFuelVent:
            if(currentState == VehicleState::test || currentState == VehicleState::offNominal)
            {
                tankPressControllerArray.at(FuelTankController_ArrayPointer)->testSetTankVentState(ValveState::CloseCommanded);
            }           
            break;
        case command_openFuelVent:
             if(currentState == VehicleState::test || currentState == VehicleState::offNominal)
            {
                tankPressControllerArray.at(FuelTankController_ArrayPointer)->testSetTankVentState(ValveState::OpenCommanded);
            }
            break;
        case command_closeFuelPressValve:
            if(currentState == VehicleState::test || currentState == VehicleState::offNominal)
            {
                tankPressControllerArray.at(FuelTankController_ArrayPointer)->testSetPrimaryPressValveState(ValveState::CloseCommanded);
            }
            break;
        case command_openFuelPressValve:
             if(currentState == VehicleState::test || currentState == VehicleState::offNominal)
            {
                tankPressControllerArray.at(FuelTankController_ArrayPointer)->testSetPrimaryPressValveState(ValveState::OpenCommanded);
            }
            break; 
        case command_closeFuelPressLineVent:
            if(currentState == VehicleState::test || currentState == VehicleState::offNominal)
            {
                tankPressControllerArray.at(FuelTankController_ArrayPointer)->testSetPressLineVentState(ValveState::CloseCommanded);
            }        
            break;
        case command_openFuelPressLineVent:
             if(currentState == VehicleState::test || currentState == VehicleState::offNominal)
            {
                tankPressControllerArray.at(FuelTankController_ArrayPointer)->testSetPressLineVentState(ValveState::OpenCommanded);
            }           
            break; 
        case command_closeFuelMV:
            if(currentState == VehicleState::test || currentState == VehicleState::offNominal)
            {
                engineControllerArray.at(Engine1Controller_ArrayPointer)->testSetPilotMVFuelValveState(ValveState::CloseCommanded);
            }        
            break;
        case command_openFuelMV:
             if(currentState == VehicleState::test || currentState == VehicleState::offNominal)
            {
                engineControllerArray.at(Engine1Controller_ArrayPointer)->testSetPilotMVFuelValveState(ValveState::OpenCommanded);
            }     
            break;
        case command_closeLoxMV:
            if(currentState == VehicleState::test || currentState == VehicleState::offNominal)
            {
                engineControllerArray.at(Engine1Controller_ArrayPointer)->testSetPilotMVLoxValveState(ValveState::CloseCommanded);
            }            
            break;
        case command_openLoxMV:
             if(currentState == VehicleState::test || currentState == VehicleState::offNominal)
            {
                engineControllerArray.at(Engine1Controller_ArrayPointer)->testSetPilotMVLoxValveState(ValveState::OpenCommanded);
            }            
            break;
        case command_engineIgniterPyro1_Off:
            if(currentState == VehicleState::test || currentState == VehicleState::offNominal)
            {
                engineControllerArray.at(Engine1Controller_ArrayPointer)->setIgniter1State(PyroState::OffCommanded);
            }
            break;
        case command_engineIgniterPyro1_On:
            if(currentState == VehicleState::test || currentState == VehicleState::offNominal)
            {
                engineControllerArray.at(Engine1Controller_ArrayPointer)->setIgniter1State(PyroState::OnCommanded);
            }           
            break;
        case command_engineIgniterPyro2_Off:
            if(currentState == VehicleState::test || currentState == VehicleState::offNominal)
            {
                engineControllerArray.at(Engine1Controller_ArrayPointer)->setIgniter2State(PyroState::OffCommanded);
            }          
            break;
        case command_engineIgniterPyro2_On:
            if(currentState == VehicleState::test || currentState == VehicleState::offNominal)
            {
                engineControllerArray.at(Engine1Controller_ArrayPointer)->setIgniter2State(PyroState::OnCommanded);
            }
            break;

        default:
            break;
    }
}

///// ----- NEW FUNCTIONS, WORK IN PROGRESS ----- /////

// state machine for the mission state (launch, ascent, apogee, descent et cetera)
void missionStateMachine(VehicleState& currentState, VehicleState& priorState, Command& currentCommand, const std::array<Valve*, NUM_VALVES>& valveArray, const std::array<Pyro*, NUM_PYROS>& pyroArray, const std::array<AutoSequence*, NUM_AUTOSEQUENCES>& autoSequenceArray, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, bool &HaltFlag)
{
    //eventually give a shit to do stuff for flight
}

void controllerDeviceSync(VehicleState& currentState, VehicleState& priorState, Command& currentCommand, const std::array<Valve*, NUM_VALVES>& valveArray, const std::array<Pyro*, NUM_PYROS>& pyroArray, const std::array<AutoSequence*, NUM_AUTOSEQUENCES>& autoSequenceArray, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const std::array<TankPressController*, NUM_TANKPRESSCONTROLLERS>& tankPressControllerArray, const std::array<EngineController*, NUM_ENGINECONTROLLERS>& engineControllerArray, bool & haltFlag)
{
    cli(); // disables interrupts during controller sync to protect from partial propulsion system states
        // Pasa Bang SF Config
        // Lox Tank
/*          valveArray.at(LoxBang_ArrayPointer)->setState(tankPressControllerArray.at(LoxTankController_ArrayPointer)->getPrimaryPressValveState());
        tankPressControllerArray.at(HighPressTankController_ArrayPointer)->setPressVentLineStateBang1(tankPressControllerArray.at(LoxTankController_ArrayPointer)->getPressLineVentState());
        valveArray.at(LoxVent_ArrayPointer)->setState(tankPressControllerArray.at(LoxTankController_ArrayPointer)->getTankVentState());
        // Fuel Tank
        valveArray.at(FuelBang_ArrayPointer)->setState(tankPressControllerArray.at(FuelTankController_ArrayPointer)->getPrimaryPressValveState());
        tankPressControllerArray.at(HighPressTankController_ArrayPointer)->setPressVentLineStateBang2(tankPressControllerArray.at(FuelTankController_ArrayPointer)->getPressLineVentState());
        valveArray.at(FuelVent_ArrayPointer)->setState(tankPressControllerArray.at(FuelTankController_ArrayPointer)->getTankVentState());
        // COPV/High Press - RUN AFTER PROP TANKS (for now, not sure if necessary long term) - for bang controllers to sync vent line settings
        valveArray.at(HiPressVent_ArrayPointer)->setState(tankPressControllerArray.at(HighPressTankController_ArrayPointer)->getPressLineVentState());
         // Engine 1
        valveArray.at(FuelMV_ArrayPointer)->setState(engineControllerArray.at(Engine1Controller_ArrayPointer)->getPilotMVFuelValveState());
        valveArray.at(LoxMV_ArrayPointer)->setState(engineControllerArray.at(Engine1Controller_ArrayPointer)->getPilotMVLoxValveState());
        pyroArray.at(EngineIgniter1_ArrayPointer)->setState(engineControllerArray.at(Engine1Controller_ArrayPointer)->getIgniter1State());
        pyroArray.at(EngineIgniter2_ArrayPointer)->setState(engineControllerArray.at(Engine1Controller_ArrayPointer)->getIgniter2State());
 */
        sensorArray.at(FuelTank1PT_ArrayPointer)->setState(tankPressControllerArray.at(FuelTankController_ArrayPointer)->getControllerSensorState());
        sensorArray.at(FuelTank2PT_ArrayPointer)->setState(tankPressControllerArray.at(FuelTankController_ArrayPointer)->getControllerSensorState());
        sensorArray.at(LoxTank1PT_ArrayPointer)->setState(tankPressControllerArray.at(LoxTankController_ArrayPointer)->getControllerSensorState());
        sensorArray.at(LoxTank2PT_ArrayPointer)->setState(tankPressControllerArray.at(LoxTankController_ArrayPointer)->getControllerSensorState());
        sensorArray.at(HiPressPT_ArrayPointer)->setState(tankPressControllerArray.at(HighPressTankController_ArrayPointer)->getControllerSensorState());
        sensorArray.at(ChamberPT1_ArrayPointer)->setState(engineControllerArray.at(Engine1Controller_ArrayPointer)->getControllerSensorState());
        sensorArray.at(FuelLinePT_ArrayPointer)->setState(engineControllerArray.at(Engine1Controller_ArrayPointer)->getControllerSensorState());
        sensorArray.at(LoxLinePT_ArrayPointer)->setState(engineControllerArray.at(Engine1Controller_ArrayPointer)->getControllerSensorState());

    sei(); // reenables interrupts after controller sync
 
}

void controllerSensorSetup(const std::array<Valve*, NUM_VALVES>& valveArray, const std::array<Pyro*, NUM_PYROS>& pyroArray, const std::array<AutoSequence*, NUM_AUTOSEQUENCES>& autoSequenceArray, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const std::array<TankPressController*, NUM_TANKPRESSCONTROLLERS>& tankPressControllerArray, const std::array<EngineController*, NUM_ENGINECONTROLLERS>& engineControllerArray)
{
    //array initializing doesn't currently work, the input array size is doing nothing right now
    sensorArray.at(LoxTank1PT_ArrayPointer)->initializeLinReg(10);
    sensorArray.at(LoxTank1PT_ArrayPointer)->setEnableIntegralCalc(true);
    sensorArray.at(LoxTank2PT_ArrayPointer)->initializeLinReg(10);
    sensorArray.at(LoxTank2PT_ArrayPointer)->setEnableIntegralCalc(true);
    sensorArray.at(FuelTank1PT_ArrayPointer)->initializeLinReg(10);
    sensorArray.at(FuelTank1PT_ArrayPointer)->setEnableIntegralCalc(true);
    sensorArray.at(FuelTank2PT_ArrayPointer)->initializeLinReg(10);
    sensorArray.at(FuelTank2PT_ArrayPointer)->setEnableIntegralCalc(true);
}

void controllerDataSync(const std::array<Valve*, NUM_VALVES>& valveArray, const std::array<Pyro*, NUM_PYROS>& pyroArray, const std::array<AutoSequence*, NUM_AUTOSEQUENCES>& autoSequenceArray, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const std::array<TankPressController*, NUM_TANKPRESSCONTROLLERS>& tankPressControllerArray, const std::array<EngineController*, NUM_ENGINECONTROLLERS>& engineControllerArray)
{
    //cli(); // disables interrupts during controller sync to protect from partial propulsion system states
    // update the controller valves based on the valve objects before Controller stateOperations
    // Lox Tank
/*     tankPressControllerArray.at(LoxTankController_ArrayPointer)->setPrimaryPressValveState(valveArray.at(LoxBang_ArrayPointer)->getSyncState());
    tankPressControllerArray.at(LoxTankController_ArrayPointer)->setTankVentState(valveArray.at(LoxVent_ArrayPointer)->getSyncState());
    // Fuel Tank
    tankPressControllerArray.at(FuelTankController_ArrayPointer)->setPrimaryPressValveState(valveArray.at(FuelBang_ArrayPointer)->getSyncState());
    tankPressControllerArray.at(FuelTankController_ArrayPointer)->setTankVentState(valveArray.at(FuelVent_ArrayPointer)->getSyncState());
    // Engine Controller
    engineControllerArray.at(Engine1Controller_ArrayPointer)->setIgniter1State(pyroArray.at(EngineIgniter1_ArrayPointer)->getSyncState());
    engineControllerArray.at(Engine1Controller_ArrayPointer)->setIgniter2State(pyroArray.at(EngineIgniter2_ArrayPointer)->getSyncState());
    engineControllerArray.at(Engine1Controller_ArrayPointer)->setPilotMVFuelValveState(valveArray.at(FuelMV_ArrayPointer)->getSyncState());
    engineControllerArray.at(Engine1Controller_ArrayPointer)->setPilotMVLoxValveState(valveArray.at(LoxMV_ArrayPointer)->getSyncState());
 */
    //integral unwinds
    sensorArray.at(LoxTank1PT_ArrayPointer)->resetIntegralCalc(tankPressControllerArray.at(LoxTankController_ArrayPointer)->getResetIntegralCalcBool(), 0);
    sensorArray.at(FuelTank1PT_ArrayPointer)->resetIntegralCalc(tankPressControllerArray.at(FuelTankController_ArrayPointer)->getResetIntegralCalcBool(), 0);
    
    //Lox Tank sensor target value update
    sensorArray.at(LoxTank1PT_ArrayPointer)->setTargetValue(tankPressControllerArray.at(LoxTankController_ArrayPointer)->getTargetValue());
    //Fuel Tank sensor target value update
    sensorArray.at(FuelTank1PT_ArrayPointer)->setTargetValue(tankPressControllerArray.at(FuelTankController_ArrayPointer)->getTargetValue());
    //Lox Tank Controller Sensor Data fetch
    tankPressControllerArray.at(LoxTankController_ArrayPointer)->setPIDSensorInput1(sensorArray.at(LoxTank1PT_ArrayPointer)->getEMAConvertedValue(), sensorArray.at(LoxTank1PT_ArrayPointer)->getIntegralSum(), sensorArray.at(LoxTank1PT_ArrayPointer)->getLinRegSlope());
    tankPressControllerArray.at(LoxTankController_ArrayPointer)->setPIDSensorInput2(sensorArray.at(LoxTank2PT_ArrayPointer)->getEMAConvertedValue(), sensorArray.at(LoxTank2PT_ArrayPointer)->getIntegralSum(), sensorArray.at(LoxTank2PT_ArrayPointer)->getLinRegSlope());
    tankPressControllerArray.at(LoxTankController_ArrayPointer)->setPIDSensorInput3(sensorArray.at(FakeLoxTankPT_ArrayPointer)->getEMAConvertedValue(), sensorArray.at(FakeLoxTankPT_ArrayPointer)->getIntegralSum(), sensorArray.at(FakeLoxTankPT_ArrayPointer)->getLinRegSlope());
    //Lox Tank Controller Sensor Data fetch
    tankPressControllerArray.at(FuelTankController_ArrayPointer)->setPIDSensorInput1(sensorArray.at(FuelTank1PT_ArrayPointer)->getEMAConvertedValue(), sensorArray.at(FuelTank1PT_ArrayPointer)->getIntegralSum(), sensorArray.at(FuelTank1PT_ArrayPointer)->getLinRegSlope());
    tankPressControllerArray.at(FuelTankController_ArrayPointer)->setPIDSensorInput2(sensorArray.at(FuelTank2PT_ArrayPointer)->getEMAConvertedValue(), sensorArray.at(FuelTank2PT_ArrayPointer)->getIntegralSum(), sensorArray.at(FuelTank2PT_ArrayPointer)->getLinRegSlope());
    tankPressControllerArray.at(FuelTankController_ArrayPointer)->setPIDSensorInput3(sensorArray.at(FakeFuelTankPT_ArrayPointer)->getEMAConvertedValue(), sensorArray.at(FakeFuelTankPT_ArrayPointer)->getIntegralSum(), sensorArray.at(FakeFuelTankPT_ArrayPointer)->getLinRegSlope());
   
    //sei(); // reenables interrupts after controller sync
}