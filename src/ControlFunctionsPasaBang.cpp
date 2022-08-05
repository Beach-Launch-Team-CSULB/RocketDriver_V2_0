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
        currentCommand = command_TankPressPressurized;
        break;
    case VehicleState::fireArmed:
        currentCommand = command_fireArm;
        break;
    case VehicleState::fire: // if we powercycle mid fire, we just abort (maybe shouldn't always be true with multinode systems)
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

void commandExecute(VehicleState& currentState, VehicleState& priorState, Command& currentCommand, bool& newCommand, const std::array<AutoSequence*, NUM_AUTOSEQUENCES>& autoSequenceArray, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const std::array<TankPressController*, NUM_TANKPRESSCONTROLLERS>& tankPressControllerArray, const std::array<EngineController*, NUM_ENGINECONTROLLERS>& engineControllerArray)
{
    if (newCommand)
    {
        switch (currentCommand)
        {
            case command_debug:
                currentState = VehicleState::debug;
                break;
            case command_passive:
                //Serial.println(" Is command Passive case happening");
                currentState = VehicleState::passive;
                break;
            case command_test:
                if(currentState == VehicleState::passive)
                {
                currentState = VehicleState::test;
                }
                break;
            case command_EnterOffNominal:
                priorState = currentState; //for remembering the state the system was in when entering Off Nominal
                currentState = VehicleState::offNominal;
                break;            
            case command_ExitOffNominal:
                if(currentState == VehicleState::offNominal)
                {
                currentState = priorState; //returns to prior state when exiting offNominal
                }
                break;
            case command_abort:
                currentState = VehicleState::abort;
                break;
            case command_vent:
                currentState = VehicleState::vent;
                break;
    // Fire Sequence commands will only be executed from the proper state
            case command_HiPressArm:
                if(currentState == VehicleState::passive)
                {
                currentState = VehicleState::HiPressArm;
                }
                break;
            case command_HiPressPressurized:
                if(currentState == VehicleState::HiPressArm || currentState == VehicleState::TankPressArm) //added second conditional to allow entry backwards in a "disarm" state change
                {
                currentState = VehicleState::HiPressPressurized;
                }
                break;
            case command_TankPressArm:
                if(currentState == VehicleState::HiPressPressurized)
                {
                currentState = VehicleState::TankPressArm;
                }
                break;
            case command_TankPressPressurized:
                //if(currentState == VehicleState::TankPressArm || currentState ==VehicleState::fireArmed) //do we want to be able to go backwards out of fire armed?
                if(currentState == VehicleState::TankPressArm)
                {
                currentState = VehicleState::TankPressPressurized;
                }
                break;
            case command_fireArm:
                if(currentState == VehicleState::TankPressPressurized)
                {
                currentState = VehicleState::fireArmed;
                }
                break;
            case command_fire:
                if(currentState == VehicleState::fireArmed)
                {
                currentState = VehicleState::fire;
                // one time actions when the state is commanded
                tankPressControllerArray.at(LoxTankController_ArrayPointer)->setK_i();  //turms K_i back on
                tankPressControllerArray.at(FuelTankController_ArrayPointer)->setK_i(); //turms K_i back on
                tankPressControllerArray.at(LoxTankController_ArrayPointer)->resetIntegralCalc(true);
                tankPressControllerArray.at(FuelTankController_ArrayPointer)->resetIntegralCalc(true);
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
    newCommand = false; //reset new command flag after command is processed
    }
}

void vehicleStateMachine(VehicleState& currentState, VehicleState& priorState, Command& currentCommand, const std::array<AutoSequence*, NUM_AUTOSEQUENCES>& autoSequenceArray, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const std::array<TankPressController*, NUM_TANKPRESSCONTROLLERS>& tankPressControllerArray, const std::array<EngineController*, NUM_ENGINECONTROLLERS>& engineControllerArray, bool & haltFlag)
{
    switch (currentState)
    {
        case VehicleState::debug:
            //doesn't do anything for now
            //plan was to make this disable all real outputs for code only testing
            break;
        case VehicleState::passive:
            autoSequenceArray.at(0)->setState(AutoSequenceState::Standby);
            tankPressControllerArray.at(LoxTankController_ArrayPointer)->setK_i(0);  //turms K_i back off
            tankPressControllerArray.at(FuelTankController_ArrayPointer)->setK_i(0); //turms K_i back off
            tankPressControllerArray.at(HighPressTankController_ArrayPointer)->setState(TankPressControllerState::Passive);
            tankPressControllerArray.at(LoxTankController_ArrayPointer)->setState(TankPressControllerState::Passive);
            tankPressControllerArray.at(FuelTankController_ArrayPointer)->setState(TankPressControllerState::Passive);
            engineControllerArray.at(Engine1Controller_ArrayPointer)->setState(EngineControllerState::Passive);
            haltFlag = false;
            break;
        case VehicleState::test:
            tankPressControllerArray.at(HighPressTankController_ArrayPointer)->setState(TankPressControllerState::TestPassthrough);
            tankPressControllerArray.at(LoxTankController_ArrayPointer)->setState(TankPressControllerState::TestPassthrough);
            tankPressControllerArray.at(FuelTankController_ArrayPointer)->setState(TankPressControllerState::TestPassthrough);
            engineControllerArray.at(Engine1Controller_ArrayPointer)->setState(EngineControllerState::TestPassthrough);
            break;
        case VehicleState::offNominal:
            tankPressControllerArray.at(HighPressTankController_ArrayPointer)->setState(TankPressControllerState::OffNominalPassthrough);
            tankPressControllerArray.at(LoxTankController_ArrayPointer)->setState(TankPressControllerState::OffNominalPassthrough);
            tankPressControllerArray.at(FuelTankController_ArrayPointer)->setState(TankPressControllerState::OffNominalPassthrough);
            engineControllerArray.at(Engine1Controller_ArrayPointer)->setState(EngineControllerState::OffNominalPassthrough);
            break;            
        case VehicleState::abort:
            //haltFlag = true; //does this stay here???
            tankPressControllerArray.at(HighPressTankController_ArrayPointer)->setState(TankPressControllerState::Abort);
            tankPressControllerArray.at(LoxTankController_ArrayPointer)->setState(TankPressControllerState::Abort);
            tankPressControllerArray.at(FuelTankController_ArrayPointer)->setState(TankPressControllerState::Abort);
            engineControllerArray.at(Engine1Controller_ArrayPointer)->setState(EngineControllerState::Shutdown);
            autoSequenceArray.at(0)->setState(AutoSequenceState::Hold);
            break;
        case VehicleState::vent:
            autoSequenceArray.at(0)->setState(AutoSequenceState::Hold);
            tankPressControllerArray.at(HighPressTankController_ArrayPointer)->setState(TankPressControllerState::Passive);
            tankPressControllerArray.at(LoxTankController_ArrayPointer)->setState(TankPressControllerState::Vent);
            tankPressControllerArray.at(FuelTankController_ArrayPointer)->setState(TankPressControllerState::Vent);
            engineControllerArray.at(Engine1Controller_ArrayPointer)->setState(EngineControllerState::Passive);
            break;
        case VehicleState::HiPressArm:
            autoSequenceArray.at(0)->setState(AutoSequenceState::Standby);
            tankPressControllerArray.at(HighPressTankController_ArrayPointer)->setState(TankPressControllerState::Armed);
            tankPressControllerArray.at(LoxTankController_ArrayPointer)->setState(TankPressControllerState::Passive);
            tankPressControllerArray.at(FuelTankController_ArrayPointer)->setState(TankPressControllerState::Passive);
            engineControllerArray.at(Engine1Controller_ArrayPointer)->setState(EngineControllerState::Passive);
            break;
        case VehicleState::HiPressPressurized:
            autoSequenceArray.at(0)->setState(AutoSequenceState::Standby);
            tankPressControllerArray.at(HighPressTankController_ArrayPointer)->setState(TankPressControllerState::RegPressActive);
            tankPressControllerArray.at(LoxTankController_ArrayPointer)->setState(TankPressControllerState::Passive);
            tankPressControllerArray.at(FuelTankController_ArrayPointer)->setState(TankPressControllerState::Passive);
            engineControllerArray.at(Engine1Controller_ArrayPointer)->setState(EngineControllerState::Passive);
            break;
        case VehicleState::TankPressArm:
            autoSequenceArray.at(0)->setState(AutoSequenceState::Standby);
            tankPressControllerArray.at(HighPressTankController_ArrayPointer)->setState(TankPressControllerState::RegPressActive);
            tankPressControllerArray.at(LoxTankController_ArrayPointer)->setState(TankPressControllerState::Armed);
            tankPressControllerArray.at(FuelTankController_ArrayPointer)->setState(TankPressControllerState::Armed);
            engineControllerArray.at(Engine1Controller_ArrayPointer)->setState(EngineControllerState::Passive);
            break;
        case VehicleState::TankPressPressurized:
            autoSequenceArray.at(0)->setState(AutoSequenceState::Standby);
            tankPressControllerArray.at(HighPressTankController_ArrayPointer)->setState(TankPressControllerState::RegPressActive);
            tankPressControllerArray.at(LoxTankController_ArrayPointer)->setState(TankPressControllerState::BangBangActive);
            tankPressControllerArray.at(FuelTankController_ArrayPointer)->setState(TankPressControllerState::BangBangActive);
            engineControllerArray.at(Engine1Controller_ArrayPointer)->setState(EngineControllerState::Passive);
            break;
        case VehicleState::fireArmed:
            autoSequenceArray.at(0)->setState(AutoSequenceState::Standby);
            tankPressControllerArray.at(HighPressTankController_ArrayPointer)->setState(TankPressControllerState::RegPressActive);
            tankPressControllerArray.at(LoxTankController_ArrayPointer)->setState(TankPressControllerState::BangBangActive);
            tankPressControllerArray.at(FuelTankController_ArrayPointer)->setState(TankPressControllerState::BangBangActive);
            engineControllerArray.at(Engine1Controller_ArrayPointer)->setState(EngineControllerState::Armed);
            break;
        case VehicleState::fire:
            autoSequenceArray.at(0)->setState(AutoSequenceState::RunCommanded);            
            tankPressControllerArray.at(HighPressTankController_ArrayPointer)->setState(TankPressControllerState::RegPressActive);
            tankPressControllerArray.at(LoxTankController_ArrayPointer)->setState(TankPressControllerState::BangBangActive);
            tankPressControllerArray.at(FuelTankController_ArrayPointer)->setState(TankPressControllerState::BangBangActive);
            engineControllerArray.at(Engine1Controller_ArrayPointer)->setState(EngineControllerState::FiringAutosequence);
            break;

        default:
            break;
    }
}

// old format vehicleStateMachine
/* void vehicleStateMachine(VehicleState& currentState, VehicleState& priorState, Command& currentCommand, const std::array<AutoSequence*, NUM_AUTOSEQUENCES>& autoSequenceArray, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const std::array<TankPressController*, NUM_TANKPRESSCONTROLLERS>& tankPressControllerArray, const std::array<EngineController*, NUM_ENGINECONTROLLERS>& engineControllerArray, bool & haltFlag)
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
            engineControllerArray.at(Engine1Controller_ArrayPointer)->setState(EngineControllerState::Armed);
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
 */
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
    // Engine Controller to Tank Controller Pc Target set
    tankPressControllerArray.at(LoxTankController_ArrayPointer)->setPcTarget(engineControllerArray.at(Engine1Controller_ArrayPointer)->getCurrentPcTarget());
    tankPressControllerArray.at(FuelTankController_ArrayPointer)->setPcTarget(engineControllerArray.at(Engine1Controller_ArrayPointer)->getCurrentPcTarget());

    //sei(); // reenables interrupts after controller sync
}

void configMSGread(configMSG& currentConfigMSG, bool& NewConfigMessage, const std::array<Valve*, NUM_VALVES>& valveArray, const std::array<Pyro*, NUM_PYROS>& pyroArray, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const std::array<AutoSequence*, NUM_AUTOSEQUENCES>& autoSequenceArray, const std::array<TankPressController*, NUM_TANKPRESSCONTROLLERS>& tankPressControllerArray, const std::array<EngineController*, NUM_ENGINECONTROLLERS>& engineControllerArray, FluidSystemSimulation& fluidSim)
{
// bool for whether to continue to check the next object array (object ID has not been found yet)
// NON UNIQUE OBJECT IDS WILL BREAK THIS SYSTEM
bool idSearch = true; 

if (NewConfigMessage) //only run all this nonsense if there is a new config message
    {

/*         Serial.println("inside configMSGread: ");
        Serial.print("Target ID: ");
        Serial.print(currentConfigMSG.TargetObjectID);
        Serial.print("Setting ID: ");
        Serial.print(currentConfigMSG.ObjectSettingID);
        Serial.print("uint32 config rcv: ");
        Serial.print(currentConfigMSG.uint32Value,10);
        Serial.print("float config rcv: ");
        Serial.println(currentConfigMSG.floatValue,10); */

    ////// ----- Valve Sets ----- /////
    if (idSearch)
    {
        //Serial.println("do I get past idSearch: valve:  ");
        for (auto valve : valveArray)
        {
            if (currentConfigMSG.TargetObjectID == valve->getValveID())
            {
                switch (currentConfigMSG.ObjectSettingID)
                {
                case 0:
                    valve->resetAll();
                    break;
                case 1:
                    valve->setValveType(currentConfigMSG.uint8Value);
                    break;
                case 2:
                    valve->setFullDutyTime(currentConfigMSG.uint32Value);
                    break;
                case 3:
                    valve->setFullDutyCyclePWM(currentConfigMSG.uint16Value);
                    break;
                case 4:
                    valve->setHoldDutyCyclePWM(currentConfigMSG.uint8Value);
                    break;
                case 5:
                    valve->setWarmDutyCyclePWM(currentConfigMSG.uint16Value);
                    break;
                
                default:
                    break;
                }
                idSearch = false;
                break;
            }
            
        }
    }

    ////// ----- Pyro Sets ----- /////
    if (idSearch)
    {
        //Serial.println("do I get past idSearch: pyro:  ");
        for (auto pyro : pyroArray)
        {
            if (currentConfigMSG.TargetObjectID == pyro->getPyroID())
            {
                switch (currentConfigMSG.ObjectSettingID)
                {
                case 0:
                    pyro->resetAll();
                    break;
                case 1:
                    pyro->setLiveOutTime(currentConfigMSG.uint32Value);
                    break;
                case 2:
                    //pyro->
                    break;
                case 3:
                    //pyro->
                    break;
                
                default:
                    break;
                }
                idSearch = false;
                break;
            }
            
        }
    }

    ////// ----- Sensor Sets ----- /////
    if (idSearch)
    {
        //Serial.println("do I get past idSearch: sensor:  ");
        for (auto sensor : sensorArray)
        {
            if (currentConfigMSG.TargetObjectID == sensor->getSensorID())
            {
                switch (currentConfigMSG.ObjectSettingID)
                {
                case 0:
                    sensor->resetAll();
                    break;
                case 1:
                    sensor->setSampleRateSlowMode(currentConfigMSG.uint32Value);
                    break;
                case 2:
                    sensor->setSampleRateMedMode(currentConfigMSG.uint32Value);
                    break;
                case 3:
                    sensor->setSampleRateFastMode(currentConfigMSG.uint32Value);
                    break;
                case 4:
                    sensor->setAlphaEMA(currentConfigMSG.floatValue);
                    break;
                
                default:
                    break;
                }
                idSearch = false;
                break;
            }
            
        }
    }

    ////// ----- AutoSequence Sets ----- /////
    if (idSearch)
    {
        //Serial.println("do I get past idSearch: autosequence:  ");
        for (auto autosequence : autoSequenceArray)
        {
            if (currentConfigMSG.TargetObjectID == autosequence->getAutoSequenceID())
            {
                switch (currentConfigMSG.ObjectSettingID)
                {
                case 0:
                    autosequence->resetAll();
                    break;
                case 1:
                    autosequence->setCountdownStart(currentConfigMSG.uint32Value);
                    break;
                case 2:
                    //autosequence->
                    break;
                case 3:
                    //autosequence->
                    break;
                
                default:
                    break;
                }
                idSearch = false;
                break;
            }
            
        }
    }

    ////// ----- Tank Press Controller Sets ----- /////
    if (idSearch)
    {
        //Serial.println("do I get past idSearch: tankPressController:  ");
        for (auto tankPressController : tankPressControllerArray)
        {
            if (currentConfigMSG.TargetObjectID == tankPressController->getControllerID())
            {
                switch (currentConfigMSG.ObjectSettingID)
                {
                case 0:
                    tankPressController->resetAll();
                    break;
                case 1:
                    tankPressController->setK_p(currentConfigMSG.floatValue);
                    break;
                case 2:
                    tankPressController->setK_i(currentConfigMSG.floatValue);
                    break;
                case 3:
                    tankPressController->setK_d(currentConfigMSG.floatValue);
                    break;
                case 4:
                    tankPressController->setControllerThreshold(currentConfigMSG.floatValue);
                    break;
                case 5:
                    tankPressController->setVentFailsafePressure(currentConfigMSG.floatValue);
                    break;
                case 6:
                    tankPressController->setValveMinimumEnergizeTime(currentConfigMSG.uint32Value);
                    break;
                case 7:
                    tankPressController->setValveMinimumDeenergizeTime(currentConfigMSG.uint32Value);
                    break;
                //need to add stuff for the fluid math for the dp calc, for the sensor filtering algorithm settings, and any other math/logic still to come
                //maybe need the valve warming/partial on/off duty cycle values here to pass to the valves? Probably just toggle for a bool? 
                default:
                    break;
                }
                idSearch = false;
                break;
            }
            
        }
    }

    ////// ----- Tank Press Controller Sets ----- /////
    if (idSearch)
    {
        //Serial.println("do I get past idSearch: engineController:  ");
        for (auto engineController : engineControllerArray)
        {
            if (currentConfigMSG.TargetObjectID == engineController->getControllerID())
            {
                switch (currentConfigMSG.ObjectSettingID)
                {
                case 0:
                    engineController->resetAll();
                    break;
                case 1:
                    engineController->setFuelMVAutosequenceActuation(currentConfigMSG.int32Value);
                    break;
                case 2:
                    engineController->setLoxMVAutosequenceActuation(currentConfigMSG.int32Value);
                    break;
                case 3:
                    engineController->setIgniter1ActuationActuation(currentConfigMSG.int32Value);
                    break;
                case 4:
                    engineController->setIgniter2ActuationActuation(currentConfigMSG.int32Value);
                    break;
                case 5:
                    engineController->setThrottleProgramPoint(currentConfigMSG.uint16Value2X[0],currentConfigMSG.uint16Value2X[1]);
                    break;
                case 6:
                    engineController->throttleProgramReset();   //resets entire throttle program, wipes all points and goes back to only T=0 default
                    break;
                case 7:
                    engineController->throttleProgramReset(currentConfigMSG.uint16Value2X[0]); //removes any throttle program points with matching time value
                    break;
                
                default:
                    break;
                }
                idSearch = false;
                break;
            }
            
        }
    }
    ////// ----- Fluid System Simulation ----- /////
    if (idSearch)
    {
        //Serial.println("do I get past idSearch: engineController:  ");
        //no for statement, only one fluid sim for whole system running passed in without the pointer array style
        //{
            if (currentConfigMSG.TargetObjectID == fluidSim.getSimID())
            {
                switch (currentConfigMSG.ObjectSettingID)
                {
                case 0:
                    fluidSim.resetAll();
                    break;
                case 1:
                    fluidSim.resetSim();
                    break;
                case 2:
                    //fluidSim.set          //Need to build out all the possible simulation configuration set functions as we finish maturing the sim
                    break;
                case 3:
                    //fluidSim.set
                    break;
                case 4:
                    //fluidSim.set
                    break;
                
                default:
                    break;
                }
                idSearch = false;
                //break;
            }
        //}
    }
}