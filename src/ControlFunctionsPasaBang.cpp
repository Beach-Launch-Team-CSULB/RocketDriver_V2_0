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


void vehicleStateMachine(VehicleState& currentState, VehicleState& priorState, Command& currentCommand, const std::array<Valve*, NUM_VALVES>& valveArray, const std::array<Pyro*, NUM_PYROS>& pyroArray, const std::array<AutoSequence*, NUM_AUTOSEQUENCES>& autoSequenceArray, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const std::array<TankPressController*, NUM_TANKPRESSCONTROLLERS>& tankPressControllerArray, const std::array<EngineController*, NUM_ENGINECONTROLLERS>& engineControllerArray, bool & haltFlag)
{
    switch (currentCommand)
    {
        case command_debug:
            currentState = VehicleState::debug;
            break;
        case command_passive:
            autoSequenceArray.at(0)->setState(AutoSequenceState::Hold);
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
            currentState = priorState;              //IS THIS STILL TRUE???? - Beware, this currently doesn't function fully as desired. Will leave ValveEnables all on and not actually enter the prior command
            }
            break;
        case command_abort:
            haltFlag = true;
            currentState = VehicleState::abort;
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
            tankPressControllerArray.at(HighPressTankController_ArrayPointer)->setState(TankPressControllerState::DomePressActive);
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
            tankPressControllerArray.at(HighPressTankController_ArrayPointer)->setState(TankPressControllerState::DomePressActive);
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
            tankPressControllerArray.at(HighPressTankController_ArrayPointer)->setState(TankPressControllerState::DomePressActive);
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
            tankPressControllerArray.at(HighPressTankController_ArrayPointer)->setState(TankPressControllerState::DomePressActive);
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
            tankPressControllerArray.at(HighPressTankController_ArrayPointer)->setState(TankPressControllerState::DomePressActive);
            tankPressControllerArray.at(LoxTankController_ArrayPointer)->setState(TankPressControllerState::BangBangActive);
            tankPressControllerArray.at(FuelTankController_ArrayPointer)->setState(TankPressControllerState::BangBangActive);
            engineControllerArray.at(Engine1Controller_ArrayPointer)->setState(EngineControllerState::FiringAutosequence);
            currentState = VehicleState::fire;
            }
            break;
        case command_closeHiPress:
            if(currentState == VehicleState::test)
            {
                valveArray.at(0)->setState(ValveState::CloseCommanded);
            }
            else if (currentState == VehicleState::offNominal)
            {
                valveArray.at(0)->setState(ValveState::CloseCommanded);
            }
            break;
        case command_openHiPress:
             if(currentState == VehicleState::test)
            {
                valveArray.at(0)->setState(ValveState::OpenCommanded);
            }
            else if (currentState == VehicleState::offNominal)
            {
                valveArray.at(0)->setState(ValveState::OpenCommanded);
            }
            break;
        case command_closeHiPressVent:
            if(currentState == VehicleState::test)
            {
                valveArray.at(1)->setState(ValveState::CloseCommanded);
            }
            else if (currentState == VehicleState::offNominal)
            {
                valveArray.at(1)->setState(ValveState::CloseCommanded);
            }            
            break;
        case command_openHiPressVent:
             if(currentState == VehicleState::test)
            {
                valveArray.at(1)->setState(ValveState::OpenCommanded);
            }
            else if (currentState == VehicleState::offNominal)
            {
                valveArray.at(1)->setState(ValveState::OpenCommanded);
            }              
            break;
        case command_closeLoxVent:
            if(currentState == VehicleState::test)
            {
                valveArray.at(2)->setState(ValveState::CloseCommanded);
            }
            else if (currentState == VehicleState::offNominal)
            {
                valveArray.at(2)->setState(ValveState::CloseCommanded);
            }              
            break;
        case command_openLoxVent:
             if(currentState == VehicleState::test)
            {
                valveArray.at(2)->setState(ValveState::OpenCommanded);
            }
            else if (currentState == VehicleState::offNominal)
            {
                valveArray.at(2)->setState(ValveState::OpenCommanded);
            }              
            break;
        case command_closeLoxDomeReg:
            if(currentState == VehicleState::test)
            {
                valveArray.at(3)->setState(ValveState::CloseCommanded);
            }
            else if (currentState == VehicleState::offNominal)
            {
                valveArray.at(3)->setState(ValveState::CloseCommanded);
            }              
            break;
        case command_openLoxDomeReg:
             if(currentState == VehicleState::test)
            {
                valveArray.at(3)->setState(ValveState::OpenCommanded);
            }
            else if (currentState == VehicleState::offNominal)
            {
                valveArray.at(3)->setState(ValveState::OpenCommanded);
            }              
            break; 
        case command_closeLoxDomeRegVent:
            if(currentState == VehicleState::test)
            {
                valveArray.at(4)->setState(ValveState::CloseCommanded);
            }
            else if (currentState == VehicleState::offNominal)
            {
                valveArray.at(4)->setState(ValveState::CloseCommanded);
            }              
            break;
        case command_openLoxDomeRegVent:
             if(currentState == VehicleState::test)
            {
                valveArray.at(4)->setState(ValveState::OpenCommanded);
            }
            else if (currentState == VehicleState::offNominal)
            {
                valveArray.at(4)->setState(ValveState::OpenCommanded);
            }              
            break; 
        case command_closeFuelVent:
            if(currentState == VehicleState::test)
            {
                valveArray.at(5)->setState(ValveState::CloseCommanded);
            }
            else if (currentState == VehicleState::offNominal)
            {
                valveArray.at(5)->setState(ValveState::CloseCommanded);
            }              
            break;
        case command_openFuelVent:
             if(currentState == VehicleState::test)
            {
                valveArray.at(5)->setState(ValveState::OpenCommanded);
            }
            else if (currentState == VehicleState::offNominal)
            {
                valveArray.at(5)->setState(ValveState::OpenCommanded);
            }              
            break;
        case command_closeFuelDomeReg:
            if(currentState == VehicleState::test)
            {
                valveArray.at(6)->setState(ValveState::CloseCommanded);
            }
            else if (currentState == VehicleState::offNominal)
            {
                valveArray.at(6)->setState(ValveState::CloseCommanded);
            }              
            break;
        case command_openFuelDomeReg:
             if(currentState == VehicleState::test)
            {
                valveArray.at(6)->setState(ValveState::OpenCommanded);
            }
            else if (currentState == VehicleState::offNominal)
            {
                valveArray.at(6)->setState(ValveState::OpenCommanded);
            }              
            break; 
        case command_closeFuelDomeRegVent:
            if(currentState == VehicleState::test)
            {
                valveArray.at(7)->setState(ValveState::CloseCommanded);
            }
            else if (currentState == VehicleState::offNominal)
            {
                valveArray.at(7)->setState(ValveState::CloseCommanded);
            }              
            break;
        case command_openFuelDomeRegVent:
             if(currentState == VehicleState::test)
            {
                valveArray.at(7)->setState(ValveState::OpenCommanded);
            }
            else if (currentState == VehicleState::offNominal)
            {
                valveArray.at(7)->setState(ValveState::OpenCommanded);
            }              
            break; 
        case command_closeFuelMV:
            if(currentState == VehicleState::test)
            {
                valveArray.at(8)->setState(ValveState::CloseCommanded);
            }
            else if (currentState == VehicleState::offNominal)
            {
                valveArray.at(8)->setState(ValveState::CloseCommanded);
            }              
            break;
        case command_openFuelMV:
             if(currentState == VehicleState::test)
            {
                valveArray.at(8)->setState(ValveState::OpenCommanded);
            }
            else if (currentState == VehicleState::offNominal)
            {
                valveArray.at(8)->setState(ValveState::OpenCommanded);
            }              
            break;
        case command_closeLoxMV:
            if(currentState == VehicleState::test)
            {
                valveArray.at(9)->setState(ValveState::CloseCommanded);
            }
            else if (currentState == VehicleState::offNominal)
            {
                valveArray.at(9)->setState(ValveState::CloseCommanded);
            }              
            break;
        case command_openLoxMV:
             if(currentState == VehicleState::test)
            {
                valveArray.at(9)->setState(ValveState::OpenCommanded);
            }
            else if (currentState == VehicleState::offNominal)
            {
                valveArray.at(9)->setState(ValveState::OpenCommanded);
            }              
            break;
        case command_engineIgniterPyro1_Off:
            if(currentState == VehicleState::test)
            {
            pyroArray.at(0)->setState(PyroState::OffCommanded);             // Renegade SF Igniter1
            }
            else if (currentState == VehicleState::offNominal)
            {
            pyroArray.at(0)->setState(PyroState::OffCommanded);             // Renegade SF Igniter1
            }              
            break;
        case command_engineIgniterPyro1_On:
            if(currentState == VehicleState::test)
            {
            pyroArray.at(0)->setState(PyroState::OnCommanded);             // Renegade SF Igniter1
            }
            else if (currentState == VehicleState::offNominal)
            {
            pyroArray.at(0)->setState(PyroState::OnCommanded);             // Renegade SF Igniter1
            }              
            break;
        case command_engineIgniterPyro2_Off:
            if(currentState == VehicleState::test)
            {
            pyroArray.at(1)->setState(PyroState::OffCommanded);             // Renegade SF Igniter1
            }
            else if (currentState == VehicleState::offNominal)
            {
            pyroArray.at(1)->setState(PyroState::OffCommanded);             // Renegade SF Igniter1
            }              
            break;
        case command_engineIgniterPyro2_On:
            if(currentState == VehicleState::test)
            {
            pyroArray.at(1)->setState(PyroState::OnCommanded);             // Renegade SF Igniter1
            }
            else if (currentState == VehicleState::offNominal)
            {
            pyroArray.at(1)->setState(PyroState::OnCommanded);             // Renegade SF Igniter1
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
        valveArray.at(LoxBang_ArrayPointer)->setState(tankPressControllerArray.at(LoxTankController_ArrayPointer)->getPrimaryPressValveState());
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

    sei(); // reenables interrupts after controller sync
 
}

void controllerDataSync(const std::array<Valve*, NUM_VALVES>& valveArray, const std::array<Pyro*, NUM_PYROS>& pyroArray, const std::array<AutoSequence*, NUM_AUTOSEQUENCES>& autoSequenceArray, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const std::array<TankPressController*, NUM_TANKPRESSCONTROLLERS>& tankPressControllerArray, const std::array<EngineController*, NUM_ENGINECONTROLLERS>& engineControllerArray)
{
    //cli(); // disables interrupts during controller sync to protect from partial propulsion system states
    //Lox Tank sensor target value update
    sensorArray.at(LoxTank1PT_ArrayPointer)->setTargetValue(tankPressControllerArray.at(LoxTankController_ArrayPointer)->getTargetValue());
    //Fuel Tank sensor target value update
    sensorArray.at(FuelTank1PT_ArrayPointer)->setTargetValue(tankPressControllerArray.at(FuelTankController_ArrayPointer)->getTargetValue());
    //Lox Tank Controller Sensor Data fetch
    tankPressControllerArray.at(LoxTankController_ArrayPointer)->setPIDSensorInputs(sensorArray.at(LoxTank1PT_ArrayPointer)->getEMAConvertedValue(), sensorArray.at(LoxTank1PT_ArrayPointer)->getIntegralSum(), sensorArray.at(LoxTank1PT_ArrayPointer)->getLinRegSlope());
    //Lox Tank Controller Sensor Data fetch
    tankPressControllerArray.at(FuelTankController_ArrayPointer)->setPIDSensorInputs(sensorArray.at(FuelTank1PT_ArrayPointer)->getEMAConvertedValue(), sensorArray.at(FuelTank1PT_ArrayPointer)->getIntegralSum(), sensorArray.at(FuelTank1PT_ArrayPointer)->getLinRegSlope());
    //sei(); // reenables interrupts after controller sync
}