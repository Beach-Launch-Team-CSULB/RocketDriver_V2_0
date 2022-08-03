#include "ValveClass.h"
#include <Arduino.h>
#include "extendedIO/extendedIO.h"

Valve::Valve(uint32_t setValveID, uint8_t setValveNodeID, ValveType setValveType, uint8_t setPinPWM, uint8_t setPinDigital, uint32_t setFullDutyTime, bool setAbortHaltDeviceBool, ValveState setAbortedState, uint8_t setHoldDuty,  bool setNodeIDCheck)
                : valveID{setValveID}, valveNodeID{setValveNodeID}, valveType{setValveType}, pinPWM{setPinPWM}, pinDigital{setPinDigital}, fullDutyTime{setFullDutyTime}, abortHaltDeviceBool{setAbortHaltDeviceBool}, abortedState{setAbortedState}, holdDuty{setHoldDuty}, nodeIDCheck{setNodeIDCheck}
{
    switch (valveType)
    {
        case NormalClosed:
            state = ValveState::Closed;
            priorState = ValveState::Closed;
            break;
        case NormalOpen:
            state = ValveState::Open;
            priorState = ValveState::Open;
            break;
        default:
            state = ValveState::Closed;
            priorState = ValveState::Closed;
            break;
    }
    timer = 0;
    
}

Valve::Valve(ValveType setValveType, ValveState setAbortedState, bool setNodeIDCheck) : valveType{setValveType}, abortedState{setAbortedState}, nodeIDCheck{setNodeIDCheck}
{
    
}

void Valve::begin()
{
    if (nodeIDCheck)
    {       
        pinModeExtended(pinPWM, OUTPUT);
        pinModeExtended(pinDigital, OUTPUT);
        analogWrite(pinPWM, 0);
        digitalWriteExtended(pinDigital, LOW);
    }
}

void Valve::resetTimer()
{
    timer = 0;
}

void Valve::resetAll()
{
    //
}

ValveState Valve::getSyncState()
{
    if(controllerUpdate)
    {
        controllerUpdate = false;
        return state;
    }
    else {return ValveState::NullReturn;}
}


void Valve::stateOperations()
{
    switch (state)
    {
    // if a valve is commanded open, if its normal closed it needs to fully actuate, if normal open it needs to drop power to zero
/*     case ValveState::OpenCommanded:
        if (!(priorState == ValveState::Open))
        {
            switch (valveType)
            {
                case NormalClosed:
                    analogWrite(pinPWM, fullDuty);
                    digitalWriteFast(pinDigital, HIGH);
                    timer = 0;
                    //state = ValveState::OpenProcess;
                    controllerUpdate = true;
                    //Serial.print("NC OpenCommanded: ");
                    //Serial.println(valveID);
                    break;
                case NormalOpen:
                    analogWrite(pinPWM, 0);
                    digitalWriteFast(pinDigital, LOW);
                    timer = 0;
                    //state = ValveState::Open;
                    controllerUpdate = true;
                    //Serial.print("NO OpenCommanded: ");
                    //Serial.println(valveID);                
                    break;
                default:
                    break;
            }
        }
        else
        {
            state = ValveState::Open;
        }
        break;
 */
/*     case ValveState::BangOpenCommanded:
        if (priorState != ValveState::Open)
        {
            switch (valveType)
            {
                case NormalClosed:
                    analogWrite(pinPWM, fullDuty);
                    digitalWriteFast(pinDigital, HIGH);
                    timer = 0;
                    state = ValveState::BangOpenProcess;
                    controllerUpdate = true;
                    //Serial.print("NC OpenCommanded: ");
                    //Serial.println(valveID);
                    break;
                case NormalOpen:
                    analogWrite(pinPWM, 0);
                    digitalWriteFast(pinDigital, LOW);
                    timer = 0;
                    state = ValveState::Open;
                    controllerUpdate = true;
                    //Serial.print("NO OpenCommanded: ");
                    //Serial.println(valveID);                
                    break;
                default:
                    break;
            }
        }
        else
        {
            state = ValveState::Open;
            //controllerUpdate = true; //do I need it here?
        }
        break;
 */
    // if a valve is commanded closed, a normal closed removes power, normal open starts activation sequence
/*     case ValveState::CloseCommanded:
        if (priorState != ValveState::Closed)
        {
            switch (valveType)
            {
                case NormalClosed:
                    analogWrite(pinPWM, 0);
                    digitalWriteFast(pinDigital, LOW);
                    timer = 0;
                    state = ValveState::Closed;
                    controllerUpdate = true;
                    break;
                case NormalOpen:
                    analogWrite(pinPWM, fullDuty);
                    digitalWriteFast(pinDigital, HIGH);
                    timer = 0;
                    state = ValveState::CloseProcess;
                    controllerUpdate = true;
                    break;
                default:
                    break;
            }
        }
        else
        {
            state = ValveState::Closed;
        }
        break;
 */        
/*     case ValveState::BangCloseCommanded:
        if (priorState != ValveState::Closed)
        {
            switch (valveType)
            {
                case NormalClosed:
                    analogWrite(pinPWM, 0);
                    digitalWriteFast(pinDigital, LOW);
                    timer = 0;
                    state = ValveState::BangingClosed;
                    controllerUpdate = true;
                    break;
                case NormalOpen:    //I think this is bogus??
                    analogWrite(pinPWM, fullDuty);
                    digitalWriteFast(pinDigital, HIGH);
                    timer = 0;
                    state = ValveState::CloseProcess;
                    controllerUpdate = true;
                    break;
                default:
                    break;
            }
        }
        else
        {
            state = ValveState::Closed;
            //controllerUpdate = true; //do I need it here
        }
        break;
 */
    // if a valve is in OpenProcess, check if the fullDutyTime has passed. If it has, cycle down to hold duty
    case ValveState::OpenProcess:
        //if(timer >= fullDutyTime)
        //{
            analogWrite(pinPWM, holdDuty);
            digitalWriteExtended(pinDigital, HIGH);
            //timer = 0;
            //state = ValveState::Open;
            //controllerUpdate = true;
        //}
        break;
    case ValveState::BangOpenProcess:
        //if(timer >= fullDutyTime)
        //{
            analogWrite(pinPWM, holdDuty);
            digitalWriteExtended(pinDigital, HIGH);
            //timer = 0;
            //state = ValveState::BangingOpen;
            //controllerUpdate = true;
        //}
        break;

    // if a valve is in CloseProcess, check if the fullDutyTime has passed. If it has, cycle down to hold duty
    case ValveState::CloseProcess:
        //if(timer >= fullDutyTime)
        //{
            analogWrite(pinPWM, holdDuty);
            digitalWriteExtended(pinDigital, HIGH);
            //timer = 0;
            //state = ValveState::Closed;
            //controllerUpdate = true;
        //}
        break;
    case ValveState::Closed:
        switch (valveType)
        {
            case NormalClosed:
                analogWrite(pinPWM, 0);
                digitalWriteExtended(pinDigital, LOW);
                break;
            case NormalOpen:
                analogWrite(pinPWM, holdDuty);
                digitalWriteExtended(pinDigital, HIGH);
            default:
                break;
        }
        break;
    case ValveState::BangingOpen:
        digitalWriteExtended(pinPWM, HIGH);
        digitalWriteExtended(pinDigital, HIGH);
        break;
    case ValveState::BangingClosed:
        digitalWriteExtended(pinPWM, LOW);
        digitalWriteExtended(pinDigital, LOW);
        break;
    
    case ValveState::FireCommanded:
        //not sure what to do here to fix issues


/*         switch (valveType)
        {
            case NormalClosed:
                analogWrite(pinPWM, 0);
                digitalWriteFast(pinDigital, LOW);
                break;
            case NormalOpen:
                analogWrite(pinPWM, holdDuty);
                digitalWriteFast(pinDigital, HIGH);
            default:
                break; */
        break;
    // All other states require no action
    default:
        break;
    }
}

//banging the old version while I brick this
/* void Valve::stateOperations()
{
    switch (state)
    {
    // if a valve is commanded open, if its normal closed it needs to fully actuate, if normal open it needs to drop power to zero
    case ValveState::OpenCommanded:
        if (!(priorState == ValveState::Open))
        {
            switch (valveType)
            {
                case NormalClosed:
                    analogWrite(pinPWM, fullDuty);
                    digitalWriteFast(pinDigital, HIGH);
                    timer = 0;
                    state = ValveState::OpenProcess;
                    controllerUpdate = true;
                    //Serial.print("NC OpenCommanded: ");
                    //Serial.println(valveID);
                    break;
                case NormalOpen:
                    analogWrite(pinPWM, 0);
                    digitalWriteFast(pinDigital, LOW);
                    timer = 0;
                    state = ValveState::Open;
                    controllerUpdate = true;
                    //Serial.print("NO OpenCommanded: ");
                    //Serial.println(valveID);                
                    break;
                default:
                    break;
            }
        }
        else
        {
            state = ValveState::Open;
        }
        break;
    case ValveState::BangOpenCommanded:
        if (priorState != ValveState::Open)
        {
            switch (valveType)
            {
                case NormalClosed:
                    analogWrite(pinPWM, fullDuty);
                    digitalWriteFast(pinDigital, HIGH);
                    timer = 0;
                    state = ValveState::BangOpenProcess;
                    controllerUpdate = true;
                    //Serial.print("NC OpenCommanded: ");
                    //Serial.println(valveID);
                    break;
                case NormalOpen:
                    analogWrite(pinPWM, 0);
                    digitalWriteFast(pinDigital, LOW);
                    timer = 0;
                    state = ValveState::Open;
                    controllerUpdate = true;
                    //Serial.print("NO OpenCommanded: ");
                    //Serial.println(valveID);                
                    break;
                default:
                    break;
            }
        }
        else
        {
            state = ValveState::Open;
            //controllerUpdate = true; //do I need it here?
        }
        break;

    // if a valve is commanded closed, a normal closed removes power, normal open starts activation sequence
    case ValveState::CloseCommanded:
        if (priorState != ValveState::Closed)
        {
            switch (valveType)
            {
                case NormalClosed:
                    analogWrite(pinPWM, 0);
                    digitalWriteFast(pinDigital, LOW);
                    timer = 0;
                    state = ValveState::Closed;
                    controllerUpdate = true;
                    break;
                case NormalOpen:
                    analogWrite(pinPWM, fullDuty);
                    digitalWriteFast(pinDigital, HIGH);
                    timer = 0;
                    state = ValveState::CloseProcess;
                    controllerUpdate = true;
                    break;
                default:
                    break;
            }
        }
        else
        {
            state = ValveState::Closed;
        }
        break;
    case ValveState::BangCloseCommanded:
        if (priorState != ValveState::Closed)
        {
            switch (valveType)
            {
                case NormalClosed:
                    analogWrite(pinPWM, 0);
                    digitalWriteFast(pinDigital, LOW);
                    timer = 0;
                    state = ValveState::BangingClosed;
                    controllerUpdate = true;
                    break;
                case NormalOpen:    //I think this is bogus??
                    analogWrite(pinPWM, fullDuty);
                    digitalWriteFast(pinDigital, HIGH);
                    timer = 0;
                    state = ValveState::CloseProcess;
                    controllerUpdate = true;
                    break;
                default:
                    break;
            }
        }
        else
        {
            state = ValveState::Closed;
            //controllerUpdate = true; //do I need it here
        }
        break;

    // if a valve is in OpenProcess, check if the fullDutyTime has passed. If it has, cycle down to hold duty
    case ValveState::OpenProcess:
        if(timer >= fullDutyTime)
        {
            analogWrite(pinPWM, holdDuty);
            digitalWriteFast(pinDigital, HIGH);
            timer = 0;
            state = ValveState::Open;
            controllerUpdate = true;
        }
        break;
    case ValveState::BangOpenProcess:
        if(timer >= fullDutyTime)
        {
            analogWrite(pinPWM, holdDuty);
            digitalWriteFast(pinDigital, HIGH);
            timer = 0;
            state = ValveState::BangingOpen;
            controllerUpdate = true;
        }
        break;

    // if a valve is in CloseProcess, check if the fullDutyTime has passed. If it has, cycle down to hold duty
    case ValveState::CloseProcess:
        if(timer >= fullDutyTime)
        {
            analogWrite(pinPWM, holdDuty);
            digitalWriteFast(pinDigital, HIGH);
            timer = 0;
            state = ValveState::Closed;
            controllerUpdate = true;
        }
        break;
    case ValveState::Closed:
        switch (valveType)
        {
            case NormalClosed:
                analogWrite(pinPWM, 0);
                digitalWriteFast(pinDigital, LOW);
                break;
            case NormalOpen:
                analogWrite(pinPWM, holdDuty);
                digitalWriteFast(pinDigital, HIGH);
            default:
                break;
        }
        break;
    case ValveState::BangingOpen:
        digitalWriteFast(pinPWM, HIGH);
        digitalWriteFast(pinDigital, HIGH);
        break;
    case ValveState::BangingClosed:
        digitalWriteFast(pinPWM, LOW);
        digitalWriteFast(pinDigital, LOW);
        break;
    
    case ValveState::FireCommanded:
        //not sure what to do here to fix issues



        break;
    // All other states require no action
    default:
        break;
    }
}
 */


void Valve::controllerStateOperations()
{


    switch (state)
    {
    // if a valve is commanded open, if its normal closed it needs to fully actuate, if normal open it needs to drop power to zero
    case ValveState::OpenCommanded:
        if (!(priorState == ValveState::Open))
        {
            switch (valveType)
            {
                case NormalClosed:
                    timer = 0;
                    state = ValveState::OpenProcess;
                    //controllerUpdate = true;
                    //Serial.print("NC OpenCommanded: ");
                    //Serial.println(valveID);
                    break;
                case NormalOpen:
                    timer = 0;
                    state = ValveState::Open;
                    //controllerUpdate = true;
                    //Serial.print("NO OpenCommanded: ");
                    //Serial.println(valveID);                
                    break;
                default:
                    break;
            }
        }
        else
        {
            state = ValveState::Open;
        }
        break;
    case ValveState::BangOpenCommanded:
        if (priorState != ValveState::Open)
        {
            switch (valveType)
            {
                case NormalClosed:
                    timer = 0;
                    state = ValveState::BangOpenProcess;
                    //controllerUpdate = true;
                    //Serial.print("NC OpenCommanded: ");
                    //Serial.println(valveID);
                    break;
                case NormalOpen:
                    timer = 0;
                    state = ValveState::Open;
                    //controllerUpdate = true;
                    //Serial.print("NO OpenCommanded: ");
                    //Serial.println(valveID);                
                    break;
                default:
                    break;
            }
        }
        else
        {
            state = ValveState::Open;
            //controllerUpdate = true; //do I need it here?
        }
        break;

    // if a valve is commanded closed, a normal closed removes power, normal open starts activation sequence
    case ValveState::CloseCommanded:
        if (priorState != ValveState::Closed)
        {
            switch (valveType)
            {
                case NormalClosed:
                    timer = 0;
                    state = ValveState::Closed;
                    //controllerUpdate = true;
                    break;
                case NormalOpen:
                    timer = 0;
                    state = ValveState::CloseProcess;
                    //controllerUpdate = true;
                    break;
                default:
                    break;
            }
        }
        else
        {
            state = ValveState::Closed;
        }
        break;
    case ValveState::BangCloseCommanded:
        if (priorState != ValveState::Closed)
        {
            switch (valveType)
            {
                case NormalClosed:
                    timer = 0;
                    state = ValveState::BangingClosed;
                    //controllerUpdate = true;
                    break;
                case NormalOpen:    //I think this is bogus??
                    timer = 0;
                    state = ValveState::CloseProcess;
                    //controllerUpdate = true;
                    break;
                default:
                    break;
            }
        }
        else
        {
            state = ValveState::Closed;
            //controllerUpdate = true; //do I need it here
        }
        break;

    // if a valve is in OpenProcess, check if the fullDutyTime has passed. If it has, cycle down to hold duty
    case ValveState::OpenProcess:
        if(timer >= fullDutyTime)
        {
            timer = 0;
            state = ValveState::Open;
            //controllerUpdate = true;
        }
        break;
    case ValveState::BangOpenProcess:
        if(timer >= fullDutyTime)
        {
            timer = 0;
            state = ValveState::BangingOpen;
            controllerUpdate = true;
        }
        break;

    // if a valve is in CloseProcess, check if the fullDutyTime has passed. If it has, cycle down to hold duty
    case ValveState::CloseProcess:
        if(timer >= fullDutyTime)
        {
            //analogWrite(pinPWM, holdDuty);
            //digitalWriteFast(pinDigital, HIGH);
            timer = 0;
            state = ValveState::Closed;
            //controllerUpdate = true;
        }
        break;
    case ValveState::Closed:
        switch (valveType)
        {
            case NormalClosed:
                break;
            case NormalOpen:
            default:
                break;
        }
        break;
    case ValveState::BangingOpen:
        break;
    case ValveState::BangingClosed:
        break;
    
    case ValveState::FireCommanded:
        //not sure what to do here to fix issues

        break;
    // All other states require no action
    default:
        break;
    }
}
