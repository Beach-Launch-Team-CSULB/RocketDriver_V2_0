#include "ValveClass.h"
#include <Arduino.h>
#include "extendedIO/extendedIO.h"

Valve::Valve(uint32_t setValveID, uint8_t setValveNodeID, ValveType setValveType_Default, uint8_t setALARA_HP_Channel, uint32_t setFullDutyTime_Default, bool setAbortHaltDeviceBool, uint16_t setHoldDuty_Default,  bool setNodeIDCheck)
                : valveID{setValveID}, valveNodeID{setValveNodeID}, valveType_Default{setValveType_Default}, ALARA_HP_Channel{setALARA_HP_Channel}, fullDutyTime_Default{setFullDutyTime_Default}, abortHaltDeviceBool{setAbortHaltDeviceBool}, holdDuty_Default{setHoldDuty_Default}, nodeIDCheck{setNodeIDCheck}
{
    //set values to default values when intstantiated
    valveType = valveType_Default;
    fullDutyTime = fullDutyTime_Default;
    fullDuty = fullDuty_Default;
    holdDuty = holdDuty_Default;
    warmDuty = warmDuty_Default;

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

Valve::Valve(ValveType setValveType_Default, bool setNodeIDCheck) : valveType_Default{setValveType_Default}, nodeIDCheck{setNodeIDCheck}
{
    //probably don't need anything here for the standin valve objects for unused controller valves
    
    //set values to default values when intstantiated
    valveType = valveType_Default;
    fullDutyTime = fullDutyTime_Default;
    fullDuty = fullDuty_Default;
    holdDuty = holdDuty_Default;
    warmDuty = warmDuty_Default;

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
}

void Valve::begin(uint8_t pinArrayIn[][11])
{
    if (nodeIDCheck)
    {       
        pinDigital = pinArrayIn[0][ALARA_HP_Channel];
        pinPWM = pinArrayIn[1][ALARA_HP_Channel];
        pinADC = pinArrayIn[2][ALARA_HP_Channel];

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
            analogWrite(pinPWM, fullDuty);
            digitalWriteExtended(pinDigital, HIGH);
            //timer = 0;
            //state = ValveState::Open;
            //controllerUpdate = true;
        //}
        break;
    case ValveState::BangOpenProcess:
        //if(timer >= fullDutyTime)
        //{
            analogWrite(pinPWM, fullDuty);
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
            analogWrite(pinPWM, fullDuty);
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
        analogWrite(pinPWM, HIGH);
        digitalWriteExtended(pinDigital, HIGH);
        break;
    case ValveState::BangingClosed:
        analogWrite(pinPWM, LOW);
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
        //if (priorState == ValveState::Closed)
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
        //if (priorState == ValveState::Open)
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
