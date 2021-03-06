#include "ValveClass.h"
#include <Arduino.h>


Valve::Valve(uint32_t setValveID, uint8_t setValveNodeID, ValveType setValveType, uint8_t setPinPWM, uint8_t setPinDigital, uint32_t setFullDutyTime, bool setAbortHaltDeviceBool = false, ValveState setAbortedState = ValveState::CloseCommanded, uint8_t setHoldDuty,  bool setNodeIDCheck)
                : valveID{setValveID}, valveNodeID{setValveNodeID}, valveType{setValveType}, pinPWM{setPinPWM}, pinDigital{setPinDigital}, fullDutyTime{setFullDutyTime}, abortHaltDeviceBool{setAbortHaltDeviceBool}, abortedState{setAbortedState}, holdDuty{setHoldDuty}, nodeIDCheck{setNodeIDCheck}
{
    switch (valveType)
    {
        case NormalClosed:
            state = ValveState::Closed;
            break;
        case NormalOpen:
            state = ValveState::Open;
            break;
        default:
            state = ValveState::Closed;
            break;
    }
    timer = 0;
    
}

void Valve::begin()
{
    if (nodeIDCheck)
    {
        pinMode(pinPWM, OUTPUT);
        pinMode(pinDigital, OUTPUT);
        analogWrite(pinPWM, 0);
        digitalWriteFast(pinDigital, LOW);
    }
}

void Valve::resetTimer()
{
    timer = 0;
}

void Valve::stateOperations()
{


    switch (state)
    {
    // if a valve is commanded open, if its normal closed it needs to fully actuate, if normal open it needs to drop power to zero
    case ValveState::OpenCommanded:
        if (priorState != ValveState::Open)
        {
            switch (valveType)
            {
                case NormalClosed:
                    analogWrite(pinPWM, fullDuty);
                    digitalWriteFast(pinDigital, HIGH);
                    timer = 0;
                    state = ValveState::OpenProcess;
                    //Serial.print("NC OpenCommanded: ");
                    //Serial.println(valveID);
                    break;
                case NormalOpen:
                    analogWrite(pinPWM, 0);
                    digitalWriteFast(pinDigital, LOW);
                    timer = 0;
                    state = ValveState::Open;
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
                    //Serial.print("NC OpenCommanded: ");
                    //Serial.println(valveID);
                    break;
                case NormalOpen:
                    analogWrite(pinPWM, 0);
                    digitalWriteFast(pinDigital, LOW);
                    timer = 0;
                    state = ValveState::Open;
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
                    break;
                case NormalOpen:
                    analogWrite(pinPWM, fullDuty);
                    digitalWriteFast(pinDigital, HIGH);
                    timer = 0;
                    state = ValveState::CloseProcess;
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
                    break;
                case NormalOpen:
                    analogWrite(pinPWM, fullDuty);
                    digitalWriteFast(pinDigital, HIGH);
                    timer = 0;
                    state = ValveState::CloseProcess;
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

    // if a valve is in OpenProcess, check if the fullDutyTime has passed. If it has, cycle down to hold duty
    case ValveState::OpenProcess:
        if(timer >= fullDutyTime)
        {
            analogWrite(pinPWM, holdDuty);
            digitalWriteFast(pinDigital, HIGH);
            timer = 0;
            state = ValveState::Open;
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
    
    
    
    // All other states require no action
    default:
        break;
    }
}
