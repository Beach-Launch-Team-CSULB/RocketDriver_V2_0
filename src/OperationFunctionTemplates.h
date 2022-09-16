#ifndef OPERATIONFUNCTIONTEMPLATES_H
#define OPERATIONFUNCTIONTEMPLATES_H

#include <array>
//#include <bitset>
//#include <FlexCAN.h>
//#include <ADC.h>
#include "ToMillisTimeTracker.h"

// This contains some of the functions to be used during operations they are templates, and so defined in the header. BEWARE

//  CALL THIS FUNCTION EVERY LOOP 
    // This function takes the array of pointers that point to the valve objects, and then calls the .stateOperations() method for each valve
    // Make sure valveArray is an array of pointers, as defined in ValveDefinitions.h
template <typename T, std::size_t size>
void valveTasks(const std::array<T, size>& valveArray, uint8_t& nodeIDReadIn, bool& outputOverride, AutoSequence& mainAutoSequence)
{
    if (!outputOverride)    //bool will block all stateOps
    {
        // iterate through valve array and run the stateOperations method
        for(auto valve : valveArray)
        {
            if (valve->getValveNodeID() == nodeIDReadIn)
            {
                valve->setCurrentAutoSequenceTime(mainAutoSequence.getCurrentCountdown());
                valve->controllerStateOperations();
                valve->stateOperations();
            }
        }
    }
}

template <typename T, std::size_t size>
void pyroTasks(const std::array<T, size>& pyroArray, uint8_t& nodeIDReadIn, bool& outputOverride, AutoSequence& mainAutoSequence)
{
    if (!outputOverride)    //bool will block all stateOps
    {
        // iterate through valve array and run the stateOperations method
        for(auto pyro : pyroArray)
        {
        if (pyro->getPyroNodeID() == nodeIDReadIn)
            {
                pyro->setCurrentAutoSequenceTime(mainAutoSequence.getCurrentCountdown());
                pyro->controllerStateOperations();
                pyro->stateOperations();
            }
        }
    }
}

template <typename T, std::size_t size>
void tankPressControllerTasks(const std::array<T, size>& tankPressControllerArray, uint8_t& nodeIDReadIn, AutoSequence& ignitionAutoSequenceRef)
{
    // iterate through valve array and run the stateOperations method
    for(auto tankPressController : tankPressControllerArray)
    {
    
        if (tankPressController->getControllerNodeID() == nodeIDReadIn)
        {
            tankPressController->setCurrentAutosequenceTime(ignitionAutoSequenceRef.getCurrentCountdown());
            tankPressController->stateOperations();
        }
    }
}

template <typename T, std::size_t size>
void engineControllerTasks(const std::array<T, size>& engineControllerArray, uint8_t& nodeIDReadIn, AutoSequence& ignitionAutoSequenceRef)
{
    // iterate through valve array and run the stateOperations method
    for(auto engineController : engineControllerArray)
    {
    
        if (engineController->getControllerNodeID() == nodeIDReadIn)
        {
            engineController->setCurrentAutosequenceTime(ignitionAutoSequenceRef.getCurrentCountdown());
            engineController->stateOperations();
        }
    }
}

template <typename T, std::size_t size>
void autoSequenceTasks(const std::array<T, size>& autoSequenceArray, uint8_t& nodeIDReadIn)
{
    // iterate through valve array and run the stateOperations method
    for(auto autoSequence : autoSequenceArray)
    {
        
    if (autoSequence->getHostNodeID() == nodeIDReadIn)
        {
            autoSequence->stateOperations();
        }
    }
}

template <typename T, std::size_t size>
//void sensorTasks(const std::array<T, size>& sensorArray, ADC*adc, uint32_t& secondsRD,uint32_t& microsecondsRD, uint8_t& nodeIDReadIn)
void sensorTasks(const std::array<T, size>& sensorArray, ADC& adc, uint8_t& nodeIDReadIn, uint32_t& rocketDriverSeconds, uint32_t&  rocketDriverMicros)
{
    // iterate through valve array and run the stateOperations method
    for(auto sensor : sensorArray)
    {
    
        if (sensor->getSensorNodeID() == nodeIDReadIn)
        {
            sensor->stateOperations();
            //Serial.print("LoopRan");
            sensor->read(adc);
            myTimeTrackingFunction(rocketDriverSeconds, rocketDriverMicros);
            sensor->setSYSTimestamp(rocketDriverSeconds, rocketDriverMicros);
            //sensor->linearConversion();
        }
/*         else if (nodeIDReadIn == 6) //shitty way to make logger node only convert
        {
            sensor->linearConversion();
        } */
    }
}

template <typename T, std::size_t size>
//void sensorTasks(const std::array<T, size>& sensorArray, ADC*adc, uint32_t& secondsRD,uint32_t& microsecondsRD, uint8_t& nodeIDReadIn)
void ALARAHPsensorTasks(const std::array<T, size>& sensorArray, ADC& adc, uint8_t& nodeIDReadIn, uint32_t& rocketDriverSeconds, uint32_t&  rocketDriverMicros, bool outputOverrideIn)
{
    // iterate through valve array and run the stateOperations method
    for(auto sensor : sensorArray)
    {
    
        if (sensor->getSensorNodeID() == nodeIDReadIn)
        {
            sensor->stateOperations();
            //Serial.println("LoopRan for HP sensor tasks: ");
            sensor->read(adc);
            sensor->setDeenergizeOffset(adc, outputOverrideIn);
            myTimeTrackingFunction(rocketDriverSeconds, rocketDriverMicros);
            sensor->setSYSTimestamp(rocketDriverSeconds, rocketDriverMicros);
            //sensor->linearConversion();
        }
/*         else if (nodeIDReadIn == 6) //shitty way to make logger node only convert
        {
            sensor->linearConversion();
        } */
    }
}

template <typename T, std::size_t size>
//void sensorTasks(const std::array<T, size>& sensorArray, ADC*adc, uint32_t& secondsRD,uint32_t& microsecondsRD, uint8_t& nodeIDReadIn)
void TCsensorTasks(const std::array<T, size>& TCsensorArray, ADC& adc, uint8_t& nodeIDReadIn, uint32_t& rocketDriverSeconds, uint32_t&  rocketDriverMicros)
{
    // iterate through valve array and run the stateOperations method
    for(auto sensor : TCsensorArray)
    {
        if (sensor->getSensorNodeID() == nodeIDReadIn)
        {
            sensor->stateOperations();
            //Serial.println("LoopRan for HP sensor tasks: ");
            sensor->read(adc);
            myTimeTrackingFunction(rocketDriverSeconds, rocketDriverMicros);
            sensor->setSYSTimestamp(rocketDriverSeconds, rocketDriverMicros);
            //sensor->linearConversion();
        }
    }
}

// CALL THIS FUNCTION ONCE IN SETUP, THIS SETS THE VALVE PINMODES
    // make sure to pass this function valveArray, as defined in ValveDefinitions.h
template <typename T, std::size_t size>
void valveSetUp(const std::array<T, size>& valveArray, uint8_t pinArrayIn[][11])
{
    // iterate through valve array and run the stateOperations method
    for(auto valve : valveArray)
    {
        valve->begin(pinArrayIn);
    }
}

template <typename T, std::size_t size>
void pyroSetUp(const std::array<T, size>& pyroArray, uint8_t pinArrayIn[][11])
{
    // iterate through valve array and run the stateOperations method
    for(auto pyro : pyroArray)
    {
        pyro->begin(pinArrayIn);
    }
}

template <typename T, std::size_t size>
void autoSequenceSetUp(const std::array<T, size>& autoSequenceArray)
{
    // iterate through valve array and run the stateOperations method
    for(auto autoSequence : autoSequenceArray)
    {
        autoSequence->begin();
    }
}

template <typename T, std::size_t size>
void engineControllerSetup(const std::array<T, size>& engineControllerArray)
{
    // iterate through engine controller array and run begin
    for(auto engineController : engineControllerArray)
    {
        engineController->begin();
    }
}

template <typename T, std::size_t size>
void tankPressControllerSetup(const std::array<T, size>& tankPressControllerArray)
{
    // iterate through tank controller array and run begin
    for(auto tankPressController : tankPressControllerArray)
    {
        tankPressController->begin();
    }
}

template <typename T, std::size_t size>
void sensorSetUp(const std::array<T, size>& sensorArray)
//void sensorSetUp(const std::array<T, size>& sensorArray, uint32_t& rocketDriverSeconds, uint32_t& rocketDriverMicros, void (*myTimeTrackingFunction)(uint32_t, uint32_t))
{
    // iterate through sensor array and run begin
    for(auto sensor : sensorArray)
    {
        sensor->begin();
        //sensor->setSYSTimestamp(secondsRD, microsecondsRD);
/*         myTimeTrackingFunction(rocketDriverSeconds, rocketDriverMicros);
        Serial.println("rocketDriverSeconds");
        Serial.println(rocketDriverSeconds);
        Serial.println("rocketDriverMicros");
        Serial.println(rocketDriverMicros); */
        //Serial.print("LoopRan");
    }
}

void fakesensorShit(uint32_t& rocketDriverSeconds, uint32_t& rocketDriverMicros, void (*myTimeTrackingFunction)(uint32_t, uint32_t))
{
    // iterate through sensor array and run begin
    //for(auto sensor : sensorArray)
    //{
        //sensor->begin();
        //sensor->setSYSTimestamp(secondsRD, microsecondsRD);
        myTimeTrackingFunction(rocketDriverSeconds, rocketDriverMicros);
        Serial.println("rocketDriverSeconds");
        Serial.println(rocketDriverSeconds);
        Serial.println("rocketDriverMicros");
        Serial.println(rocketDriverMicros);
        //Serial.print("LoopRan");
    //}
}

template <typename T, std::size_t size>
void ValveNodeIDCheck(const std::array<T, size>& valveArray, uint8_t nodeIDfromMain)
{
    // iterate through valve array and run the stateOperations method
    for (auto valve : valveArray)
    {
        if (valve->getValveNodeID() == nodeIDfromMain)
        {
            valve->setNodeIDCheck(true);
        }
    }
}

template <typename T, std::size_t size>
void PyroNodeIDCheck(const std::array<T, size>& pyroArray, uint8_t nodeIDfromMain)
{
    // iterate through pyro array and run the stateOperations method
    for (auto pyro : pyroArray)
    {
        if (pyro->getPyroNodeID() == nodeIDfromMain)
        {
            pyro->setNodeIDCheck(true);
        }
    }
}

template <typename T, std::size_t size>
void SensorNodeIDCheck(const std::array<T, size>& sensorArray, uint8_t nodeIDfromMain)
{
    // iterate through sensor array and run the stateOperations method
    for (auto sensor : sensorArray)
    {
        if (sensor->getSensorNodeID() == nodeIDfromMain)
        {
            sensor->setNodeIDCheck(true);
        }
        else if (nodeIDfromMain == 6)    //Logger nodeID so it generates array for all sensors
        {
            sensor->setNodeIDCheck(true);
        }
    }
}



#endif