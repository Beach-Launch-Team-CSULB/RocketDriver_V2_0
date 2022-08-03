#include <Arduino.h>
#include <FlexCAN.h>
#include <array>
#include <vector>
#include "CANRead.h"
using std::array;
using std::vector;


void vectorBufferInitialize(size_t bufferSize)
{
    configMSGvecBuffer.reserve(bufferSize); // should reserve by element count
} 
   


void writeVectorBuffer(configMSG configStructIn)
{
    //writes to the vector buffer, up to the allocated size
    if (!(configMSGvecBuffer.size() == configMSGvecBuffer.capacity()))  //only proceed if not at capacity
    {
        //add to back of vector
        configMSGvecBuffer.push_back(configStructIn);
    }
    else 
    {
        Serial.println("vector buffer overrun");
    }
}

bool readRemoveVectorBuffer(configMSG& configMSGOut)
{
    //reads and deletes once read vector elements out of the buffer, once per func call
    // returns true if there are elements left in buffer
    if(configMSGvecBuffer.empty())
    {
        return false;
    }
    else 
    {
    Serial.println(" here? ");
    configMSGOut = configMSGvecBuffer.front();
    Serial.println(" there? ");
    configMSGvecBuffer.erase(configMSGvecBuffer.begin());
    Serial.println(" everywhere? ");
        return true;
    }
}

// function to be run every loop to check for a new CAN message

bool CANread(FlexCAN& CANbus, uint8_t configVerificationKey, bool& NewConfigMessage, Command& CurrentCommand, configMSG& currentConfigMSG, uint8_t propNodeIDIn)
{
    // New Message Flag
    bool NewMessage {false};
    NewConfigMessage = false;
    //configMSGvecBuffer.reserve(32);
    configMSG configStruct {};
    // create a buffer to hold command messages (using a static array for speed)
    static std::array<Command, 64> CommandBuffer{};                                         // large enough for a back up of 8 full frames
    static uint32_t CommandBufferIndex {0};                                                 // keeps track of where we are in command buffer, works just like a stack
    static uint32_t CommandBufferPull {0};                                                  // lets us pull from oldest without shuffling the array

    // create a variable to hold the current message
    CAN_message_t msg {};

    // create a variable to hold the current config message
    //configMSG emptyConfigStruct {0,0,0};
    
    // create a variable to hold serial input fake CAN command
    //uint8_t fakeCANmsg;

    // check if new message in the mailbox
    if(CANbus.available())
    {
        if(CANbus.read(msg))                                                                // read occurs inside if statement
        {
            //if(msg.id == 0||1) //id = 0 is the only command frame ID to be used for state control, temporarily added ID = 1 as well because of dumb GUI versions
            if(msg.id == 1)
            {
                NewMessage = true;                                                              // set new message flag to true if message recieved and read

                // add CAN messages to internal buffer
                for(uint8_t index{0}; index < msg.len; ++index)                                 // restrict to length of message
                {
                
                
                    if(msg.buf[index] < command_SIZE)                                           // this checks if the message at that location in the buffer could be a valid command
                    {
                        // convert message to a Command type and save it to the buffer
                        CommandBuffer.at(CommandBufferIndex) = static_cast<Command>(msg.buf[index]);
                        ++CommandBufferIndex;                                                   // increment buffer index
                    }
                
                }
            }
            else if(msg.id == propNodeIDIn && msg.buf[0] == configVerificationKey && msg.len > 3) // must be right node ID, matching verification key, and msg at least 4 bytes long to be valid config message
            //else // must be right ID and msg at least 3 bytes long to be valid config message
            {
                //NewConfigMessage = true;                                                              // set new message flag to true if message recieved and read
                configStruct.TargetObjectID = msg.buf[1];
                configStruct.ObjectSettingID = msg.buf[2];
                if (msg.len == 4)
                {
                    configStruct.uint32Value = msg.buf[3];
                }
                else if (msg.len == 5)
                {
                    configStruct.uint32Value = msg.buf[3] + (msg.buf[4] << 8);
                }
                else if (msg.len == 6)
                {
                    configStruct.uint32Value = msg.buf[3] + (msg.buf[4] << 8) + (msg.buf[5] << 16);
                }
                else
                {
                    configStruct.uint32Value = msg.buf[3] + (msg.buf[4] << 8) + (msg.buf[5] << 16) + (msg.buf[6] << 24);    //might have endianness backwards, test
                }

                Serial.print("Target ID: ");
                Serial.print(configStruct.TargetObjectID);
                Serial.print("Setting ID: ");
                Serial.print(configStruct.ObjectSettingID);
                Serial.print("uint32 config rcv: ");
                Serial.println(configStruct.uint32Value,10);
                //Serial.print("float config rcv: ");
                //Serial.println(configStruct.floatValue,10);
                
                //write to vector buffer
                writeVectorBuffer(configStruct);




                // CAN buffer bytes past this are meaningless in current format
                //configMSGBuffer.at(configMSGBufferIndex) = configStruct;
                //configMSGBufferIndex += sizeof(configStruct); 
            }

        }
    }

    // check if there are commands waiting in the buffer to be executed
    if ((CommandBufferIndex > 0) && (CommandBufferPull != CommandBufferIndex))
    {
        CurrentCommand = CommandBuffer.at(CommandBufferPull);                               // THIS IS WHERE THE WRITE TO CURRENTCOMMAND HAPPENS
        ++CommandBufferPull;      
    }
    else if (CommandBufferPull >= CommandBufferIndex)                                       // check if we've caught up and executed all commands, ie when CommandBufferPull == CommandBufferIndex. CommandBufferPull should NEVER exceed CommandBufferIndex, but for safety I use >=
    {
        CommandBufferPull = 0;
        CommandBufferIndex = 0;
        CurrentCommand = command_NOCOMMAND;                                                 // if we caught up, set command to no command
    }

//currentConfigMSG.TargetObjectID = configStruct.TargetObjectID;
//currentConfigMSG.ObjectSettingID = configStruct.ObjectSettingID;
//currentConfigMSG.uint32Value = configStruct.uint32Value;


    // at this point the mailbox has been checked, any waiting messages have been moved to the buffer, and the oldest command has been moved into the global CurrentCommand variable
    // return the message read bool
    
    // Adding setup to use serial input to put artificial CAN commands in
    // Upgrade this to be toggled on/off from main?
/*     while (Serial.available()) 
    {
        if(Serial.read() == 0) //enter 0 inter serial to trigger command read
        {
            //add in code here to prompt for command code and update current command from this
            Serial.println("Enter Command Byte");
            //CurrentCommand = Serial.read();

                    fakeCANmsg = Serial.read();
                    if(fakeCANmsg < command_SIZE)                                           // this checks if the message at that location in the buffer could be a valid command
                    {
                        // convert message to a Command type and save it to the buffer
                        CommandBuffer.at(CommandBufferIndex) = static_cast<Command>(fakeCANmsg);
                        ++CommandBufferIndex;                                                   // increment buffer index
                    }


            Serial.println("Command Entered");
                
                //return;
        }
    } */
    
    
    
    
    return NewMessage;

}