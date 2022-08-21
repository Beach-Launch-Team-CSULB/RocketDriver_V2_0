#ifndef ALARAUTILITYFUNCTIONS_H
#define ALARAUTILITYFUNCTIONS_H
#pragma once

#include <Arduino.h>
#include <EEPROM.h>
#include <cstring>
//#include "piezoSongs.h"

void ALARAHPOverride(uint8_t pinArrayIn[][11], bool& outputOverride);

void tripleEEPROMwrite(uint8_t byteToWrite, uint32_t byteAddress1, uint32_t byteAddress2, uint32_t byteAddress3);

uint8_t tripleEEPROMread(uint32_t byteAddress1, uint32_t byteAddress2, uint32_t byteAddress3, uint32_t &errorFlag);

bool readOnlyMUX(uint8_t pinToReadMUX, uint8_t pinMUX_S0, uint8_t pinMUX_S1, uint8_t pinMUX_S2, uint8_t pinMUX_A); // SN74CB3Q3251 MUX pin read operation for use only when MUX has OE tied to GND

void SPI_CS_MUX(uint8_t pinToWriteMUX, uint8_t pinMUX_S0, uint8_t pinMUX_S1, uint8_t pinMUX_S2); // SN74CB3Q3251 MUX pin output operation for use as multiplexed CS pins

void MUXSetup(bool MUX_Input, uint8_t pinMUX_S0, uint8_t pinMUX_S1, uint8_t pinMUX_S2, uint8_t pinMUX_A = 0);       // SN74CB3Q3251 MUX pin output setup function, set MUX_Input = false and leave pinMUX_A empty for output MUX

// -------------------------------------------------------------
//Function to run the nodeID hardware addressing
uint8_t NodeIDDetect(uint8_t pinToReadMUXNodeID, uint8_t pinMUX_S0NodeID, uint8_t pinMUX_S1NodeID, uint8_t pinMUX_S2NodeID, uint8_t pinMUX_ANodeID,
                    uint16_t nodeIDDetermineAddress1, uint16_t nodeIDDetermineAddress2, uint16_t nodeIDDetermineAddress3, uint16_t NodeIDAddress1, uint16_t NodeIDAddress2, uint16_t NodeIDAddress3, uint8_t nodeID, bool nodeIDdetermine, uint8_t nodeIDfromEEPROM1, uint8_t nodeIDfromEEPROM2, uint8_t nodeIDfromEEPROM3);



void TeensyInternalReset (bool& localNodeResetFlagIn, uint8_t addressIn1, uint8_t addressIn2, uint8_t addressIn3);

void analogWriteSoft(uint8_t outputPin, bool outputState, uint32_t freqIN, uint16_t dutyCycleIn);

float float_from32bits(uint32_t f);
uint32_t uint32_fromfloat(float u);

// utility function for running a rolling array
// float array version - !!!!! Not Protected from if you put negative signed floats for the index values !!!!!
void writeToRollingArray(float rollingArray[], float newInputArrayValue, uint32_t numberSizeIndex = 1);

void writeToRollingArray(uint32_t rollingArray[], uint32_t newInputArrayValue, uint32_t numberSizeIndex = 1);

void writeToRollingArray(uint16_t rollingArray[], uint16_t newInputArrayValue, uint8_t numberSizeIndex = 1);

void writeToRollingArray(uint8_t rollingArray[], uint8_t newInputArrayValue, uint8_t numberSizeIndex = 1);

/* struct buzzerTone
{
    float toneLoudness = 100;       // Loudness as a % of the buzzer range minimum audible = 0%, max volume = 100%
    float toneFrequency;            // Tone Frequency in Hz
    uint32_t toneDuration;          // Length in micros of tone before rest or next tone
};
 */

/* class ALARAbuzzer
{
    private:
        const uint8_t buzzerPin;
        const float maxToneFrequency;
        const float minToneFrequency;
        const uint8_t minPWM;
        const uint8_t maxPWM;
        elapsedMicros songTimer;
        uint32_t currentNoteTime;   // Time of start of note, in micros since song start
        bool songStart = true;
        bool songEnd = false;
        bool songRepeat = false; //default to not looping songs
        bool notePlaying = false;
        bool songPlaying = false;   //To block a second song from mixing with one already playing
        bool otherSongPlaying = false;   //To block a second song from mixing with one already playing
        std::vector<buzzerTone>::iterator songNoteIt;

    public:
        // Constructor
        ALARAbuzzer (uint8_t setBuzzerPin, float setMaxToneFrequency = 20000, float setMinToneFrequency = 25, uint8_t setMinPWM = 0, uint8_t setMaxPWM = 50);
        // Play tone function
        void playTone(buzzerTone toneInput);
        void playSongVector(std::vector<buzzerTone> songIn, uint16_t BPMset = 120);

};
 */
void toneBuzzer();

#endif