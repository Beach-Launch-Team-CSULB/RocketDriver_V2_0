#ifndef ALARAUTILITYFUNCTIONS_H
#define ALARAUTILITYFUNCTIONS_H
#pragma once

#include <Arduino.h>
#include <EEPROM.h>
#include <cstring>

void tripleEEPROMwrite(uint8_t byteToWrite, uint32_t byteAddress1, uint32_t byteAddress2, uint32_t byteAddress3);

uint8_t tripleEEPROMread(uint32_t byteAddress1, uint32_t byteAddress2, uint32_t byteAddress3, uint32_t &errorFlag);

bool readOnlyMUX(uint8_t pinToReadMUX, uint8_t pinMUX_S0, uint8_t pinMUX_S1, uint8_t pinMUX_S2, uint8_t pinMUX_A); // SN74CB3Q3251 MUX pin read operation for use only when MUX has OE tied to GND

void SPI_CS_MUX(uint8_t pinToWriteMUX, uint8_t pinMUX_S0, uint8_t pinMUX_S1, uint8_t pinMUX_S2); // SN74CB3Q3251 MUX pin output operation for use as multiplexed CS pins

void MUXSetup(bool MUX_Input, uint8_t pinMUX_S0, uint8_t pinMUX_S1, uint8_t pinMUX_S2, uint8_t pinMUX_A = 0);       // SN74CB3Q3251 MUX pin output setup function, set MUX_Input = false and leave pinMUX_A empty for output MUX

// -------------------------------------------------------------
//Function to run the nodeID hardware addressing
uint8_t NodeIDDetect(uint8_t pinToReadMUXNodeID, uint8_t pinMUX_S0NodeID, uint8_t pinMUX_S1NodeID, uint8_t pinMUX_S2NodeID, uint8_t pinMUX_ANodeID,
                    uint16_t nodeIDDetermineAddress1, uint16_t nodeIDDetermineAddress2, uint16_t nodeIDDetermineAddress3, uint16_t NodeIDAddress1, uint16_t NodeIDAddress2, uint16_t NodeIDAddress3, uint8_t nodeID, bool nodeIDdetermine, uint8_t nodeIDfromEEPROM1, uint8_t nodeIDfromEEPROM2, uint8_t nodeIDfromEEPROM3);

// ----- Teensy Internal Reset -----
// Teensy 3.5/3.6 MCU restart register definitions
#define RESTART_ADDR       0xE000ED0C
#define READ_RESTART()     (*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))

void TeensyInternalReset (bool& localNodeResetFlagIn, uint8_t addressIn1, uint8_t addressIn2, uint8_t addressIn3);

void analogWriteSoft(uint8_t outputPin, bool outputState, uint32_t freqIN, uint16_t dutyCycleIn);

float float_from32bits(uint32_t f);

// utility function for running a rolling array
// float array version - !!!!! Not Protected from if you put negative signed floats for the index values !!!!!
void writeToRollingArray(float rollingArray[], float newInputArrayValue, uint32_t numberSizeIndex = 1);

void writeToRollingArray(uint32_t rollingArray[], uint32_t newInputArrayValue, uint32_t numberSizeIndex = 1);

void writeToRollingArray(uint16_t rollingArray[], uint16_t newInputArrayValue, uint8_t numberSizeIndex = 1);

void writeToRollingArray(uint8_t rollingArray[], uint8_t newInputArrayValue, uint8_t numberSizeIndex = 1);


#endif