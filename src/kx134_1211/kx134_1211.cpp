/*
THIS DRIVER IS NOT IN A FUNCTIONAL STATE, HAD ISSUES COMMUNICATING OVER SPI

KX134-1211 64g 3-axis Accelerometer driver

Written by Brandon Summers
Last updated June 13, 2022

Arduino platform friendly - Made specifically to be used with Teensy 3.6
*/

#include "kx134_1211.h"


void kx134_1211::init_asyncronous()
{
    pinMode(KX134_CS_PIN,OUTPUT);
    digitalWrite(KX134_CS_PIN,HIGH);

    set_mode_standby();

    // Write 0x00 to Control 1 (CNTL1) to set the accelerometer in stand-by mode
    SPI.beginTransaction(SPISettings(100000, MSBFIRST, KX134_SPI_MODE));
    digitalWrite(KX134_CS_PIN,0);
    SPI.transfer(0xFF); // SPI dummy read
    SPI.transfer(KX134_1211_ODCNTL);
    SPI.transfer(0x06);
    digitalWrite(KX134_CS_PIN,1);
    SPI.endTransaction();
    
    set_mode_operating();
}


// Write 0x00 to Control 1 (CNTL1) to set the accelerometer in stand-by mode
void kx134_1211::set_mode_standby()
{
    SPI.beginTransaction(SPISettings(100000, MSBFIRST, KX134_SPI_MODE));
    digitalWrite(KX134_CS_PIN,0);
    SPI.transfer(0xFF); // SPI dummy read
    SPI.transfer(KX134_1211_CNTL1);
    SPI.transfer(0x00);
    digitalWrite(KX134_CS_PIN,1);
    SPI.endTransaction();
}


// Write 0xC0 to Control 1 (CNTL1) to set the accelerometer into operating mode (PC1=1)
// full power mode (RES=1), data ready disabled (DRDYE=0), range to ±8g (GSEL=0).
void kx134_1211::set_mode_operating()
{
    SPI.beginTransaction(SPISettings(100000, MSBFIRST, KX134_SPI_MODE));
    digitalWrite(KX134_CS_PIN,0);
    SPI.transfer(0xFF); // SPI dummy read
    SPI.transfer(KX134_1211_CNTL1);
    SPI.transfer(0xC0);
    digitalWrite(KX134_CS_PIN,1);
    SPI.endTransaction();
}