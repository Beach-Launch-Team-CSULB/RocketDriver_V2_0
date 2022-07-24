/*
THIS DRIVER IS NOT IN A FUNCTIONAL STATE, HAD ISSUES COMMUNICATING OVER SPI

KX134-1211 64g 3-axis Accelerometer driver

Written by Brandon Summers
Last updated June 13, 2022

Arduino platform friendly - Made specifically to be used with Teensy 3.6
*/


#pragma once
#include <SPI.h>
#include <Arduino.h>
#include "kx134_1211_registers.h"
#include "extendedIO/extendedIO.h"

#define KX134_CS_PIN ED9
#define KX134_SPI_MODE SPI_MODE0

class kx134_1211
{
private:

public:
    void init_asyncronous();
    void set_mode_standby();
    void set_mode_operating();
};