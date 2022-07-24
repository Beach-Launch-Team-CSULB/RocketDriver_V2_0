/*
MS5607 Barometric Pressure Sensor driver

Written by Brandon Summers
Last updated May 31, 2022
Based on MS5607 datasheet information

Arduino platform friendly - Made specifically to be used with Teensy 3.6
*/

#pragma once
#include <SPI.h>
#include <Arduino.h>

#define BPS_CS_PIN 10
#define BPS_SPI_RATE 20000000 // Hz
#define BPS_FIR_SIZE 101
#define BPS_BUFFER_SIZE 200

enum D1_OSR : const uint8_t
{
    CONV_D1_256 = 0x40,   // Pressure Convert OSR 256 Command Byte  
    CONV_D1_512 = 0x42,   // Pressure Convert OSR 512 Command Byte  
    CONV_D1_1024 = 0x44,  // Pressure Convert OSR 1024 Command Byte  
    CONV_D1_2048 = 0x46,  // Pressure Convert OSR 2048 Command Byte  
    CONV_D1_4096 = 0x48   // Pressure Convert OSR 4096 Command Byte  
};

enum D2_OSR : const uint8_t
{
    CONV_D2_256 = 0x50,   // Temperature Convert OSR 256 Command Byte
    CONV_D2_512 = 0x52,   // Temperature Convert OSR 512 Command Byte
    CONV_D2_1024 = 0x54,  // Temperature Convert OSR 1024 Command Byte
    CONV_D2_2048 = 0x56,  // Temperature Convert OSR 2048 Command Byte
    CONV_D2_4096 = 0x58   // Temperature Convert OSR 4096 Command Byte
};

enum PROM_COEF : const uint8_t
{
    PROM_READ_C1 = 0xA2,
    PROM_READ_C2 = 0xA4,
    PROM_READ_C6 = 0xAC,
    PROM_READ_C3 = 0xA6,
    PROM_READ_C4 = 0xA8,
    PROM_READ_C5 = 0xAA
};

enum CONV_OSR : const uint16_t
{
    OSR_256 = 256,
    OSR_512 = 512,
    OSR_1024 = 1024,
    OSR_2048 = 2048,
    OSR_4096 = 4096
};


class MS5607
{
private:
    uint8_t CS_PIN = BPS_CS_PIN;

    // command bytes
    const uint16_t RESET = 0x1E;    // Reset Command Byte    
    const uint16_t ADC_READ = 0x00; // ADC Read Command Byte

    // oversampling rates
    D1_OSR d1_osr = CONV_D1_256;
    D2_OSR d2_osr = CONV_D2_256;
    
    // Calibration data from PROM
    int64_t C1 = 0;
    int64_t C2 = 0;
    int64_t C3 = 0;
    int64_t C4 = 0;
    int64_t C5 = 0;
    int64_t C6 = 0;

    int64_t D1 = 0;
    int64_t D2 = 0;

    int64_t dT = 0;
    int64_t TEMP = 0; // last temperature value

    int64_t OFF = 0;
    int64_t SENS = 0;
    int64_t PRES = 0; // last pressure value

    uint32_t ALT = 0;  // last altitude value

    uint32_t altRingBuffer[BPS_BUFFER_SIZE] = {0};
    uint16_t altRingBufferIndex = 0;
    uint32_t altFIRFiltered = 0;
    uint32_t altRAFiltered = 0;

public:
    void init(uint8_t new_cs_pin, CONV_OSR new_osr);
    void reset(); 

    void set_CS_pin(uint8_t new_cs_pin);
    uint8_t get_CS_pin();

    void set_D1_OSR(D1_OSR new_d1_osr);
    D1_OSR get_D1_OSR();
    void set_D2_OSR(D2_OSR new_d2_osr);
    D2_OSR get_D2_OSR();

    uint16_t get_D1_OSR_delay();
    uint16_t get_D2_OSR_delay();

    void get_PROM();
    void get_coef(int64_t* coef, uint16_t coef_reg);
private:
    void update_temperature();
    void update_pressure();
    void update_altitude();
public:
    void update();

    int32_t get_temperature();
    uint32_t get_pressure();
    uint32_t get_altitude();    

    void calculate_altitude_FIR_filtered();
    uint32_t get_altitude_FIR_filtered();

    void calculate_altitude_RA_filtered();
    uint32_t get_altitude_RA_filtered();

    void print_all();
    void print_data();

private:
    double bpsFilterCoefficients[BPS_FIR_SIZE] = 
    {
    0.000000000000000000,
    0.000008471711691010,
    0.000034031005548846,
    0.000077106988255137,
    0.000138403662837941,
    0.000218883366724262,
    0.000319743982133028,
    0.000442390308259176,
    0.000588400085152093,
    0.000759485251766625,
    0.000957449103869573,
    0.001184140089975318,
    0.001441403044086650,
    0.001731028701745490,
    0.002054702379971361,
    0.002413952721518276,
    0.002810101409173712,
    0.003244214746448159,
    0.003717057977084964,
    0.004229053177713048,
    0.004780241506250726,
    0.005370250524144369,
    0.005998267234192322,
    0.006663017388751364,
    0.007362751526909165,
    0.008095238095238093,
    0.008857763896654724,
    0.009647141997425276,
    0.010459727105281005,
    0.011291438313783771,
    0.012137788991368519,
    0.012993923479729190,
    0.013854660157205743,
    0.014714540320298524,
    0.015567882242006970,
    0.016408839680862878,
    0.017231464040654450,
    0.018029769319095220,
    0.018797798935072635,
    0.019529693489398869,
    0.020219758493749275,
    0.020862531097052692,
    0.021452844848101230,
    0.021985891557450114,
    0.022457279360424725,
    0.022863086135656491,
    0.023199907499233917,
    0.023464898672271538,
    0.023655809608274510,
    0.023771012864745100,
    0.023809523809523808,
    0.023771012864745100,
    0.023655809608274510,
    0.023464898672271532,
    0.023199907499233920,
    0.022863086135656491,
    0.022457279360424732,
    0.021985891557450125,
    0.021452844848101230,
    0.020862531097052703,
    0.020219758493749282,
    0.019529693489398876,
    0.018797798935072642,
    0.018029769319095220,
    0.017231464040654450,
    0.016408839680862878,
    0.015567882242006961,
    0.014714540320298522,
    0.013854660157205732,
    0.012993923479729190,
    0.012137788991368524,
    0.011291438313783765,
    0.010459727105281005,
    0.009647141997425281,
    0.008857763896654724,
    0.008095238095238098,
    0.007362751526909175,
    0.006663017388751366,
    0.005998267234192327,
    0.005370250524144368,
    0.004780241506250728,
    0.004229053177713054,
    0.003717057977084971,
    0.003244214746448154,
    0.002810101409173709,
    0.002413952721518279,
    0.002054702379971364,
    0.001731028701745494,
    0.001441403044086652,
    0.001184140089975317,
    0.000957449103869573,
    0.000759485251766627,
    0.000588400085152096,
    0.000442390308259177,
    0.000319743982133028,
    0.000218883366724262,
    0.000138403662837942,
    0.000077106988255137,
    0.000034031005548847,
    0.000008471711691010,
    0.000000000000000000
    };
};

