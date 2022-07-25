/*
MS5607 Barometric Pressure Sensor driver

Written by Brandon Summers
Last updated May 31, 2022
Based on MS5607 datasheet information

Arduino platform friendly - Made specifically to be used with Teensy 3.6
*/

#include "ms5607.h"
#include "extendedIO/extendedIO.h"


// Initializes the MS5607
void MS5607::init(uint8_t new_cs_pin, CONV_OSR new_osr)
{
    Serial.println("BPS INIT BEGIN");
    set_CS_pin(new_cs_pin);
    Serial.print("BPS CS PIN SET TO PIN ");
    Serial.println(get_CS_pin());
    // Reset MS5607 after powerup to ensure PROM load to internal register
    reset();
    Serial.println("BPS RESET DONE");
    get_PROM();
    Serial.println("BPS GOT PROM");
    
    // Set Oversampling rates
    switch(new_osr)
    {
    case OSR_256:
        set_D1_OSR(CONV_D1_256);
        set_D2_OSR(CONV_D2_256);   
        Serial.println("BPS OSR AT 256");
        break;
    case OSR_512:
        set_D1_OSR(CONV_D1_512);
        set_D2_OSR(CONV_D2_512);   
        Serial.println("BPS OSR AT 512");
        break;
    case OSR_1024:
        set_D1_OSR(CONV_D1_1024);
        set_D2_OSR(CONV_D2_1024);   
        Serial.println("BPS OSR AT 1024");
        break;
    case OSR_2048:
        set_D1_OSR(CONV_D1_2048);
        set_D2_OSR(CONV_D2_2048);   
        Serial.println("BPS OSR AT 2048");
        break;
    case OSR_4096:
        set_D1_OSR(CONV_D1_4096);
        set_D2_OSR(CONV_D2_4096);   
        Serial.println("BPS OSR AT 4096");
        break;
    }
    Serial.println("BPS INIT COMPLETE");
}


// Hardware resets the MS5607
void MS5607::reset()
{    
    Serial.println("SPI BEGINNING TRANSACTION");
    SPI.beginTransaction(SPISettings(BPS_SPI_RATE, MSBFIRST, SPI_MODE0));
    Serial.println("SPI SETTING CS PIN LOW");
    //digitalWrite(CS_PIN,0);
    digitalWrite(ED19,0);
    Serial.println("SPI SENDING RESET COMMAND");
    SPI.transfer(RESET); // send reset command
    Serial.println("SPI SETTING CS PIN HIGH");
    //digitalWrite(CS_PIN,1);
    digitalWrite(ED19,1);
    Serial.println("SPI ENDING TRANSACTION");
    SPI.endTransaction();
    Serial.println("SPI TRANSACTION ENDED");
    delay(10); // wait for reset to complete
}


// Set the SPI Chip Select pin for the MS5607
void MS5607::set_CS_pin(uint8_t new_cs_pin)
{
    CS_PIN = new_cs_pin;
    pinMode(new_cs_pin,OUTPUT);
}


// Return the SPI Chip Select pin for the MS5607
uint8_t MS5607::get_CS_pin()
{
    return CS_PIN;
}


// Sets the pressure oversampling rate command byte
void MS5607::set_D1_OSR(D1_OSR new_d1_osr)
{
    d1_osr = new_d1_osr;
}


// Returns the pressure oversampling rate command byte
D1_OSR MS5607::get_D1_OSR()
{
    return d1_osr;
}


// Sets the temperature oversampling rate command byte
void MS5607::set_D2_OSR(D2_OSR new_d2_osr)
{
    d2_osr = new_d2_osr;
}


// Returns the temperature oversampling rate command byte
D2_OSR MS5607::get_D2_OSR()
{
    return d2_osr;
}


// Gets the delay time for the sensor to complete a read of the pressure data
// - Delays are in microseconds and are based on max ADC conversion times in datasheet
uint16_t MS5607:: get_D1_OSR_delay()
{
    switch(d1_osr)
    {
    case CONV_D1_256:
        return 600;
        break;
    case CONV_D1_512:
        return 1170;
        break;
    case CONV_D1_1024:
        return 2280;
        break;
    case CONV_D1_2048:
        return 4540;
        break;
    case CONV_D1_4096:
        return 9040;
        break;
    }
}


// Gets the delay time for the sensor to complete a read of the temperature data
// - Delays are in microseconds and are based on max ADC conversion times in datasheet
uint16_t MS5607:: get_D2_OSR_delay()
{
    switch(d2_osr)
    {
    case CONV_D2_256:
        return 600;
        break;
    case CONV_D2_512:
        return 1170;
        break;
    case CONV_D2_1024:
        return 2280;
        break;
    case CONV_D2_2048:
        return 4540;
        break;
    case CONV_D2_4096:
        return 9040;
        break;
    }
}


// Gets all of the calibration coefficient values from onboard PROM
void MS5607::get_PROM()
{    
    get_coef(&C1,PROM_READ_C1);
    get_coef(&C2,PROM_READ_C2);
    get_coef(&C3,PROM_READ_C3);
    get_coef(&C4,PROM_READ_C4);
    get_coef(&C5,PROM_READ_C5);
    get_coef(&C6,PROM_READ_C6);    
}


// Gets an individual calibration coefficient value from onboard PROM
void MS5607::get_coef(int64_t* coef, uint16_t coef_reg)
{
    SPI.beginTransaction(SPISettings(BPS_SPI_RATE, MSBFIRST, SPI_MODE0));
    //digitalWrite(CS_PIN,0);
    digitalWrite(ED19,0);
    SPI.transfer(coef_reg);                 // throw away first byte read
    *coef = 0;                              // reset coefficient value 
    *coef += SPI.transfer(coef_reg)*256;    // get most significant byte
    *coef += SPI.transfer(coef_reg);        // get least significant byte
    //digitalWrite(CS_PIN,1);
    digitalWrite(ED19,1);
    SPI.endTransaction();
}
 

// Gets the temperature in degrees centicelsius (Celsius times 100)
void MS5607::update_temperature()
{
    SPI.beginTransaction(SPISettings(BPS_SPI_RATE, MSBFIRST, SPI_MODE0));
    //digitalWrite(CS_PIN,0);
    digitalWrite(ED19,0);
    SPI.transfer(get_D2_OSR());
    //digitalWrite(CS_PIN,1);
    digitalWrite(ED19,1);
    SPI.endTransaction();

    delayMicroseconds(get_D2_OSR_delay());

    SPI.beginTransaction(SPISettings(BPS_SPI_RATE, MSBFIRST, SPI_MODE0));
    //digitalWrite(CS_PIN,0);  
    digitalWrite(ED19,0);
    SPI.transfer(ADC_READ); // throw away first byte read
    uint8_t temp_data[3] = {0};

    for(int i = 0; i < 3; i++)
    { // transfer all 3 bytes of the temperature data
        temp_data[i] = SPI.transfer(ADC_READ);
    }

    //digitalWrite(CS_PIN,1);
    digitalWrite(ED19,1);
    SPI.endTransaction();

    D2 = temp_data[0]*65536 + temp_data[1]*256 + temp_data[2]; // combine data
    dT = (int64_t)D2 - (int64_t)C5*256;
    TEMP = 2000 + (int64_t)dT*(int64_t)C6/8388608; 
}


// Gets the air pressure in Pascals
void MS5607::update_pressure()
{
    SPI.beginTransaction(SPISettings(BPS_SPI_RATE, MSBFIRST, SPI_MODE0));
    //digitalWrite(CS_PIN,0);
    digitalWrite(ED19,0);
    SPI.transfer(get_D1_OSR());
    //digitalWrite(CS_PIN,1);
    digitalWrite(ED19,1);
    SPI.endTransaction();

    delayMicroseconds(get_D1_OSR_delay());

    SPI.beginTransaction(SPISettings(BPS_SPI_RATE, MSBFIRST, SPI_MODE0));
    //digitalWrite(CS_PIN,0);  
    digitalWrite(ED19,0);
    SPI.transfer(ADC_READ); // throw away first byte read
    uint8_t pres_data[3] = {0};

    for(int i = 0; i < 3; i++)
    { // transfer all 3 bytes of the temperature data
        pres_data[i] = SPI.transfer(ADC_READ);
    }

    //digitalWrite(CS_PIN,1);
    digitalWrite(ED19,1);
    SPI.endTransaction();

    D1 = pres_data[0]*65536 + pres_data[1]*256 + pres_data[2]; // combine data
    OFF = (int64_t)C2*131072 + ((int64_t)C4*(int64_t)dT)/64;
    SENS = (int64_t)C1*65536 + ((int64_t)C3*(int64_t)dT)/128;
    PRES = ((int64_t)D1*SENS/2097152 - OFF)/32768;
}


// Gets the altitude in centimeters
// - Based on NASA's Earth Atmosphere Model (Metric Units)
//   https://www.grc.nasa.gov/www/k-12/airplane/atmosmet.html
void MS5607::update_altitude()
{
    // Troposphere
    double altitude = (2.408058-pow((double)PRES/1000.0,1.0/5.256))/0.00005074287;

    // Lower Stratosphere
    if(altitude > 11000)
    {
        altitude = (1.7028-log(PRES/22.65))/0.000157;
    }

    // Add Upper stratosphere math if expected altitude is above 25Km (82k feet)
    // Honestly you're really getting towards the theoretical pressure limit
    // of the sensor if you're above 25Km

    altitude *= 100.0; // convert meters to centimeters for integer value
    ALT = uint32_t(altitude);
    altRingBuffer[altRingBufferIndex] = ALT;
    altRingBufferIndex++;
    if(altRingBufferIndex >= BPS_BUFFER_SIZE)
    {
        altRingBufferIndex = 0;
    }
}


void MS5607::update()
{
    update_temperature();
    update_pressure();
    update_altitude();
}


// Returns the last temperature data
int32_t MS5607::get_temperature()
{  return TEMP;  }


// Returns the last pressure data// 
uint32_t MS5607::get_pressure()
{  return PRES;  }


// Returns the last altitude data
uint32_t MS5607::get_altitude()
{  return ALT;  }


// Calculate the filtered altitude based on the FIR coefficients
void MS5607::calculate_altitude_FIR_filtered()
{
    double altitudeFiltered = 0;
    for(int i = 0; i < BPS_FIR_SIZE; i++)
    {
        int16_t bufferIndex = altRingBufferIndex - i;
        if(bufferIndex < 0)
        {
            bufferIndex += BPS_BUFFER_SIZE;
        }
        altitudeFiltered += altRingBuffer[bufferIndex]*bpsFilterCoefficients[i];
    }
    altFIRFiltered = uint32_t(altitudeFiltered);
}


// Return the Finite Impulse Response filtered altitude data
uint32_t MS5607::get_altitude_FIR_filtered()
{  return altFIRFiltered;  }


// Calculate the filtered altitude based on the rolling average
void MS5607::calculate_altitude_RA_filtered()
{
    double altitudeFiltered = 0;
    for(int i = 0; i < BPS_BUFFER_SIZE; i++)
    {
        altitudeFiltered += altRingBuffer[i];
    }
    altRAFiltered = uint32_t(altitudeFiltered)/BPS_BUFFER_SIZE;
}


// Return the rolling average filtered altitude data
uint32_t MS5607::get_altitude_RA_filtered()
{  return altRAFiltered;  }


// Prints all data from MS5607 to serial monitor
void MS5607::print_all()
{
    Serial.print("C1:");
    Serial.print(C1);
    Serial.print(" C2:");
    Serial.print(C2);
    Serial.print(" C3:");
    Serial.print(C3);
    Serial.print(" C4:");
    Serial.print(C4);
    Serial.print(" C5:");
    Serial.print(C5);
    Serial.print(" C6:");
    Serial.print(C6);

    Serial.print(" D1:");
    Serial.print(D1);
    Serial.print(" D2:");
    Serial.print(D2);

    Serial.print(" dT:");
    Serial.print(dT);
    Serial.print(" TEMP:");
    Serial.print(TEMP);

    Serial.print(" OFF:");
    Serial.print(OFF);
    Serial.print(" SENS:");
    Serial.print(SENS);
    Serial.print(" PRES:");
    Serial.println(PRES);
}


void MS5607::print_data()
{
    Serial.print("Temp ");
    Serial.print(get_temperature()); 
    Serial.print(" centicelsius - Pressure ");
    Serial.print(get_pressure());
    Serial.print(" Pa - Altitude ");
    Serial.print(get_altitude());
    Serial.println(" centimeters");  
}