#ifndef ALARABOARDCONTROLLERLASS_H
#define ALARABOARDCONTROLLERLASS_H

#include <Arduino.h>
#include "PCA9685.h"
#include <Wire.h>
#include "piezoSongs.h"
#include "ALARApinDefines.h"
#include "extendedIO/extendedIO.h"

struct RGB_12bitColor
{
    uint16_t Red_PWM12bit;
    uint16_t Green_PWM12bit;
    uint16_t Blue_PWM12bit;
};

class ALARAbuzzer
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

class RGB_LED
{
    public:
    RGB_12bitColor outputColor;
    uint8_t PWMbitDepth;
    uint16_t Red_PWMpin;
    uint16_t Green_PWMpin;
    uint16_t Blue_PWMpin;
    uint16_t Red_PWM;
    uint16_t Green_PWM;
    uint16_t Blue_PWM;

    // Constructor
    RGB_LED(uint16_t setRed_PWMpin, uint16_t setGreen_PWMpin, uint16_t setBlue_PWMpin, uint8_t setPWMbitDepth = 12);
    // Sets the PWM values to the LED driver IC
    void writeLEDOutput(PCA9685 LEDExtPWMIn, RGB_12bitColor ledInput);

};

class ALARABoardController
{
    private:
    PCA9685 &ALARALEDExtPWM;
    uint8_t PCA9685_OE_Pin;
    RGB_LED &Led1;
    RGB_LED &Led2;
    ALARAbuzzer &buzzerr;

    public:
    
    
    // Constructor
    ALARABoardController (PCA9685* setALARALEDExtPWM, uint8_t setPCA9685_OE_Pin, RGB_LED* setLed1, RGB_LED* setLed2, ALARAbuzzer* setbuzzer);
    //ALARABoardController (RGB_LED* setLed1, RGB_LED* setLed2, ALARAbuzzer* setbuzzer);
    // Run begin to setup the necessary device operations
    void begin();
    //
    void boardTasks();
    void setLED(uint8_t LedN, RGB_12bitColor ledInput);
};

#endif