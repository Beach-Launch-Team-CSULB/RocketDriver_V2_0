#include "ALARABoardControllerClass.h"

ALARABoardController::ALARABoardController(PCA9685* setALARALEDExtPWM, uint8_t setPCA9685_OE_Pin, RGB_LED* setLed1, RGB_LED* setLed2, ALARAbuzzer* setbuzzer)
    : ALARALEDExtPWM{*setALARALEDExtPWM}, PCA9685_OE_Pin{setPCA9685_OE_Pin}, Led1{*setLed1}, Led2{*setLed2}, buzzerr{*setbuzzer}
{

}


void ALARABoardController::begin()
{
    // LED stuff
    ALARALEDExtPWM.resetDevices();
    ALARALEDExtPWM.init();
    ALARALEDExtPWM.setPWMFrequency(2000);
    pinMode(PCA9685_OE_Pin,OUTPUT);
    digitalWriteExtended(PCA9685_OE_Pin,HIGH);
    ALARALEDExtPWM.setChannelOn(0);
    ALARALEDExtPWM.setChannelOn(1);
    ALARALEDExtPWM.setChannelOn(2);
    ALARALEDExtPWM.setChannelOn(3);
    ALARALEDExtPWM.setChannelOn(4);
    ALARALEDExtPWM.setChannelOn(5);
    ALARALEDExtPWM.setChannelOff(6);
    ALARALEDExtPWM.setChannelOff(7);
    ALARALEDExtPWM.setChannelOff(8);
    ALARALEDExtPWM.setChannelOff(9);
    ALARALEDExtPWM.setChannelOff(10);
    ALARALEDExtPWM.setChannelOff(11);
    ALARALEDExtPWM.setChannelOff(12);
    ALARALEDExtPWM.setChannelOff(13);
    ALARALEDExtPWM.setChannelOff(14);
    ALARALEDExtPWM.setChannelOff(15);    
}

void ALARABoardController::boardTasks()
{
    //stuff to do at regular program interval
    // Led and buzzer routines beyond single color/tone would update here
}

void ALARABoardController::setLED(uint8_t LedN, RGB_12bitColor ledInput)
{
    // Replace this ifdef with the ALARA Map present bool
    #ifdef ALARAV2_1
    // For LedN of 1, write to Led1
    if (LedN == 1)
    {
    Led1.writeLEDOutput(ALARALEDExtPWM, ledInput);
    }
    // For LedN of 2, write to Led2
    else if (LedN == 2)
    {
    Led2.writeLEDOutput(ALARALEDExtPWM, ledInput);
    }
    #endif
}


RGB_LED::RGB_LED(uint16_t setRed_PWMpin, uint16_t setGreen_PWMpin, uint16_t setBlue_PWMpin, uint8_t setPWMbitDepth)
    : Red_PWMpin{setRed_PWMpin}, Green_PWMpin{setGreen_PWMpin}, Blue_PWMpin{setBlue_PWMpin}, PWMbitDepth{setPWMbitDepth}
{

}

void RGB_LED::writeLEDOutput(PCA9685 LEDExtPWMIn, RGB_12bitColor ledInput)
{
// Set Led object PWM values
Red_PWM = ledInput.Red_PWM12bit;
Green_PWM = ledInput.Green_PWM12bit;
Blue_PWM = ledInput.Blue_PWM12bit;
// Write Led object PWM values via I2C
LEDExtPWMIn.setChannelPWM(Red_PWMpin, Red_PWM);
LEDExtPWMIn.setChannelPWM(Green_PWMpin, Green_PWM);
LEDExtPWMIn.setChannelPWM(Blue_PWMpin, Blue_PWM);
}

ALARAbuzzer::ALARAbuzzer(uint8_t setBuzzerPin, float setMaxToneFrequency, float setMinToneFrequency, uint8_t setMinPWM, uint8_t setMaxPWM)
         : buzzerPin{setBuzzerPin}, maxToneFrequency{setMaxToneFrequency}, minToneFrequency{setMinToneFrequency}, minPWM{setMinPWM}, maxPWM{setMaxPWM}
{
  //nothing for now
}

void ALARAbuzzer::playTone(buzzerTone toneInput)
{
  // cap the loudness at the defined min and max PWM duty cycles
  // slighly odd looking because I have volume as 0-100% for notes and need to convert to 8bit PWM duty ycle
  uint8_t pwmValue = 0;
  if (toneInput.toneLoudness >= float(maxPWM)/float(255)*100)
  {
    //toneInput.toneLoudness = (float(maxPWM)/float(255))*100;
    pwmValue = maxPWM;
  }
  else if (toneInput.toneLoudness <= float(minPWM)/float(255)*100)
  {
    //toneInput.toneLoudness = (float(minPWM)/float(255))*100;
    pwmValue = minPWM;
  }
  else 
  {
    pwmValue = (toneInput.toneLoudness/100)*255;
  }
  // Only play if tone is within the buzzer defined range, otherwise analogWrite value 0 to turn off output
  // It is critical to keep a min freq because I define rests as tones at 0 frequency, which will cause this to turn off buzzer
  if (toneInput.toneFrequency >= minToneFrequency && toneInput.toneFrequency <= maxToneFrequency)
  {
    analogWriteFrequency(buzzerPin,toneInput.toneFrequency);
    analogWrite(buzzerPin,pwmValue);
  }
  else analogWrite(buzzerPin,0);
}

void ALARAbuzzer::playSongVector(std::vector<buzzerTone> songIn, uint16_t BPMset)
{
  // All the note durations are set with BPM of 120 as baseline
  if (BPMset == 0)
  {
    // No ur not
    BPMset = 120;
  }
  float durationScalar = float(120)/float(BPMset);
  //break between notes to make audible distinction in micros
  uint32_t noteSeparation  = 15000*durationScalar;
  // Tolerance in micros for polling to start the note (if timer is within tolerance of making a comparison operator true, it executes)
  uint32_t noteStartTol  = 125; // keep less than note Separation or weirdness could ensue
  // If song finished but is also set to repeat, reset songStart
  if (songEnd)
  {
    if (songRepeat)
    {
    songStart = true;
    songEnd = false;
    }
    else songPlaying = false;
  }
  // When song begins, set song timer to zero, start vector iterator at first note
  if (songStart)
  {
    songNoteIt = songIn.begin();
    songTimer = 0;
    currentNoteTime = 0;
    songStart = false;
    songPlaying = true;
  }
  // Play the current note by working through the vector with the iterator
  for (; songNoteIt != songIn.end();)
  {
    // If not already playing note, check if it's time to do so
    if (!notePlaying)
    {
      // Play current note if time to do so
      if ((songTimer + noteStartTol) >= uint32_t(currentNoteTime*durationScalar))
      {
        playTone(*songNoteIt);
        notePlaying = true;
      }
    }
    // If note is already playing, check if it's time to turn it off and move to next note
    if (notePlaying)
    {
      // Check note duration to turn note off
      // I am creating the note separation by chopping the end of note durations
      if (songTimer >= (currentNoteTime + ((songNoteIt->toneDuration)*durationScalar) - noteSeparation))
      {
        analogWrite(buzzerPin,0);
        notePlaying = false;
        // set next note time at end of this note
        currentNoteTime += ((songNoteIt->toneDuration)*durationScalar);
        // Only increment the song iterator when turning off a note
        ++songNoteIt;
      }
    }
  // break from for loop
  break;
  }
  // When reaching end of vector with iterator detect end of song
  if ((songNoteIt == songIn.end()))
  {
    songEnd = true;
  }
}
