#pragma once

// Define LED ring
#define PIXEL_PIN 10
#define LED_COUNT 12
Adafruit_NeoPixel strip(LED_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

typedef struct
{
  uint32_t fPixelColor[12][12];
  uint32_t fDelay;
  int fNumCyclesBlock;
} LedMap_t;


// /****************************    CONNECTING ANNIMATION   ***********************************/
uint32_t connectSequenceZero[12] = {strip.Color(0, 255, 0), strip.Color(0, 255, 0), strip.Color(0, 255, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0),
                              strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0),
                              strip.Color(0, 0, 0), strip.Color(0, 0, 0)};
uint32_t connectSequenceOne[12] = {strip.Color(0, 0, 0), strip.Color(0, 255, 0), strip.Color(0, 255, 0), strip.Color(0, 255, 0), strip.Color(0, 0, 0),
                              strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0),
                              strip.Color(0, 0, 0), strip.Color(0, 0, 0)};
uint32_t connectSequenceTwo[12] = {strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 255, 0), strip.Color(0, 255, 0), strip.Color(0, 255, 0),
                              strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0),
                              strip.Color(0, 0, 0), strip.Color(0, 0, 0)};
uint32_t connectSequenceThree[12] = {strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 255, 0), strip.Color(0, 255, 0),
                              strip.Color(0, 255, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0),
                              strip.Color(0, 0, 0), strip.Color(0, 0, 0)};
uint32_t connectSequenceFour[12] = {strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 255, 0),
                              strip.Color(0, 255, 0), strip.Color(0, 255, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0),
                              strip.Color(0, 0, 0), strip.Color(0, 0, 0)};
uint32_t connectSequenceFive[12] = {strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0),
                              strip.Color(0, 255, 0), strip.Color(0, 255, 0), strip.Color(0, 255, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0),
                              strip.Color(0, 0, 0), strip.Color(0, 0, 0)};
uint32_t connectSequenceSix[12] = {strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0),
                              strip.Color(0, 0, 0), strip.Color(0, 255, 0), strip.Color(0, 255, 0), strip.Color(0, 255, 0), strip.Color(0, 0, 0),
                              strip.Color(0, 0, 0), strip.Color(0, 0, 0)};
uint32_t connectSequenceSeven[12] = {strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0),
                              strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 255, 0), strip.Color(0, 255, 0), strip.Color(0, 255, 0),
                              strip.Color(0, 0, 0), strip.Color(0, 0, 0)};
uint32_t connectSequenceEight[12] = {strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0),
                              strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 255, 0), strip.Color(0, 255, 0),
                              strip.Color(0, 255, 0), strip.Color(0, 0, 0)};
uint32_t connectSequenceNine[12] = {strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0),
                              strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 255, 0),
                              strip.Color(0, 255, 0), strip.Color(0, 255, 0)};                             
uint32_t connectSequenceTen[12] = {strip.Color(0, 255, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0),
                              strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0),
                              strip.Color(0, 255, 0), strip.Color(0, 255, 0)};    
uint32_t connectSequenceEleven[12] = {strip.Color(0, 255, 0), strip.Color(0, 255, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0),
                              strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0),
                              strip.Color(0, 0, 0), strip.Color(0, 255, 0)};    



/****************************    CONNECTED ANNIMATION   ***********************************/
uint32_t connectedSequenceZero[12] = {strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0),
                              strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0),
                              strip.Color(0, 0, 0), strip.Color(0, 0, 0)};
uint32_t connectedSequenceOne[12] = {strip.Color(0, 42, 0), strip.Color(0, 42, 0), strip.Color(0, 42, 0), strip.Color(0, 42, 0), strip.Color(0, 42, 0),
                              strip.Color(0, 42, 0), strip.Color(0, 42, 0), strip.Color(0, 42, 0), strip.Color(0, 42, 0), strip.Color(0, 42, 0),
                              strip.Color(0, 42, 0), strip.Color(0, 42, 0)};
uint32_t connectedSequenceTwo[12] = {strip.Color(0, 84, 0), strip.Color(0, 84, 0), strip.Color(0, 84, 0), strip.Color(0, 84, 0), strip.Color(0, 84, 0),
                              strip.Color(0, 84, 0), strip.Color(0, 84, 0), strip.Color(0, 84, 0), strip.Color(0, 84, 0), strip.Color(0, 84, 0),
                              strip.Color(0, 84, 0), strip.Color(0, 84, 0)};
uint32_t connectedSequenceThree[12] = {strip.Color(0, 127, 0), strip.Color(0, 127, 0), strip.Color(0, 127, 0), strip.Color(0, 127, 0), strip.Color(0, 127, 0),
                              strip.Color(0, 127, 0), strip.Color(0, 127, 0), strip.Color(0, 127, 0), strip.Color(0, 127, 0), strip.Color(0, 127, 0),
                              strip.Color(0, 127, 0), strip.Color(0, 127, 0)};
uint32_t connectedSequenceFour[12] = {strip.Color(0, 169, 0), strip.Color(0, 169, 0), strip.Color(0, 169, 0), strip.Color(0, 169, 0), strip.Color(0, 169, 0),
                              strip.Color(0, 169, 0), strip.Color(0, 169, 0), strip.Color(0, 169, 0), strip.Color(0, 169, 0), strip.Color(0, 169, 0),
                              strip.Color(0, 169, 0), strip.Color(0, 169, 0)};
uint32_t connectedSequenceFive[12] = {strip.Color(0, 211, 0), strip.Color(0, 211, 0), strip.Color(0, 211, 0), strip.Color(0, 211, 0), strip.Color(0, 211, 0),
                              strip.Color(0, 211, 0), strip.Color(0, 211, 0), strip.Color(0, 211, 0), strip.Color(0, 211, 0), strip.Color(0, 211, 0),
                              strip.Color(0, 211, 0), strip.Color(0, 211, 0)};
uint32_t connectedSequenceSix[12] = {strip.Color(0, 255, 0), strip.Color(0, 255, 0), strip.Color(0, 255, 0), strip.Color(0, 255, 0), strip.Color(0, 255, 0),
                              strip.Color(0, 255, 0), strip.Color(0, 255, 0), strip.Color(0, 255, 0), strip.Color(0, 255, 0), strip.Color(0, 255, 0),
                              strip.Color(0, 255, 0), strip.Color(0, 255, 0)};
uint32_t connectedSequenceSeven[12] = {strip.Color(0, 211, 0), strip.Color(0, 211, 0), strip.Color(0, 211, 0), strip.Color(0, 211, 0), strip.Color(0, 211, 0),
                              strip.Color(0, 211, 0), strip.Color(0, 211, 0), strip.Color(0, 211, 0), strip.Color(0, 211, 0), strip.Color(0, 211, 0),
                              strip.Color(0, 211, 0), strip.Color(0, 211, 0)};
uint32_t connectedSequenceEight[12] = {strip.Color(0, 169, 0), strip.Color(0, 169, 0), strip.Color(0, 169, 0), strip.Color(0, 169, 0), strip.Color(0, 169, 0),
                              strip.Color(0, 169, 0), strip.Color(0, 169, 0), strip.Color(0, 169, 0), strip.Color(0, 169, 0), strip.Color(0, 169, 0),
                              strip.Color(0, 169, 0), strip.Color(0, 169, 0)};
uint32_t connectedSequenceNine[12] = {strip.Color(0, 127, 0), strip.Color(0, 127, 0), strip.Color(0, 127, 0), strip.Color(0, 127, 0), strip.Color(0, 127, 0),
                              strip.Color(0, 127, 0), strip.Color(0, 127, 0), strip.Color(0, 127, 0), strip.Color(0, 127, 0), strip.Color(0, 127, 0),
                              strip.Color(0, 127, 0), strip.Color(0, 127, 0)};                             
uint32_t connectedSequenceTen[12] = {strip.Color(0, 84, 0), strip.Color(0, 84, 0), strip.Color(0, 84, 0), strip.Color(0, 84, 0), strip.Color(0, 84, 0),
                              strip.Color(0, 84, 0), strip.Color(0, 84, 0), strip.Color(0, 84, 0), strip.Color(0, 84, 0), strip.Color(0, 84, 0),
                              strip.Color(0, 84, 0), strip.Color(0, 84, 0)};    
uint32_t connectedSequenceEleven[12] = {strip.Color(0, 42, 0), strip.Color(0, 42, 0), strip.Color(0, 42, 0), strip.Color(0, 42, 0), strip.Color(0, 42, 0),
                              strip.Color(0, 42, 0), strip.Color(0, 42, 0), strip.Color(0, 42, 0), strip.Color(0, 42, 0), strip.Color(0, 42, 0),
                              strip.Color(0, 42, 0), strip.Color(0, 42, 0)};  



/****************************    ERROR ANNIMATION   ***********************************/
uint32_t errorSequenceZero[12] = {strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0),
                              strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0),
                              strip.Color(0, 0, 0), strip.Color(0, 0, 0)};
uint32_t errorSequenceOne[12] = {strip.Color(42, 0, 0), strip.Color(42, 0, 0), strip.Color(42, 0, 0), strip.Color(42, 0, 0), strip.Color(42, 0, 0),
                              strip.Color(42, 0, 0), strip.Color(42, 0, 0), strip.Color(42, 0, 0), strip.Color(42, 0, 0), strip.Color(42, 0, 0),
                              strip.Color(42, 0, 0), strip.Color(42, 0, 0)};
uint32_t errorSequenceTwo[12] = {strip.Color(84, 0, 0), strip.Color(84, 0, 0), strip.Color(84, 0, 0), strip.Color(84, 0, 0), strip.Color(84, 0, 0),
                              strip.Color(84, 0, 0), strip.Color(84, 0, 0), strip.Color(84, 0, 0), strip.Color(84, 0, 0), strip.Color(84, 0, 0),
                              strip.Color(84, 0, 0), strip.Color(84, 0, 0)};
uint32_t errorSequenceThree[12] = {strip.Color(127, 0, 0), strip.Color(127, 0, 0), strip.Color(127, 0, 0), strip.Color(127, 0, 0), strip.Color(127, 0, 0),
                              strip.Color(127, 0, 0), strip.Color(127, 0, 0), strip.Color(127, 0, 0), strip.Color(127, 0, 0), strip.Color(127, 0, 0),
                              strip.Color(127, 0, 0), strip.Color(127, 0, 0)};
uint32_t errorSequenceFour[12] = {strip.Color(169, 0, 0), strip.Color(169, 0, 0), strip.Color(169, 0, 0), strip.Color(169, 0, 0), strip.Color(169, 0, 0),
                              strip.Color(169, 0, 0), strip.Color(169, 0, 0), strip.Color(169, 0, 0), strip.Color(169, 0, 0), strip.Color(169, 0, 0),
                              strip.Color(169, 0, 0), strip.Color(169, 0, 0)};
uint32_t errorSequenceFive[12] = {strip.Color(211, 0, 0), strip.Color(211, 0, 0), strip.Color(211, 0, 0), strip.Color(211, 0, 0), strip.Color(211, 0, 0),
                              strip.Color(211, 0, 0), strip.Color(211, 0, 0), strip.Color(211, 0, 0), strip.Color(211, 0, 0), strip.Color(211, 0, 0),
                              strip.Color(211, 0, 0), strip.Color(211, 0, 0)};
uint32_t errorSequenceSix[12] = {strip.Color(255, 0, 0), strip.Color(255, 0, 0), strip.Color(255, 0, 0), strip.Color(255, 0, 0), strip.Color(255, 0, 0),
                              strip.Color(255, 0, 0), strip.Color(255, 0, 0), strip.Color(255, 0, 0), strip.Color(255, 0, 0), strip.Color(255, 0, 0),
                              strip.Color(255, 0, 0), strip.Color(255, 0, 0)};
uint32_t errorSequenceSeven[12] = {strip.Color(211, 0, 0), strip.Color(211, 0, 0), strip.Color(211, 0, 0), strip.Color(211, 0, 0), strip.Color(211, 0, 0),
                              strip.Color(211, 0, 0), strip.Color(211, 0, 0), strip.Color(211, 0, 0), strip.Color(211, 0, 0), strip.Color(211, 0, 0),
                              strip.Color(211, 0, 0), strip.Color(211, 0, 0)};
uint32_t errorSequenceEight[12] = {strip.Color(169, 0, 0), strip.Color(169, 0, 0), strip.Color(169, 0, 0), strip.Color(169, 0, 0), strip.Color(169, 0, 0),
                              strip.Color(169, 0, 0), strip.Color(169, 0, 0), strip.Color(169, 0, 0), strip.Color(169, 0, 0), strip.Color(169, 0, 0),
                              strip.Color(169, 0, 0), strip.Color(169, 0, 0)};
uint32_t errorSequenceNine[12] = {strip.Color(127, 0, 0), strip.Color(127, 0, 0), strip.Color(127, 0, 0), strip.Color(127, 0, 0), strip.Color(127, 0, 0),
                              strip.Color(127, 0, 0), strip.Color(127, 0, 0), strip.Color(127, 0, 0), strip.Color(127, 0, 0), strip.Color(127, 0, 0),
                              strip.Color(127, 0, 0), strip.Color(127, 0, 0)};                          
uint32_t errorSequenceTen[12] = {strip.Color(84, 0, 0), strip.Color(84, 0, 0), strip.Color(84, 0, 0), strip.Color(84, 0, 0), strip.Color(84, 0, 0),
                              strip.Color(84, 0, 0), strip.Color(84, 0, 0), strip.Color(84, 0, 0), strip.Color(84, 0, 0), strip.Color(84, 0, 0),
                              strip.Color(84, 0, 0), strip.Color(84, 0, 0)}; 
uint32_t errorSequenceEleven[12] = {strip.Color(42, 0, 0), strip.Color(42, 0, 0), strip.Color(42, 0, 0), strip.Color(42, 0, 0), strip.Color(42, 0, 0),
                              strip.Color(42, 0, 0), strip.Color(42, 0, 0), strip.Color(42, 0, 0), strip.Color(42, 0, 0), strip.Color(42, 0, 0),
                              strip.Color(42, 0, 0), strip.Color(42, 0, 0)};                             
