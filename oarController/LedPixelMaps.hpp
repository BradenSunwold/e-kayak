#pragma once

#include <functional>

// Define LED ring
#define PIXEL_PIN 10
#define LED_COUNT 12
Adafruit_NeoPixel strip(LED_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

typedef struct
{
  std::function<void(int, uint32_t*)> fFrameGenerator;  // Function to generate frame
  uint32_t fDelayInMs;
  int fNumCyclesBlock;
  int fNumFrames;
} LedMap_t;

// Global variables for pixel map tests - too big to store in local stack
const uint32_t totalAnimations = 6;
LedMap_t pixelMapList[totalAnimations];

// Set battery indication LED map
// uint32_t batteryColorIndex[] = {strip.Color(0, 255, 0), strip.Color(75, 255, 0), strip.Color(200, 255, 0), strip.Color(255, 170, 0), 
//                                     strip.Color(255, 50, 0), strip.Color(255, 0, 0)};
uint32_t batteryColorIndex[] = {strip.Color(255, 0, 0), strip.Color(255, 50, 0), strip.Color(255, 170, 0), strip.Color(200, 255, 0),
                                        strip.Color(75, 255, 0), strip.Color(0, 255, 0)};


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



/****************************    MODE CHANGE ANNIMATION   ***********************************/
uint32_t modeChangeSequenceZero[12] = {strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0),
                              strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0),
                              strip.Color(0, 0, 0), strip.Color(0, 0, 0)};
uint32_t modeChangeSequenceOne[12] = {strip.Color(20, 20, 42), strip.Color(20, 20, 42), strip.Color(20, 20, 42), strip.Color(20, 20, 42), strip.Color(20, 20, 42),
                              strip.Color(20, 20, 42), strip.Color(20, 20, 42), strip.Color(20, 20, 42), strip.Color(20, 20, 42), strip.Color(20, 20, 42),
                              strip.Color(20, 20, 42), strip.Color(20, 20, 42)};
uint32_t modeChangeSequenceTwo[12] = {strip.Color(40, 40, 84), strip.Color(40, 40, 84), strip.Color(40, 40, 84), strip.Color(40, 40, 84), strip.Color(40, 40, 84),
                              strip.Color(40, 40, 84), strip.Color(40, 40, 84), strip.Color(40, 40, 84), strip.Color(40, 40, 84), strip.Color(40, 40, 84),
                              strip.Color(40, 40, 84), strip.Color(40, 40, 84)};
uint32_t modeChangeSequenceThree[12] = {strip.Color(60, 60, 127), strip.Color(60, 60, 127), strip.Color(60, 60, 127), strip.Color(60, 60, 127), strip.Color(60, 60, 127),
                              strip.Color(60, 60, 127), strip.Color(60, 60, 127), strip.Color(60, 60, 127), strip.Color(60, 60, 127), strip.Color(60, 60, 127),
                              strip.Color(60, 60, 127), strip.Color(60, 60, 127)};
uint32_t modeChangeSequenceFour[12] = {strip.Color(80, 80, 169), strip.Color(80, 80, 169), strip.Color(80, 80, 169), strip.Color(80, 80, 169), strip.Color(80, 80, 169),
                              strip.Color(80, 80, 169), strip.Color(80, 80, 169), strip.Color(80, 80, 169), strip.Color(80, 80, 169), strip.Color(80, 80, 169),
                              strip.Color(80, 80, 169), strip.Color(80, 80, 169)};
uint32_t modeChangeSequenceFive[12] = {strip.Color(100, 100, 211), strip.Color(100, 100, 211), strip.Color(100, 100, 211), strip.Color(100, 100, 211), strip.Color(100, 100, 211),
                              strip.Color(100, 100, 211), strip.Color(100, 100, 211), strip.Color(100, 100, 211), strip.Color(100, 100, 211), strip.Color(100, 100, 211),
                              strip.Color(100, 100, 211), strip.Color(100, 100, 211)};
uint32_t modeChangeSequenceSix[12] = {strip.Color(120, 120, 255), strip.Color(120, 120, 255), strip.Color(120, 120, 255), strip.Color(120, 120, 255), strip.Color(120, 120, 255),
                              strip.Color(120, 120, 255), strip.Color(120, 120, 255), strip.Color(120, 120, 255), strip.Color(120, 120, 255), strip.Color(120, 120, 255),
                              strip.Color(120, 120, 255), strip.Color(120, 120, 255)};
uint32_t modeChangeSequenceSeven[12] = {strip.Color(100, 100, 211), strip.Color(100, 100, 211), strip.Color(100, 100, 211), strip.Color(100, 100, 211), strip.Color(100, 100, 211),
                              strip.Color(100, 100, 211), strip.Color(100, 100, 211), strip.Color(100, 100, 211), strip.Color(100, 100, 211), strip.Color(100, 100, 211),
                              strip.Color(100, 100, 211), strip.Color(100, 100, 211)};
uint32_t modeChangeSequenceEight[12] = {strip.Color(80, 80, 169), strip.Color(80, 80, 169), strip.Color(80, 80, 169), strip.Color(80, 80, 169), strip.Color(80, 80, 169),
                              strip.Color(80, 80, 169), strip.Color(80, 80, 169), strip.Color(80, 80, 169), strip.Color(80, 80, 169), strip.Color(80, 80, 169),
                              strip.Color(80, 80, 169), strip.Color(80, 80, 169)};
uint32_t modeChangeSequenceNine[12] = {strip.Color(60, 60, 127), strip.Color(60, 60, 127), strip.Color(60, 60, 127), strip.Color(60, 60, 127), strip.Color(60, 60, 127),
                              strip.Color(60, 60, 127), strip.Color(60, 60, 127), strip.Color(60, 60, 127), strip.Color(60, 60, 127), strip.Color(60, 60, 127),
                              strip.Color(60, 60, 127), strip.Color(60, 60, 127)};                        
uint32_t modeChangeSequenceTen[12] = {strip.Color(40, 40, 84), strip.Color(40, 40, 84), strip.Color(40, 40, 84), strip.Color(40, 40, 84), strip.Color(40, 40, 84),
                              strip.Color(40, 40, 84), strip.Color(40, 40, 84), strip.Color(40, 40, 84), strip.Color(40, 40, 84), strip.Color(40, 40, 84),
                              strip.Color(40, 40, 84), strip.Color(40, 40, 84)};
uint32_t modeChangeSequenceEleven[12] = {strip.Color(20, 20, 42), strip.Color(20, 20, 42), strip.Color(20, 20, 42), strip.Color(20, 20, 42), strip.Color(20, 20, 42),
                              strip.Color(20, 20, 42), strip.Color(20, 20, 42), strip.Color(20, 20, 42), strip.Color(20, 20, 42), strip.Color(20, 20, 42),
                              strip.Color(20, 20, 42), strip.Color(20, 20, 42)};    



/****************************    START UP ANNIMATION   ***********************************/
uint32_t startSequenceZero[12] = {strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0),
                              strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0),
                              strip.Color(0, 0, 0), strip.Color(0, 0, 0)};
uint32_t startSequenceOne[12] = {strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0),
                              strip.Color(120, 120, 255), batteryColorIndex[0], strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0),
                              strip.Color(0, 0, 0), strip.Color(0, 0, 0)};
uint32_t startSequenceTwo[12] = {strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(120, 120, 255),
                              strip.Color(120, 120, 255), batteryColorIndex[0], batteryColorIndex[1], strip.Color(0, 0, 0), strip.Color(0, 0, 0),
                              strip.Color(0, 0, 0), strip.Color(0, 0, 0)};
uint32_t startSequenceThree[12] = {strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(120, 120, 255), strip.Color(120, 120, 255),
                              strip.Color(120, 120, 255), batteryColorIndex[0], batteryColorIndex[1], batteryColorIndex[2], strip.Color(0, 0, 0),
                              strip.Color(0, 0, 0), strip.Color(0, 0, 0)};
uint32_t startSequenceFour[12] = {strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(120, 120, 255), strip.Color(120, 120, 255), strip.Color(120, 120, 255),
                              strip.Color(120, 120, 255), batteryColorIndex[0], batteryColorIndex[1], batteryColorIndex[2], batteryColorIndex[3],
                              strip.Color(0, 0, 0), strip.Color(0, 0, 0)};
uint32_t startSequenceFive[12] = {strip.Color(0, 0, 0), strip.Color(120, 120, 255), strip.Color(120, 120, 255), strip.Color(120, 120, 255), strip.Color(120, 120, 255),
                              strip.Color(120, 120, 255), batteryColorIndex[0], batteryColorIndex[1], batteryColorIndex[2], batteryColorIndex[3],
                              batteryColorIndex[4], strip.Color(0, 0, 0)};
uint32_t startSequenceSix[12] = {strip.Color(120, 120, 255), strip.Color(120, 120, 255), strip.Color(120, 120, 255), strip.Color(120, 120, 255), strip.Color(120, 120, 255),
                              strip.Color(120, 120, 255), batteryColorIndex[0], batteryColorIndex[1], batteryColorIndex[2], batteryColorIndex[3],
                              batteryColorIndex[4], batteryColorIndex[5]};
uint32_t startSequenceSeven[12] = {strip.Color(0, 0, 0), strip.Color(120, 120, 255), strip.Color(120, 120, 255), strip.Color(120, 120, 255), strip.Color(120, 120, 255),
                              strip.Color(120, 120, 255), batteryColorIndex[0], batteryColorIndex[1], batteryColorIndex[2], batteryColorIndex[3],
                              batteryColorIndex[4], strip.Color(0, 0, 0)};
uint32_t startSequenceEight[12] = {strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(120, 120, 255), strip.Color(120, 120, 255), strip.Color(120, 120, 255),
                              strip.Color(120, 120, 255), batteryColorIndex[0], batteryColorIndex[1], batteryColorIndex[2], batteryColorIndex[3],
                              strip.Color(0, 0, 0), strip.Color(0, 0, 0)};
uint32_t startSequenceNine[12] = {strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(120, 120, 255), strip.Color(120, 120, 255),
                              strip.Color(120, 120, 255), batteryColorIndex[0], batteryColorIndex[1], batteryColorIndex[2], strip.Color(0, 0, 0),
                              strip.Color(0, 0, 0), strip.Color(0, 0, 0)};
uint32_t startSequenceTen[12] = {strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(120, 120, 255),
                              strip.Color(120, 120, 255), batteryColorIndex[0], batteryColorIndex[1], strip.Color(0, 0, 0), strip.Color(0, 0, 0),
                              strip.Color(0, 0, 0), strip.Color(0, 0, 0)};                      
uint32_t startSequenceEleven[12] = {strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0),
                              strip.Color(120, 120, 255), batteryColorIndex[0], strip.Color(0, 0, 0), strip.Color(0, 0, 0), strip.Color(0, 0, 0),
                              strip.Color(0, 0, 0), strip.Color(0, 0, 0)};                                                 
