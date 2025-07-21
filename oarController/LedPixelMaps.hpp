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
