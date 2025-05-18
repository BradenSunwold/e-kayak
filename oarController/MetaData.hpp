
#pragma once

#include <stdint.h>
#include <FreeRTOS_SAMD21.h>
#include "TimingStats.hpp"

class MetaData
{
public:
  MetaData();

  void UpdateTimestamp();

  // Getters 
  uint32_t GetTimestampInTicks();
  double GetTimestampInMs();
  TimingStats& GetUpdateRateStats();

private:
  uint32_t mTimestampInTicks;
  uint32_t mPreviousTimestampInTicks;

  TimingStats mUpdateRateStats;
};
