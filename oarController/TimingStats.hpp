
#pragma once

#include <stdint.h>
#include <FreeRTOS_SAMD21.h>

class TimingStats
{
public:
  TimingStats();

  void UpdateAllStatsInTicks(uint32_t newExecutionTimeInTicks);

  // Getters
  uint32_t GetMaxTimeInTicks();
  double GetMaxTimeInMs();

  uint32_t GetMinTimeInTicks();
  double GetMinTimeInMs();

  uint32_t GetAverageTimeInTicks();
  double GetAverageTimeInMs();

private:

  void CalculateMinExecutionTimeInTicks(uint32_t newExecutionTimeInTicks);
  void CalculateMaxExecutionTimeInTicks(uint32_t newExecutionTimeInTicks);
  void CalculateAverageExecutionTimeInTicks(uint32_t newExecutionTimeInTicks);

  uint32_t mMinExecutionTimeInTicks;
  uint32_t mMaxExecutionTimeInTicks;
  uint32_t mAverageExecutionTimeInTicks;

  uint32_t mTimesSumInTicks;
  uint32_t mTotalTimesCount;

};