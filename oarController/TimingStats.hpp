
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
  float GetMaxTimeInMs();

  uint32_t GetMinTimeInTicks();
  float GetMinTimeInMs();

  uint32_t GetAverageTimeInTicks();
  float GetAverageTimeInMs();

  uint32_t GetVarianceInTicks();
  float GetVarianceInMs();

  uint32_t GetStdDeviationInTicks();
  float GetStdDeviationInMs();

private:

  void CalculateMinExecutionTimeInTicks(uint32_t newExecutionTimeInTicks);
  void CalculateMaxExecutionTimeInTicks(uint32_t newExecutionTimeInTicks);
  void CalculateAverageExecutionTimeInTicks(uint32_t newExecutionTimeInTicks);
  void CalculateVarianceAndStdDevInTicks(uint32_t newExecutionTimeInTicks);

  uint32_t mMinExecutionTimeInTicks;
  uint32_t mMaxExecutionTimeInTicks;
  uint32_t mAverageExecutionTimeInTicks;

  float mVarianceInTicks;
  float mStdDeviationInTicks;

  uint64_t mTimesSumInTicks;
  uint64_t mTimesSquareSumInTicks; // sum of squares of execution times
  uint32_t mTotalTimesCount;

};