
#include "TimingStats.hpp"


TimingStats::TimingStats()
  :
  mMinExecutionTimeInTicks(0xFFFFFFFFULL),
  mMaxExecutionTimeInTicks(0),
  mAverageExecutionTimeInTicks(0),
  mTimesSumInTicks(0),
  mTotalTimesCount(0)
{
}

void TimingStats::UpdateAllStatsInTicks(uint32_t newExecutionTimeInTicks)
{
  CalculateMinExecutionTimeInTicks(newExecutionTimeInTicks);
  CalculateMaxExecutionTimeInTicks(newExecutionTimeInTicks);
  CalculateAverageExecutionTimeInTicks(newExecutionTimeInTicks);
}

// Getters
uint32_t TimingStats::GetMaxTimeInTicks()
{
  return mMaxExecutionTimeInTicks;
}

double TimingStats::GetMaxTimeInMs()
{
  double maxTimeInMs = static_cast<double>(mMaxExecutionTimeInTicks) * (1 /static_cast<double>(portTICK_PERIOD_MS));

  return maxTimeInMs;
}

uint32_t TimingStats::GetMinTimeInTicks()
{
  return mMinExecutionTimeInTicks;
}

double TimingStats::GetMinTimeInMs()
{
  double minTimeInMs = static_cast<double>(mMinExecutionTimeInTicks) * (1 /static_cast<double>(portTICK_PERIOD_MS));

  return minTimeInMs;
}

uint32_t TimingStats::GetAverageTimeInTicks()
{
  return mAverageExecutionTimeInTicks;
}

double TimingStats::GetAverageTimeInMs()
{
  double averageTimeInMs = static_cast<double>(mAverageExecutionTimeInTicks) * (1 /static_cast<double>(portTICK_PERIOD_MS));

  return averageTimeInMs;
}

// Private
void TimingStats::CalculateMinExecutionTimeInTicks(uint32_t newExecutionTimeInTicks)
{
  if(newExecutionTimeInTicks < mMinExecutionTimeInTicks)
  {
    mMinExecutionTimeInTicks = newExecutionTimeInTicks;
  }
}

void TimingStats::CalculateMaxExecutionTimeInTicks(uint32_t newExecutionTimeInTicks)
{
  if(newExecutionTimeInTicks > mMaxExecutionTimeInTicks)
  {
    mMaxExecutionTimeInTicks = newExecutionTimeInTicks;
  }
}

void TimingStats::CalculateAverageExecutionTimeInTicks(uint32_t newExecutionTimeInTicks)
{
  mTimesSumInTicks += newExecutionTimeInTicks;
  mTotalTimesCount++;

  mAverageExecutionTimeInTicks = mTimesSumInTicks / mTotalTimesCount;
}

