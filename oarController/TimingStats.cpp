
#include "TimingStats.hpp"


TimingStats::TimingStats()
  :
  mMinExecutionTimeInTicks(0xFFFFFFFFULL),
  mMaxExecutionTimeInTicks(0),
  mAverageExecutionTimeInTicks(0),
  mVarianceInTicks(0),
  mStdDeviationInTicks(0),
  mTimesSumInTicks(0),
  mTimesSquareSumInTicks(0),
  mTotalTimesCount(0)
{
}

void TimingStats::UpdateAllStatsInTicks(uint32_t newExecutionTimeInTicks)
{
  CalculateMinExecutionTimeInTicks(newExecutionTimeInTicks);
  CalculateMaxExecutionTimeInTicks(newExecutionTimeInTicks);
  CalculateAverageExecutionTimeInTicks(newExecutionTimeInTicks);
  // CalculateVarianceAndStdDevInTicks(newExecutionTimeInTicks);
}

// Getters
uint32_t TimingStats::GetMaxTimeInTicks()
{
  return mMaxExecutionTimeInTicks;
}

float TimingStats::GetMaxTimeInMs()
{
  float maxTimeInMs = static_cast<double>(mMaxExecutionTimeInTicks) * (1 /static_cast<double>(portTICK_PERIOD_MS));

  return maxTimeInMs;
}

uint32_t TimingStats::GetMinTimeInTicks()
{
  return mMinExecutionTimeInTicks;
}

float TimingStats::GetMinTimeInMs()
{
  float minTimeInMs = static_cast<float>(mMinExecutionTimeInTicks) * (1 /static_cast<float>(portTICK_PERIOD_MS));

  return minTimeInMs;
}

uint32_t TimingStats::GetAverageTimeInTicks()
{
  return mAverageExecutionTimeInTicks;
}

float TimingStats::GetAverageTimeInMs()
{
  float averageTimeInMs = static_cast<float>(mAverageExecutionTimeInTicks) * (1 /static_cast<float>(portTICK_PERIOD_MS));

  return averageTimeInMs;
}

uint32_t TimingStats::GetVarianceInTicks()
{
  return static_cast<uint32_t>(std::llround(mVarianceInTicks));
}

float TimingStats::GetVarianceInMs()
{
  return mVarianceInTicks * (1.0 / static_cast<float>(portTICK_PERIOD_MS * portTICK_PERIOD_MS));
}

uint32_t TimingStats::GetStdDeviationInTicks()
{
  return static_cast<uint32_t>(std::llround(mStdDeviationInTicks));
}

float TimingStats::GetStdDeviationInMs()
{
  return mStdDeviationInTicks * (1.0 / static_cast<float>(portTICK_PERIOD_MS));
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
  mTimesSumInTicks += static_cast<uint64_t>(newExecutionTimeInTicks);
  mTotalTimesCount++;

  mAverageExecutionTimeInTicks = mTimesSumInTicks / mTotalTimesCount;
}

void TimingStats::CalculateVarianceAndStdDevInTicks(uint32_t newExecutionTimeInTicks)
{
  mTimesSquareSumInTicks += static_cast<uint64_t>(newExecutionTimeInTicks) *
                                    static_cast<uint64_t>(newExecutionTimeInTicks);

  if (mTotalTimesCount > 0)
  {
    float mean = static_cast<float>(mTimesSumInTicks) / mTotalTimesCount;
    float meanSquares = static_cast<float>(mTimesSquareSumInTicks) / mTotalTimesCount;

    mVarianceInTicks = meanSquares - (mean * mean);

    if (mVarianceInTicks < 0.0) mVarianceInTicks = 0.0; // numerical guard

    mStdDeviationInTicks = std::sqrt(mVarianceInTicks);
  }
}

