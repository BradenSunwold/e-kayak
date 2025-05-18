
#include "MetaData.hpp"

MetaData::MetaData()
  :
  mTimestampInTicks(xTaskGetTickCount()),
  mPreviousTimestampInTicks(0),
  mUpdateRateStats()
{
}

void MetaData::UpdateTimestamp()
{
  // Update times
  mPreviousTimestampInTicks = mTimestampInTicks;
  mTimestampInTicks = xTaskGetTickCount();

  // Update all stats
  uint32_t newPeriod = mTimestampInTicks - mPreviousTimestampInTicks;
  mUpdateRateStats.UpdateAllStatsInTicks(newPeriod);
}

// Getters
uint32_t MetaData::GetTimestampInTicks()
{
  return mTimestampInTicks;
}

double MetaData::GetTimestampInMs()
{
  double timeStampInMilliSec = static_cast<double>(mTimestampInTicks) * (1 /static_cast<double>(portTICK_PERIOD_MS));

  return timeStampInMilliSec;
}

 TimingStats& MetaData::GetUpdateRateStats()
 {
  return mUpdateRateStats;
 }



