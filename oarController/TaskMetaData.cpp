
#include "TaskMetaData.hpp"

TaskMetaData::TaskMetaData(TaskHandle_t& handle)
  :
  mMetaData(), 
  mTaskExecutionTimeStats(),
  mTaskExecutionTimer([this](uint32_t val) { mTaskExecutionTimeStats.UpdateAllStatsInTicks(val); }),
  mTaskHandle(handle),
  mStackUsageHighWaterMark(uxTaskGetStackHighWaterMark(handle))
{
}

// Getters
UBaseType_t TaskMetaData::GetStackUsageHighWaterMark()
{
  mStackUsageHighWaterMark = uxTaskGetStackHighWaterMark(mTaskHandle);

  return mStackUsageHighWaterMark;
}

MetaData& TaskMetaData::GetMetaData()
{
  return mMetaData;
}

TimingStats& TaskMetaData::GetTaskExecutionTimeStats()
{
  return mTaskExecutionTimeStats;
}

TaskHandle_t& TaskMetaData::GetTaskHandle()
{
  return mTaskHandle;
}

ExecutionTimer& TaskMetaData::GetExecutionTimer()
{
  return mTaskExecutionTimer;
}


