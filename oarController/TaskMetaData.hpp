
#pragma once

#include <stdint.h>
#include <FreeRTOS_SAMD21.h>
#include "MetaData.hpp"
#include "ExecutionTimer.hpp"
#include "TimingStats.hpp"

class TaskMetaData
{
public:
  TaskMetaData(TaskHandle_t& handle);

  // Getters 
  UBaseType_t GetStackUsageHighWaterMark();
  MetaData& GetMetaData();
  TimingStats& GetTaskExecutionTimeStats();
  TaskHandle_t& GetTaskHandle();
  ExecutionTimer& GetExecutionTimer();

private:
  MetaData mMetaData;

  ExecutionTimer mTaskExecutionTimer;
  TimingStats mTaskExecutionTimeStats;

  TaskHandle_t& mTaskHandle;                // Task associated with this metaData
  UBaseType_t mStackUsageHighWaterMark;
};



// ExecutionTimer readRfExecutionTime([readRfExecutionTimeStats](uint32_t val) { readRfExecutionTimeStats.UpdateAllStatsInTicks(val); });