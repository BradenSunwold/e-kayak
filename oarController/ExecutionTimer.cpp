
#include "ExecutionTimer.hpp"

ExecutionTimer::ExecutionTimer()
  :
  mPreviousTimeInTicks(xTaskGetTickCount()),
  mCallback(NULL)
{
}

ExecutionTimer::ExecutionTimer(std::function<void(uint32_t)> callback)
  :
  mPreviousTimeInTicks(xTaskGetTickCount()),
  mCallback(callback)
{
}

ExecutionTimer::~ExecutionTimer()
{
  uint32_t currentExecutionTimeInTicks = xTaskGetTickCount() - mPreviousTimeInTicks;

  mCallback(currentExecutionTimeInTicks);
}

void ExecutionTimer::Start()
{
  mPreviousTimeInTicks = xTaskGetTickCount();
}

void ExecutionTimer::Stop()
{
  uint32_t currentExecutionTimeInTicks = xTaskGetTickCount() - mPreviousTimeInTicks;

  mCallback(currentExecutionTimeInTicks);
}


