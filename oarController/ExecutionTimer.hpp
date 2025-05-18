
#pragma once

#include <FreeRTOS_SAMD21.h>
#include <functional>

class ExecutionTimer
{
public:
  ExecutionTimer();
  ExecutionTimer(std::function<void(uint32_t)>);
  ~ExecutionTimer();
  
  void Start();
  void Stop();

private:
  uint32_t mPreviousTimeInTicks;
  std::function<void(uint32_t)> mCallback;
};