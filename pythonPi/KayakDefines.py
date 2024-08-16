
from enum import IntEnum


class StatusType(IntEnum):
  eBatteryVolt = 0
  eStartup = 1
  eLowBattery = 2 
  eHighCurrent = 3 
  eComsLoss = 4
  eUnknownFault = 5
  eFaultCleared = 6
  
class MotorCmd(IntEnum):
  eMode = 0
  eRpm = 1