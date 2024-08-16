from enum import IntEnum
import sys
import argparse
import time
import struct
from pyrf24 import RF24, RF24_PA_LOW
import sched
import yaml
import logging
import threading
import queue

from yaml import load, SafeLoader

from RfManager import RfManager
from MotorManager import MotorManager
from KayakDefines import StatusType



configStream = open("config/config.yaml", 'r')
config = yaml.safe_load(configStream)

logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s', filemode='w')
loggerRf = logging.getLogger('loggerRf')
loggerMotor = logging.getLogger('loggerMotor')

# Create logging handlers
rfFileHandler = logging.FileHandler(config['rfManager']['rfLoggerFilePath'])
rfFileHandler.setLevel(logging.DEBUG)
motorFileHandler = logging.FileHandler(config['motorManager']['motorLoggerFilePath'])
motorFileHandler.setLevel(logging.DEBUG)

rfLoggerFormatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
rfFileHandler.setFormatter(rfLoggerFormatter)
motorLoggerFormatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
motorFileHandler.setFormatter(motorLoggerFormatter)

loggerRf.addHandler(rfFileHandler)
loggerMotor.addHandler(motorFileHandler)


# Create queues for thread comms
motorToRfQueue = queue.Queue()
rfToMotorQueue = queue.Queue()


# Create rfManager object
rfManager = RfManager(config['rfManager'], loggerRf, motorToRfQueue, rfToMotorQueue)
motorManager = MotorManager(config['motorManager'], loggerMotor, rfToMotorQueue, motorToRfQueue)


# Start all threads running
rfManager.start()
motorManager.start()

rfManager.join()
motorManager.join()



# Send out initial fault cleared / startup message
#rfManager.RfSend(StatusType.eFaultCleared, 0.0)
#rfManager.RfSend(StatusType.eStartup, 0.0)

#rfManager.StartScheduler()

#while(1) :
#  rfManager.RfReceive()

#batteryVoltage = 25.0

#rfManager.RfSend(StatusType.eFaultCleared, 0.0)
#for i in range(50) :
#  if (i < 25) :
#    batteryVoltage = batteryVoltage - .2
#  else :
#    batteryVoltage = batteryVoltage + .2
#  rfManager.RfSend(StatusType.eBatteryVolt, batteryVoltage)
