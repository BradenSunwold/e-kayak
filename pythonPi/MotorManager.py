from enum import IntEnum
import sys
import argparse
import time
import struct
import sched
import yaml
import logging
import RPi.GPIO as GPIO
import serial
import pyvesc
from pyvesc import GetValues, SetRPM, SetCurrent
from yaml import load, SafeLoader
import threading
import queue

from KayakDefines import MotorCmd


class MotorManager(threading.Thread):
    def __init__(self, configDictionary, logger, incomingQueue, outgoingQueue):
        super().__init__()
        self.mLogger = logger
        self.mLogger.info('Motor Manager Starting Up') 
        
        # Init member variables
        self.mRpm = 0.0
        self.mIncomingQueue = incomingQueue
        self.mOutgoingQueue = outgoingQueue
        
        # Variables coming from RF manager
        self.mMotorSpeed = 0
        self.mMode = 0
        
        # Variables coming from motor
        self.mFaultCode = 0
        self.mVoltage = 0.0
        self.mCurrentIn = 0.0
        self.mCurrentMotor = 0
        self.mDutyCycle = 0.0
        self.mTemp = 0.0
        self.mPowerMotor = 0
        
        # Grab variables from configuration file
        self.mConfigurator = configDictionary
        self.mMaxCurrent = self.mConfigurator['maxCurrentInMilliAmps']
        self.mNumberOfSpeedSettings = self.mConfigurator['numberOfSpeedSettings']
        # Saves array of same size as numberOfSpeedSettings
        self.mSpeedSettingToCurrentMap = self.mConfigurator['speedSettingToCurrentMapInPercent']     
        self.mUnderVoltage = self.mConfigurator['underVoltage']
        self.mMaxTemp = self.mConfigurator['maxTemp']
        self.mFaultClearPersistence = self.mConfigurator['faultClearTimeoutInMilliseconds']
        self.mFaultLatchPersistence = ['faultLatchTimeoutInMilliseconds']
        
        # Set up scheduler
        self.mScheduler = sched.scheduler(time.time, time.sleep)
        self.mRxInterval = self.mConfigurator['motorCommandReadRateInMilliseconds'] / 1000 
        self.mTxInterval = self.mConfigurator['motorCommandWriteRateInMilliseconds'] / 1000
        
        # Init serial communication
        self.Serial = serial.Serial ("/dev/ttyS0", 115200, timeout=0.05)


    # Grab Rf commands from oar and write new speeds to the motor. Also read motor status back
    def WriteMotor(self) :
        # First, check if we have new data from the RF manager
        try :
            newCommand = self.mIncomingQueue.get(timeout=.05)
            
            # Check what msg we got
            tmpMode, tmpRpm = struct.unpack("?B", newCommand)
            
            numberSpeeds = self.mConfigurator['numberOfSpeedSettings']
            
            if(tmpRpm >= 0 and tmpRpm <= numberSpeeds) :
                self.mMode = tmpMode
                self.mRpm = tmpRpm
                
                self.mLogger.info('Mode: %s', self.mMode)
                self.mLogger.info('Speed: %s', self.mRpm)
            else :
                self.mLogger.info("Received invalid motor commands")
                
        except queue.Empty:
            print("Queue is empty after timeout.")
        
        self.mScheduler.enter(self.mTxInterval, 1, self.WriteMotor)
    
    # Support function to manage any fault conditions from the motor
    def FaultManager(self) :
        print("test")
    
    def StartScheduler(self) :
        # Schedule the initial run of functions
        print(self.mScheduler.enter)
        self.mScheduler.enter(0, 1, self.WriteMotor)
        #self.mScheduler.enter(0, 1, self.RfReceive)
    
        # Start the scheduler
        self.mScheduler.run()
  
    def run(self) :
        self.StartScheduler()
        

#while(1) :
#    ser.write(b'Hello')

# while True :
#     try :
#         ser.write(pyvesc.encode(SetRPM(3000)))

#         ser.write(pyvesc.encode_request(GetValues))

#         if ser.in_waiting > 78 :
#             buffer = ser.read(79)
#             (response, consumed) = pyvesc.decode(buffer)

#             try : 
#                 print("NEW MESSAGE")
#                 print("Fet temp: ")
#                 print(response.temp_fets)
#                 print("Duty Cycle: ")
#                 print(response.duty_now)
#                 print("RPM: ")
#                 print(response.rpm)
#                 print("Voltage: ")
#                 print(response.v_in)
#                 print("Current In: ")
#                 print(response.current_in)

#             except:
#                 # Dont know what to do here
#                 pass

#         time.sleep(0.1)
    
#     except KeyboardInterrupt :
#         ser.write(pyvesc.encode(SetCurrent(0)))
#         ser.close()