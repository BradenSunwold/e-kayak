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
import FIR

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
        self.mMotorSpeedManual = 0
        self.mMode = 0
        
        # Variables coming from ML model
        self.mEstimatedSpeed = 0.0
        
        # Variables coming from motor
        self.mFaultCode = 0
        self.mVoltage = 0.0
        self.mCurrentIn = 0.0
        self.mCurrentMotor = 0.0
        self.mDutyCycle = 0.0
        self.mTemp = 0.0
        self.mPowerMotor = 0.0
        self.mRpmFeedback = 0.0
        
        # Grab variables from configuration file
        self.mConfigurator = configDictionary
        self.mMaxRpms = self.mConfigurator['maxRpms']
        self.mNumberOfSpeedSettings = self.mConfigurator['numberOfSpeedSettings']
        # Saves array of same size as numberOfSpeedSettings
        self.mSpeedSettingToRpmMap = self.mConfigurator['speedSettingToRpmMapInPercent']     
        self.mUnderVoltage = self.mConfigurator['underVoltage']
        self.mMaxTemp = self.mConfigurator['maxTemp']
        self.mFaultClearPersistence = self.mConfigurator['faultClearTimeoutInMilliseconds']

        # Init RPM rate limiter
        self.mRateLimiterTaps = self.mConfigurator['rpmRateLimiterTaps']
        self.mRateLimiter = FIR.FIR(self.mRateLimiterTaps)
        self.mRateLimiter.Clear()
                
        # Set up scheduler
        self.mScheduler = sched.scheduler(time.time, time.sleep)
        self.mCommandReadInterval = self.mConfigurator['motorCommandReadRateInMilliseconds'] / 1000 
        self.mMotorWriteInterval = self.mConfigurator['motorWriteRateInMilliseconds'] / 1000
        self.mMotorReadInterval = self.mConfigurator['motorReadRateInMilliseconds'] / 1000
        
        # Init serial communication
        self.mSerial = serial.Serial ("/dev/ttyS0", 115200, timeout=0.05)


    # Read commands from both oar and ML module - decide which one to use later
    def ReadCommands(self) :
        # First, check if we have new data from the RF manager - manaul oar mode
        try :
            newCommand = self.mIncomingQueue.get(timeout=.05)
            
            # Check what msg we got
            tmpMode, tmpRpm = struct.unpack("?B", newCommand)
            
            numberSpeeds = self.mConfigurator['numberOfSpeedSettings']
            
            if(tmpRpm >= 0 and tmpRpm <= numberSpeeds) :
                self.mMode = tmpMode
                self.mMotorSpeedManual = tmpRpm
                
                self.mLogger.info('Oar Mode Comand: %s', self.mMode)
                self.mLogger.info('Oar Speed Command: %s', self.mMotorSpeedManual)
            else :
                self.mLogger.info("Received invalid motor commands")
                
        except queue.Empty:
            print("Queue is empty after timeout.")
            
        # Read ML module - Empty for now
        
        self.mScheduler.enter(self.mCommandReadInterval, 1, self.ReadCommands)
    
    # Update RPM speed based on received commands
    def WriteMotor(self) :
        newRpmCommand = 0
        
        # Check which mode we are in 
        if self.mMode == 0 :
            # We are in manual mode - use speed lookup table
            self.mRpm = (self.mSpeedSettingToRpmMap[self.mMotorSpeedManual] / 100) * self.mMaxRpms
        else :
            # Use estimated speed based on ML model
            # self.mRpm = self.mEstimatedSpeed
            self.mRpm = self.mMotorSpeedManual      # For now, force manual mode 
            
        # Send RPM command to the motor
        self.mLogger.info('Rpm Step Command: %s', self.mRpm)
        filteredRpm = self.mRateLimiter.Feed(self.mRpm)
        self.mLogger.info('Filtered RPM: %s', filteredRpm)
        
        filteredRpm *= 7
        self.mSerial.write(pyvesc.encode(SetRPM(int(filteredRpm))))
        
        self.mScheduler.enter(self.mMotorWriteInterval, 1, self.WriteMotor)
    
    def ReadMotor(self) :
        # Read status from motor and handle any faults
        self.mSerial.write(pyvesc.encode_request(GetValues))

        if self.mSerial.in_waiting > 78 :
            buffer = self.mSerial.read(79)
            (response, consumed) = pyvesc.decode(buffer)

            try : 
                # Store motor values 
                self.mFaultCode = response.mc_fault_code
                self.mVoltage = response.v_in
                self.mCurrentIn = response.current_in
                self.mCurrentMotor = response.current_motor
                self.mDutyCycle = response.duty_now
                self.mTemp = response.temp_fets
                self.mPowerMotorIn = self.mVoltage * self.mCurrentIn
                self.mRpmFeedback = response.rpm
                
                self.mLogger.info('*** MOTOR STATUS ***:')
                self.mLogger.info('Fet Temperature: %s', response.temp_fets)
                self.mLogger.info('Duty Cycle: %s', response.duty_now)
                self.mLogger.info('RPM: %s', response.rpm)
                self.mLogger.info('Input Voltage: %s', response.v_in)
                self.mLogger.info('Current In: %s', response.current_in)
                self.mLogger.info('Motor Current: %s', response.current_motor)
                self.mLogger.info('Amp Hours: %s', response.amp_hours)
                self.mLogger.info('Watt Hours: %s', response.watt_hours)
                self.mLogger.info('Fault Code: %s', response.mc_fault_code)       
                self.mLogger.info('Power: %s', self.mPowerMotorIn)  

            except:
                # Dont know what to do here
                pass
        self.mScheduler.enter(self.mMotorReadInterval, 1, self.ReadMotor)
    
    def StartScheduler(self) :
        # Schedule the initial run of functions
        print(self.mScheduler.enter)
        self.mScheduler.enter(0, 1, self.ReadCommands)
        self.mScheduler.enter(0, 1, self.WriteMotor)
        # self.mScheduler.enter(0, 1, self.ReadMotor)
    
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