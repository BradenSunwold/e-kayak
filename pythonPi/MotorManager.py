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

from KayakDefines import StatusType
from KayakDefines import MotorCmd
from KayakDefines import VescFaultCodes


class MotorManager(threading.Thread):
    def __init__(self, configDictionary, logger, incomingQueue, outgoingQueue):
        super().__init__()
        self.mLogger = logger
        self.mLogger.info('**** Motor Manager Starting Up *****') 
        self.mLogger.info('*************************************') 
        
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
        self.mStatus = StatusType.eBatteryVoltPercentage
        self.mRfStatus = StatusType.eFaultCleared
        self.mFaultTime = 0
        self.mFaultLatched = False
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
        self.mVoltageOffset = self.mConfigurator['batteryCalibrationOffset']
        self.mMaxTemp = self.mConfigurator['maxTemp']
        self.mFaultClearPersistence = self.mConfigurator['faultClearTimeoutInMilliseconds']
        self.mStartupReversalTimeout = self.mConfigurator['startupReverseTimeInMilliseconds']
        self.mStartupLatched = True
        self.mStartupTime = time.time()

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
            tmpMode, tmpRpm = struct.unpack('?B', newCommand)
            
            numberSpeeds = self.mConfigurator['numberOfSpeedSettings']
            
            if(tmpRpm == 255) :
                # We have triggered a comms loss fault from the RF manager
                self.mRfStatus = StatusType.eComsLoss
                self.mLogger.info("Received comms loss")
            elif(tmpRpm >= 0 and tmpRpm <= numberSpeeds) :
                self.mMode = tmpMode
                self.mMotorSpeedManual = tmpRpm
                
                self.mRfStatus = StatusType.eFaultCleared   # Receiving valid cmds so clear coms fault
                
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
        # Check on faults 
        if(self.mFaultLatched) :
            self.mRpm = 0.0       # if any faults are latching, force to 0 no matter what
            self.mLogger.info('Latching fault = 0 RPM')
        elif (self.mRfStatus == StatusType.eComsLoss or self.mStatus == StatusType.eComsLoss or self.mStatus == StatusType.eHighCurrent or self.mStatus == StatusType.eLowBattery or self.mStatus == StatusType.eUnknownFault) :
            self.mRpm = 0.0
            self.mLogger.info('Soft fault = 0 RPM')
        # Check which mode we are in and calculate RPM
        elif self.mMode == 0 :
            # We are in manual mode - use speed lookup table
            self.mRpm = (self.mSpeedSettingToRpmMap[self.mMotorSpeedManual] / 100) * self.mMaxRpms
            self.mLogger.info('In Mode 0')
        else :
            # Use estimated speed based on ML model
            # self.mRpm = self.mEstimatedSpeed
            self.mMode = 0      # For now, force manual mode 
            self.mLogger.info('Force Mode 0')
            
        # If we are in a startup, ignore all faults and commands - Force a reversal for X seconds
        if(self.mStartupLatched == True) :
            self.mRpm = (self.mSpeedSettingToRpmMap[1] / 100) * self.mMaxRpms * -1
            if(time.time() - self.mStartupTime > (self.mStartupReversalTimeout / 1000)) :
                self.mStartupLatched = False
                self.mLogger.info('Exiting Startup Reversal')
            
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
                self.mVoltage = response.v_in + self.mVoltageOffset
                self.mCurrentIn = response.current_in
                self.mCurrentMotor = response.current_motor
                self.mDutyCycle = response.duty_now
                self.mTemp = response.temp_fets
                self.mPowerMotorIn = self.mVoltage * self.mCurrentIn
                self.mRpmFeedback = response.rpm / 6                    # Divide ERPM by # poles to get RPM
                
                # Log motor values
                self.mLogger.info('*** MOTOR STATUS ***:')
                self.mLogger.info('Fet Temperature: %s', response.temp_fets)
                self.mLogger.info('Duty Cycle: %s', response.duty_now)
                self.mLogger.info('RPM: %s', response.rpm / 6)
                self.mLogger.info('VESC measured Input Voltage: %s', response.v_in)
                self.mLogger.info('Calibrated Input Voltage: %s', self.mVoltage)
                self.mLogger.info('Current In: %s', response.current_in)
                self.mLogger.info('Motor Current: %s', response.current_motor)
                self.mLogger.info('Amp Hours: %s', response.amp_hours)
                self.mLogger.info('Watt Hours: %s', response.watt_hours)
                self.mLogger.info('Fault Code: %s', response.mc_fault_code)       
                self.mLogger.info('Power: %s', self.mPowerMotorIn)  
                self.mLogger.info('RF Status: %s', self.mRfStatus)
                self.mLogger.info('Motor Status: %s', self.mStatus)
                
                # Send out status to oar 
                if(self.mRfStatus != StatusType.eComsLoss and self.mStatus != StatusType.eComsLoss and self.mStatus != StatusType.eHighCurrent and self.mStatus != StatusType.eLowBattery and self.mStatus != StatusType.eUnknownFault) :
                    # Only send out battery and speed info if not faulted 
                    self.mStatus = StatusType.eBatteryVoltPercentage
                    self.mLogger.info('Sending Battery Status Voltage: %s', self.mVoltage)
                    
                    # Normalize battery voltages between 0 - 100% - equation based on voltage curves
                    kayakBatteryPercentage = (1.0234 * (self.mVoltage * self.mVoltage * self.mVoltage)) - (69.144 * (self.mVoltage * self.mVoltage))  + (1556 * self.mVoltage) - 11654
                    
                    if(kayakBatteryPercentage > 100) :
                        kayakBatteryPercentage = 100
                    elif(kayakBatteryPercentage < 0) :
                       kayakBatteryPercentage = 0
                       
                    self.mLogger.info('Sending Battery Status Voltage Percentage: %s', kayakBatteryPercentage)
                    payload = struct.pack('hf', self.mStatus, kayakBatteryPercentage) 
                    self.mOutgoingQueue.put(payload)
                    
                    # Send speed percentage
                    self.mStatus = StatusType.eSpeedPercentageReport
                    speedPercentage = (abs(self.mRpmFeedback) / self.mMaxRpms) * 100
                    
                    # Cap speed percentage to 0 - 100%
                    if(speedPercentage > 100) :
                        speedPercentage = 100
                    elif(speedPercentage < 0) :
                        speedPercentage = 0
                        
                    self.mLogger.info('Sending Speed Percentage: %s', speedPercentage)
                    payload = struct.pack('hf', self.mStatus, speedPercentage) 
                    self.mOutgoingQueue.put(payload)
                
                # Check on any faults - For some reason this is not entering when no faults are present
                if int.from_bytes(self.mFaultCode, byteorder='big') == VescFaultCodes.FAULT_CODE_NONE and self.mRfStatus != StatusType.eComsLoss:
                    self.mLogger.info('No Faults')
                    # If faults have been cleared for more than faultClearPersistence timer then update flag
                    if self.mFaultLatched == False and (self.mStatus == StatusType.eComsLoss or self.mStatus == StatusType.eHighCurrent or self.mStatus == StatusType.eLowBattery or self.mStatus == StatusType.eUnknownFault) : 
                        
                        # We had a fault that is cleared, now ensure it stays cleared for X seconds
                        if (time.time() - self.mFaultTime) > (self.mFaultClearPersistence / 1000) :
                            # If we are clearing high current fault then decrement max rpm 
                            if (self.mStatus == StatusType.eHighCurrent) :
                                self.mMaxRpms -= 5
                                self.mLogger.info('New Max RPM: %s', self.mMaxRpms)
                            
                            # Init fault cleared sequence
                            self.mStatus = StatusType.eFaultCleared       # Send fault cleared then startup commands to init oar fault reset
                            data = 0.0                                    # Pack emtpy data
                            
                            payload = struct.pack('hf', self.mStatus, data)
                            self.mOutgoingQueue.put(payload)
                            
                            self.mStartupLatched = True
                            self.mStartupTime = time.time()
                            self.mLogger.info('Entering Startup Reversal')
                            
                        else :
                            # Keep sending out fault to keep coms connected 
                            data = 0
                            payload = struct.pack('hf', self.mStatus, data)
                            self.mOutgoingQueue.put(payload)
                        
                else :
                    # We have an active fault - check which one and set flags accordingly
                    self.mFaultTime = time.time()
                    
                    # Check which fault - Let RF manager handling clearning comms loss fault, this is only motor faults
                    if(int.from_bytes(self.mFaultCode, byteorder='big') == VescFaultCodes.FAULT_CODE_UNDER_VOLTAGE) :
                        self.mStatus = StatusType.eLowBattery
                        self.mFaultLatched = True
                    elif(int.from_bytes(self.mFaultCode, byteorder='big') == VescFaultCodes.FAULT_CODE_ABS_OVER_CURRENT) :
                        self.mStatus = StatusType.eHighCurrent
                        self.mFaultLatched = True
                    elif(self.mRfStatus == StatusType.eComsLoss) :
                        self.mStatus = StatusType.eComsLoss
                        self.mFaultLatched = True
                    else :
                        self.mStatus = StatusType.eUnknownFault
                        self.mFaultLatched = True
                       
                    self.mLogger.info('Sending RF fault: ') 
                    self.mLogger.info('Kayak Fault: %s', self.mStatus)
                    
                    # Send out fault
                    data = 0
                    payload = struct.pack('hf', self.mStatus, data)
                    self.mOutgoingQueue.put(payload)

            except:
                # Dont know what to do here
                pass
                
        self.mScheduler.enter(self.mMotorReadInterval, 1, self.ReadMotor)
    
    def StartScheduler(self) :
        # Schedule the initial run of functions
        print(self.mScheduler.enter)
        self.mScheduler.enter(0, 1, self.ReadCommands)
        self.mScheduler.enter(0, 1, self.WriteMotor)
        self.mScheduler.enter(0, 1, self.ReadMotor)
    
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