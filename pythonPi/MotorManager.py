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


class FaultType(IntEnum):
    NONE = 0
    OVER_TEMP = 1
    OVER_CURRENT = 2
    UNDER_VOLTAGE = 3
    COMS_LOSS = 4
    VESC_COMS_LOSS = 5
    UNKNOWN = 6

# Maps internal fault type to the StatusType sent to the oar controller
FAULT_TO_STATUS = {
    FaultType.OVER_TEMP: StatusType.eUnknownFault,
    FaultType.OVER_CURRENT: StatusType.eHighCurrent,
    FaultType.UNDER_VOLTAGE: StatusType.eLowBattery,
    FaultType.COMS_LOSS: StatusType.eComsLoss,
    FaultType.VESC_COMS_LOSS: StatusType.eUnknownFault,
    FaultType.UNKNOWN: StatusType.eUnknownFault,
}

# Maps internal fault type to config key in faultConfig
FAULT_CONFIG_KEY = {
    FaultType.OVER_TEMP: 'overTemp',
    FaultType.OVER_CURRENT: 'overCurrent',
    FaultType.UNDER_VOLTAGE: 'underVoltage',
    FaultType.COMS_LOSS: 'comsLoss',
    FaultType.VESC_COMS_LOSS: 'vescComsLoss',
    FaultType.UNKNOWN: 'unknown',
}


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
        self.mVoltage = 0.0
        self.mCurrentIn = 0.0
        self.mCurrentMotor = 0.0
        self.mDutyCycle = 0.0
        self.mTemp = 0.0
        self.mPowerMotor = 0.0
        self.mRpmFeedback = 0.0

        # Fault tracking
        self.mActiveFault = FaultType.NONE
        self.mFaultLatched = False
        self.mFaultClearStartTime = None
        self.mLastVescResponseTime = time.time()
        self.mLastOarMessageTime = time.time()

        # Grab variables from configuration file
        self.mConfigurator = configDictionary
        self.mMaxRpms = self.mConfigurator['maxRpms']
        self.mMaxRpmsFloor = self.mMaxRpms - self.mConfigurator['maxRpmReduction']
        self.mNumberOfSpeedSettings = self.mConfigurator['numberOfSpeedSettings']
        # Saves array of same size as numberOfSpeedSettings
        self.mSpeedSettingToRpmMap = self.mConfigurator['speedSettingToRpmMapInPercent']
        self.mUnderVoltage = self.mConfigurator['underVoltage']
        self.mVoltageOffset = self.mConfigurator['batteryCalibrationOffset']
        self.mMaxTemp = self.mConfigurator['maxTemp']
        self.mFaultClearPersistence = self.mConfigurator['faultClearTimeoutInMilliseconds']
        self.mVescComsLossTimeout = self.mConfigurator['vescComsLossTimeoutInMilliseconds']
        self.mOarComsLossTimeout = self.mConfigurator['oarComsLossTimeoutInMilliseconds']
        self.mStartupReversalTimeout = self.mConfigurator['startupReverseTimeInMilliseconds']
        self.mStartupLatched = True
        self.mStartupTime = time.time()
        self.mFaultConfig = self.mConfigurator['faultConfig']

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


    def _DetectFault(self, faultCode):
        """Check all fault sources and return the highest priority active fault."""
        # Software over-temp check
        if self.mTemp > self.mMaxTemp:
            return FaultType.OVER_TEMP

        # VESC fault codes (ignore OVER_TEMP_MOTOR - no motor temp sensor installed)
        if faultCode == VescFaultCodes.FAULT_CODE_OVER_TEMP_FET:
            return FaultType.OVER_TEMP
        if faultCode == VescFaultCodes.FAULT_CODE_ABS_OVER_CURRENT:
            return FaultType.OVER_CURRENT
        if faultCode == VescFaultCodes.FAULT_CODE_UNDER_VOLTAGE:
            return FaultType.UNDER_VOLTAGE
        if faultCode != VescFaultCodes.FAULT_CODE_NONE:
            return FaultType.UNKNOWN

        # VESC serial loss - no response for longer than timeout
        if (time.time() - self.mLastVescResponseTime) > (self.mVescComsLossTimeout / 1000):
            return FaultType.VESC_COMS_LOSS

        # Oar coms loss - no message received within timeout
        if (time.time() - self.mLastOarMessageTime) > (self.mOarComsLossTimeout / 1000):
            return FaultType.COMS_LOSS

        return FaultType.NONE

    def _IsFaultLatching(self, faultType):
        """Check config to determine if this fault type latches or auto-recovers."""
        configKey = FAULT_CONFIG_KEY.get(faultType)
        if configKey and configKey in self.mFaultConfig:
            return self.mFaultConfig[configKey]['latching']
        return True  # Default to latching for safety

    def _SendFaultStatus(self):
        """Send current fault status to the oar."""
        statusType = FAULT_TO_STATUS.get(self.mActiveFault, StatusType.eUnknownFault)
        payload = struct.pack('hf', statusType, 0.0)
        self.mOutgoingQueue.put(payload)

    def _SendBatteryAndSpeedStatus(self):
        """Send battery percentage and speed percentage to the oar."""
        # Normalize battery voltages between 0 - 100% - equation based on voltage curves
        kayakBatteryPercentage = (1.0234 * (self.mVoltage ** 3)) - (69.144 * (self.mVoltage ** 2)) + (1556 * self.mVoltage) - 11654
        kayakBatteryPercentage = max(0, min(100, kayakBatteryPercentage))

        self.mLogger.info('Sending Battery Status Voltage: %s (%s%%)', self.mVoltage, kayakBatteryPercentage)
        payload = struct.pack('hf', StatusType.eBatteryVoltPercentage, kayakBatteryPercentage)
        self.mOutgoingQueue.put(payload)

        # Speed percentage
        speedPercentage = (abs(self.mRpmFeedback) / self.mMaxRpms) * 100
        speedPercentage = max(0, min(100, speedPercentage))

        self.mLogger.info('Sending Speed Percentage: %s', speedPercentage)
        payload = struct.pack('hf', StatusType.eSpeedPercentageReport, speedPercentage)
        self.mOutgoingQueue.put(payload)

    def _UpdateFaultState(self, vescFaultCode, gotVescResponse):
        """Run the fault state machine. Called every ReadMotor cycle regardless of VESC response."""
        currentFault = self._DetectFault(vescFaultCode)

        if currentFault == FaultType.NONE:
            if self.mActiveFault == FaultType.NONE:
                # No fault, normal operation - send battery and speed to oar
                if gotVescResponse:
                    self._SendBatteryAndSpeedStatus()

            elif self.mFaultLatched:
                # Latched fault stays until restart
                self.mLogger.info('Fault latched: %s', self.mActiveFault.name)
                self._SendFaultStatus()

            else:
                # Non-latching fault condition cleared, run persistence timer
                if self.mFaultClearStartTime is None:
                    self.mFaultClearStartTime = time.time()
                    self.mLogger.info('Fault condition cleared, starting persistence timer')

                elapsed = time.time() - self.mFaultClearStartTime
                if elapsed > (self.mFaultClearPersistence / 1000):
                    # Over-current recovery: decrement max RPM, latch if floor hit
                    if self.mActiveFault == FaultType.OVER_CURRENT:
                        self.mMaxRpms -= 5
                        if self.mMaxRpms <= self.mMaxRpmsFloor:
                            self.mMaxRpms = self.mMaxRpmsFloor
                            self.mFaultLatched = True
                            self.mLogger.info('Max RPM floor hit (%s), latching %s fault', self.mMaxRpmsFloor, self.mActiveFault.name)
                            self._SendFaultStatus()
                            return
                        self.mLogger.info('Over-current recovery: new Max RPM: %s', self.mMaxRpms)

                    # Fault cleared long enough - recover
                    self.mLogger.info('Fault cleared: %s', self.mActiveFault.name)
                    self.mActiveFault = FaultType.NONE
                    self.mFaultClearStartTime = None

                    # Send fault cleared and trigger startup reversal
                    payload = struct.pack('hf', StatusType.eFaultCleared, 0.0)
                    self.mOutgoingQueue.put(payload)

                    self.mStartupLatched = True
                    self.mStartupTime = time.time()
                    self.mLogger.info('Entering Startup Reversal')
                else:
                    # Still waiting for persistence - keep sending fault to oar
                    self._SendFaultStatus()
        else:
            # Fault is active
            self.mActiveFault = currentFault
            self.mFaultClearStartTime = None  # Reset persistence timer

            if self._IsFaultLatching(currentFault):
                self.mFaultLatched = True

            self.mLogger.info('Active fault: %s (latched: %s)', currentFault.name, self.mFaultLatched)
            self._SendFaultStatus()


    # Read commands from both oar and ML module - decide which one to use later
    def ReadCommands(self) :
        # First, check if we have new data from the RF manager - manual oar mode
        try :
            newCommand = self.mIncomingQueue.get(timeout=.05)

            # Check what msg we got
            tmpMode, tmpRpm = struct.unpack('?B', newCommand)

            numberSpeeds = self.mConfigurator['numberOfSpeedSettings']

            if(tmpRpm >= 0 and tmpRpm <= numberSpeeds) :
                self.mMode = tmpMode
                self.mMotorSpeedManual = tmpRpm
                self.mLastOarMessageTime = time.time()

                self.mLogger.info('Oar Mode Command: %s', self.mMode)
                self.mLogger.info('Oar Speed Command: %s', self.mMotorSpeedManual)
            else :
                self.mLogger.info("Received invalid motor commands")

        except queue.Empty:
            self.mLogger.debug("Command queue empty")

        # Read ML module - Empty for now

        self.mScheduler.enter(self.mCommandReadInterval, 1, self.ReadCommands)

    # Update RPM speed based on received commands
    def WriteMotor(self) :
        # Check on faults
        if self.mActiveFault != FaultType.NONE:
            self.mRpm = 0.0
            if self.mFaultLatched:
                self.mLogger.info('Latching fault = 0 RPM')
            else:
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
        # Read status from motor
        self.mSerial.write(pyvesc.encode_request(GetValues))

        vescFaultCode = VescFaultCodes.FAULT_CODE_NONE
        gotVescResponse = False

        if self.mSerial.in_waiting > 78 :
            buffer = self.mSerial.read(79)
            (response, consumed) = pyvesc.decode(buffer)

            try :
                # Store motor values - decode fault code once
                vescFaultCode = int.from_bytes(response.mc_fault_code, byteorder='big')
                self.mVoltage = response.v_in + self.mVoltageOffset
                self.mCurrentIn = response.current_in
                self.mCurrentMotor = response.current_motor
                self.mDutyCycle = response.duty_now
                self.mTemp = response.temp_fets
                self.mPowerMotorIn = self.mVoltage * self.mCurrentIn
                self.mRpmFeedback = response.rpm / 6                    # Divide ERPM by # poles to get RPM
                self.mLastVescResponseTime = time.time()
                gotVescResponse = True

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
                self.mLogger.info('Fault Code: %s', vescFaultCode)
                self.mLogger.info('Power: %s', self.mPowerMotorIn)
                self.mLogger.info('Active Fault: %s', self.mActiveFault.name)

            except Exception as e:
                self.mLogger.error('ReadMotor exception: %s', e)

        # Fault state machine always runs regardless of VESC response
        self._UpdateFaultState(vescFaultCode, gotVescResponse)

        self.mScheduler.enter(self.mMotorReadInterval, 1, self.ReadMotor)

    def StartScheduler(self) :
        # Schedule the initial run of functions
        self.mScheduler.enter(0, 1, self.ReadCommands)
        self.mScheduler.enter(0, 1, self.WriteMotor)
        self.mScheduler.enter(0, 1, self.ReadMotor)

        # Start the scheduler
        self.mScheduler.run()

    def run(self) :
        self.StartScheduler()
