from enum import IntEnum
import time
import struct
import sched
import threading
import queue
import serial
import RPi.GPIO as GPIO
import pyvesc
from pyvesc import GetValues, SetRPM, SetCurrent
from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory
import FIR
import RCFilter
import PIDController

from KayakDefines import StatusType
from KayakDefines import VescFaultCodes
from KayakDefines import MotorMode


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
    FaultType.OVER_TEMP: StatusType.eOverTemp,
    FaultType.OVER_CURRENT: StatusType.eHighCurrent,
    FaultType.UNDER_VOLTAGE: StatusType.eLowBattery,
    FaultType.COMS_LOSS: StatusType.eComsLoss,
    FaultType.VESC_COMS_LOSS: StatusType.eVescComsLoss,
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
    def __init__(self, configDictionary, logger, incomingQueue, outgoingQueue,
                 imuQueue=None, oarImuQueue=None, mlQueue=None, influxWriter=None):
        super().__init__()
        self.mLogger = logger
        self.mInfluxWriter = influxWriter
        self.mLogger.info('**** Motor Manager Starting Up *****')
        self.mLogger.info('*************************************')

        # Init member variables
        self.mRpm = 0.0
        self.mIncomingQueue = incomingQueue
        self.mOutgoingQueue = outgoingQueue
        self.mImuQueue = imuQueue              # Kayak heading from ImuManager
        self.mOarImuQueue = oarImuQueue        # Oar pitch/roll/yaw from RfManager
        self.mMlQueue = mlQueue                # Stroke probability from MlManager

        # Variables coming from RF manager
        self.mMotorSpeedManual = 0
        self.mMode = MotorMode.TRAINING
        self.mPreviousMode = MotorMode.TRAINING

        # Variables coming from ML model
        self.mStrokeProbability = 0.0

        # Kayak IMU state
        self.mKayakHeading = 0.0

        # Oar IMU state
        self.mOarRoll = 0.0
        self.mOarPitch = 0.0
        self.mOarYaw = 0.0

        # Fin controller state
        self.mHeadingSetpoint = 0.0
        self.mHeadingInitialized = False  # True once first IMU reading seeds the setpoint
        self.mFinOpenLoop = False       # True when user is actively steering via oar pitch
        self.mFinAngle = 0.0            # Fin angle in degrees: -MAX to +MAX (positive = right turn)

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
        self.mMotorPolePairs = self.mConfigurator['motorPolePairs']
        self.mFaultConfig = self.mConfigurator['faultConfig']

        # Init RPM rate limiter (RC filter replaces FIR for simpler config)
        self.mRpmFilterTau = self.mConfigurator.get('rpmFilterTauInSeconds', 1.3)
        self.mMotorWriteInterval = self.mConfigurator['motorWriteRateInMilliseconds'] / 1000
        self.mRateLimiter = RCFilter.RCFilter(self.mRpmFilterTau, self.mMotorWriteInterval)
        self.mRateLimiter.Clear()

        # Auto mode config and asymmetric RC filter
        autoConfig = self.mConfigurator.get('autoMode', {})
        self.mAutoDeadbandRpm = autoConfig.get('deadbandRpm', 100)
        self.mAutoStrokeRpm = autoConfig.get('strokeRpm', 400)
        self.mStrokeProbabilityThreshold = autoConfig.get('strokeProbabilityThreshold', 0.75)
        self.mAutoRampUpTau = autoConfig.get('rampUpTauInSeconds', 0.5)
        self.mAutoRampDownTau = autoConfig.get('rampDownTauInSeconds', 2.0)
        self.mAutoRpmFilter = RCFilter.RCFilter(self.mAutoRampDownTau, self.mMotorWriteInterval)
        self.mAutoRpmFilter.Clear()

        # Fin controller config
        self.mFinConfig = self.mConfigurator.get('finController', {})
        self.mPitchDeadbandDeg = self.mFinConfig.get('pitchDeadbandDeg', 5.0)
        self.mOpenLoopGain = self.mFinConfig.get('openLoopGain', 1.0)       # Fin degrees per degree of oar pitch
        self.mMaxFinAngle = self.mFinConfig.get('maxFinAngleDeg', 30.0)     # Max fin deflection in degrees
        self.mServoNeutralAngle = 90.0                                       # Servo angle that corresponds to 0 fin deflection
        self.mFinFilterTau = self.mFinConfig.get('finFilterTauInSeconds', 0.3)

        # Heading PID controller config
        headingPid = self.mFinConfig.get('headingPid', {})
        self.mHeadingKp = headingPid.get('kP', 0.0)
        self.mHeadingKi = headingPid.get('kI', 0.0)
        self.mHeadingKd = headingPid.get('kD', 0.0)
        self.mHeadingIntegralLimit = headingPid.get('integralLimit', 30.0)
        self.mHeadingOutputMin = headingPid.get('outputMinDeg', -self.mMaxFinAngle)
        self.mHeadingOutputMax = headingPid.get('outputMaxDeg', self.mMaxFinAngle)

        # Init fin servo via gpiozero + pigpio for jitter-free PWM
        servoPin = self.mFinConfig.get('servoGpioPin', 12)
        self.mPiGpioFactory = PiGPIOFactory()
        self.mFinServo = AngularServo(
            servoPin,
            min_angle=0,
            max_angle=180,
            min_pulse_width=0.0005,   # 500us
            max_pulse_width=0.0025,   # 2500us
            pin_factory=self.mPiGpioFactory,
            initial_angle=None,
        )
        # Zero the fin on startup
        self._SetFinAngle(0.0)
        self.mLogger.info('Fin servo initialized on GPIO %s, zeroed to %.1f°',
                          servoPin, self.mServoNeutralAngle)

        # Set up scheduler
        self.mScheduler = sched.scheduler(time.time, time.sleep)
        self.mCommandReadInterval = self.mConfigurator['motorCommandReadRateInMilliseconds'] / 1000
        self.mMotorReadInterval = self.mConfigurator['motorReadRateInMilliseconds'] / 1000
        self.mFinControlInterval = self.mFinConfig.get('controlRateInMilliseconds', 50) / 1000
        self.mFinAngleFilter = RCFilter.RCFilter(self.mFinFilterTau, self.mFinControlInterval)
        self.mHeadingPid = PIDController.PIDController(
            self.mHeadingKp, self.mHeadingKi, self.mHeadingKd,
            self.mFinControlInterval,
            self.mHeadingOutputMin, self.mHeadingOutputMax,
            self.mHeadingIntegralLimit)
        self.mNextCommandReadTime = 0
        self.mNextMotorWriteTime = 0
        self.mNextMotorReadTime = 0
        self.mNextFinControlTime = 0

        # Init VESC enable GPIO
        self.mVescEnablePin = self.mConfigurator['vescEnableGpioPin']
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.mVescEnablePin, GPIO.OUT)
        GPIO.output(self.mVescEnablePin, True)
        self.mLogger.info('VESC enabled on GPIO %s', self.mVescEnablePin)
        time.sleep(4)   

        # Shutdown flag
        self.mShutdownRequested = False

        # Init serial communication
        self.mSerial = serial.Serial("/dev/ttyS0", 115200, timeout=0.05)
        self.mStartupTime = time.time()


    def _DetectFault(self, faultCode):
        """Check all fault sources and return the highest priority active fault."""
        # Software over-temp check
        if self.mTemp > self.mMaxTemp:
            return FaultType.OVER_TEMP

        # Software under-voltage check (guard against startup when mVoltage is 0)
        if self.mVoltage > 0 and self.mVoltage < self.mUnderVoltage:
            return FaultType.UNDER_VOLTAGE

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
        if self.mShutdownRequested:
            return
        # Drain the queue and only use the latest command. The oar sends
        # packets faster than ReadCommands runs, so the queue grows over time
        # if we only read one message per call.
        latestCommand = None
        while True:
            try:
                latestCommand = self.mIncomingQueue.get_nowait()
            except queue.Empty:
                break

        if latestCommand is not None:
            tmpMode, tmpRpm = struct.unpack('BB', latestCommand)

            numberSpeeds = self.mConfigurator['numberOfSpeedSettings']

            if(tmpRpm < numberSpeeds) :
                self.mMode = MotorMode(tmpMode)
                self.mMotorSpeedManual = tmpRpm
                self.mLastOarMessageTime = time.time()

                self.mLogger.debug('Oar Mode Command: %s', self.mMode.name)
                self.mLogger.debug('Oar Speed Command: %s', self.mMotorSpeedManual)
            else :
                self.mLogger.info("Received invalid motor commands")
        else:
            self.mLogger.debug("Command queue empty")

        # Read stroke probability from ML module
        if self.mMlQueue is not None:
            latestMl = None
            while True:
                try:
                    latestMl = self.mMlQueue.get_nowait()
                except queue.Empty:
                    break
            if latestMl is not None:
                (self.mStrokeProbability,) = struct.unpack('f', latestMl)
                self.mLogger.debug('Stroke probability: %.3f', self.mStrokeProbability)

        self.mNextCommandReadTime += self.mCommandReadInterval
        self.mScheduler.enterabs(self.mNextCommandReadTime, 1, self.ReadCommands)

    # Update RPM speed based on received commands
    def WriteMotor(self) :
        if self.mShutdownRequested:
            return
        # Check on faults
        if self.mActiveFault != FaultType.NONE:
            self.mRpm = 0.0
            if self.mFaultLatched:
                self.mLogger.info('Latching fault = 0 RPM')
            else:
                self.mLogger.info('Soft fault = 0 RPM')
        # Check which mode we are in and calculate RPM
        elif self.mMode == MotorMode.MANUAL:
            self.mRpm = (self.mSpeedSettingToRpmMap[self.mMotorSpeedManual] / 100) * self.mMaxRpms
            self.mLogger.debug('In Mode MANUAL')
        elif self.mMode == MotorMode.TRAINING:
            self.mRpm = 0.0
            self.mLogger.debug('In Mode TRAINING')
        elif self.mMode == MotorMode.AUTO:
            # Auto mode — on entry, clear filter so RPM ramps up from 0
            if self.mPreviousMode != MotorMode.AUTO:
                self.mAutoRpmFilter.Clear()
                self.mLogger.info('Entered AUTO mode')

            # Determine target RPM based on stroke detection
            strokeDetected = self.mStrokeProbability >= self.mStrokeProbabilityThreshold
            if strokeDetected:
                autoTargetRpm = self.mAutoStrokeRpm
                self.mAutoRpmFilter.SetTau(self.mAutoRampUpTau, self.mMotorWriteInterval)
            else:
                autoTargetRpm = self.mAutoDeadbandRpm
                self.mAutoRpmFilter.SetTau(self.mAutoRampDownTau, self.mMotorWriteInterval)

            self.mRpm = self.mAutoRpmFilter.Feed(autoTargetRpm)
            self.mLogger.debug('AUTO stroke=%.3f target=%d filtered_rpm=%.1f',
                               self.mStrokeProbability, autoTargetRpm, self.mRpm)

        # If we are in a startup, ignore all faults and commands - Force a reversal for X seconds
        if(self.mStartupLatched == True) :
            self.mRpm = (self.mSpeedSettingToRpmMap[1] / 100) * self.mMaxRpms * -1
            if(time.time() - self.mStartupTime > (self.mStartupReversalTimeout / 1000)) :
                self.mStartupLatched = False
                self.mLastVescResponseTime = time.time()
                self.mLogger.info('Exiting Startup Reversal')

        # Send RPM command to the motor
        # In auto mode, mRpm is already filtered by the asymmetric auto RC filter
        # so skip the manual rate limiter to avoid double-filtering
        self.mLogger.debug('Rpm Step Command: %s', self.mRpm)
        if self.mMode == MotorMode.AUTO:
            filteredRpm = self.mRpm
            self.mRateLimiter.mOutput = self.mRpm  # Keep manual filter synced for mode transitions
        else:
            filteredRpm = self.mRateLimiter.Feed(self.mRpm)
        self.mLogger.debug('Filtered RPM: %s', filteredRpm)

        if self.mInfluxWriter:
            self.mInfluxWriter.write_point("motor_cmd", {
                "commanded_rpm": self.mRpm,
                "filtered_rpm": filteredRpm,
                "speed_setting": self.mMotorSpeedManual,
                "mode": int(self.mMode),
                "max_rpms": float(self.mMaxRpms),
            })

        filteredRpm *= self.mMotorPolePairs
        self.mSerial.write(pyvesc.encode(SetRPM(int(filteredRpm))))

        self.mNextMotorWriteTime += self.mMotorWriteInterval
        self.mScheduler.enterabs(self.mNextMotorWriteTime, 1, self.WriteMotor)

    def ReadMotor(self) :
        if self.mShutdownRequested:
            return
        # Read status from motor
        self.mSerial.write(pyvesc.encode_request(GetValues))

        vescFaultCode = VescFaultCodes.FAULT_CODE_NONE
        gotVescResponse = False

        if self.mSerial.in_waiting > 78 :
            buffer = self.mSerial.read(79)

            try :
                (response, consumed) = pyvesc.decode(buffer)
                # Store motor values - decode fault code once
                vescFaultCode = int.from_bytes(response.mc_fault_code, byteorder='big')
                self.mVoltage = response.v_in + self.mVoltageOffset
                self.mCurrentIn = response.current_in
                self.mCurrentMotor = response.current_motor
                self.mDutyCycle = response.duty_now
                self.mTemp = response.temp_fets
                self.mPowerMotor = self.mVoltage * self.mCurrentIn
                self.mRpmFeedback = response.rpm / self.mMotorPolePairs   # Divide ERPM by # poles to get RPM
                self.mLastVescResponseTime = time.time()
                gotVescResponse = True

                # Log motor values
                self.mLogger.debug('*** MOTOR STATUS ***:')
                self.mLogger.debug('Fet Temperature: %s', response.temp_fets)
                self.mLogger.debug('Duty Cycle: %s', response.duty_now)
                self.mLogger.debug('RPM: %s', response.rpm / self.mMotorPolePairs)
                self.mLogger.debug('VESC measured Input Voltage: %s', response.v_in)
                self.mLogger.debug('Calibrated Input Voltage: %s', self.mVoltage)
                self.mLogger.debug('Current In: %s', response.current_in)
                self.mLogger.debug('Motor Current: %s', response.current_motor)
                self.mLogger.debug('Amp Hours: %s', response.amp_hours)
                self.mLogger.debug('Watt Hours: %s', response.watt_hours)
                self.mLogger.debug('Fault Code: %s', vescFaultCode)
                self.mLogger.debug('Power: %s', self.mPowerMotor)
                self.mLogger.debug('Active Fault: %s', self.mActiveFault.name)

                if self.mInfluxWriter:
                    self.mInfluxWriter.write_point("motor", {
                        "rpm": self.mRpmFeedback,
                        "duty_cycle": self.mDutyCycle,
                        "voltage_in": self.mVoltage,
                        "current_in": self.mCurrentIn,
                        "motor_current": self.mCurrentMotor,
                        "power": self.mPowerMotor,
                        "temp_fets": self.mTemp,
                        "amp_hours": response.amp_hours,
                        "watt_hours": response.watt_hours,
                        "fault_code": vescFaultCode,
                    }, tags={"fault": self.mActiveFault.name})

            except Exception as e:
                self.mLogger.error('ReadMotor exception: %s', e)

        # Skip fault detection during startup reversal - VESC may not respond yet
        if not self.mStartupLatched:
            self._UpdateFaultState(vescFaultCode, gotVescResponse)

        self.mNextMotorReadTime += self.mMotorReadInterval
        self.mScheduler.enterabs(self.mNextMotorReadTime, 1, self.ReadMotor)

    def _SetFinAngle(self, angleDeg):
        """Set the fin angle in degrees. 0 = straight, positive = right turn, negative = left turn.

        Clamps to ±mMaxFinAngle, converts to servo angle (90° = neutral), and writes to servo.
        """
        clamped = max(-self.mMaxFinAngle, min(self.mMaxFinAngle, angleDeg))
        self.mFinAngle = clamped
        servoAngle = self.mServoNeutralAngle - clamped
        self.mFinServo.angle = servoAngle

    def _ReadKayakImu(self):
        """Drain the kayak IMU queue and store the latest heading."""
        if self.mImuQueue is None:
            return
        latestPayload = None
        while True:
            try:
                latestPayload = self.mImuQueue.get_nowait()
            except queue.Empty:
                break
        if latestPayload is not None:
            (self.mKayakHeading,) = struct.unpack('f', latestPayload)
            self.mLogger.debug('Kayak heading: %.2f', self.mKayakHeading)

            if not self.mHeadingInitialized:
                self.mHeadingSetpoint = self.mKayakHeading
                self.mHeadingInitialized = True
                self.mLogger.info('Heading initialized to %.2f', self.mKayakHeading)

    def _ReadOarImu(self):
        """Drain the oar IMU queue and store the latest pitch/roll/yaw."""
        if self.mOarImuQueue is None:
            return
        latestPayload = None
        while True:
            try:
                latestPayload = self.mOarImuQueue.get_nowait()
            except queue.Empty:
                break
        if latestPayload is not None:
            self.mOarRoll, self.mOarPitch, self.mOarYaw = struct.unpack('fff', latestPayload)
            self.mLogger.debug('Oar IMU  roll=%.2f  pitch=%.2f  yaw=%.2f',
                               self.mOarRoll, self.mOarPitch, self.mOarYaw)

    def _UpdateHeadingSetpoint(self):
        """Pre-conditioning step: read all IMU data and determine heading setpoint + control mode.

        In manual mode:
          - If |oar_pitch| > deadband: open-loop steering. Fin deflects proportional to
            oar pitch (negative pitch = right turn). Heading setpoint is NOT updated so
            it retains the last latched value for when the user releases.
          - If |oar_pitch| <= deadband: latch current kayak heading as setpoint,
            switch to closed-loop heading hold.

        In auto mode (future):
          - ML algorithm provides heading setpoint directly.
        """
        self._ReadKayakImu()
        self._ReadOarImu()

        if self.mMode == MotorMode.MANUAL:
            # Transition into manual — latch current heading so we don't hold a stale setpoint
            if self.mPreviousMode != MotorMode.MANUAL:
                self.mHeadingSetpoint = self.mKayakHeading
                self.mHeadingPid.Reset()
                self.mFinAngleFilter.Clear()
                self.mLogger.info('Entered MANUAL from %s — latch heading=%.2f',
                                  self.mPreviousMode.name, self.mHeadingSetpoint)

            # Manual mode — oar pitch drives steering intent
            if abs(self.mOarPitch) > self.mPitchDeadbandDeg:
                # Open-loop: proportionally map oar pitch to fin angle in degrees
                # Subtract the deadband so effective pitch starts at 0 once outside the deadband
                # Positive oar pitch = left turn intent = negative fin angle (invert)
                self.mFinOpenLoop = True
                if self.mOarPitch > 0:
                    effectivePitch = self.mOarPitch - self.mPitchDeadbandDeg
                else:
                    effectivePitch = self.mOarPitch + self.mPitchDeadbandDeg
                self.mFinAngle = max(-self.mMaxFinAngle,
                                     min(self.mMaxFinAngle,
                                         effectivePitch * self.mOpenLoopGain))
                self.mLogger.debug('Setpoint STEERING  oar_pitch=%.2f  fin_angle=%.2f',
                                   self.mOarPitch, self.mFinAngle)
            else:
                # Closed-loop: latch current heading when user releases the pitch
                if self.mFinOpenLoop:
                    # Transition from steering to hold — latch now
                    self.mHeadingSetpoint = self.mKayakHeading
                    self.mHeadingPid.Reset()
                    self.mLogger.debug('Setpoint LATCH  heading=%.2f', self.mHeadingSetpoint)
                self.mFinOpenLoop = False
        elif self.mMode == MotorMode.TRAINING:
            # Training mode — no fin control, center fin
            self.mFinOpenLoop = False
            self.mFinAngle = 0.0
        else:
            # Auto mode — ML provides setpoint (placeholder)
            self.mFinOpenLoop = False

        self.mPreviousMode = self.mMode

    def _FinController(self):
        """Pure heading controller. Computes fin angle from setpoint and feedback, writes to servo.

        If open-loop (user actively steering): fin angle was already set by _UpdateHeadingSetpoint.
        If closed-loop (heading hold): compute error and drive fin to maintain setpoint.
        """
        if self.mFinOpenLoop:
            # Open-loop angle already computed in _UpdateHeadingSetpoint — smooth it
            filteredFinAngle = self.mFinAngleFilter.Feed(self.mFinAngle)
            self._SetFinAngle(filteredFinAngle)
            self.mLogger.debug('FinCtrl OPEN_LOOP  raw=%.2f  filtered=%.2f', self.mFinAngle, filteredFinAngle)
        else:
            # Closed-loop heading hold
            # Wrap-safe heading error: result in [-180, 180]
            headingError = (self.mHeadingSetpoint - self.mKayakHeading + 180) % 360 - 180

            # PID controller outputs fin angle in degrees
            finAngle = self.mHeadingPid.Update(headingError, self.mKayakHeading)

            # Smooth through the same fin angle filter so the transition from
            # open-loop decays naturally instead of snapping
            filteredFinAngle = self.mFinAngleFilter.Feed(finAngle)
            self._SetFinAngle(filteredFinAngle)
            self.mLogger.debug('FinCtrl CLOSED_LOOP  setpoint=%.2f  heading=%.2f  error=%.2f  pid=%.2f  fin=%.2f',
                               self.mHeadingSetpoint, self.mKayakHeading, headingError, finAngle, filteredFinAngle)

    def FinControlLoop(self):
        """Scheduled entry point for the fin control pipeline."""
        if self.mShutdownRequested:
            return

        self._UpdateHeadingSetpoint()

        if self.mMode == MotorMode.TRAINING or self.mMode == MotorMode.AUTO:
            # Training / Auto mode — force fin to center, skip PID
            self._SetFinAngle(0.0)
            self.mFinAngleFilter.Clear()
            self.mHeadingPid.Reset()
            self.mLogger.debug('FinCtrl %s  fin=0.00', self.mMode.name)
        else:
            self._FinController()

        if self.mInfluxWriter:
            self.mInfluxWriter.write_point("fin_controller", {
                "kayak_heading": self.mKayakHeading,
                "heading_setpoint": self.mHeadingSetpoint,
                "oar_pitch": self.mOarPitch,
                "oar_roll": self.mOarRoll,
                "oar_yaw": self.mOarYaw,
                "fin_angle": self.mFinAngle,
                "servo_angle": self.mServoNeutralAngle + self.mFinAngle,
                "open_loop": int(self.mFinOpenLoop),
                "pid_p_term": self.mHeadingPid.mLastPTerm,
                "pid_i_term": self.mHeadingPid.mLastITerm,
                "pid_d_term": self.mHeadingPid.mLastDTerm,
                "pid_output": self.mHeadingPid.mLastOutput,
                "pid_integral": self.mHeadingPid.mIntegral,
            })

        self.mNextFinControlTime += self.mFinControlInterval
        self.mScheduler.enterabs(self.mNextFinControlTime, 1, self.FinControlLoop)

    def StartScheduler(self) :
        # Reset timeout baselines so VESC boot delay doesn't trigger false coms loss
        now = time.time()
        self.mLastVescResponseTime = now
        self.mLastOarMessageTime = now
        self.mStartupTime = now

        # Initialize absolute schedule times
        self.mNextCommandReadTime = now
        self.mNextMotorWriteTime = now
        self.mNextMotorReadTime = now
        self.mNextFinControlTime = now

        # Schedule the initial run of functions
        self.mScheduler.enterabs(self.mNextCommandReadTime, 1, self.ReadCommands)
        self.mScheduler.enterabs(self.mNextMotorWriteTime, 1, self.WriteMotor)
        self.mScheduler.enterabs(self.mNextMotorReadTime, 1, self.ReadMotor)
        self.mScheduler.enterabs(self.mNextFinControlTime, 1, self.FinControlLoop)

        # Start the scheduler
        self.mScheduler.run()

    def shutdown(self):
        """Safely shut down the motor and VESC."""
        self.mLogger.info('Shutdown requested')
        self.mShutdownRequested = True

        # Command motor to zero
        try:
            self.mSerial.write(pyvesc.encode(SetCurrent(0)))
            self.mLogger.info('Motor set to 0 current')
        except Exception as e:
            self.mLogger.error('Failed to zero motor: %s', e)

        # Close serial port
        try:
            self.mSerial.close()
            self.mLogger.info('Serial port closed')
        except Exception as e:
            self.mLogger.error('Failed to close serial: %s', e)

        # Center fin servo before shutdown
        try:
            self._SetFinAngle(0.0)
            self.mLogger.info('Fin servo centered')
        except Exception as e:
            self.mLogger.error('Failed to center fin servo: %s', e)

        # Disable VESC
        try:
            GPIO.output(self.mVescEnablePin, False)
            GPIO.cleanup(self.mVescEnablePin)
            self.mLogger.info('VESC disabled on GPIO %s', self.mVescEnablePin)
        except Exception as e:
            self.mLogger.error('Failed to disable VESC GPIO: %s', e)

    def run(self):
        self.StartScheduler()
