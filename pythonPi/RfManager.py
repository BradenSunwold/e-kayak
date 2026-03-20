import time
import struct
import sched
import threading
import queue
from pyrf24 import RF24, RF24_PA_LOW, RF24_2MBPS


class RfManager(threading.Thread):
    def __init__(self, configDictionary, logger, incomingQueue, outgoingQueue):
        super().__init__()
        self.mLogger = logger
        self.mLogger.info('**** RF Manager Starting Up *****')
        self.mLogger.info('*************************************')

        # Initialize class member variables
        self.mIncomingQueue = incomingQueue
        self.mOutgoingQueue = outgoingQueue
        self.mRfReceiveDataFormatMsgOne = 'B?Bffffff'
        self.mRfReceiveDataFormatMsgTwo = 'Bffffff'
        self.mCurrentBatteryVolt = 100.0
        self.mCurrentMotorMode = 0
        self.mCurrentMotorSpeed = 0

        self.mPrevReadTimeStamp = time.time()
        self.mPrevWriteTimeStamp = time.time()
        self.mDataRate = 0

        # IMU member vars
        self.mCurrentMsgNum = 0
        self.mCurrentRoll = 0
        self.mCurrentPitch = 0
        self.mCurrentYaw = 0
        self.mCurrentAccelX = 0
        self.mCurrentAccelY = 0
        self.mCurrentAccelZ = 0
        self.mCurrentGyroX = 0
        self.mCurrentGyroY = 0
        self.mCurrentGyroZ = 0

        # Parse configs
        self.mConfigurator = configDictionary

        self.mScheduler = sched.scheduler(time.time, time.sleep)
        self.mRxInterval = self.mConfigurator['rfReadDataRateInMilliseconds'] / 1000
        self.mTxInterval = self.mConfigurator['rfWriteDataRateInMilliseconds'] / 1000

        # Set up radio for coms
        self.mRadio = RF24(22, 0)
        self.mAddress = [b"1Node", b"2Node"]
        self.mRadioNumber = 1

        if not self.mRadio.begin():
            raise OSError("nRF24L01 hardware isn't responding")

        self.mRadio.set_pa_level(RF24_PA_LOW)  # RF24_PA_LOW tested in box at 15 feet
        self.mRadio.open_tx_pipe(self.mAddress[self.mRadioNumber])
        self.mRadio.open_rx_pipe(1, self.mAddress[not self.mRadioNumber])
        self.mRadio.setRetries(4, 6)
        self.mRadio.setDataRate(RF24_2MBPS)  # Set data rate to 2Mbps for faster data transfer
        self.mRadio.dynamic_payloads = True

        # Shutdown flag
        self.mShutdownRequested = False

    def shutdown(self):
        """Signal the RF manager to stop."""
        self.mLogger.info('RF shutdown requested')
        self.mShutdownRequested = True

    def RfSend(self):
        if self.mShutdownRequested:
            return
        # Read from motor status queue
        try:
            newCommand = self.mIncomingQueue.get(timeout=.05)

            status, data = struct.unpack('hf', newCommand)

            self.mRadio.listen = False  # ensures the nRF24L01 is in TX mode

            payload = struct.pack('hf', status, data)
            self.mLogger.debug('RF sending status: %s', status)
            self.mLogger.debug('RF sending data: %s', data)

            self.mLogger.debug('Tx message timing: %.4f', time.time() - self.mPrevWriteTimeStamp)
            self.mPrevWriteTimeStamp = time.time()

            result = self.mRadio.write(payload)

            if not result:
                self.mLogger.error("RF transmission failed or timed out")

        except queue.Empty:
            self.mLogger.debug("Queue is empty after timeout. No new motor status")

        self.mScheduler.enter(self.mTxInterval, 1, self.RfSend)

    def RfReceive(self):
        if self.mShutdownRequested:
            return
        self.mRadio.listen = True

        has_payload, pipe_number = self.mRadio.available_pipe()

        if has_payload:
            length = self.mRadio.getDynamicPayloadSize()
            received = self.mRadio.read(length)

            # If this is first half of message
            if struct.unpack("B", received[:1])[0] == 1:
                self.mCurrentMsgNum, self.mCurrentMotorMode, self.mCurrentMotorSpeed, self.mCurrentRoll, self.mCurrentPitch, self.mCurrentYaw, self.mCurrentAccelX, self.mCurrentGyroX, self.mCurrentAccelY = struct.unpack(self.mRfReceiveDataFormatMsgOne, received)

                # Always forward motor commands to MotorManager as heartbeat
                motorModeCommand = struct.pack("?B", self.mCurrentMotorMode, self.mCurrentMotorSpeed)
                self.mOutgoingQueue.put(motorModeCommand)

                # Log IMU data
                self.mLogger.debug('Msg 1')
                self.mLogger.debug('Mode: %s', self.mCurrentMotorMode)
                self.mLogger.debug('Speed: %s', self.mCurrentMotorSpeed)
                self.mLogger.debug('Roll: %.4f', self.mCurrentRoll)
                self.mLogger.debug('Pitch: %.4f', self.mCurrentPitch)
                self.mLogger.debug('Yaw: %.4f', self.mCurrentYaw)

                self.mDataRate = 1 / (time.time() - self.mPrevReadTimeStamp)
                self.mLogger.debug('Rx message timing: %.4f', time.time() - self.mPrevReadTimeStamp)
                self.mPrevReadTimeStamp = time.time()
            else:
                self.mCurrentMsgNum, self.mCurrentAccelX, self.mCurrentGyroX, self.mCurrentAccelY, self.mCurrentGyroY, self.mCurrentAccelZ, self.mCurrentGyroZ = struct.unpack(self.mRfReceiveDataFormatMsgTwo, received)

                # Log IMU data
                self.mLogger.debug('Msg 2')
                self.mLogger.debug('X Acceleration: %.4f', self.mCurrentAccelX)
                self.mLogger.debug('X Gyroscope: %.4f', self.mCurrentGyroX)
                self.mLogger.debug('Y Acceleration: %.4f', self.mCurrentAccelY)
                self.mLogger.debug('Y Gyroscope: %.4f', self.mCurrentGyroY)
                self.mLogger.debug('Z Acceleration: %.4f', self.mCurrentAccelZ)
                self.mLogger.debug('Z Gyroscope: %.4f', self.mCurrentGyroZ)
        self.mScheduler.enter(self.mRxInterval, 1, self.RfReceive)

    def StartScheduler(self):
        # Schedule the initial run of functions
        self.mScheduler.enter(0, 1, self.RfSend)
        self.mScheduler.enter(0, 1, self.RfReceive)

        # Start the scheduler
        self.mScheduler.run()

    def run(self):
        self.StartScheduler()
