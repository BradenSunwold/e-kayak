import time
import struct
import sched
import threading
import queue
from pyrf24 import RF24, RF24_PA_LOW, RF24_2MBPS
from EventTimer import EventTimer


class RfManager(threading.Thread):
    def __init__(self, configDictionary, logger, incomingQueue, outgoingQueue):
        super().__init__()
        self.mLogger = logger
        self.mLogger.info('**** RF Manager Starting Up *****')
        self.mLogger.info('*************************************')

        # Initialize class member variables
        self.mIncomingQueue = incomingQueue
        self.mOutgoingQueue = outgoingQueue
        self.mRfReceiveFormatManual = 'B?Bfff'
        self.mRfReceiveFormatAuto = 'B?Bffffff'
        self.mCurrentMotorMode = 0
        self.mCurrentMotorSpeed = 0

        self.mEventTimer = EventTimer(self.mLogger)

        # RX rate drop tracking
        self.mRxDropCount = 0
        self.mRxTotalCount = 0
        self.mRxDropLogInterval = 100  # Log every N received packets
        self.mRxDropThresholdHz = 17.0

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
        self.mNextRxTime = 0
        self.mNextTxTime = 0

        # Set up radio for coms
        self.mRadio = RF24(22, 0)
        self.mAddress = [b"1Node", b"2Node"]
        self.mRadioNumber = 1

        if not self.mRadio.begin():
            raise OSError("nRF24L01 hardware isn't responding")

        self.mRadio.set_pa_level(RF24_PA_LOW)  # RF24_PA_LOW tested in box at 15 feet
        self.mRadio.open_tx_pipe(self.mAddress[self.mRadioNumber])
        self.mRadio.open_rx_pipe(1, self.mAddress[not self.mRadioNumber])
        self.mRadio.setRetries(3, 7)
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

        # Drain all pending messages from the outgoing queue
        messages = []
        while True:
            try:
                messages.append(self.mIncomingQueue.get_nowait())
            except queue.Empty:
                break

        if messages:
            self.mRadio.listen = False
            for newCommand in messages:
                status, data = struct.unpack('hf', newCommand)
                payload = struct.pack('hf', status, data)
                result = self.mRadio.write(payload)

                self.mEventTimer.mark('rf_tx')
                self.mLogger.debug('RF sending status: %s', status)
                self.mLogger.debug('RF sending data: %s', data)

                if not result:
                    self.mLogger.error("RF transmission failed or timed out")

        self.mRadio.listen = True

        # Schedule next transmission
        self.mNextTxTime += self.mTxInterval
        self.mScheduler.enterabs(self.mNextTxTime, 1, self.RfSend)

    def RfReceive(self):
        if self.mShutdownRequested:
            return
        self.mRadio.listen = True

        has_payload, pipe_number = self.mRadio.available_pipe()

        if has_payload:
            length = self.mRadio.getDynamicPayloadSize()
            received = self.mRadio.read(length)

            try:
                # Use payload length to determine format (accounts for C struct padding)
                # RfManualMsg_t: sizeof = 16 bytes (1+1+1+pad+3×float)
                # RfAutoMsg_t:   sizeof = 28 bytes (1+1+1+pad+6×float)
                manualSize = struct.calcsize(self.mRfReceiveFormatManual)
                autoSize = struct.calcsize(self.mRfReceiveFormatAuto)

                if length == manualSize:
                    self.mCurrentMsgNum, self.mCurrentMotorMode, self.mCurrentMotorSpeed, self.mCurrentRoll, self.mCurrentPitch, self.mCurrentYaw = struct.unpack(self.mRfReceiveFormatManual, received)

                    self.mLogger.debug('Manual mode packet')
                    self.mLogger.debug('Speed: %s', self.mCurrentMotorSpeed)
                    self.mLogger.debug('Roll: %.4f', self.mCurrentRoll)
                    self.mLogger.debug('Pitch: %.4f', self.mCurrentPitch)
                    self.mLogger.debug('Yaw: %.4f', self.mCurrentYaw)
                elif length == autoSize:
                    self.mCurrentMsgNum, self.mCurrentMotorMode, self.mCurrentMotorSpeed, self.mCurrentAccelX, self.mCurrentGyroX, self.mCurrentAccelY, self.mCurrentGyroY, self.mCurrentAccelZ, self.mCurrentGyroZ = struct.unpack(self.mRfReceiveFormatAuto, received)

                    self.mLogger.debug('Auto mode packet')
                    self.mLogger.debug('Speed: %s', self.mCurrentMotorSpeed)
                    self.mLogger.debug('X Acceleration: %.4f', self.mCurrentAccelX)
                    self.mLogger.debug('X Gyroscope: %.4f', self.mCurrentGyroX)
                    self.mLogger.debug('Y Acceleration: %.4f', self.mCurrentAccelY)
                    self.mLogger.debug('Y Gyroscope: %.4f', self.mCurrentGyroY)
                    self.mLogger.debug('Z Acceleration: %.4f', self.mCurrentAccelZ)
                    self.mLogger.debug('Z Gyroscope: %.4f', self.mCurrentGyroZ)
                else:
                    self.mLogger.error('RfReceive unexpected packet length: %d bytes', length)
                    raise ValueError('unexpected packet length: %d' % length)

                # Always forward motor commands to MotorManager as heartbeat
                motorModeCommand = struct.pack("?B", self.mCurrentMotorMode, self.mCurrentMotorSpeed)
                self.mOutgoingQueue.put(motorModeCommand)

                result = self.mEventTimer.mark('rf_rx')
                if result is not None:
                    delta, rate, stats = result
                    self.mRxTotalCount += 1
                    if rate < self.mRxDropThresholdHz:
                        self.mRxDropCount += 1
                    if self.mRxTotalCount % self.mRxDropLogInterval == 0:
                        self.mLogger.info(
                            'rf_rx drops below %g Hz: %d / %d (%.1f%%)',
                            self.mRxDropThresholdHz,
                            self.mRxDropCount, self.mRxTotalCount,
                            100.0 * self.mRxDropCount / self.mRxTotalCount
                        )
            except Exception as e:
                self.mLogger.error('RfReceive malformed packet (%d bytes): %s', length, e)
        self.mNextRxTime += self.mRxInterval
        self.mScheduler.enterabs(self.mNextRxTime, 1, self.RfReceive)

    def StartScheduler(self):
        # Initialize absolute schedule times
        now = time.time()
        self.mNextTxTime = now
        self.mNextRxTime = now

        # Schedule the initial run of functions
        self.mScheduler.enterabs(self.mNextTxTime, 1, self.RfSend)
        self.mScheduler.enterabs(self.mNextRxTime, 1, self.RfReceive)

        # Start the scheduler
        self.mScheduler.run()

    def run(self):
        self.StartScheduler()
