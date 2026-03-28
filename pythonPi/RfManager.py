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
        self.mRadio.setRetries(4, 6)
        self.mRadio.setDataRate(RF24_2MBPS)  # Set data rate to 2Mbps for faster data transfer
        self.mRadio.dynamic_payloads = True

        # Shutdown flag
        self.mShutdownRequested = False

        # Diagnostics
        self.mRxCallCount = 0
        self.mRxPacketCount = 0
        self.mTxCallCount = 0
        self.mTxFailCount = 0

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
            t0 = time.time()
            self.mRadio.listen = False
            t1 = time.time()
            if (t1 - t0) > 0.05:
                self.mLogger.warning('listen=False took %.3fs', t1 - t0)

            for newCommand in messages:
                status, data = struct.unpack('hf', newCommand)
                payload = struct.pack('hf', status, data)
                tWrite = time.time()
                result = self.mRadio.write(payload)
                writeElapsed = time.time() - tWrite

                self.mEventTimer.mark('rf_tx')
                self.mLogger.debug('RF sending status: %s', status)
                self.mLogger.debug('RF sending data: %s', data)

                if not result:
                    self.mTxFailCount += 1
                    self.mLogger.error("RF TX failed (total fails: %d, write took %.3fs)", self.mTxFailCount, writeElapsed)
                if writeElapsed > 0.05:
                    self.mLogger.warning('radio.write() took %.3fs (result=%s)', writeElapsed, result)

            t2 = time.time()
            self.mLogger.debug('RfSend burst: %d msgs in %.3fs', len(messages), t2 - t0)

        tListen = time.time()
        self.mRadio.listen = True
        listenElapsed = time.time() - tListen
        if listenElapsed > 0.05:
            self.mLogger.warning('listen=True took %.3fs', listenElapsed)

        # Schedule next transmission
        self.mNextTxTime += self.mTxInterval
        self.mScheduler.enterabs(self.mNextTxTime, 1, self.RfSend)

    def RfReceive(self):
        if self.mShutdownRequested:
            return
        self.mRxCallCount += 1
        if self.mRxCallCount % 100 == 0:
            self.mLogger.info('RfReceive heartbeat: calls=%d, packets=%d, txFails=%d',
                              self.mRxCallCount, self.mRxPacketCount, self.mTxFailCount)

        tListen = time.time()
        self.mRadio.listen = True
        tAvail = time.time()
        if (tAvail - tListen) > 0.05:
            self.mLogger.warning('RX listen=True took %.3fs', tAvail - tListen)

        has_payload, pipe_number = self.mRadio.available_pipe()
        tAfterAvail = time.time()
        if (tAfterAvail - tAvail) > 0.05:
            self.mLogger.warning('RX available_pipe() took %.3fs', tAfterAvail - tAvail)

        if has_payload:
            tRead = time.time()
            length = self.mRadio.getDynamicPayloadSize()
            received = self.mRadio.read(length)
            readElapsed = time.time() - tRead
            if readElapsed > 0.05:
                self.mLogger.warning('RX read() took %.3fs (len=%d)', readElapsed, length)

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

                self.mRxPacketCount += 1
                self.mEventTimer.mark('rf_rx')
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
