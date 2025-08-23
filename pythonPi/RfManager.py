from enum import IntEnum
import sys
import argparse
import time
import struct
from pyrf24 import RF24, RF24_PA_LOW,  RF24_250KBPS, RF24_1MBPS, RF24_2MBPS
import sched
import yaml
import logging
import threading
import queue

from yaml import load, SafeLoader

from KayakDefines import StatusType
from KayakDefines import MotorCmd


class RfManager(threading.Thread):
  def __init__(self, configDictionary, logger, incomingQueue, outgoingQueue):
    super().__init__()
    self.mLogger = logger
    self.mLogger.info('**** Motor Manager Starting Up *****') 
    self.mLogger.info('*************************************') 

    # Initialize class member variables
    self.mIncomingQueue = incomingQueue
    self.mOutgoingQueue = outgoingQueue
    self.mRfReceiveDataFormatMsgOne = 'B?Bffffff'
    self.mRfReceiveDataFormatMsgTwo = 'Bffffff'
    self.mCurrentBatteryVolt = 100.0	# Variables to hold last RF messages received
    self.mCurrentMotorMode = 0
    self.mCurrentMotorSpeed = 0
    self.mPrevMotorMode = 0
    self.mPrevMotorSpeed = 0


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
    # self.mReadDataRate = self.mConfigurator['rfReadDataRateInMilliseconds']
    # print(self.mReadDataRate)
    # self.mWriteDataRate = self.mConfigurator['rfWriteDataRateInMilliseconds']
    # print(self.mWriteDataRate)

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
    self.mRadio.setRetries(4, 5)
    self.mRadio.setDataRate(RF24_2MBPS)  # Set data rate to 2Mbps for faster data transfer
    self.mRadio.dynamic_payloads = True
    # self.mRadio.setChannel(90) # Set channel to 90 for less interference
    # self.mRadio.set_auto_ack(False)

  def RfSend(self):
   
    # Read from motor status queue
    try :
      newCommand = self.mIncomingQueue.get(timeout=.05)
      print("Writing")
      
      status, data = struct.unpack('hf', newCommand)
      
      # Send to oar
      # self.mRadio.payload_size = struct.calcsize('hf') # Payload consists of status type and byte value
      txLength = struct.calcsize('hf')
      self.mRadio.listen = False  # ensures the nRF24L01 is in TX mode
      
      payload = struct.pack('hf', status, data)
      self.mLogger.info('RF sending status: %s', status)
      self.mLogger.info('RF sending data: %s', data)
      
      self.mLogger.info('Tx message timing: %.4f', time.time() - self.mPrevWriteTimeStamp)
      self.mPrevWriteTimeStamp = time.time()     # Capture previous data read timestamp in order to detect data rate issues
            
      result = self.mRadio.write(payload)
      
      if( not result):
        self.mLogger.error("RF transmission failed or timed out")
      
    except :
      print("Queue is empty after timeout. No new motor status")

    self.mScheduler.enter(self.mTxInterval, 1, self.RfSend)

  def RfReceive(self):
    print("Reading")
    # self.mRadio.payload_size = 28   # standard incoming data will be 28 bytes
    self.mRadio.listen = True 
   
    

#    activeRead = True
#    #has_payload, pipe_number = self.mRadio.available_pipe()
#    while(activeRead) :
    has_payload, pipe_number = self.mRadio.available_pipe()
    
    if has_payload :
        length = self.mRadio.getDynamicPayloadSize()  # grab the payload length
        #print(length)
        # fetch 1 payload from RX FIFO
        received = self.mRadio.read(length)  # also clears radio.irq_dr status flag
      
        # If this is first half of message
        if(struct.unpack("B", received[:1])[0] == 1) :
            self.mCurrentMsgNum, self.mCurrentMotorMode, self.mCurrentMotorSpeed, self.mCurrentRoll, self.mCurrentPitch, self.mCurrentYaw, self.mCurrentAccelX, self.mCurrentGyroX, self.mCurrentAccelY = struct.unpack(self.mRfReceiveDataFormatMsgOne, received)

            if(self.mPrevMotorMode != self.mCurrentMotorMode or self.mPrevMotorSpeed != self.mCurrentMotorSpeed) :
              # Send new motor commands to the MotorManager thread 
              motorModeCommand = struct.pack("?B", self.mCurrentMotorMode, self.mCurrentMotorSpeed)
              self.mOutgoingQueue.put(motorModeCommand)
              
            self.mPrevMotorMode = self.mCurrentMotorMode
            self.mPrevMotorSpeed = self.mCurrentMotorSpeed
            
            # Log IMU data
            self.mLogger.debug('Msg 1')
            self.mLogger.info('Mode: %s', self.mCurrentMotorMode)
            self.mLogger.info('Speed: %s', self.mCurrentMotorSpeed)
            self.mLogger.info('Roll: %.4f', self.mCurrentRoll)
            self.mLogger.info('Pitch: %.4f', self.mCurrentPitch)
            self.mLogger.info('Yaw: %.4f', self.mCurrentYaw)

            self.mDataRate = 1 / (time.time() - self.mPrevReadTimeStamp) 
            self.mLogger.info('Rx message timing: %.4f', time.time() - self.mPrevReadTimeStamp)
            self.mPrevReadTimeStamp = time.time()     # Capture previous data read timestamp in order to detect data rate issues
            #self.mLogger.info('data Rate: %.4f', self.mDataRate)
        else :
            self.mCurrentMsgNum, self.mCurrentAccelX, self.mCurrentGyroX, self.mCurrentAccelY, self.mCurrentGyroY, self.mCurrentAccelZ, self.mCurrentGyroZ = struct.unpack(self.mRfReceiveDataFormatMsgTwo, received)
          
            # Log IMU data
            self.mLogger.debug('Msg 2')
            self.mLogger.info('X Acceleration: %.4f', self.mCurrentAccelX)
            self.mLogger.info('X Gyroscope: %.4f', self.mCurrentGyroX)
            self.mLogger.info('Y Acceleration: %.4f', self.mCurrentAccelY)
            self.mLogger.info('Y Gyroscope: %.4f', self.mCurrentGyroY)
            self.mLogger.info('Z Acceleration: %.4f', self.mCurrentAccelZ)
            self.mLogger.info('Z Gyroscope: %.4f', self.mCurrentGyroZ)
    else :
      activeRead = False
        
      # If we don't get a new message within 7 seconds, trigger comms loss fault
      if(time.time() - self.mPrevReadTimeStamp > 7) : 
        commsLossCommand = struct.pack("?B", self.mCurrentMotorMode, 255)
        self.mOutgoingQueue.put(commsLossCommand)
        self.mLogger.info("Comms loss triggered")

    self.mScheduler.enter(self.mRxInterval, 1, self.RfReceive)

  def StartScheduler(self) :
    # Schedule the initial run of functions
    self.mScheduler.enter(0, 1, self.RfSend)
    self.mScheduler.enter(0, 1, self.RfReceive)
    
    # Start the scheduler
    self.mScheduler.run()
  
  def run(self) :
    self.StartScheduler()


#def testSend(count: int = 5):
#    radio.listen = False  # ensures the nRF24L01 is in TX mode
#    
#    #send fault cleared message
#    payload = struct.pack('hf', 6, 20)
#    result = radio.write(payload)
#
#    time.sleep(1)
#
#    # Send startup message
#    payload = struct.pack('hf', 1, 20)
#    result = radio.write(payload)
#    
#    time.sleep(1)
#    
#    # Send battery status message
#    payload = struct.pack('hf', 0, 20)
#    result = radio.write(payload)
#    
#    time.sleep(1)
#    
#    while count:
#        start_timer = time.monotonic_ns()  # start timer
#        result = radio.write(payload)
#        end_timer = time.monotonic_ns()  # end timer
#        if not result:
#            print("Transmission failed or timed out")
#        else:
#            print(
#                "Transmission successful! Time to Transmit:",
#                f"{(end_timer - start_timer) / 1000} us. Sent: {payload[0]}",
#            )
#        time.sleep(1)
#        count -= 1