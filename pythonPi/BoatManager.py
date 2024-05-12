from enum import IntEnum
import sys
import argparse
import time
import struct
from pyrf24 import RF24, RF24_PA_LOW
import schedule
import yaml
import logging

from yaml import load, SafeLoader

class StatusType(IntEnum):
  eBatteryVolt = 0
  eStartup = 1
  eLowBattery = 2 
  eHighCurrent = 3 
  eComsLoss = 4
  eUnknownFault = 5
  eFaultCleared = 6

class RfManager:
  def __init__(self, configDictionary, logger):
    
    self.mLogger = logger
    self.mLogger.info('RF Manager Starting Up')

    # Initialize class member variables
    self.mRfReceiveDataFormatMsgOne = 'B?Bffffff'
    self.mRfReceiveDataFormatMsgTwo = 'Bffffff'
    self.mCurrentBatteryVolt = 100	# Variables to hold last RF messages received
    self.mCurrentMotorMode = 0
    self.mCurrentMotorSpeed = 0

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
    self.mReadDataRate = self.mConfigurator['rfReadDataRateInMilliseconds']
    print(self.mReadDataRate)
    self.mWriteDataRate = self.mConfigurator['rfWriteDataRateInMilliseconds']
    print(self.mWriteDataRate)
    
    # Set up radio for coms
    self.mRadio = RF24(22, 0)
    self.mAddress = [b"1Node", b"2Node"]
    self.mRadioNumber = 1;

    if not self.mRadio.begin():
      raise OSError("nRF24L01 hardware isn't responding")
    
    self.mRadio.set_pa_level(RF24_PA_LOW)  # RF24_PA_LOW tested in box at 15 feet
    self.mRadio.open_tx_pipe(self.mAddress[self.mRadioNumber])  
    self.mRadio.open_rx_pipe(1, self.mAddress[not self.mRadioNumber])  

  def RfSend(self, status, data):
    self.mRadio.payload_size = struct.calcsize('hf') # Payload consists of status type and float value
    self.mRadio.listen = False  # ensures the nRF24L01 is in TX mode
    
    payload = struct.pack('hf', status, data)
    result = self.mRadio.write(payload)

    time.sleep(.5)

  def RfReceive(self):
    self.mRadio.payload_size = 28   # standard incoming data will be 28 bytes
    self.mRadio.listen = True 
    
    has_payload, pipe_number = self.mRadio.available_pipe()
    if has_payload:
      length = self.mRadio.payload_size  # grab the payload length
      #print(length)
      # fetch 1 payload from RX FIFO
      received = self.mRadio.read(length)  # also clears radio.irq_dr status flag
      
      # If this is first half of message
      if(struct.unpack("B", received[:1])[0] == 1) :
          self.mCurrentMsgNum, self.mCurrentMotorMode, self.mCurrentMotorSpeed, self.mCurrentRoll, self.mCurrentPitch, self.mCurrentYaw, self.mCurrentAccelX, self.mCurrentGyroX, self.mCurrentAccelY = struct.unpack(self.mRfReceiveDataFormatMsgOne, received)
          
          # Log IMU data
          self.mLogger.debug('Msg 1')
          self.mLogger.info('Mode: %s', self.mCurrentMotorMode)
          self.mLogger.info('Speed: %s', self.mCurrentMotorSpeed)
          self.mLogger.info('Roll: %.4f', self.mCurrentRoll)
          self.mLogger.info('Pitch: %.4f', self.mCurrentPitch)
          self.mLogger.info('Yaw: %.4f', self.mCurrentYaw)
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



configStream = open("config/config.yaml", 'r')
config = yaml.safe_load(configStream)

logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s', filemode='w')
loggerRf = logging.getLogger('loggerRf')

# Create logging handlers
rfFileHandler = logging.FileHandler(config['rfManager']['rfLoggerFilePath'])
rfFileHandler.setLevel(logging.DEBUG)

rfLoggerFormatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
rfFileHandler.setFormatter(rfLoggerFormatter)

loggerRf.addHandler(rfFileHandler)


# Create rfManager object
rfManager = RfManager(config['rfManager'], loggerRf)

while(1) :
  rfManager.RfReceive()

batteryVoltage = 25.0

rfManager.RfSend(StatusType.eFaultCleared, 0.0)
for i in range(50) :
  if (i < 25) :
    batteryVoltage = batteryVoltage - .2
  else :
    batteryVoltage = batteryVoltage + .2
  rfManager.RfSend(StatusType.eBatteryVolt, batteryVoltage)
