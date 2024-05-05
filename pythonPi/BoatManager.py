from enum import IntEnum
import sys
import argparse
import time
import struct
from pyrf24 import RF24, RF24_PA_LOW
import schedule
import yaml

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
  def __init__(self, configDictionary):
    
    # Initialize class member variables
    self.mCurrentBatteryVolt = 100	# Variables to hold last RF messages received
    self.mCurrentMotorMode = 0
    self.mCurrentMotorSpeed = 0

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
    self.mRadio.payload_size = struct.calcsize('?Hfffffffff') # Receive consists of: mode (bool), speed (uint8_t), IMU (F, F, F, F, F, F, F, F, F)
    self.mRadio.listen = True 
#    print(struct.calcsize('?Hfffffffff'))
    
    has_payload, pipe_number = self.mRadio.available_pipe()
    if has_payload:
      length = self.mRadio.payload_size  # grab the payload length
      # fetch 1 payload from RX FIFO
      received = self.mRadio.read(length)  # also clears radio.irq_dr status flag
      # expecting a little endian float, thus the format string "<f"
      # received[:4] truncates padded 0s in case dynamic payloads are disabled
      #payload = struct.unpack("<f", received[:4])[0]
      #print(struct.unpack('?Hfffffffff', received))
      #print(struct.calcsize(received))
      # print details about the received packet
      #print(f"Received {length} bytes on pipe {pipe_number}: {payload}")


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

rfManager = RfManager(config['rfManager'])

#while(1) :
#  rfManager.RfReceive()

batteryVoltage = 25.0

rfManager.RfSend(StatusType.eFaultCleared, 0.0)
for i in range(50) :
  if (i < 25) :
    batteryVoltage = batteryVoltage - .2
  else :
    batteryVoltage = batteryVoltage + .2
  rfManager.RfSend(StatusType.eBatteryVolt, batteryVoltage)
