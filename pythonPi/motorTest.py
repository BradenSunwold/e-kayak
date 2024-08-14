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


# Test serial
ser = serial.Serial ("/dev/ttyS0", 115200, timeout=0.05)

#while(1) :
#    ser.write(b'Hello')

while True :
    try :
        ser.write(pyvesc.encode(SetRPM(3000)))

        ser.write(pyvesc.encode_request(GetValues))

        if ser.in_waiting > 78 :
            buffer = ser.read(79)
            (response, consumed) = pyvesc.decode(buffer)

            try : 
                print("NEW MESSAGE")
                print("Fet temp: ")
                print(response.temp_fets)
                print("Duty Cycle: ")
                print(response.duty_now)
                print("RPM: ")
                print(response.rpm)
                print("Voltage: ")
                print(response.v_in)
                print("Current In: ")
                print(response.current_in)

            except:
                # Dont know what to do here
                pass

        time.sleep(0.1)
    
    except KeyboardInterrupt :
        ser.write(pyvesc.encode(SetCurrent(0)))
        ser.close()


