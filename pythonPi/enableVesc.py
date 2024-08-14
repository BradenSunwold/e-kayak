from enum import IntEnum
import sys
import argparse
import time
import struct
import sched
import yaml
import logging
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)

# Enable motor controller
try :
    while True :
        GPIO.output(18, True)

except KeyboardInterrupt :
    GPIO.output(18, False)
    


