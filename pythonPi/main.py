import yaml
from enum import IntEnum
import sys
import argparse
import time
import struct
import schedule

from yaml import load, SafeLoader


configStream = open("config/config.yaml", 'r')
test = yaml.safe_load(configStream)

print(test['rfManager']['rfReadDataRateInMilliseconds'])
