from enum import IntEnum
import sys
import argparse
import time
import struct
from yaml import load, SafeLoader


class FIR():
    def __init__(self, taps):
        self.mTaps = taps
        self.mSize = len(taps)
        
        self.mInputBuffer = [0] * self.mSize
        self.mCurrentOutput = 0.0
        self.mIndex = 0
        
        
    def Feed(self, dataIn) :
        
        # Store new data and update index
        self.mInputBuffer[self.mIndex] = dataIn
        self.mIndex += 1
        
        if(self.mIndex == self.mSize) :
            self.mIndex = 0
            
        # Reset output
        self.mCurrentOutput = 0
        
        # Feed new data through taps
        tempIndex = self.mIndex
        
        for i in range(self.mSize)  :
            if(tempIndex > 0) :
                tempIndex -= 1
            else :
                tempIndex = self.mSize - 1
            
            tempTap = self.mTaps[i]
            tempBuffValue = self.mInputBuffer[tempIndex]
            self.mCurrentOutput += tempTap * tempBuffValue
        
        return self.mCurrentOutput
            
        
    def Clear(self) :
        self.mCurrentOutput = 0
        self.mIndex = 0
        
        for i in range(self.mSize) :
            self.mInputBuffer[i] = 0
            
    
    def LastOutput(self) :
        return self.mCurrentOutput
    
    
    def Load(self, taps) :
        self.mTaps = taps
        self.mSize = taps.len()
        
        self.Clear()