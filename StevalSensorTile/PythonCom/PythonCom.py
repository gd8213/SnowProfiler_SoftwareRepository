# -*- coding: utf-8 -*-
"""
Created on Thu Oct 28 20:45:36 2021

@author: Felbermayr Simon
"""
# Global definitions
Stop=0;
Start=1;

import serial
import time
start = time.time()
# your code
stop=0;

print("The time of the run:", stop - start)
try:
    s = serial.Serial(
        port='COM4',\
        baudrate=9600,\
        parity=serial.PARITY_ODD,\
        stopbits=serial.STOPBITS_ONE,\
        bytesize=serial.EIGHTBITS,\
            timeout=5)
    s.isOpen() # try to open port, if possible print message and proceed with 'while True:'
    print ("Port is opened!")
except IOError: # if port is already opened, close it and open it again and print message
  print ("Port was already open, was closed and opened again!")
  s.close()
  s.open()
  
    
print("connected to: " + s.portstr)
datafile = open("hexdatafile.txt","a+")
s.write(b'1') #start measurement
while True:
#    x = s.readline()
    stop = time.time()
    datafile.write(str(s.read(10000)))
    if stop-start>20:
        print('Measured time:',stop-start)
        datafile.close()
        break
        
s.write(b'0') #stop measurement
s.close()