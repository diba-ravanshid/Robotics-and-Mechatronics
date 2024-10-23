import time
import serial
import numpy as np
from vpython import *

arduino_data = serial.Serial('com9', 115200)
time.sleep(1)

toRad = np.pi/180.0
toDeg = 1/toRad

while True:
    while arduino_data.inWaiting() == 0:
        pass
    data_packet = arduino_data.readline()
    try:
        data_packet = str(data_packet, 'utf-8')
        split_packet = data_packet.split(",")
        w = float(split_packet[0])*toRad
        x = float(split_packet[1])*toRad
        y = float(split_packet[2])*toRad
        z = float(split_packet[3])*toRad
        print(w*toDeg, x*toDeg, y*toDeg, z*toDeg)
        
        # Change the attributes of your object to syncronize it with real time motions

    except:
        pass