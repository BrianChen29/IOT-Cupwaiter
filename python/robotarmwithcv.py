import numpy as np
import cv2
import time
import sys
from adafruit_servokit import ServoKit
from gpiozero import DistanceSensor
from time import sleep
from signal import pause

#distance sensor
#sensor = DistanceSensor(23,24, max_distance = 1, threshold_distance = 0.5)


#robotarm
#Constants
nbPCAServo=3 
#Parameters
MIN_IMP  =[500, 500, 500]
MAX_IMP  =[2500, 2500, 2500]
MIN_ANG  =[0, 0, 0]
MAX_ANG  =[180, 180, 180]
#Objects
pca = ServoKit(channels=16)

#while True:
 #   print('Distance to nearest object is', sensor.distance,'m')
  #  sleep(1)
def init():

    for i in range(nbPCAServo):
        pca.servo[i+1].set_pulse_width_range(MIN_IMP[i] , MAX_IMP[i])

# function main 
def main():

    global angle
    pcaScenario();
    
# function pcaScenario 
def pcaScenario():
    """Scenario to test servo"""
    # go
    for j in range(MIN_ANG[0],2,1):
        print("Send angle {} to Servo1".format(j))
        pca.servo[1].angle = j
        time.sleep(0.01)
    for j in range(MIN_ANG[0],30,1):
        print("Send angle {} to Servo1".format(j))
        pca.servo[1].angle = j
        time.sleep(0.01)
    for j in range(MIN_ANG[0],60,1):
        print("Send angle {} to Servo2".format(j))
        pca.servo[2].angle = j
        time.sleep(0.01)
    for j in range(MIN_ANG[0],30,1):
        print("Send angle {} to Servo3".format(j))
        pca.servo[3].angle = j
        time.sleep(0.01)
    for j in range(10,MIN_ANG[0],-1):
        print("Send angle {} to Servo4".format(j))
        pca.servo[4].angle = j
        time.sleep(0.1)        
    #back
"""     for j in range(10,MIN_ANG[0],-1):
        print("Send angle {} to Servo2".format(j))
        pca.servo[2].angle = j
        time.sleep(0.1)
    for j in range(10,MIN_ANG[0],-1):
        print("Send angle {} to Servo3".format(j))
        pca.servo[3].angle = j
        time.sleep(0.1)
    #for j in range(MIN_ANG[0],30,1):
     #   print("Send angle {} to Servo1".format(j))
     #   pca.servo[4].angle = j
     #   time.sleep(0.01)
    # go back
    pca.servo[0].angle=None #disable channel
    time.sleep(0.5)
    sys.exit()
"""

if __name__ == '__main__':
    init()
    main()


class Capture(object):
    def _init_(self, deviceID=0):
        self.deviceID=deviceID
        self.capture =cv2.VideoCapture(self.deviceID)
    def read(self):
        frame = self.capture.read()
        frame = cv2.bilateralFilter(frame,5,50,100)
        image = Image.fromarray(frame)
        return image