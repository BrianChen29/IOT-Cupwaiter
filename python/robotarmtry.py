#!/usr/bin/env python
# -*- coding: utf-8 -*-

#Libraries
import time    #https://docs.python.org/fr/3/library/time.html
import sys
from adafruit_servokit import ServoKit    #https://circuitpython.readthedocs.io/projects/servokit/en/latest/

#Constants
nbPCAServo=3 

#Parameters
MIN_IMP  =[500, 500, 500]
MAX_IMP  =[2500, 2500, 2500]
MIN_ANG  =[0, 0, 0]
MAX_ANG  =[180, 180, 180]

#Objects
pca = ServoKit(channels=16)

# function init 
def init():

    for i in range(nbPCAServo):
        pca.servo[i].set_pulse_width_range(MIN_IMP[i] , MAX_IMP[i])


# function main 
def main():

    global first
    first = input('input the test angle ')
    pcaFirstScenario();


# function pcaScenario 
def pcaFirstScenario():
    """Scenario to test servo"""
    # go
    for j in range(MIN_ANG[0],int(first),1):
       # print("Send angle {} to Servo1".format(j))
        pca.servo[0].angle = j
    # go back
    #for j in range(int(first),MIN_ANG[0],-1):
        #print("Send angle {} to Servo1".format(j))
      #  pca.servo[0].angle = j
        time.sleep(0.01)
    pca.servo[0].angle=None #disable channel
    time.sleep(0.5)
    sys.exit()


if __name__ == '__main__':
    init()
    main()