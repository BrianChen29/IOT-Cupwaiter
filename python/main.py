#Libraries
import time
import sys
from adafruit_servokit import ServoKit
from gpiozero import DistanceSensor
from time import sleep
from signal import pause

#distance sensor
sensor = DistanceSensor(23,24, max_distance = 1.5, threshold_distance = 0.1)


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
    if(sensor.when_in_range == True):
        pcaScenario()
    pause()
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
"""
"""
    pca.servo[0].angle=None #disable channel
    time.sleep(0.5)
    sensor.when_out_of_range = sys.exit()
"""

if __name__ == '__main__':
    init()
    main()