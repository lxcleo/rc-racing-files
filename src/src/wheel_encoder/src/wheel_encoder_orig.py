#!/usr/bin/env python
import rospy
import math
import time
import numpy
import std_msgs.msg
from time import sleep
import pigpio
import time
import datetime
import RPi.GPIO as GPIO

#
# OH3144E or equivalent Hall effect sensor
#
# Pin 1 - 5V
# Pin 2 - Ground
# Pin 3 - GPIO pin
#
# The internal gpio pull-up is enabled so that the sensor
# normally reads high (FALLING edge).  It reads low (RISING edge) when a magnet is close.
#


##############################################
## FUNCTIONS
##############################################

# Setting Back Right Wheel Magnet Interaction
def sensorCallback_BRW(channel):
  global BRW_state
  global BRW_counter

  # high condition
  if GPIO.input(channel):
    # Check the state before. If state changed, increment tick.   
    if (BRW_state == 0):
      BRW_state = 1 # Change state to 1 (high).
      BRW_counter += 1
  # low condition     
  else:
    if (BRW_state == 1):
      BRW_state = 0
      BRW_counter += 1

# Setting Back Left Wheel Magnet Interaction
def sensorCallback_BLW(channel):
  global BLW_state
  global BLW_counter

# high condition
  if GPIO.input(channel):
    # Check the state before. If state changed, increment tick.   
    if (BLW_state == 0):
      BLW_state = 1 # Change state to 1 (high).
      BLW_counter += 1
  # low condition     
  else:
    if (BLW_state == 1):
      BLW_state = 0
      BLW_counter += 1

# Setting Front Right Wheel Magnet Interaction
def sensorCallback_FRW(channel):
  global FRW_state
  global FRW_counter

# high condition
  if GPIO.input(channel):
    # Check the state before. If state changed, increment tick.   
    if (FRW_state == 0):
      FRW_state = 1 # Change state to 1 (high).
      FRW_counter += 1
  # low condition     
  else:
    if (FRW_state == 1):
      FRW_state = 0
      FRW_counter += 1

# Setting Front Left Wheel Magnet Interaction
def sensorCallback_FLW(channel):
  global FLW_state
  global FLW_counter
# high condition
  if GPIO.input(channel):
    # Check the state before. If state changed, increment tick.   
    if (FLW_state == 0):
      FLW_state = 1 # Change state to 1 (high).
      FLW_counter += 1
  # low condition     
  else:
    if (FLW_state == 1):
      FLW_state = 0
      FLW_counter += 1

##################################################
## MAIN FUNCTION
##################################################
GPIO.setmode(GPIO.BOARD)

GPIO.setup(31, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(31, GPIO.BOTH, callback=sensorCallback_BRW, bouncetime=1)
GPIO.setup(29 , GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(29, GPIO.BOTH, callback=sensorCallback_BLW, bouncetime=1)
GPIO.setup(19 , GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(19, GPIO.BOTH, callback=sensorCallback_FRW, bouncetime=1)
GPIO.setup(21 , GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(21, GPIO.BOTH, callback=sensorCallback_FLW, bouncetime=1)

# Initialize Global Variables
BRW_state = 0
BRW_counter = 0.0
BLW_state = 0
BLW_counter = 0.0
FRW_state = 0
FRW_counter = 0.0
FLW_state = 0
FLW_counter = 0.0
time_count = 0

def main():
  # Defining Global Variables as wheel states
  global BRW_state
  global BRW_counter
  global BLW_state
  global BLW_counter
  global FRW_state
  global FRW_counter
  global FLW_state
  global FLW_counter
  global time_count
  global timer
  rospy.init_node('wheel_encoder');
  pub = rospy.Publisher('back_right_wheel', std_msgs.msg.String, queue_size=1) 
  pub1 = rospy.Publisher('back_left_wheel', std_msgs.msg.String, queue_size=1)
  pub2 = rospy.Publisher('front_right_wheel', std_msgs.msg.String, queue_size=1)
  pub3 = rospy.Publisher('front_left_wheel', std_msgs.msg.String, queue_size=1)
  pub4 = rospy.Publisher('all_wheel', std_msgs.msg.String, queue_size=1)
  start = time.time()
  while True:
      if ((time.time()-start) >= 10):
        # The code below outputs the wheel encoder data as a vector. This vector will enventually be of the form:
        # [ Back Right Wheel, Back Left Wheel, Front Right Wheel, Front Left Wheel ]
        #pub.publish(std_msgs.msg.String("Back Right: [ " + str(BRW_counter) + " ]"))
        #pub1.publish(std_msgs.msg.String("Back Left: [ " + str(BLW_counter) + " ]"))
        #pub2.publish(std_msgs.msg.String("Front Right: [ " + str(FRW_counter) + " ]"))  
        #pub3.publish(std_msgs.msg.String("Front Left: [ " + str(FLW_counter) + " ]"))    
        pub4.publish("rr :" + str(BRW_counter) + "+rl :" + str(BLW_counter) + "+fr :" + str(FRW_counter) + "+fl :" + str(FLW_counter))
        start = time.time() 
        # Reset counter and time
        BRW_counter = 0.0
        BLW_counter = 0.0
        FRW_counter = 0.0
        FLW_counter = 0.0  
      sensorCallback_BRW(31)
      sensorCallback_BLW(29)
      sensorCallback_FRW(19)
      sensorCallback_FLW(21) 
      
print("Setup GPIO pins as input for #16 (back right), #11 (back left), #31 (front right), and #16 (front left)")

if __name__=="__main__":
   main()
