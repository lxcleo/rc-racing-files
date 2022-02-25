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
def sensorCallback_BRW(Input_Sig):
  global BRW_counter, BRW_valid_edge, BRW_falling

  if BRW_falling == True: # falling edge detection
      # discard any subsequent falling events
      if BRW_valid_edge == True :
          # only the first event is valid
          # check if we have a logical low
          if GPIO.input(Input_Sig) == 0:
              BRW_counter = BRW_counter + 1
              BRW_valid_edge = False
      # check if we have a logical high (reached bottom of hysteresis range)
      if GPIO.input(Input_Sig) == 1:
      # reverse the edge detection
          BRW_falling = False
          BRW_valid_edge = True
          GPIO.remove_event_detect(Input_Sig)
          GPIO.add_event_detect(Input_Sig, GPIO.RISING, callback=sensorCallback_BRW, bouncetime=5)

  else: # rising edge detection
      # discard any subsequent rising events
      if BRW_valid_edge == True :
          # only the first event is valid
          # check if we have a logical high
          if GPIO.input(Input_Sig) == 1:
              BRW_counter = BRW_counter + 1
              BRW_valid_edge = False
      # check if we have a logical low (reached top of hysteresis range)
      if GPIO.input(Input_Sig) == 0:
          # reverse the edge detection
          BRW_falling = True
          BRW_valid_edge = True
          GPIO.remove_event_detect(Input_Sig)
          GPIO.add_event_detect(Input_Sig, GPIO.FALLING, callback=sensorCallback_BRW, bouncetime=5)

# Setting Back Left Wheel Magnet Interaction
def sensorCallback_BLW(Input_Sig):
  global BLW_counter, BLW_valid_edge, BLW_falling

  if BLW_falling == True: # falling edge detection
      # discard any subsequent falling events
      if BLW_valid_edge == True :
          # only the first event is valid
          # check if we have a logical low
          if GPIO.input(Input_Sig) == 0:
              BLW_counter = BLW_counter + 1
              BLW_valid_edge = False
      # check if we have a logical high (reached bottom of hysteresis range)
      if GPIO.input(Input_Sig) == 1:
      # reverse the edge detection
          BLW_falling = False
          BLW_valid_edge = True
          GPIO.remove_event_detect(Input_Sig)
          GPIO.add_event_detect(Input_Sig, GPIO.RISING, callback=sensorCallback_BLW, bouncetime=5)

  else: # rising edge detection
      # discard any subsequent rising events
      if BLW_valid_edge == True :
          # only the first event is valid
          # check if we have a logical high
          if GPIO.input(Input_Sig) == 1:
              BLW_counter = BLW_counter + 1
              BLW_valid_edge = False
      # check if we have a logical low (reached top of hysteresis range)
      if GPIO.input(Input_Sig) == 0:
          # reverse the edge detection
          BLW_falling = True
          BLW_valid_edge = True
          GPIO.remove_event_detect(Input_Sig)
          GPIO.add_event_detect(Input_Sig, GPIO.FALLING, callback=sensorCallback_BLW, bouncetime=5)

# Setting Front Right Wheel Magnet Interaction
def sensorCallback_FRW(Input_Sig):
  global FRW_counter, FRW_valid_edge, FRW_falling

  if FRW_falling == True: # falling edge detection
      # discard any subsequent falling events
      if FRW_valid_edge == True :
          # only the first event is valid
          # check if we have a logical low
          if GPIO.input(Input_Sig) == 0:
              FRW_counter = FRW_counter + 1
              FRW_valid_edge = False
      # check if we have a logical high (reached bottom of hysteresis range)
      if GPIO.input(Input_Sig) == 1:
      # reverse the edge detection
          FRW_falling = False
          FRW_valid_edge = True
          GPIO.remove_event_detect(Input_Sig)
          GPIO.add_event_detect(Input_Sig, GPIO.RISING, callback=sensorCallback_FRW, bouncetime=5)

  else: # rising edge detection
      # discard any subsequent rising events
      if FRW_valid_edge == True :
          # only the first event is valid
          # check if we have a logical high
          if GPIO.input(Input_Sig) == 1:
              FRW_counter = FRW_counter + 1
              FRW_valid_edge = False
      # check if we have a logical low (reached top of hysteresis range)
      if GPIO.input(Input_Sig) == 0:
          # reverse the edge detection
          FRW_falling = True
          FRW_valid_edge = True
          GPIO.remove_event_detect(Input_Sig)
          GPIO.add_event_detect(Input_Sig, GPIO.FALLING, callback=sensorCallback_FRW, bouncetime=5)

# Setting Front Left Wheel Magnet Interaction
def sensorCallback_FLW(Input_Sig):
  global FLW_counter, FLW_valid_edge, FLW_falling

  if FLW_falling == True: # falling edge detection
      # discard any subsequent falling events
      if FLW_valid_edge == True :
          # only the first event is valid
          # check if we have a logical low
          if GPIO.input(Input_Sig) == 0:
              FLW_counter = FLW_counter + 1
              FLW_valid_edge = False
      # check if we have a logical high (reached bottom of hysteresis range)
      if GPIO.input(Input_Sig) == 1:
      # reverse the edge detection
          FLW_falling = False
          FLW_valid_edge = True
          GPIO.remove_event_detect(Input_Sig)
          GPIO.add_event_detect(Input_Sig, GPIO.RISING, callback=sensorCallback_FLW, bouncetime=5)

  else: # rising edge detection
      # discard any subsequent rising events
      if FLW_valid_edge == True :
          # only the first event is valid
          # check if we have a logical high
          if GPIO.input(Input_Sig) == 1:
              FLW_counter = FLW_counter + 1
              FLW_valid_edge = False
      # check if we have a logical low (reached top of hysteresis range)
      if GPIO.input(Input_Sig) == 0:
          # reverse the edge detection
          FLW_falling = True
          FLW_valid_edge = True
          GPIO.remove_event_detect(Input_Sig)
          GPIO.add_event_detect(Input_Sig, GPIO.FALLING, callback=sensorCallback_FLW, bouncetime=5)

##################################################
## MAIN FUNCTION
##################################################
GPIO.setmode(GPIO.BOARD)



def main():
  # Defining Global Variables as wheel states
  global BRW_falling
  global BRW_counter
  global BLW_falling
  global BLW_counter
  global FRW_falling
  global FRW_counter
  global FLW_falling
  global FLW_counter
  global BRW_valid_edge
  global FRW_valid_edge
  global FLW_valid_edge
  global BLW_valid_edge
  # Initialize Global Variables
  BRW_valid_edge = True
  FRW_valid_edge = True
  FLW_valid_edge = True
  BLW_valid_edge = True
  BRW_counter = 0
  BLW_counter = 0
  FRW_counter = 0
  FLW_counter = 0
  GPIO.setup(11, GPIO.IN, pull_up_down=GPIO.PUD_UP)
  # define the event, based on the active high or low condition of the pin
  if GPIO.input(11) == 0:
     # active low
     GPIO.add_event_detect(11, GPIO.FALLING, callback=sensorCallback_BRW, bouncetime=5)
     BRW_falling = True
  else:
     # active high
     GPIO.add_event_detect(11, GPIO.RISING, callback=sensorCallback_BRW, bouncetime=5)
     BRW_falling = False

  GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_UP)
  # define the event, based on the active high or low condition of the pin
  if GPIO.input(18) == 0:
     # active low
     GPIO.add_event_detect(18, GPIO.FALLING, callback=sensorCallback_BLW, bouncetime=5)
     BLW_falling = True
  else:
     # active high
     GPIO.add_event_detect(18, GPIO.RISING, callback=sensorCallback_BLW, bouncetime=5)
     BLW_falling = False

  GPIO.setup(31 , GPIO.IN, pull_up_down=GPIO.PUD_UP)
  # define the event, based on the active high or low condition of the pin
  if GPIO.input(31) == 0:
     # active low
     GPIO.add_event_detect(31, GPIO.FALLING, callback=sensorCallback_FRW, bouncetime=5)
     FRW_falling = True
  else:
     # active high
     GPIO.add_event_detect(31, GPIO.RISING, callback=sensorCallback_FRW, bouncetime=5)
     FRW_falling = False

  GPIO.setup(16 , GPIO.IN, pull_up_down=GPIO.PUD_UP)
  # define the event, based on the active high or low condition of the pin
  if GPIO.input(16) == 0:
     # active low
     GPIO.add_event_detect(16, GPIO.FALLING, callback=sensorCallback_FLW, bouncetime=5)
     FLW_falling = True
  else:
     # active high
     GPIO.add_event_detect(16, GPIO.RISING, callback=sensorCallback_FLW, bouncetime=5)
     FLW_falling = False

  rospy.init_node('wheel_encoder');
  pub = rospy.Publisher('back_right_wheel', std_msgs.msg.String, queue_size=1) 
  pub1 = rospy.Publisher('back_left_wheel', std_msgs.msg.String, queue_size=1)
  pub2 = rospy.Publisher('front_right_wheel', std_msgs.msg.String, queue_size=1)
  pub3 = rospy.Publisher('front_left_wheel', std_msgs.msg.String, queue_size=1)
  pub4 = rospy.Publisher('all_wheel', std_msgs.msg.String, queue_size=1)
  start = time.time()

  while True:
      if ((time.time()-start) >= 0.025):
        # The code below outputs the wheel encoder data as a vector. This vector will enventually be of the form:
        # [ Back Right Wheel, Back Left Wheel, Front Right Wheel, Front Left Wheel ]
        pub.publish(std_msgs.msg.String("Back Right: [ " + str(BRW_counter) + " ]"))
        pub1.publish(std_msgs.msg.String("Back Left: [ " + str(BLW_counter) + " ]"))
        pub2.publish(std_msgs.msg.String("Front Right: [ " + str(FRW_counter) + " ]"))  
        pub3.publish(std_msgs.msg.String("Front Left: [ " + str(FLW_counter) + " ]"))    
        pub4.publish(std_msgs.msg.String(str(BRW_counter) + "d" + str(BLW_counter) + "d" + str(FRW_counter) + "d" + str(FLW_counter)))
        start = time.time() 
        # Reset counter and time
        BRW_counter = 0
        BLW_counter = 0
        FRW_counter = 0
        FLW_counter = 0 
      sensorCallback_BRW(11)
      sensorCallback_BLW(18)
      sensorCallback_FRW(31)
      sensorCallback_FLW(16) 
      
print("Setup GPIO pins as input for #11 (back right), #18 (back left), #31 (front right), and #16 (front left)")

if __name__=="__main__":
   main()
