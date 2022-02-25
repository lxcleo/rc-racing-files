#!/usr/bin/env python3.7
# -*- coding: utf-8 -*-
import rospy
import std_msgs
import time
import pigpio

def Read(pin):
    pi.set_mode(pin, pigpio.OUTPUT) #Set your chosen pin to an output
    pi.write(pin, 1) #Drive the output high, charging the capacitor
    time.sleep(0.03) #wait for the cap to charge. 
    count = 0 # set the counter to 0
    pi.set_mode(pin, pigpio.INPUT) # set pin to input 
    if pi.read(pin) == 0:
        count = 1
    return count

def tof_rl_callback(data):
    global rl
    rl = str(data)

def tof_rr_callback(data):
    global rr
    rr = str(data)

def tof_fl_callback(data):
    global fl
    fl = str(data)

#def tof_fr(data):
#    global fr
#    fr = str(data)

if __name__ == '__main__':
        global fl
        global fr
        global rr
        global rl
        fl = fr = rl = rr = 0.0
        pi = pigpio.pi()
        rospy.init_node('ir_lap_detector')
        pub1 = rospy.Publisher("lap_times", std_msgs.msg.Float32, queue_size =1) 
        #rospy.Subscriber("tof_rl", std_msgs.msg.Float32)
        #rospy.Subscriber("tof_rr", std_msgs.msg.Float32)
        #rospy.Subscriber("tof_fl", std_msgs.msg.Float32)
        #rospy.Subscriber("tof_fr", std_msgs.msg.Float32, tof_fr, queue_size =1)
        rate = rospy.Rate(20)  # 10hz
        pin = 4
        count = 0
        lap_time = 99999
        best_lap_time = 9999
        start_time = time.time()
        while not rospy.is_shutdown():
                ir = Read(pin)  
                rospy.Subscriber("tof_rl", std_msgs.msg.Float32, tof_rl_callback)  
                rospy.Subscriber("tof_rr", std_msgs.msg.Float32, tof_rr_callback)     
                rospy.Subscriber("tof_fl", std_msgs.msg.Float32, tof_fl_callback)
                #rospy.loginfo(str(ir))   
                if ir > 0 and count == 0:
                    rospy.loginfo("Start!")
                    count = count + 1
                    start_time = time.time()
                current_time = time.time()-start_time
                if ir > 0 and count > 0 and current_time > 5: 
                    lap_time = time.time()-start_time    
                    rospy.loginfo("Crossed Finishline at " + str(lap_time) + "seconds.")
                    start_time = time.time()                    
                    pub1.publish(lap_time)                                          
                    rospy.loginfo("dist_fl: " + str(fl))
                    rospy.loginfo("dist_rl: " + str(rl))
                if lap_time < best_lap_time:
                    best_lap_time = lap_time
                    rospy.loginfo("New Best Lap time at " + str(best_lap_time) + "seconds.")   
