#!/usr/bin/env python3.7
# -*- coding: utf-8 -*-
import rospy
import std_msgs
import time
import adafruit_bno08x
from math import atan2, sqrt, pi
from adafruit_extended_bus import ExtendedI2C as I2C
from adafruit_bno08x import (
    BNO_REPORT_STEP_COUNTER,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_GAME_ROTATION_VECTOR,
    BNO_REPORT_LINEAR_ACCELERATION
)
from adafruit_bno08x.i2c import BNO08X_I2C
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3, PoseStamped, WrenchStamped


# quat_real, quat_i, quat_j, quat_k
def find_heading(dqw, dqx, dqy, dqz):
    norm = sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz)
    dqw = dqw / norm
    dqx = dqx / norm
    dqy = dqy / norm
    dqz = dqz / norm
    ysqr = dqy * dqy
    t3 = +2.0 * (dqw * dqz + dqx * dqy)
    t4 = +1.0 - 2.0 * (ysqr + dqz * dqz)
    yaw_raw = atan2(t3, t4)
    yaw = yaw_raw * 180.0 / pi
    if yaw > 0:
        yaw = 360 - yaw
    else:
        yaw = abs(yaw)
    return yaw  # heading in 360 clockwise

if __name__ == '__main__':
        i2c = I2C(1)
        bno = BNO08X_I2C(i2c, address=0x4b)
        bno.enable_feature(BNO_REPORT_STEP_COUNTER)
        bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        bno.enable_feature(BNO_REPORT_GYROSCOPE)
        bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
        bno.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)
        bno.begin_calibration()
        counter = 0
        '''
        while counter < 10:
           time.sleep(0.5)
           calibration_status = bno.calibration_status
           print(
           "Magnetometer Calibration quality:",
           adafruit_bno08x.REPORT_ACCURACY_STATUS[calibration_status],
           " (%d)" % calibration_status,
           )
           if calibration_status >= 1:
               bno.save_calibration_data()
               print("calibration done")
               break
           else:
              print("Calibrating..." + str(counter))
           counter = counter+1
        '''
        #bno.save_calibration_data()
        rospy.init_node('bno_imu')
        pub = rospy.Publisher("yaw", std_msgs.msg.Float64, queue_size =1) 
        #pubtwo = rospy.Publisher("geo_yaw", std_msgs.msg.Float64, queue_size =1) 
        pubthree = rospy.Publisher("bnoaccel_x", std_msgs.msg.Float64, queue_size =1) 
        pubfour = rospy.Publisher("bnoaccel_y", std_msgs.msg.Float64, queue_size =1) 
        imu_pub = rospy.Publisher("bnoimu", Imu, queue_size =1)
        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'bnoimu'
        rate = rospy.Rate(60)  # 20hz
        while not rospy.is_shutdown():
                quat_i, quat_j, quat_k, quat_real = bno.quaternion # pylint:disable=no-member
                heading = find_heading(quat_real, quat_i, quat_j, quat_k)
                gyro_x, gyro_y, gyro_z = bno.gyro
                #geo_quat_i, geo_quat_j, geo_quat_k, geo_quat_real = bno.geomagnetic_quaternion
                #heading_geo = find_heading(geo_quat_real, geo_quat_i, geo_quat_j, geo_quat_k)
                accel_x, accel_y, accel_z = bno.linear_acceleration 
                msg.orientation.x = quat_i
                msg.orientation.y = quat_j
                msg.orientation.z = quat_k
                msg.orientation.w = quat_real
                msg.orientation_covariance[0] = quat_i * quat_i
                msg.orientation_covariance[0] = quat_j * quat_j
                msg.orientation_covariance[0] = quat_k * quat_k		
                msg.angular_velocity.x = gyro_x
                msg.angular_velocity.y = gyro_y
                msg.angular_velocity.z = gyro_z
                msg.angular_velocity_covariance[0] = gyro_x * gyro_x
                msg.angular_velocity_covariance[4] = gyro_y * gyro_y
                msg.angular_velocity_covariance[8] = gyro_z * gyro_z
                msg.linear_acceleration.x = accel_x
                msg.linear_acceleration.y = accel_y
                msg.linear_acceleration.z = accel_z
                msg.linear_acceleration_covariance[0] = accel_x * accel_x
                msg.linear_acceleration_covariance[4] = accel_y * accel_y
                msg.linear_acceleration_covariance[8] = accel_z * accel_z
                imu_pub.publish(msg)
                pub.publish(heading) 
                pubthree.publish(accel_x)
                pubfour.publish(accel_y)
                rate.sleep()
                #rospy.loginfo("heading: " + str(heading) + " accel x: " + str(accel_x)
                #+ " accel y: " + str(accel_y))
