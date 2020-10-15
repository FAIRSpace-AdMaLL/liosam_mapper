#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix


imu_pub = rospy.Publisher('/imu_correct', Imu, queue_size=10)
gps_pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=10)

def imu_callback(data):
    rospy.loginfo("pub imu")
    data.header.frame_id = "base_link"

    data.orientation_covariance = [
        0.1, 0.0, 0.0,
        0.0, 0.1, 0.0,
        0.0, 0.0, 0.11
    ]

    data.linear_acceleration_covariance = [
        0.05, 0.0, 0.0,
        0.0, 0.05, 0.0,
        0.0, 0.0, 0.05
    ]

    imu_pub.publish(data) 

def gps_callback(data):
    rospy.loginfo("pub gps")
    data.header.frame_id = "navsat_link"

    data.position_covariance = [
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    ]
    gps_pub.publish(data)
    
def listener():
    rospy.init_node('frame_fixer', anonymous=True)

    rospy.Subscriber("/imu/data", Imu, imu_callback)
    rospy.Subscriber("/gnss", NavSatFix, gps_callback)

    rospy.spin()

if __name__ == '__main__':
    listener()