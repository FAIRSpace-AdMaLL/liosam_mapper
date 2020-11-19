#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


imu = rospy.Publisher('/yaw/imu', Float64, queue_size=10)
lidar = rospy.Publisher('/yaw/lidar', Float64, queue_size=10)

# f_imu = open("/home/marwan/fyp/csv/imu_200.txt", "w")
# f_lidar = open("/home/marwan/fyp/csv/lidar_200.txt", "w")

def imu_callback(data):
    yaw_vel = tf.transformations.euler_from_quaternion([
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w])
    imu.publish(yaw_vel[2])
    # f_imu.write(str(yaw_vel[2])+ "\n")

def lidar_callback(data):
    yaw_vel = tf.transformations.euler_from_quaternion([
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w])
    lidar.publish(yaw_vel[2])
    # f_lidar.write(str(yaw_vel[2]) + "\n")

def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/odometry/imu", Odometry, imu_callback)
    rospy.Subscriber("/lio_sam/mapping/odometry", Odometry, lidar_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
