#!/usr/bin/env python

## change config file nime from simple_control_v2.yaml -> simple_control.yaml
## waypoints don't have radian values for setting the final heading
##      Remove theta from all comments    
##
## Task description says they should use odom for getting to the waypoint. 
##      Maybe should use ground_truth to make sure that robot is navigating to correct positions
##      Can we visualize the waypoints in RViZ
##
##  def isWaypointReached(self, pose, wpt): -> Do we need the input arguments or will class variables suffice? Don't need to tell them to do it this way
##  
##  Why do we want the closest waypoint instead of just the first waypoint in the list?
## Task numbering 1, 5, 3, 4, 5

"""
Solution to home assignment 7 (Robot Control). Node to take a set of
waypoints and to drive a differential drive robot through those waypoints
using a simple PD controller and provided odometry data.

@author: 
@date: 
@input: Odometry as nav_msgs Odometry message
@output: body velocity commands as geometry_msgs Twist message.
"""

import math
import rospy
import numpy as np
from geometry_msgs.msg import Point, PoseStamped, Twist, Pose, Quaternion
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from tf import TransformListener

def quat_msg_to_tf(q):
    return [q.x, q.y, q.z, q.w]

class PoseErrorPublisher:
    def __init__(self, rate):
        self.rate = rospy.Rate(rate)
        
        self.pose_error_pub = rospy.Publisher("/pose_error", Pose, queue_size=10)
        self.pose_error_corrected_pub = rospy.Publisher("/pose_error_corrected", Pose, queue_size=10)
        
        self.pose_error = Pose()
        self.pose_error_corrected = Pose()
        
        self.gt_sub = rospy.Subscriber('/ground_truth', Odometry, callback=self.onGroundTruth, queue_size=10)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, callback=self.onOdom, queue_size=10)
        
        self.odometry = Odometry()
        self.ground_truth = Odometry()

        self.tf_listener = TransformListener()
        self.got_transformation = False
        self.got_gt = False
        self.got_odom = False

    def wrapAngle(self, angle):
        """
        helper function that returns angle wrapped between +- Pi
        @param: self
        @param: angle - angle  to be wrapped in [rad]
        @result: returns wrapped angle -Pi <= angle <= Pi
        """        
        if angle > np.pi:
            return angle - 2 * np.pi
        if angle < -np.pi:
            return angle + 2 * np.pi
        return angle

    def run(self):
        """
        Main loop of class.
        @param: self
        @result: runs the step function for controller update
        """

        while not rospy.is_shutdown():
            self.step()
            self.rate.sleep()

    def step(self):
        """
        Perform an iteration of the control loop.
        @param: self
        @result: publishes computed body velocity command.
        """

        if self.got_odom and self.got_gt:

            self.pose_error.position.x = self.odometry.pose.pose.position.x - self.ground_truth.pose.pose.position.x
            self.pose_error.position.y = self.odometry.pose.pose.position.y - self.ground_truth.pose.pose.position.y
            self.pose_error.position.z = self.odometry.pose.pose.position.z - self.ground_truth.pose.pose.position.z

            gt_rot = np.array(self.quaternionMsgToList(self.ground_truth.pose.pose.orientation), dtype=np.float64)
            odom_rot_inv = np.array(self.quaternionMsgToList(self.odometry.pose.pose.orientation), dtype=np.float64)
            odom_rot_inv[3] = -odom_rot_inv[3]
            odom_orientation_error = odom_rot_inv * gt_rot

            self.pose_error.orientation = Quaternion(*(quaternion_multiply(odom_rot_inv, gt_rot)))

        self.pose_error_pub.publish(self.pose_error)
        self.pose_error_corrected_pub.publish(self.pose_error_corrected)

    def getTransformation(self):
        try:
            (position, orientation_quat_list) = self.tf_listener.lookupTransform('/base_footprint', '/ground_truth_base_footprint', rospy.Time(0))
            self.pose_error_corrected.position = Point(*position)
            self.pose_error_corrected.orientation = Quaternion(*orientation_quat_list)
            self.got_transformation = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("No transform!")

    def onOdom(self, odom_msg):
        """
        Handles incoming odometry updates. Ok, if running on ground truth, otherwise need to add transform in the function (not present)
        @param: self
        @param odom_msg - odometry geometry message
        @result: update of relevant vehicle state variables
        """
        self.odometry = odom_msg
        self.got_odom = True
        # self.x = odom_msg.pose.pose.position.x
        # self.y = odom_msg.pose.pose.position.y
        # _, _, self.theta = euler_from_quaternion(quat_msg_to_tf(odom_msg.pose.pose.orientation))

    def onGroundTruth(self, odom_msg):
        """
        Handles incoming ground_truth odometry updates.
        @param: self
        @param odom_msg - odometry geometry message
        @result: update of relevant vehicle state variables
        """        
        self.ground_truth = odom_msg
        self.got_gt = True
    
    def quaternionMsgToList(self, q):
        return [q.x, q.y, q.z, q.w]

    def pointMsgToList(self, p):
        return [p.x, p.y, p.z]

if __name__ == '__main__':
    rospy.init_node("WPController")
    controller = PoseErrorPublisher(30)
    controller.run()