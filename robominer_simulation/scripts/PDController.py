#!/usr/bin/env python

import math
import rospy
import numpy as np
from geometry_msgs.msg import Point, PoseStamped, Twist
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

def quat_msg_to_tf(q):
    return [q.x, q.y, q.z, q.w]

class PDController:
    def __init__(self, rate):
        self.rate = rospy.Rate(rate)
        [self.p_x, self.p_y, self.p_angular] = rospy.get_param('/controller/Kp')
        [self.d_x, self.d_y, self.d_angular] = rospy.get_param('/controller/Kd')
        [self.i_x, self.i_y, self.i_angular] = rospy.get_param('/controller/Ki')
        self.distance_margin = rospy.get_param('/mission/distance_margin')
        self.heading_margin = rospy.get_param('/mission/heading_margin')
        self.waypoints = rospy.get_param('/mission/waypoints')
        self.marker_pub = rospy.Publisher("/waypoints", Marker, queue_size=1)
        self.sub_goal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.onGoal)

        self.odom_sub = rospy.Subscriber("/ground_truth", Odometry, callback=self.onOdom, queue_size=1)
        self.vel_cmd_msg = Twist()
        self.vel_cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.x, self.y, self.theta = [0,0,0]
        self.prev_heading_diff = 0
        self.prev_dist_x = 0
        self.prev_dist_y = 0
        self.headin_integ = 0
        self.dist_x_integ = 0
        self.dist_y_integ = 0
        self.vel_limit_xy = rospy.get_param('/controller/vel_limits_xy')
        
        [self.min_vel_theta, self.max_vel_theta] = rospy.get_param('/controller/vel_limits_theta')


    def wrapAngle(self, angle):
        if angle > np.pi:
            return angle - 2 * np.pi
        if angle < -np.pi:
            return angle + 2 * np.pi
        return angle

    def run(self):
        while not rospy.is_shutdown():
            self.step()
            self.rate.sleep()

    def step(self):
        """
        Perform an iteration of the control loop.
        @param: self
        @result: publishes computed body velocity command.
        """

        x_vel = 0
        y_vel = 0
        angular_vel = 0

        self.isWaypointReached()
        if self.waypoints:
            target = self.waypoints[0]
            dist_x_world = target[0] - self.x
            dist_y_world = target[1] - self.y

            dist_x = dist_x_world * math.cos(-self.theta) - dist_y_world * math.sin(-self.theta)
            dist_y = dist_x_world * math.sin(-self.theta) + dist_y_world * math.cos(-self.theta)
            dist_x_deriv = dist_x - self.prev_dist_x
            dist_y_deriv = dist_y - self.prev_dist_y

            self.prev_dist_x = dist_x
            self.prev_dist_y = dist_y

            target_heading = target[2]
            heading_diff = self.wrapAngle(target_heading - self.theta)
            heading_deriv = heading_diff - self.prev_heading_diff
            self.prev_heading_diff = heading_diff


            angular_vel = self.p_angular * heading_diff + self.i_angular * self.headin_integ + self.d_angular * heading_deriv
            x_vel = self.p_x * dist_x + self.i_x * self.dist_x_integ  + self.d_y * dist_x_deriv
            y_vel = self.p_y * dist_y + self.i_y * self.dist_y_integ + self.d_y * dist_y_deriv

            angular_vel = max(self.min_vel_theta, min(self.max_vel_theta, angular_vel)) # clamp speed between values
            vel = math.sqrt(x_vel * x_vel + y_vel * y_vel)
            
            cmd_ratio = self.vel_limit_xy / vel if vel > self.vel_limit_xy else 1.0
            if cmd_ratio == 1.0:
                self.dist_x_integ += dist_x
                self.dist_y_integ += dist_y
                self.headin_integ += heading_diff
            
            x_vel = cmd_ratio * x_vel
            y_vel = cmd_ratio * y_vel

            # x_vel = max(self.min_vel_xy, min(self.max_vel_xy, x_vel)) # clamp speed between values
            # y_vel = max(self.min_vel_xy, min(self.max_vel_xy, y_vel)) # clamp speed between values

        # publish velocity commands
        self.vel_cmd_msg.linear.x = x_vel
        self.vel_cmd_msg.linear.y = y_vel
        self.vel_cmd_msg.angular.z = angular_vel
        self.vel_cmd_pub.publish(self.vel_cmd_msg)

    def makeMarkers(self):
        markers = Marker()
        markers.header.frame_id = "map"
        markers.type = markers.SPHERE_LIST
        markers.action = markers.ADD
        markers.scale.x, markers.scale.y, markers.scale.z = [self.distance_margin, self.distance_margin, self.distance_margin]
        markers.pose.orientation.x, markers.pose.orientation.y, markers.pose.orientation.z, markers.pose.orientation.w  = [0, 0, 0, 1]
        markers.color.a = 0.7
        markers.points = [Point(x=wp[0], y=wp[1], z=0) for wp in self.waypoints]
        return markers

    def isWaypointReached(self):
        """
        check if waypoint is reached based on user define threshold
        @param: self
        @param: pose - current robot pose (x,y,theta)
        @result: if waypoint is reached that waypoint is popped from
                 waypoint list
        """
        if self.waypoints:
            dist_x = self.x - self.waypoints[0][0]
            dist_y = self.y - self.waypoints[0][1]
            dist_theta = abs(self.wrapAngle(self.theta - self.waypoints[0][2]))
            if math.sqrt(dist_x * dist_x + dist_y * dist_y) < self.distance_margin and dist_theta < self.heading_margin:
                rospy.loginfo("Waypoint reached! %d more waypoints to go", len(self.waypoints))
                rospy.loginfo("Current position: x: %f , y: %f theta:  %f",self.x, self.y, self.theta)
                self.waypoints.pop(0)
                self.headin_integ = 0
                self.dist_x_integ = 0
                self.dist_y_integ = 0
            self.marker_pub.publish(self.makeMarkers())

    def onOdom(self, odom_msg):
        """
        Handles incoming odometry updates. Ok, if running on ground truth, otherwise need to add transform in the function (not present)
        @param: self
        @param odom_msg - odometry geometry message
        @result: update of relevant vehicle state variables
        """
        
        self.x = odom_msg.pose.pose.position.x
        self.y = odom_msg.pose.pose.position.y
        _, _, self.theta = euler_from_quaternion(quat_msg_to_tf(odom_msg.pose.pose.orientation))
        # self.odom_msg = odom_msg

    def onGoal(self, goal_msg):
        """
        Handles incoming goal updates.
        @param: self
        @param goal_msg - odometry geometry message
        @result: Adds new waypoint at the end of waypoints and
                 updates target angle.
        """
        self.marker_frame = goal_msg.header.frame_id

        orientation_q = goal_msg.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

        target_yaw = euler_from_quaternion(orientation_list)[2]

        self.waypoints.append([goal_msg.pose.position.x, goal_msg.pose.position.y, target_yaw])


if __name__ == '__main__':
    rospy.init_node("WPController")
    controller = PDController(30)
    rospy.sleep(rospy.Duration(secs=5))
    controller.run()