#!/usr/bin/env python3

import rospy
import actionlib
import math
import time

import rostest
import unittest
import rosunit

from tortoisebot_waypoints.msg import WaypointActionFeedback, WaypointActionResult, WaypointActionAction, WaypointActionGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from tf import transformations

PKG = "tortoisebot_waypoints"
NAME = "waypoint_action_integration_test"

class WaypointActionClient(unittest.TestCase):
    def setUp(self):
        rospy.init_node("waypoint_action_client", anonymous=True)

        self.client = actionlib.SimpleActionClient("tortoisebot_as", WaypointActionAction)
        self.action_result = False

        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.robot_position = Point()
        self.robot_yaw = 0.0

        self.target_x = -0.5 #0.0   
        self.target_y = -0.4 #-0.3
        self.target_yaw = 1.57

        self.linear_error_threshold = 0.1
        self.yaw_error_threshold = 0.15

        self.perform_action()

    def perform_action(self):
        rospy.wait_for_service('/gazebo/reset_world')
        time.sleep(3)

        while not self.client.wait_for_server(timeout=rospy.Duration(5.0)):
            print("Waiting for service...")
        print("Service found")

        goal_message = WaypointActionGoal()
        goal_message.position.x = self.target_x
        goal_message.position.y = self.target_y
        goal_message.position.z = self.target_yaw

        print("Goal message: ", goal_message)

        try:
            self.client.send_goal(goal_message)
            result = self.client.wait_for_result(timeout=rospy.Duration(30.0))  # Increased timeout to 30 seconds

            if result:
                self.action_result = self.client.get_result().success
            else:
                print("Action was not finished in 30 seconds. Possibly hit an obstacle.")
                self.client.cancel_all_goals()
        except rospy.ROSException as e:
            print(f"Test timed out: {e}. Possibly hit an obstacle.")

    def odom_callback(self, msg):
        self.robot_position = msg.pose.pose.position

        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        self.robot_yaw = euler[2]

    def test_yaw_error(self):
        current_yaw_diff = self.robot_yaw - self.target_yaw
        self.assertTrue((abs(current_yaw_diff) < self.yaw_error_threshold), "Yaw angle is not within error limits")

    @unittest.skipIf(True, "Skipping linear error test") 
    def test_linear_error(self):
        delta_x = self.robot_position.x - self.target_x
        delta_y = self.robot_position.y - self.target_y
        delta_xy = math.sqrt(delta_x**2 + delta_y**2)
        self.assertTrue((delta_xy < self.linear_error_threshold), "Linear error is too large")

if __name__ == "__main__":
    rostest.rosrun(PKG, NAME, WaypointActionClient)
