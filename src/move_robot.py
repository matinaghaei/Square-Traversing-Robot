#! /usr/bin/env python3

# Import the Python library for ROS
import rospy
import time
import math
import matplotlib.pyplot as plt

# Import the Odometry message
from nav_msgs.msg import Odometry

# Import the Twist message
from geometry_msgs.msg import Twist

from std_msgs.msg import Bool

# TF allows to perform transformations between different coordinate frames
import tf


class MoveRobot():

	def __init__(self):
		# Initiate a named node
		rospy.init_node('MoveRobot', anonymous=False)
		
		# tell user how to stop TurtleBot
		rospy.loginfo("CTRL + C to stop the turtlebot")
		
		# What function to call when ctrl + c is issued
		rospy.on_shutdown(self.shutdown)
		
		self.odom_sub = rospy.Subscriber('/odom', Odometry, self.callback_odometry)
		
		# Create a Publisher object, will publish on cmd_vel_mux/input/teleop topic
		# to which the robot (real or simulated) is a subscriber
		self.vel_pub = rospy.Publisher('/cmd_vel', Twist,
									queue_size=5)
		
		# Creates a var of msg type Twist for velocity
		self.vel = Twist()
		
		# publish a topic to notify node pose_monitor of the changed velocity
		self.new_velocity_pub = rospy.Publisher('/change', Twist, queue_size=1)
		
		self.finished_bool = Bool()
		self.finished_bool.data = True
		self.finish_pub = rospy.Publisher('/finished', Bool, queue_size=1)
		
		self.last_round_bool = Bool()
		self.last_round_bool.data = True
		self.last_round_pub = rospy.Publisher('/last_round', Bool, queue_size=1)
		
		# Set a publish velocity rate of in Hz
		self.rate = rospy.Rate(5)
		
		self.points = [(1.5, 1.5), (-1.5, 1.5), (-1.5, -1.5), (1.5, -1.5)]
		self.next_point = 0
		self.round = 0
		self.is_finished = False
		self.path_x = []
		self.path_y = []
		
		plt.plot((1.5, -1.5), (1.5, 1.5), c='b')
		plt.plot((-1.5, -1.5), (1.5, -1.5), c='b')
		plt.plot((-1.5, 1.5), (-1.5, -1.5), c='b')
		plt.plot((1.5, 1.5), (-1.5, 1.5), c='b')

	def callback_odometry(self, msg):
		if self.round == 11:
			return
		current = (msg.pose.pose.position.x, msg.pose.pose.position.y)
		self.path_x.append(current[0])
		self.path_y.append(current[1])
		target = self.points[self.next_point]
		pos_diff = math.sqrt((current[0]-target[0])**2 + (current[1]-target[1])**2)
		if pos_diff < 1.5:
			self.next_point += 1
			if self.next_point == 4:
				self.next_point = 0
			elif self.next_point == 1:
				self.round += 1
				if self.round == 11:
					self.shutdown()
					return
				if self.round == 10:
					self.last_round_pub.publish(self.last_round_bool)
			target = self.points[self.next_point]
			pos_diff = math.sqrt((current[0]-target[0])**2 + (current[1]-target[1])**2)
		print("Position Difference:", pos_diff)
		self.vel.linear.x = 0.9
		
		target_yaw = math.atan2(target[1] - current[1], target[0] - current[0])
		quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
			msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
		(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
		yaw_diff = target_yaw - yaw
		if yaw_diff > math.pi:
			yaw_diff -= 2 * math.pi
		if yaw_diff < -math.pi:
			yaw_diff += 2 * math.pi
		print("Yaw Difference:", yaw_diff)
		self.vel.angular.z = yaw_diff
		
		self.new_velocity_pub.publish(self.vel)		
		
	def send_velocity_cmd(self):
		self.vel_pub.publish(self.vel)
	
	def shutdown(self):
		print("Shutdown!")
		# stop TurtleBot
		rospy.loginfo("Stop TurtleBot")
		
		self.vel.linear.x = 0.0
		self.vel.angular.z = 0.0
		
		self.vel_pub.publish(self.vel)
		
		self.finish_pub.publish(self.finished_bool)
		
		# makes sure robot receives the stop command prior to shutting down
		rospy.sleep(1)

		self.is_finished = 1	
		
if __name__ == '__main__':
	try:
		controller = MoveRobot()
		
		# keeping doing until ctrl+c
		while not rospy.is_shutdown() and not controller.is_finished:
			
			# send velocity commands to the robots
			controller.send_velocity_cmd()
			
			# wait for the selected mseconds and publish velocity again
			controller.rate.sleep()
		
		plt.scatter(controller.path_x, controller.path_y, s=0.5, c='r')
		plt.show()
		
	except:
		rospy.loginfo("move_robot node terminated")

