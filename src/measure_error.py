#!/usr/bin/env python3

import rospy

# For getting robot’s ground truth from Gazebo
from gazebo_msgs.srv import GetModelState

from std_msgs.msg import Bool

class MeasureError():
	
	def __init__(self):
		# Initiate a named node
		rospy.init_node('measure_error', anonymous=True)
		
		print("Wait for service ....")
		rospy.wait_for_service("gazebo/get_model_state")
		
		print(" ... Got it!")
		
		self.get_ground_truth = rospy.ServiceProxy("gazebo/get_model_state",
												GetModelState)
		
		self.finished_sub = rospy.Subscriber('/finished', Bool, self.callback_finished)
		self.last_round_sub = rospy.Subscriber('/last_round', Bool, self.callback_last_round)
		
		self.rate = rospy.Rate(5)
		
		self.is_finished = False
		self.is_last_round = False
	
	def callback_finished(self, msg):
		if msg.data:
			self.is_finished = True
	
	def callback_last_round(self, msg):
		if msg.data:
			self.is_last_round = True

	def measure(self):
		position = error_calculator.get_ground_truth("turtlebot3_burger", "world").pose.position
		(x, y) = (position.x, position.y)
		error = min(abs(x - 1.5), abs(x - (-1.5)), abs(y - 1.5), abs(y - (-1.5)))
		return error

if __name__ == '__main__':
	errors = []
	error_calculator = MeasureError()
	while not error_calculator.is_finished:
		error = error_calculator.measure()
		if error_calculator.is_last_round:
			errors.append(error)
			print("(Last Round) Current Error:", error)
		else:
			print("Current Error:", error)
		error_calculator.rate.sleep()
	print("Last Round Average Error:", sum(errors) / len(errors))

