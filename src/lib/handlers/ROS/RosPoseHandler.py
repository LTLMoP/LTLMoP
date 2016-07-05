#!/usr/bin/env python
import rospy, math
from numpy import *
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
# TODO: don't rely on gazebo
from gazebo_msgs import srv

"""
=======================================
rosPose.py - ROS Interface Pose Handler
=======================================
"""

import lib.handlers.handlerTemplates as handlerTemplates

class RosPoseHandler(handlerTemplates.PoseHandler):
	def __init__(self, executor, shared_data, modelName="mobile_base"):
		"""
		Pose Handler for ROS and gazebo.

		modelName (str): The model name of the robot in gazebo to get the pose information from (default="turtlebot")
		"""

		#GetModelState expects the arguments model_name and relative_entity_name
		#In this case it is turtlebot and world respectively but can be changed for different robots and environments
		self.model_name = modelName
		self.relative_entity_name = 'world' #implies the gazebo global coordinates
		self.last_pose = None

		self.shared_data=shared_data['ROS_INIT_HANDLER']

	def getPose(self,cached = False):
		if (not cached) or self.last_pose is None:
			#Ros service call to get model state
			#This returns a GetModelStateResponse, which contains data on pose
			rospy.wait_for_service('/gazebo/get_model_state')
			try:
				gms = rospy.ServiceProxy('/gazebo/get_model_state', srv.GetModelState)
				resp = gms(self.model_name,self.relative_entity_name)
				#Cartesian Pose
				self.pos_x = resp.pose.position.x
				self.pos_y = resp.pose.position.y
				self.pos_z = resp.pose.position.z
				#Quaternion Orientation
				self.or_x = resp.pose.orientation.x
				self.or_y = resp.pose.orientation.y
				self.or_z = resp.pose.orientation.z
				self.or_w = resp.pose.orientation.w
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e
			#  Use the tf module transforming quaternions to euler
			try:
				angles = euler_from_quaternion([self.or_x, self.or_y, self.or_z, self.or_w])
				self.theta = angles[2]
				shared=self.shared_data
				#The following accounts for the maps offset in gazebo for
				#initial region placement
				self.last_pose = array([self.pos_x+shared.offset[0], self.pos_y+shared.offset[1], self.theta, self.pos_z])
			except Exception:
				print 'Pose Broke', Exception
		return self.last_pose
