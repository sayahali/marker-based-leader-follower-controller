#!/usr/bin/env python

"""
PID follower
	ar_follower.py    
"""
import rospy
import time
import math
import numpy as np
from sensor_msgs.msg import Joy, LaserScan
from geometry_msgs.msg import Twist, Vector3
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import String as StringMsg
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float64

class ARFollower():
	def __init__(self):
		roll = pitch = yaw = 0.0
		rospy.init_node("ar_tag_follower")
		self.cmdVelPublisher = rospy.Publisher('/rb1_base/cmd_vel', Twist, queue_size =3)
		self.distance_moved_pub = rospy.Publisher('/moved_distance', Float64, queue_size=1)
		self.angle_moved_pub = rospy.Publisher('/moved_angle', Float64, queue_size=1)
		
		self.target_visible = False
		self.active=True
        
		self.max_speed = rospy.get_param('~maxSpeed') 
        targetDist = rospy.get_param('~targetDist')
        PID_param = rospy.get_param('~PID_controller')
		self.PID_controller = simplePID([0, targetDist], PID_param['P'], PID_param['I'], PID_param['D'])
		rospy.loginfo("Waiting for ar_pose_marker topic...")
		rospy.wait_for_message('ar_pose_marker', AlvarMarkers)
		rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.positionUpdateCallback)
		rospy.loginfo("Marker messages detected. Starting follower...")

	def positionUpdateCallback(self, markers):
		# gets called whenever we receive a new position. It will then update the motorcomand
		global roll, pitch, yaw
		for marker in markers.markers:
			marker_identity = marker.id
			
			try:
				if not self.target_visible:
					rospy.loginfo("FOLLOWER is Tracking Target!")
					self.target_visible = True
			except:
				self.cmdVelPublisher.publish (Twist())
				if self.target_visible:
					rospy.loginfo("FOLLOWER LOST Target!")
					self.target_visible = False        
				return
		if marker_identity ==13:
			target_offset_x = marker.pose.pose.position.x
			target_offset_y = marker.pose.pose.position.y
			target_offset_z = marker.pose.pose.position.z
			print("l'angle initiale est",target_offset_z)
			#l'angle est y est la distance est x
			angleX=target_offset_y
			distance = target_offset_x
			distance_pub=Float64()
			angle_pub=Float64()
			distance_pub.data=distance
			angle_pub.data=angleX
			orientation_q = marker.pose.pose.orientation
			orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
			(roll, pitch, yaw) = euler_from_quaternion (orientation_list)

			rospy.loginfo('Angle: {}, Distance: {}, '.format(angleX, distance))
			[uncliped_ang_speed, uncliped_lin_speed] = self.PID_controller.update([angleX, distance])
			
			self.distance_moved_pub.publish(distance_pub.data)
			self.angle_moved_pub.publish(angle_pub.data)
			angularSpeed = np.clip(-uncliped_ang_speed, -self.max_speed, self.max_speed)
			linearSpeed  = np.clip(-uncliped_lin_speed, -self.max_speed, self.max_speed)
			r = rospy.Rate(25)
			velocity = Twist()	
			velocity.linear = Vector3(linearSpeed,0.,0.)
			velocity.angular= Vector3(0.,0.,angularSpeed)
			rospy.loginfo('linearSpeed: {}, angularSpeed: {}'.format(linearSpeed, angularSpeed))
			self.cmdVelPublisher.publish(velocity)
class simplePID:
	def __init__(self, target, P, I, D):
		# check if parameter shapes are compatible. 
		if(not(np.size(P)==np.size(I)==np.size(D)) or ((np.size(target)==1) and np.size(P)!=1) or (np.size(target )!=1 and (np.size(P) != np.size(target) and (np.size(P) != 1)))):
			raise TypeError('input parameters shape is not compatable')
		rospy.loginfo('PID initialised with P:{}, I:{}, D:{}'.format(P,I,D))
		self.error_angle_pub = rospy.Publisher('/distance_error', Float64, queue_size=1)
		self.error_distance_pub = rospy.Publisher('/angle_error', Float64, queue_size=1)
		self.Kp		=np.array(P)
		self.Ki		=np.array(I)
		self.Kd		=np.array(D)
		self.setPoint   =np.array(target)
		self.last_error=0
		self.integrator = 0
		self.integrator_max = float('inf')
		self.timeOfLastCall = None
		print("target is far",self.setPoint)
	def update(self, current_value):
		
		current_value=np.array(current_value)
		if(np.size(current_value) != np.size(self.setPoint)):
			raise TypeError('current_value and target do not have the same shape')
		if(self.timeOfLastCall is None):
			# the PID was called for the first time. we don't know the deltaT yet
			# no controll signal is applied
			self.timeOfLastCall = time.clock()
			return np.zeros(np.size(current_value))
		
		
		error = self.setPoint - current_value
		errorang=Float64()
		errordis=Float64()
		errorang.data=error[1]
		errordis.data=error[0]
		self.error_angle_pub.publish(errorang.data)
		self.error_distance_pub.publish(errordis.data)

		P = error

		currentTime = time.clock()
		deltaT      = (currentTime-self.timeOfLastCall)

		# integral of the error is current error * time since last update
		self.integrator = self.integrator + (error*deltaT)
		I = self.integrator
		

		# derivative is difference in error / time since last update
		
		D = (error-self.last_error)/deltaT
		

		self.last_error = error
		self.timeOfLastCall = currentTime
		# return controll signal
		
		return self.Kp*P + self.Ki*I + self.Kd*D

if __name__ == '__main__':
	try:
		ARFollower()
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("AR follower node terminated.")
