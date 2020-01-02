#!/usr/bin/python

import math
import numpy as np
import rospy
from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from mav_msgs.msg import RollPitchYawrateThrust
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
import argparse

model = "hummingbird"

x_f = []
y_f = []
z_f = []

vx_f = []
vy_f = []
vz_f = []

r_f = []
p_f = []
yaw_f = []

x_des = []
y_des = []
z_des = []


roll_sp = []
pitch_sp = []
yawrate_sp = []
thrust_sp = []

x_sp = []
y_sp = []
z_sp = []


def quaternion_to_euler_angle(w, x, y, z):
	ysqr = y * y

	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + ysqr)
	X = math.degrees(math.atan2(t0, t1))

	t2 = +2.0 * (w * y - z * x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	Y = math.degrees(math.asin(t2))

	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (ysqr + z * z)
	Z = math.degrees(math.atan2(t3, t4))

	return X, Y, Z

actuators = RollPitchYawrateThrust()
def control_cb(data):
	global actuators
	actuators = data

trajectory = MarkerArray()
def trajectory_cb(data):
	global trajectory
	trajectory = data

odom = Odometry()
def odom_cb(data):
	global odom
	odom = data

def control():
	marker_sub =  rospy.Subscriber(model+'/trajectory',MarkerArray, trajectory_cb, queue_size = 100)
	control_sub = rospy.Subscriber(model+ "/command/roll_pitch_yawrate_thrust", RollPitchYawrateThrust, control_cb, queue_size=100)
	odom_sub = rospy.Subscriber(model+'/ground_truth/odometry',Odometry, odom_cb, queue_size=100)
	traj_pub = rospy.Publisher(model+'/command/pose',PoseStamped,queue_size=100)	
	flag_pub = rospy.Publisher('/iris/trajectory_flag', Bool, queue_size=100)

	rospy.init_node('tracking',anonymous=True)
	rate = rospy.Rate(10)

	flag = Bool()
	desired_trajectory = PoseStamped()

	last_index = 0
	desired_x = 0.
	desired_y = 0.
	desired_z = 1.

	while not rospy.is_shutdown():

		if len(trajectory.markers) > 0:
			last_element = trajectory.markers[len(trajectory.markers)-1].points

			curr_x = odom.pose.pose.position.x
			curr_y = odom.pose.pose.position.y
			curr_z = odom.pose.pose.position.z
		

			for i in range(last_index,len(last_element)):
				count_index = 0
				temp_x = trajectory.markers[len(trajectory.markers)-1].points[i].x
				temp_y = trajectory.markers[len(trajectory.markers)-1].points[i].y
				temp_z = trajectory.markers[len(trajectory.markers)-1].points[i].z


				if np.abs(temp_x - curr_x) < 0.7 and np.abs(temp_y -curr_y) < 0.7 and np.abs(temp_z -curr_z) < 0.5:
					if len(last_element) > i+1:
						desired_x = trajectory.markers[len(trajectory.markers)-1].points[i+1].x
						desired_y = trajectory.markers[len(trajectory.markers)-1].points[i+1].y
						desired_z = trajectory.markers[len(trajectory.markers)-1].points[i+1].z
						last_index = i+1
					else:
						desired_x = temp_x
						desired_y = temp_y
						desired_z = temp_z
						last_index = i


			x_f.append(float(odom.pose.pose.position.x))
			y_f.append(float(odom.pose.pose.position.y))
			z_f.append(float(odom.pose.pose.position.z))

			vx_f.append(float(odom.twist.twist.linear.x))
			vy_f.append(float(odom.twist.twist.linear.y))
			vz_f.append(float(odom.twist.twist.linear.z))

			(roll,pitch, yaw) = quaternion_to_euler_angle(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x , odom.pose.pose.orientation.y, odom.pose.pose.orientation.z)
			r_f.append(float(math.radians(roll)))
			p_f.append(float(math.radians(pitch)))
			yaw_f.append(float(math.radians(yaw)))

			roll_sp.append(actuators.roll)
			pitch_sp.append(actuators.pitch)
			yawrate_sp.append(actuators.yaw_rate)
			thrust_sp.append(actuators.thrust.z)


			desired_trajectory.pose.position.x = desired_x
			desired_trajectory.pose.position.y = desired_y
			desired_trajectory.pose.position.z = desired_z

			x_sp.append(desired_trajectory.pose.position.x)
			y_sp.append(desired_trajectory.pose.position.y)
			z_sp.append(desired_trajectory.pose.position.z)


			traj_pub.publish(desired_trajectory)


			if last_index == len(trajectory.markers[len(trajectory.markers)-1].points)-1:

				state_ 	 = np.array([x_f,y_f,z_f,  vx_f,vy_f,vz_f, r_f,p_f,yaw_f],ndmin=2).transpose()
				setpoints_ = np.array([x_sp, y_sp, z_sp], ndmin=2).transpose()
				controls_ = np.array([roll_sp,pitch_sp, yawrate_sp, thrust_sp],ndmin=2).transpose()

				print('setpoints_	:',setpoints_.shape)
				print('control_ 	:',controls_.shape)
				print('state_		:',state_.shape)

				np.save('suboptimal_rose_input_state.npy',state_)
				np.save('suboptimal_rose_setpoints.npy', setpoints_)
				np.save('suboptimal_rose_controls.npy'   ,controls_)

				while np.abs(odom.pose.pose.position.x) > 0.5 and np.abs(odom.pose.pose.position.x) > 0.5:
					desired_trajectory.pose.position.x = .0
					desired_trajectory.pose.position.y = 0.
					desired_trajectory.pose.position.z = 1.
				
					traj_pub.publish(desired_trajectory)
					rate.sleep()

				last_index = 0


				flag = True
				flag_pub.publish(flag)
			else:
				flag = False
				flag_pub.publish(flag)

		rate.sleep()






if __name__ == '__main__':
	try:
		# current_state = State()
		control()
	except rospy.ROSInterruptException:
		pass