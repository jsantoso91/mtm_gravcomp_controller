#!/usr/bin/env python
import rospy
import numpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from math import cos, sin

g = 9.81
m1 = 0.9; m2 = 1; m3 = 0.9; m4 = 0.8; m5 = 0.4; m6 = 0.2; m7 = 0.2; m8 = 0.1

msg1 = Float64()
msg2 = Float64()
msg3 = Float64()
msg4 = Float64()
msg5 = Float64()
msg6 = Float64()
msg7 = Float64()

						# position[1]            position[2]               position[0]           position[4]           position[3]            position[6]                 position[5]
K_p = numpy.matrix( ((-0.5, 0, 0, 0, 0, 0, 0), (0, -0.8, 0, 0, 0, 0, 0), (0,0,-0.85,0,0,0,0), (0,0,0,-3.7,0,0,0),    (0,0,0,0,-0.5,0,0),     (0,0,0,0,0,-0.02,0),     (0,0,0,0,0,0,-0.006)) )
K_v = numpy.matrix( ((-1.95,0,0,0,0,0,0),      (0,-0.95,0,0,0,0,0),        (0,0,-1.75,0,0,0,0),    (0,0,0,-0.20,0,0,0),    (0,0,0,0,-0.1,0,0),    (0,0,0,0,0,-0.8,0),        (0,0,0,0,0,0,-0.08)) )
K_i = numpy.matrix( ((-0.00001,0,0,0,0,0,0), (0,-0.00005,0,0,0,0,0),     (0,0,-0.0001,0,0,0,0), (0,0,0,-0.00040,0,0,0), (0,0,0,0,-0.000001,0,0),(0,0,0,0,0,-0.0000001,0), (0,0,0,0,0,0,-0.00000001)) )

# This is reference value for G obtained from reading the torque values for all the joints when maintaining 0 position for all the angles using ROS built in PID position control
# G = numpy.matrix( (3.257782240190199, -0.0026742991027006724, 3.1129055161644104, -0.038113895204983095, 0.004219637783897667, 100.0, -0.001377744552755189) )
					   # joint3					joint1				joint2				    joint5				joint 4			 joint 7		joint 6

q_des = numpy.array( (0, 0, 0, 0, 0, 0, 0) )
q_error = numpy.array( (0, 0, 0, 0, 0, 0, 0) )
past_q_error = numpy.array( (0, 0, 0, 0, 0, 0, 0) )
error_sum = numpy.array( (0, 0, 0, 0, 0, 0, 0) )


def callback(dataq):
	q_cur = dataq.position
	qdot_cur = dataq.velocity
	# The /mtm/joint_states topic publish joint position data in this order, outer_yaw_joint being the second element of dataq.position and so on
	q_current = numpy.array( (q_cur[1], q_cur[2], q_cur[0], q_cur[4], q_cur[3], q_cur[6], q_cur[5]) )
	qdot_current = numpy.array( (qdot_cur[1], qdot_cur[2], qdot_cur[0], qdot_cur[4], q_cur[3], q_cur[6], q_cur[5]) )

	theta1 = q_cur[1]
	theta2 = q_cur[2]
	theta3 = q_cur[0]
	theta4 = q_cur[4]
	theta5 = q_cur[3]
	theta6 = q_cur[6]
	theta7 = q_cur[5]

	# Torque equations derived from the MTM kinematics and position of center of masses, look at the 
	torque1 = 0;

	torque2 = ((g*(3645*m5*cos(theta2 + theta3) - 390*m8*cos(theta2 + theta3 + theta4 + theta5) + 
		3645*m6*cos(theta2 + theta3) + 3645*m7*cos(theta2 + theta3) + 3645*m8*cos(theta2 + theta3) - 
		1560*m5*sin(theta2 + theta3) - 1560*m6*sin(theta2 + theta3) - 1560*m7*sin(theta2 + theta3) - 
		1560*m8*sin(theta2 + theta3) + 2794*m4*sin(theta2) + 2794*m5*sin(theta2) + 2794*m6*sin(theta2) + 
		2794*m7*sin(theta2) + 2794*m8*sin(theta2)))/10000);

	torque3 = (-(3*g*(26*m8*cos(theta2 + theta3 + theta4 + theta5) - 243*m5*cos(theta2 + theta3) - 
		243*m6*cos(theta2 + theta3) - 243*m7*cos(theta2 + theta3) - 243*m8*cos(theta2 + theta3) + 
		104*m5*sin(theta2 + theta3) + 104*m6*sin(theta2 + theta3) + 104*m7*sin(theta2 + theta3) + 
		104*m8*sin(theta2 + theta3)))/2000);

	torque4 = -(39*g*m8*cos(theta2 + theta3 + theta4 + theta5))/1000;
	torque5 = -(39*g*m8*cos(theta2 + theta3 + theta4 + theta5))/1000;
	torque6 = 0.000;
	torque7 = 0.000;
	
	G = numpy.matrix( (torque1, torque2, torque3, torque4, torque5, torque6, torque7) )

	rospy.loginfo(rospy.get_caller_id())
	
	q_error = q_current - q_des
	global error_sum
	error_sum = error_sum + q_error
	P = numpy.dot(K_p, q_error)
	D = numpy.dot(K_v, qdot_current)
	I = numpy.dot(K_i, error_sum)
	q_input = G + P + I + D

	msg1.data = q_input[0,0]       # outer_yaw_joint
	msg2.data = q_input[0,1]       # shoulder_pitch_joint
	msg3.data = q_input[0,2]       # elbow_pitch_joint
	msg4.data = q_input[0,3]       # wrist_platform_joint
	msg5.data = q_input[0,4]       # wrist_pitch_joint
	msg6.data = q_input[0,5]       # wrist_yaw_joint
	msg7.data = q_input[0,6]       # wrist_roll_joint
	print(q_input)

	pub1.publish(msg1)
	pub2.publish(msg2)
	pub3.publish(msg3)
	pub4.publish(msg4)
	pub5.publish(msg5)
	pub6.publish(msg6)
	pub7.publish(msg7)

def listener():
    rospy.init_node('mtm_joint_torque_node', anonymous=True)
    rospy.Subscriber("mtm/joint_states", JointState, callback)

    global pub1, pub2, pub3, pub4, pub4, pub5, pub6, pub7
    pub1 = rospy.Publisher('mtm/joint1_torque_controller/command', Float64, queue_size=100)
    pub2 = rospy.Publisher('mtm/joint2_torque_controller/command', Float64, queue_size=100)
    pub3 = rospy.Publisher('mtm/joint3_torque_controller/command', Float64, queue_size=100)
    pub4 = rospy.Publisher('mtm/joint4_torque_controller/command', Float64, queue_size=100)
    pub5 = rospy.Publisher('mtm/joint5_torque_controller/command', Float64, queue_size=100)
    pub6 = rospy.Publisher('mtm/joint6_torque_controller/command', Float64, queue_size=100)
    pub7 = rospy.Publisher('mtm/joint7_torque_controller/command', Float64, queue_size=100)

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
    	rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    listener()