#!/usr/bin/env python
import roslib
import numpy
import Queue
roslib.load_manifest('tiago_ltl_flexbe')
import rospy
import actionlib

from robot_fts import robot_model
import sys

import time

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from std_srvs.srv import Empty
import move_base_msgs.msg

from math import pi as PI
from math import atan2, sin, cos, sqrt, copysign

from tf.transformations import euler_from_quaternion, quaternion_from_euler


from ltl_tools.ts import MotionFts, ActionModel, MotActModel
from ltl_tools.planner import ltl_planner


def norm2(pose1, pose2):
    # 2nd norm distance
    return sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)


def check_yaw(yaw_goal):
	reach_yaw_bound = 0.1*PI
	VelPublisher = rospy.Publisher('mobile_base_controller/cmd_vel', Twist, queue_size = 10)
	ang = Twist()
	while (abs(robot_pose[1][2]-yaw_goal) > reach_yaw_bound):
		print 'diff = %s' %str(abs(abs(robot_pose[1][2])-yaw_goal))
		print 'yaw = %s' %str(robot_pose[1][2])
		ang.angular.z = copysign(1, yaw_goal-robot_pose[1][2]) # return 1 or -1
		VelPublisher.publish(ang)
		print 'pub %s' %str(ang.angular.z)
		rospy.sleep(0.1)
	ang.angular.z = 0
	VelPublisher.publish(ang)




def PoseCallback(posedata):
	reach_xy_bound = 0.1
    # PoseWithCovarianceStamped data from amcl_pose
	global robot_pose # [time, [x,y,yaw]]
	global GoalPublisher
	header = posedata.header
	pose = posedata.pose
	#print 'amcl_pose(Quaternion)\n %s' %str(pose)
	if (not robot_pose[0]) or (header.stamp > robot_pose[0]):
        # more recent pose data received
		robot_pose[0] = header.stamp
        # TODO: maybe add covariance check here?
        # print('robot position update!')
		euler = euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]) #roll, pitch, yaw
		robot_pose[1] = [pose.pose.position.x, pose.pose.position.y, euler[2]] # in radians
		#print 'euler from quaternion :\n %s' %str(euler)
		print 'robot_pose (pose.x, pose.y, euler[2]) :\n %s' %str(robot_pose[1])
		print 'GOAL STATE : %s' %str(GoalPublisher.get_state())
		print 'Current Goal CallBack %s' %str(current_goal)
		if ((norm2(robot_pose[1][0:2], current_goal[0:2]) < reach_xy_bound) and (GoalPublisher.get_state() == 1)): # 1 = ACTIVE
			GoalPublisher.cancel_goal()
			print 'GOAL CANCELLED'
			



def SendGoal(goal , time_stamp):
    # goal: [x, y, yaw]
    global GoalPublisher
    GoalMsg = PoseStamped()
    #GoalMsg.header.seq = 0
    GoalMsg.header.stamp = time_stamp
    GoalMsg.header.frame_id = 'map'
    GoalMsg.pose.position.x = goal[0]
    GoalMsg.pose.position.y = goal[1]
    #GoalMsg.pose.position.z = 0.0
    quaternion = quaternion_from_euler(0, 0, goal[2])
    GoalMsg.pose.orientation.x = quaternion[0]
    GoalMsg.pose.orientation.y = quaternion[1]
    GoalMsg.pose.orientation.z = quaternion[2]
    GoalMsg.pose.orientation.w = quaternion[3]
    goal = move_base_msgs.msg.MoveBaseGoal(target_pose = GoalMsg)
    GoalPublisher.send_goal(goal)
    #print 'GoalMsg : %s' %str(GoalMsg)




if __name__ == '__main__':
	robot_name='TIAGo'
	rospy.init_node('orientation_test_%s' %robot_name)
	global robot_pose
	robot_pose = [None, (0,0,0)]
	current_goal = (0.65, -7.58, 0.52)
	current_goal = (1.0, -4.5, 0.0)
	current_goal =(-3.25, -0.31, 0.82)
	rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, PoseCallback)
	global GoalPublisher
	GoalPublisher = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)
	GoalPublisher.wait_for_server()
	t0 = rospy.Time.now()
	#while not rospy.is_shutdown():
	#	try:
	t = rospy.Time.now()-t0
	SendGoal(current_goal, t)
	GoalPublisher.wait_for_result()
	print '-------------\nAction server over\n-------------\nCurrent Goal : %s' %str(current_goal)
	print 'Current position : %s' %str(robot_pose[1])
	check_yaw(current_goal[2])
	rospy.sleep(10)
			
	#	except rospy.ROSInterruptException:
	#		pass
			



