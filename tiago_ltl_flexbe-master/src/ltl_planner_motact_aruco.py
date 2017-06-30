#!/usr/bin/env python
import roslib
import numpy
import Queue
roslib.load_manifest('tiago_ltl_flexbe')
import rospy
import sys
import time
import actionlib

import scenario


from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from std_srvs.srv import Empty
import move_base_msgs.msg

from math import pi as PI
from math import atan2, sin, cos, sqrt, copysign

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal

from robot_fts import robot_model
from ltl_tools.ts import MotionFts, ActionModel, MotActModel
from ltl_tools.planner import ltl_planner


def norm2(pose1, pose2):
    # 2nd norm distance
    return sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)


def check_yaw(yaw_goal):
	print 'yaw_goal %s' %str(yaw_goal)
	print 'yaw_robot %s' %str(robot_pose[1][2])
	reach_yaw_bound = 0.2
	VelPublisher = rospy.Publisher('mobile_base_controller/cmd_vel', Twist, queue_size = 10)
	ang = Twist()
	while (PI-abs(abs(robot_pose[1][2]-yaw_goal)-PI) > reach_yaw_bound):
		ang.angular.z = 0.7*copysign(1, (yaw_goal-robot_pose[1][2]+3*PI)%(2*PI)-PI) # return 1 or -1
		VelPublisher.publish(ang)
		rospy.sleep(0.1)
	ang.angular.z = 0
	VelPublisher.publish(ang)



def PoseCallback(posedata):
	global current_goal
	if not isinstance(current_goal, str):
		reach_xy_bound = 0.1
		# PoseWithCovarianceStamped data from amcl_pose
		global robot_pose # [time, [x,y,yaw]]
		global GoalPublisher
		header = posedata.header
		pose = posedata.pose
		if (not robot_pose[0]) or (header.stamp > robot_pose[0]):
			# more recent pose data received
			robot_pose[0] = header.stamp
			# TODO: maybe add covariance check here?
			# print('robot position update!')
			euler = euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]) #roll, pitch, yaw
			robot_pose[1] = [pose.pose.position.x, pose.pose.position.y, euler[2]] # in radians
			print 'robot_pose (pose.x, pose.y, euler[2]) :\n %s' %str(robot_pose[1])
			print 'GOAL STATE : %s' %str(GoalPublisher.get_state())
			if ((norm2(robot_pose[1][0:2], current_goal[0:2]) < reach_xy_bound) and (GoalPublisher.get_state() == 1)): # 1 = ACTIVE
				GoalPublisher.cancel_goal()
				print '-------------\nPOSITION REACHED'
	
	

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



def SendInitialPose(InitialPosePublisher, initial_pose, time_stamp):
    # goal: [x, y, yaw]
    InitialPoseMsg = PoseWithCovarianceStamped()
    #InitialPoseMsg.header.seq = 0
    InitialPoseMsg.header.stamp = time_stamp
    InitialPoseMsg.header.frame_id = 'map'
    InitialPoseMsg.pose.pose.position.x = initial_pose[0]
    InitialPoseMsg.pose.pose.position.y = initial_pose[1]
    #InitialPoseMsg.pose.position.z = 0.0
    quaternion = quaternion_from_euler(0, 0, initial_pose[2])
    InitialPoseMsg.pose.pose.orientation.x = quaternion[0]
    InitialPoseMsg.pose.pose.orientation.y = quaternion[1]
    InitialPoseMsg.pose.pose.orientation.z = quaternion[2]
    InitialPoseMsg.pose.pose.orientation.w = quaternion[3]
    InitialPosePublisher.publish(InitialPoseMsg)    


def planner(ts, init_pose, act, robot_task, robot_name='TIAGo'):
	global robot_pose
	robot_pose = [None, init_pose]
	global current_goal
	current_goal = init_pose
	rospy.init_node('ltl_planner_%s' %robot_name)
	print 'Robot %s: ltl_planner started!' %(robot_name)
    #----------
    #publish to
    #----------
	InitialPosePublisher = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size = 100)
    # for i in xrange(10):
    #     SendInitialPose(InitialPosePublisher, init_pose, rospy.Time.now())
    #     rospy.sleep(0.1)
    # print('Initial pose set to %s.' %str(init_pose))
	global GoalPublisher
	GoalPublisher = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)
	GoalPublisher.wait_for_server()
    ####### robot information
	full_model = MotActModel(ts, act)
	planner = ltl_planner(full_model, robot_task, None)
    ####### initial plan synthesis
	planner.optimal(5)
    #######
    #----------
    #subscribe to
    #----------
	rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, PoseCallback)
	
	t0 = rospy.Time.now()
	while not rospy.is_shutdown():
		try:
			t = rospy.Time.now()-t0
			print '----------Time: %.2f----------' %t.to_sec()
			current_goal = planner.next_move
			if isinstance(current_goal, str):
				print 'Robot %s next move is action %s' %(str(robot_name), str(current_goal))
				# Octomap setup
				client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction) 
				client.wait_for_server()
				goal = PlayMotionGoal()
				goal.motion_name = 'head_look_around'
				goal.skip_planning = False
				goal.priority = 0  # Optional
				print 'Sending actionlib goal with motion: %s' %str(goal.motion_name)
				client.send_goal(goal)
				print 'Waiting for result...'
				action_ok = client.wait_for_result(rospy.Duration(30.0))
				state = client.get_state()
				print 'head_look_around state %s' %str(state)
				print 'Waiting for pick Service...'
				# Pick object with Aruco marker
				pickclient = rospy.ServiceProxy('/pick_gui', Empty)
				pickclient() 
				print 'Pick server finished'
				# Come back to default position
				goal.motion_name = 'home'
				goal.skip_planning = False
				goal.priority = 0  # Optional
				print 'Sending actionlib goal with motion: %s' %str(goal.motion_name)
				client.send_goal(goal)
				print 'Waiting for result...'
				action_ok = client.wait_for_result(rospy.Duration(30.0))
				state = client.get_state()
				print 'home state %s' %str(state)
				planner.find_next_move()
				print 'action : next move : %s' %str(planner.next_move)
			else:
				print 'current yaw : %s' %str(abs(robot_pose[1][2]))
				print 'yaw goal : %s' %str(current_goal[2])
				# move_base action server travels till (x,y) coordinates
				SendGoal(current_goal, t)
				GoalPublisher.wait_for_result()
				print 'Action server over\n-------------\nCurrent Goal : %s' %str(current_goal)
				print 'Current position : %s' %str(robot_pose[1])
				# check_yaw function set the orientation 
				check_yaw(current_goal[2])
				
				planner.find_next_move()
				print 'move : next move : %s' %str(planner.next_move)
			if current_goal == planner.next_move:
				break
		except rospy.ROSInterruptException:
			pass



if __name__ == '__main__':
		
	robot_name='TIAGo'
	rospy.init_node('ltl_planner_%s' %robot_name)
    ########
	world = rospy.get_param('world_name')
	if len(sys.argv) == 2:
		world = str(sys.argv[1])
		print('Argument: %s.' %(str(sys.argv[1])))
	else:
		print 'No argument : world set automatically : %s' %str(world)
	while 1:
		if world == 'tabletop_cube' or 'tutorial_office':
			scenario.scenario(world)
		try:
			robot_task = rospy.get_param('plan')
			if str(robot_task) == 'none':
				sys.exit()
			print('Robot task: %s.' %(str(robot_task)))
			[robot_motion, init_pose, robot_action] = robot_model
			planner(robot_motion, init_pose, robot_action, robot_task)
		except rospy.ROSInterruptException:
			pass
