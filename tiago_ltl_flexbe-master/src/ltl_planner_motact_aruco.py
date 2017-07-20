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

from ltl_tools.ts import MotionFts, ActionModel, MotActModel
from ltl_tools.planner import ltl_planner



def norm2(pose1, pose2):
    # 2nd norm distance
    return sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)


def check_yaw(yaw_goal):
	reach_yaw_bound = 0.15
	VelPublisher = rospy.Publisher('mobile_base_controller/cmd_vel', Twist, queue_size = 10)
	ang = Twist()
	diff = yaw_goal-robot_pose[1][2]
	while (abs(atan2(sin(diff),cos(diff))) > reach_yaw_bound):
		diff = yaw_goal-robot_pose[1][2]
		ang.angular.z = 0.5*copysign(1, atan2(sin(diff),cos(diff))) # return 1 or -1
		VelPublisher.publish(ang)
		rospy.sleep(0.1)
	ang.angular.z = 0
	VelPublisher.publish(ang)



def PoseCallback(posedata, GoalPublisher):
	global pose_goal
	if not isinstance(pose_goal, str):
		reach_xy_bound = 0.1
		# PoseWithCovarianceStamped data from amcl_pose
		global robot_pose # [time, [x,y,yaw]]
		header = posedata.header
		pose = posedata.pose
		if (not robot_pose[0]) or (header.stamp > robot_pose[0]):
			# more recent pose data received
			robot_pose[0] = header.stamp
			# TODO: maybe add covariance check here?
			# print('robot position update!')
			euler = euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]) #roll, pitch, yaw
			robot_pose[1] = [pose.pose.position.x, pose.pose.position.y, euler[2]] # in radians
			#print 'robot_pose (pose.x, pose.y, euler[2]) :\n %s' %str(robot_pose[1])	
			if ((norm2(robot_pose[1][0:2], pose_goal[0:2]) < reach_xy_bound) and (GoalPublisher.get_state() == 1)): # 1 = ACTIVE
				GoalPublisher.cancel_goal()
	

def SendGoal(goal , time_stamp, GoalPublisher):
    # goal: [x, y, yaw]
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



def init_pose():
	try:
		posedata = rospy.wait_for_message('/amcl_pose',  PoseWithCovarianceStamped, timeout=2)
	except rospy.ROSException:
		pass
	xi = posedata.pose.pose.position.x
	yi = posedata.pose.pose.position.y
	yawi = (euler_from_quaternion([posedata.pose.pose.orientation.x, posedata.pose.pose.orientation.y, posedata.pose.pose.orientation.z, posedata.pose.pose.orientation.w]))[2]	
	rospy.set_param('amcl_initial_pose_x', xi)
	rospy.set_param('amcl_initial_pose_y', yi)
	rospy.set_param('amcl_initial_pose_a', yawi)
	return (xi, yi, yawi)
	



def planner(ts, init_pose, act, robot_task, robot_name='TIAGo'):
	global robot_pose
	robot_pose = [None, init_pose]
	global pose_goal
	pose_goal = init_pose
	rospy.init_node('ltl_planner_%s' %robot_name)
	print 'Robot %s: ltl_planner started!' %(robot_name)
    #----------
    #publish to
    #----------
	GoalPublisher = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)
	GoalPublisher.wait_for_server()
    ####### robot information
	full_model = MotActModel(ts, act)
	planner = ltl_planner(full_model, robot_task, None)
    ####### initial plan synthesis
	planner.optimal(10)
    #######
    #----------
    #subscribe to
    #----------
	rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, PoseCallback, GoalPublisher)
	
	t0 = rospy.Time.now()

	while not rospy.is_shutdown():
		try:
			t = rospy.Time.now()-t0
			new_goal = planner.next_move
			rospy.loginfo("NEW GOAL: "+str(new_goal))
			if isinstance(new_goal, str):
				tp = rospy.Time.now()
				data = open("../pick_test/"+str(rospy.get_param('test_name'))+".txt", "a")
				data.write("Time = "+str((tp-t0).to_sec())+": Pick server starting\n")
				data.close()
				rospy.loginfo("Robot "+ str(robot_name) +" next action is action "+str(new_goal))
				rospy.loginfo("Waiting for pick Service...")
				# Pick object with Aruco marker
				rospy.wait_for_service('/pick_gui')
				rospy.loginfo("Service ready. Starting /pick_gui...")
				pickclient = rospy.ServiceProxy('/pick_gui', Empty)
				pickclient()
				data = open("../pick_test/"+str(rospy.get_param('test_name'))+".txt", "a")
				data.write("Time = "+str((rospy.Time.now()-t0).to_sec())+": Pick server finished\n")
				data.write("Pick duration: "+str((rospy.Time.now()-tp).to_sec())+"\n")
				data.close()
				rospy.loginfo("Pick server finished. Duration: "+str((rospy.Time.now()-tp).to_sec()))
				try:
					aruco_detect = rospy.wait_for_message('/detected_aruco_pose', PoseStamped, timeout=0.5)
				except rospy.ROSException:
					return
				planner.find_next_move()
			else:
				pose_goal = new_goal
				# move_base action server travels till (x,y) coordinates
				SendGoal(pose_goal, t, GoalPublisher)
				GoalPublisher.wait_for_result()
				# the check_yaw function sets the orientation 
				rospy.loginfo("Start check_yaw")
				check_yaw(pose_goal[2])
				rospy.loginfo("Current robot position : "+str(robot_pose[1]))
				data = open("../pick_test/"+str(rospy.get_param('test_name'))+".txt", "a")
				data.write("Current robot position : "+str(robot_pose[1])+"\n")
				data.close()
				rospy.loginfo("Goal "+str(pose_goal)+" reached")
				planner.find_next_move()
			if pose_goal == planner.next_move:
				data = open("../pick_test/"+str(rospy.get_param('test_name'))+".txt", "a")
				data.write("Mission duration : "+str((rospy.Time.now()-t0).to_sec())+"\n")
				data.close()
				rospy.loginfo("Mission duration : "+str((rospy.Time.now()-t0).to_sec()))
				return
		except rospy.ROSInterruptException:
			data = open("../pick_test/"+str(rospy.get_param('test_name'))+".txt", "a")
			data.write("Stopped by user\n")
			data.close()
			pass
