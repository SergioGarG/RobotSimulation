#!/usr/bin/env python
import roslib
import numpy
import Queue
import rospy
import sys
import time
import actionlib
import scenario

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
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
	reach_yaw_bound = 0.15
	VelPublisher = rospy.Publisher('mobile_base_controller/cmd_vel', Twist, queue_size = 10)
	vel = Twist()
	diff = yaw_goal-robot_pose[1][2]
	while (abs(atan2(sin(diff),cos(diff))) > reach_yaw_bound):
		diff = yaw_goal-robot_pose[1][2]
		vel.angular.z = 0.5*copysign(1, atan2(sin(diff),cos(diff))) # return 1 or -1
		VelPublisher.publish(vel)
		rospy.sleep(0.1)
	vel.angular.z = 0
	VelPublisher.publish(vel)



def PoseCallback(posedata):
	# PoseWithCovarianceStamped data from amcl_pose
	global robot_pose # [time, [x, y, yaw]]
	header = posedata.header
	pose = posedata.pose
	if (not robot_pose[0]) or (header.stamp > robot_pose[0]):
		# more recent pose data received
		robot_pose[0] = header.stamp
		euler = euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]) #roll, pitch, yaw
		robot_pose[1] = [pose.pose.position.x, pose.pose.position.y, euler[2]] # in radians

	

def SendGoal(goal , time_stamp, GoalPublisher):
	global robot_pose
	pose_goal = goal
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
	while not GoalPublisher.get_state() == 1: # 1 = Active
		rospy.Rate(5).sleep()
	reach_xy_bound = 0.1
	while (norm2(robot_pose[1][0:2], pose_goal[0:2]) > reach_xy_bound and (GoalPublisher.get_state() == 1)):
		rospy.Rate(5).sleep()
	if GoalPublisher.get_state() == 1:
		GoalPublisher.cancel_goal()
		check_yaw(pose_goal[2])


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
	#global pose_goal
	rospy.init_node('ltl_planner_%s' %robot_name)
	print 'Robot %s: ltl_planner started!' %(robot_name)
    #----------
    #publish to
    #----------
	GoalPublisher = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)
	GoalPublisher.wait_for_server()
    #----------
    #subscribe to
    #----------
	rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, PoseCallback)
    
    ####### robot information
	full_model = MotActModel(ts, act)
	planner = ltl_planner(full_model, robot_task, None)
    ####### initial plan synthesis
	planner.optimal(10)
    #######
	t0 = rospy.Time.now()
	while not rospy.is_shutdown():
		try:
			t = rospy.Time.now()-t0
			new_goal = planner.next_move
			rospy.loginfo("NEW GOAL: "+str(new_goal))
			if new_goal == 'pick':
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
				#planner.find_next_move() #a effacer
			else:
				pose_goal = new_goal
				# move_base action server travels till (x,y) coordinates
				rospy.loginfo("Robot "+str(robot_name)+" moving to "+str(pose_goal))
				SendGoal(pose_goal, rospy.Time.now()-t0, GoalPublisher)
				# the check_yaw function sets the orientation 
				rospy.loginfo("Current robot position : "+str(robot_pose[1]))
				data = open("../pick_test/"+str(rospy.get_param('test_name'))+".txt", "a")
				if GoalPublisher.get_state() == 4:
					rospy.logingo("Planning fail. Position not reached. Retrying...\n")
					data.write("Planning fail. Position not reached.\n")
				else:
					rospy.loginfo("Goal "+str(pose_goal)+" reached")
				data.write("Current robot position : "+str(robot_pose[1])+"\n")
				data.close()
			planner.find_next_move() #indent -1
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
