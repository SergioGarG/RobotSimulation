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
	yaw_goal = yaw_goal%(2*PI)
	print 'yaw_goal %s' %str(yaw_goal)
	print 'yaw_robot %s' %str((robot_pose[1][2])%(2*PI))
	reach_yaw_bound = 0.1*PI
	VelPublisher = rospy.Publisher('mobile_base_controller/cmd_vel', Twist, queue_size = 10)
	ang = Twist()
	while (abs((robot_pose[1][2])%(2*PI)-yaw_goal) > reach_yaw_bound):
		ang.angular.z = copysign(-1, yaw_goal<(robot_pose[1][2])%(2*PI)) # return 1 or -1
		VelPublisher.publish(ang)
		rospy.sleep(0.1)
	ang.angular.z = 0
	VelPublisher.publish(ang)




def PoseCallback(posedata):
	reach_xy_bound = 0.1
    # PoseWithCovarianceStamped data from amcl_pose
	global robot_pose # [time, [x,y,yaw]]
	global GoalPublisher
	global current_goal
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
	print 'Robot %s: ltl_planner started!' %(robot_name)
    #----------
    #publish to
    #----------
	InitialPosePublisher = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size = 100)
  #   for i in xrange(10):
  #       SendInitialPose(InitialPosePublisher, init_pose, rospy.Time.now())
  #       rospy.sleep(0.1)
  #   print('Initial pose set to %s.' %str(init_pose))
	global GoalPublisher
	GoalPublisher = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)
	GoalPublisher.wait_for_server()
    #----------
    ####### robot information
	full_model = MotActModel(ts, act)
	planner = ltl_planner(full_model, robot_task, None)
    ####### initial plan synthesis
	planner.optimal(5)
    #######
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
				print 'the robot next_move is an action, currently not implemented for %s' %robot_name
				break
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
			if current_goal == planner.next_move:
				break
		except rospy.ROSInterruptException:
			pass



if __name__ == '__main__':
    robot_name='TIAGo'
    rospy.init_node('ltl_planner_%s' %robot_name)

    ########
   # if len(sys.argv) == 2:
    #    robot_task = str(sys.argv[1])
     #   print('Argument: %s.' %(str(sys.argv[1])))
        # to run: python ltl_planner.py '<> (r2 && <>r3)'
        # to run: python ltl_planner.py '([]<> r2) && ([]<> r3) && ([]<> r1)'
        # to run: python ltl_planner.py '<> r2 && ([]<> r3) && ([]<> r1)'
    ###############
    try:
	robot_task = rospy.get_param('plan')
	print('Robot task: %s.' %(str(robot_task)))
        [robot_motion, init_pose, robot_action] = robot_model
        planner(robot_motion, init_pose, robot_action, robot_task)
    except rospy.ROSInterruptException:
        pass

