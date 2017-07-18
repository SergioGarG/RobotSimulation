#!/usr/bin/env python
import random
from robot_fts import *
from ltl_planner_motact_aruco import *
from moveit_msgs.msg import PickupActionFeedback
from control_msgs.msg import JointTrajectoryControllerState, FollowJointTrajectoryActionGoal
from gazebo_msgs.msg import ModelState, ModelStates
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseWithCovarianceStamped
from gazebo_msgs.srv import SetModelState
from math import pi



def mission_gen():
	global amcl_init_pose 
	ArucoSetPose = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
	x_range=[-1.75,-0.25] 
	y_range=[[-11.4,-10.6], [-9.4,-8.6], [-7.4,-6.6]]
	i=random.randint(0,2)
	x=random.uniform(x_range[0]+0.1, x_range[1]-0.1)
	y=random.uniform(y_range[i][0]+0.1, y_range[i][1]-0.1)
	print '----------------------------'
	print 'New Cube position : [%s, %s]' %(str(x), str(y))
	print '----------------------------'
	aruco_object = ModelState()
	aruco_object.model_name = 'aruco_cube'
	aruco_object.pose.position.x = x
	aruco_object.pose.position.y = y
	aruco_object.pose.position.z = 1.001
	aruco_object.reference_frame = 'world'
	ArucoSetPose(aruco_object)
	dist = map(abs, [x_range[0]-x, y_range[i][0]-y, x_range[1]-x, y_range[i][1]-y])
	print 'Distance from table edge : %s' %str(min(dist))
	print 'init_pose = %s' %str(amcl_init_pose)
	xr = amcl_init_pose[0]-1
	yr = amcl_init_pose[1]-1
	yawr = dist.index(min(dist))*pi/2
	rospy.loginfo("---------------------\nRobot pose goal (x, y, Y) generated : ("+str(xr)+", "+str(yr)+", "+str(yawr)+")\n")
	return (xr, yr, yawr)
		

def RobotPoseCallback(posedata):
	global xr, yr, yawr
	xr = posedata.pose.pose.position.x
	yr = posedata.pose.pose.position.y
	yawr = (euler_from_quaternion([posedata.pose.pose.orientation.x, posedata.pose.pose.orientation.y, posedata.pose.pose.orientation.z, posedata.pose.pose.orientation.w]))[2]



if __name__ == '__main__':
	
	rospy.set_param('test_name', str(sys.argv[1]))
	robot_name='TIAGo'
	rospy.init_node('ltl_planner_%s' %robot_name)
    ########
	#rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, RobotPoseCallback)
	########
	head_cmd = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=1)

	global amcl_init_pose
	amcl_init_pose = [0,0,0]
	try:
		posedata = rospy.wait_for_message('gazebo/model_states',  ModelStates, timeout=2)
	except rospy.ROSException:
		pass
	i = posedata.name.index('tiago_steel')
	amcl_init_pose[0] = posedata.pose[i].position.x
	amcl_init_pose[1] = posedata.pose[i].position.y
	amcl_init_pose[2] = (euler_from_quaternion([posedata.pose[i].orientation.x, posedata.pose[i].orientation.y, posedata.pose[i].orientation.z, posedata.pose[i].orientation.w]))[2]
	
	pose_goal = mission_gen()
	regions[pose_goal] = set(['sim',])
	robot_motion = MotionFts(regions, ap, 'office' )
	robot_motion.set_initial(amcl_init_pose)
	robot_motion.add_full_edges(unit_cost = 0.1)
	robot_model = [robot_motion, amcl_init_pose, robot_action]

	#########
	world = rospy.get_param('world_name')
	try:
		rospy.loginfo("New mission")
		robot_task = '<>(sim && pick)'
		print 'Robot task: %s.' %(str(robot_task))
		planner(robot_motion, amcl_init_pose, robot_action, robot_task)
		try:
			rospy.wait_for_message('/detected_aruco_pose', PoseStamped, timeout=1)
		except rospy.ROSException:
			pass
		del regions[pose_goal]
		
		rospy.loginfo("Moving head up")
		jt = JointTrajectory()
		jt.joint_names = ['head_1_joint', 'head_2_joint']
		jtp = JointTrajectoryPoint()
		jtp.positions = [0.0, 0.0]
		jtp.time_from_start = rospy.Duration(2.0)
		jt.points.append(jtp)
		head_cmd.publish(jt)
		rospy.loginfo("Done.")
	except rospy.ROSInterruptException:
		rospy.loginfo("Interrupt: "+str(rospy.ROSInterruptException))
		pass
				
