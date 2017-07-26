#!/usr/bin/env python
import random
from robot_fts import *
from ltl_planner_motact_aruco import *
from moveit_msgs.msg import PickupActionFeedback
from control_msgs.msg import JointTrajectoryControllerState
from gazebo_msgs.msg import ModelState, ModelStates
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from gazebo_msgs.srv import SetModelState
from math import pi
from datetime import datetime


def mission_gen():
	global amcl_init_pose 
	data = open("../pick_test/"+str(rospy.get_param('test_name'))+".txt", "a")
	data.write(str(datetime.now())+"\n")
	aruco_range_x=[-1.75,-0.25] 
	aruco_range_y=[[-11.4,-10.6], [-9.4,-8.6], [-7.4,-6.6]]
	i=random.randint(0,2)
	x_aruco=random.uniform(aruco_range_x[0]+0.1, aruco_range_x[1]-0.1)
	y_aruco=random.uniform(aruco_range_y[i][0]+0.1, aruco_range_y[i][1]-0.1)
	set_object_pose('aruco_cube', ((x_aruco, y_aruco, 1.001), 0.0))
	dist = map(abs, [aruco_range_x[0]-x_aruco, aruco_range_y[i][0]-y_aruco, aruco_range_x[1]-x_aruco, aruco_range_y[i][1]-y_aruco])	
	print '---------------------------------'
	data.write("New Cube position : ("+str(x_aruco)+", "+str(y_aruco)+")\n")
	print 'New Cube position : (%s, %s), distance from table edge : %s' %(str(x_aruco), str(y_aruco), str(min(dist)))
	
	if 'dining_chair' in rospy.wait_for_message('/gazebo/model_states', ModelStates).name:
		chair_range=[[1,n] for n in range(-9,-4)]+[[0, n] for n in range(-6, -4)]+ [[0,-8], [0,-10], [0,-12]]
		(x_chair, y_chair) = chair_range[random.randint(0, 9)]
		set_object_pose('dining_chair', ((x_chair, y_chair, 0.0), 0.0))
		data.write("Obstacle position : ("+str(x_chair)+", "+str(y_chair)+"), distance from table edge : "+str(min(dist))+"\n")
		print 'Obstacle position : (%s, %s)' %(str(x_chair), str(y_chair))

	print '---------------------------------'
	data.write("Initial position : "+str(amcl_init_pose)+"\n")
	rospy.loginfo ("Initial position : "+str(amcl_init_pose))
	xr = {0: aruco_range_x[0]-0.38, 1: x_aruco, 2: aruco_range_x[1]+0.38, 3: x_aruco}.get(dist.index(min(dist)), 0)
	yr = {0: y_aruco, 1: aruco_range_y[i][0]-0.38, 2: y_aruco, 3: aruco_range_y[i][1]+0.38}.get(dist.index(min(dist)), 0)
	yawr = dist.index(min(dist))*pi/2
	data.write("Robot pose goal (x, y, Y) : ("+str(xr)+", "+str(yr)+", "+str(yawr)+")\n")
	rospy.loginfo("Robot pose goal (x, y, Y) generated : ("+str(xr)+", "+str(yr)+", "+str(yawr)+")\n")
	data.close()
	return (xr, yr, yawr)
	

def GraspFeedbackCallback(graspdata):
	rospy.loginfo(graspdata.status.text)
		

def GripperCallback(gripperdata):
	global gripper_ok
	gripper_ok = (gripperdata.actual.positions[0] + gripperdata.actual.positions[1]) > 0.04
	
	
def RobotPoseCallback(posedata):
	global xr, yr, yawr
	xr = posedata.pose.pose.position.x
	yr = posedata.pose.pose.position.y
	yawr = (euler_from_quaternion([posedata.pose.pose.orientation.x, posedata.pose.pose.orientation.y, posedata.pose.pose.orientation.z, posedata.pose.pose.orientation.w]))[2]


def robot_home():
	head_cmd = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=1)
	client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction) 
	client.wait_for_server()
	goal = PlayMotionGoal()
	goal.motion_name = 'home'
	goal.skip_planning = False
	rospy.loginfo("Sending actionlib goal with motion : home")
	client.send_goal(goal)
	rospy.loginfo("Waiting for result...")
	client.wait_for_result(rospy.Duration(30.0))
	rospy.loginfo("Moving head up...")
	jt = JointTrajectory()
	jt.joint_names = ['head_1_joint', 'head_2_joint']
	jtp = JointTrajectoryPoint()
	jtp.positions = [0.0, 0.0]
	jtp.time_from_start = rospy.Duration(2.0)
	jt.points.append(jtp)
	head_cmd.publish(jt)
	rospy.loginfo("Done.")
	rospy.loginfo("Going back to (4.00, -4.00, 3.14)")
	GoalPublisher = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)
	pose_goal = (4.00, -4.00, 3.14)
	SendGoal(pose_goal, rospy.Time.now(), GoalPublisher)
	if GoalPublisher.get_state() == 4:
		rospy.loginfo("Planning fail. Position home not reached. Retrying...\n")
		SendGoal(pose_goal, rospy.Time.now()-t0, GoalPublisher)
	rospy.loginfo("Robot moved to home\nRobot current position : ("+str(xr)+", "+str(yr)+", "+str(yawr)+")")
	data = open("../pick_test/"+str(rospy.get_param('test_name'))+".txt", "a")
	data.write("Robot moved to home\nCurrent robot position : ("+str(xr)+", "+str(yr)+", "+str(yawr)+")\n")
	data.close()
	
	
def pose_tables():
	gazebo_model_states = rospy.wait_for_message('/gazebo/model_states', ModelStates)
	pose_table = (gazebo_model_states.pose[gazebo_model_states.name.index('table')].position.x, gazebo_model_states.pose[gazebo_model_states.name.index('table')].position.y)
	pose_table_0 = (gazebo_model_states.pose[gazebo_model_states.name.index('table_0')].position.x, gazebo_model_states.pose[gazebo_model_states.name.index('table_0')].position.y)
	pose_table_1 = (gazebo_model_states.pose[gazebo_model_states.name.index('table_1')].position.x, gazebo_model_states.pose[gazebo_model_states.name.index('table_1')].position.y)
	return pose_table, pose_table_0, pose_table_1
	
	
def set_object_pose(name, pose):
	# pose = ((x, y, z), Y)
	SetPose = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
	obj = ModelState()
	obj.model_name = name
	obj.pose.position.x = pose[0][0]
	obj.pose.position.y = pose[0][1]
	obj.pose.position.z = pose[0][2]
	quaternion = quaternion_from_euler(0, 0, pose[1])
	obj.pose.orientation.x = quaternion[0]
	obj.pose.orientation.y = quaternion[1]
	obj.pose.orientation.z = quaternion[2]
	obj.pose.orientation.w = quaternion[3]
	obj.reference_frame = 'world'
	SetPose(obj)



if __name__ == '__main__':
	
	rospy.set_param('test_name', str(sys.argv[1]))
	robot_name='TIAGo'
	rospy.init_node('ltl_planner_%s' %robot_name)
	print robot_name
    ########
	rospy.Subscriber('pickup/feedback', PickupActionFeedback, GraspFeedbackCallback)
	rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, RobotPoseCallback)
	rospy.Subscriber('gripper_controller/state', JointTrajectoryControllerState, GripperCallback)
	pose_init_table, pose_init_table_0, pose_init_table_1 = pose_tables()
	########
	data = open("../pick_test/"+str(rospy.get_param('test_name'))+".txt", "a")
	data.write("\n----------------New test----------------\n")
	data.close()
	global amcl_init_pose
	amcl_init_pose = init_pose()
	pose_goal = mission_gen()
	regions[pose_goal] = set(['sim',])
	robot_motion = MotionFts(regions, ap, 'office' )
	robot_motion.set_initial(amcl_init_pose)
	robot_motion.add_full_edges(unit_cost = 0.1)
	robot_model = [robot_motion, amcl_init_pose, robot_action]
	client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction) 
	client.wait_for_server()
	goal = PlayMotionGoal()
	goal.motion_name = 'close_gripper'
	goal.skip_planning = False
	goal.priority = 0  # Optional
	print 'Sending actionlib goal with motion: %s' %str(goal.motion_name)
	client.send_goal(goal)
	print 'Waiting for result...'
	client.wait_for_result(rospy.Duration(30.0))

	#########
	global gripper_ok
	aruco_ok = True
	world = rospy.get_param('world_name')
	try:
		rospy.loginfo("New mission")
		robot_task = '<>(sim && pick)'
		print 'Robot task: %s.' %(str(robot_task))
		planner(robot_motion, amcl_init_pose, robot_action, robot_task)
		try:
			new_aruco_pose = rospy.wait_for_message('/detected_aruco_pose', PoseStamped, timeout=1)
			if new_aruco_pose.header.frame_id == "old_base_footprint":
				rospy.logwarn("Aruco cube not detected")
				aruco_ok = False
			else:
				new_aruco_pose.header.frame_id = "old_base_footprint"
		except rospy.ROSException:
			rospy.logwarn("Aruco cube not detected")
			aruco_ok = False

		data = open("../pick_test/"+str(rospy.get_param('test_name'))+".txt", "a")
		print 'gripper_ok = %s, aruco_ok = %s' %(str(gripper_ok), str(aruco_ok))
		
		if aruco_ok and gripper_ok:
			data.write("GRASP SUCCESSFUL\n")
			rospy.loginfo("GRASP SUCCESSFUL")
		elif aruco_ok and not gripper_ok:
			data.write("GRASP FAIL : hand empty\n")
			rospy.logwarn("GRASP FAIL : hand empty\n")
		else:
			data.write("Aruco object not found. ABORTING\nRobot current position : ("+str(xr)+", "+str(yr)+", "+str(yawr)+")\n")
			rospy.logwarn("Aruco object not found. ABORTING")
			rospy.loginfo("Robot current position : ("+str(xr)+", "+str(yr)+", "+str(yawr)+")")
			
		del regions[pose_goal]
		rospy.loginfo("Mission finished\n==============================")
		data.close()
		robot_home()
	except rospy.ROSInterruptException:
		pass
	rospy.set_param('robot_status', 'ready')
	pose_table, pose_table_0, pose_table_1 = pose_tables()
	rospy.loginfo("shift_table_  : "+str(norm2(pose_init_table  , pose_table  )))
	rospy.loginfo("shift_table_0 : "+str(norm2(pose_init_table_0, pose_table_0)))
	rospy.loginfo("shift_table_1 : "+str(norm2(pose_init_table_1, pose_table_1)))
	set_object_pose('table'  , (pose_init_table   + (0.0,),0.0))
	set_object_pose('table_0', (pose_init_table_0 + (0.0,),0.0))
	set_object_pose('table_1', (pose_init_table_1 + (0.0,),0.0))
	data = open("../pick_test/"+str(rospy.get_param('test_name'))+".txt", "a")
	data.write("----------------End test----------------\n")
	data.close()
		
		
