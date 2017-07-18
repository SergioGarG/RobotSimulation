#!/usr/bin/env python
import random
from robot_fts import *
from ltl_planner_motact_aruco import *
from moveit_msgs.msg import PickupActionFeedback
from control_msgs.msg import JointTrajectoryControllerState, FollowJointTrajectoryActionGoal
from gazebo_msgs.msg import ModelState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseWithCovarianceStamped
from gazebo_msgs.srv import SetModelState
from math import pi



def mission_gen():
	global amcl_init_pose 
	data = open("../pick_test/"+str(rospy.get_param('test_name'))+".txt", "a")
	data.write("\n----------------New test----------------\n")
	SetPose = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
	aruco_range_x=[-1.75,-0.25] 
	aruco_range_y=[[-11.4,-10.6], [-9.4,-8.6], [-7.4,-6.6]]
	chair_range=[[1,n] for n in range(-9,-4)]+[[0, n] for n in range(-6, -4)]+ [[0,-8], [0,-10], [0,-12], [1.5,-1.5]]
	i=random.randint(0,2)
	x_aruco=random.uniform(aruco_range_x[0]+0.1, aruco_range_x[1]-0.1)
	y_aruco=random.uniform(aruco_range_y[i][0]+0.1, aruco_range_y[i][1]-0.1)
	(x_chair, y_chair)=chair_range[random.randint(0, 10)]
	print '----------------------------'
	data.write("New Cube position : ["+str(x_aruco)+", "+str(y_aruco)+"]\n")
	data.write("Obstacle position : ["+str(x_chair)+", "+str(y_chair)+"]\n")
	print 'New Cube position : [%s, %s]' %(str(x_aruco), str(y_aruco))
	print 'Obstacle position : [%s, %s]' %(str(x_chair), str(y_chair))
	print '----------------------------'
	aruco_object = ModelState()
	aruco_object.model_name = 'aruco_cube'
	aruco_object.pose.position.x = x_aruco
	aruco_object.pose.position.y = y_aruco
	aruco_object.pose.position.z = 1.001
	aruco_object.reference_frame = 'world'
	chair = ModelState()
	chair.model_name = 'dining_chair'
	chair.pose.position.x = x_chair
	chair.pose.position.y = y_chair
	chair.pose.position.z = 0.0
	chair.reference_frame = 'world'
	SetPose(aruco_object)
	SetPose(chair)
	dist = map(abs, [aruco_range_x[0]-x_aruco, aruco_range_y[i][0]-y_aruco, aruco_range_x[1]-x_aruco, aruco_range_y[i][1]-y_aruco])
	data.write("Distance from table edge : "+str(min(dist))+"\n")
	data.write("Initial position : "+str(amcl_init_pose)+"\n")
	print 'Distance from table edge : %s' %str(min(dist))
	print 'init_pose = %s' %str(amcl_init_pose)
	xr = {0: aruco_range_x[0]-0.38, 1: x_aruco, 2: aruco_range_x[1]+0.38, 3: x_aruco}.get(dist.index(min(dist)), 0)
	yr = {0: y_aruco, 1: aruco_range_y[i][0]-0.38, 2: y_aruco, 3: aruco_range_y[i][1]+0.38}.get(dist.index(min(dist)), 0)
	yawr = dist.index(min(dist))*pi/2
	data.write("Robot pose goal (x, y, Y) : ("+str(xr)+", "+str(yr)+", "+str(yawr)+")\n")
	rospy.loginfo("---------------------\nRobot pose goal (x, y, Y) generated : ("+str(xr)+", "+str(yr)+", "+str(yawr)+")\n")
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

def robot_home(head_cmd):
	client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction) 
	client.wait_for_server()
	goal = PlayMotionGoal()
	goal.motion_name = 'home'
	goal.skip_planning = False
	print 'Sending actionlib goal with motion: %s' %str(goal.motion_name)
	client.send_goal(goal)
	print 'Waiting for result...'
	action_ok = client.wait_for_result(rospy.Duration(30.0))
	state = client.get_state()
	if state >= 1:
		print 'Successful execution of action %s' %goal.motion_name
	
	rospy.loginfo("Moving head up")
	jt = JointTrajectory()
	jt.joint_names = ['head_1_joint', 'head_2_joint']
	jtp = JointTrajectoryPoint()
	jtp.positions = [0.0, 0.0]
	jtp.time_from_start = rospy.Duration(2.0)
	jt.points.append(jtp)
	head_cmd.publish(jt)
	rospy.loginfo("Done.")



if __name__ == '__main__':
	
	rospy.set_param('test_name', str(sys.argv[1]))
	robot_name='TIAGo'
	rospy.init_node('ltl_planner_%s' %robot_name)
    ########
	rospy.Subscriber('pickup/feedback', PickupActionFeedback, GraspFeedbackCallback)
	rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, RobotPoseCallback)
	rospy.Subscriber('gripper_controller/state', JointTrajectoryControllerState, GripperCallback)
	########
	head_cmd = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=1)

	#InitialPosePublisher = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size = 100)
	#for i in xrange(10):
		#SendInitialPose(InitialPosePublisher, init_pose, rospy.Time.now())
		#rospy.sleep(0.1)
		#print('Initial pose set to %s.' %str(init_pose))
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
	action_ok = client.wait_for_result(rospy.Duration(30.0))

	#########
	global gripper_ok
	planner_ok = True
	world = rospy.get_param('world_name')
	try:
		rospy.loginfo("New mission")
		robot_task = '<>(sim && pick)'
		print 'Robot task: %s.' %(str(robot_task))
		planner(robot_motion, amcl_init_pose, robot_action, robot_task)
		try:
			rospy.wait_for_message('/detected_aruco_pose', PoseStamped, timeout=1)
		except rospy.ROSException:
			planner_ok = False
		data = open("../pick_test/"+str(rospy.get_param('test_name'))+".txt", "a")
		print 'gripper_ok = %s, planner_ok = %s' %(str(gripper_ok), str(planner_ok))
		if planner_ok and gripper_ok:
			data.write("GRASP SUCCESSFUL\n")
			rospy.loginfo("GRASP SUCCESSFUL")
		elif planner_ok and not gripper_ok:
			data.write("GRASP FAIL : hand empty\n")
			rospy.loginfo("GRASP FAIL : hand empty\n")
		else:
			data.write("Aruco object not found. ABORTING\nRobot current position : ("+str(xr)+", "+str(yr)+", "+str(yawr)+")\n")
			rospy.loginfo("Aruco object not found. ABORTING\nRobot current position : ("+str(xr)+", "+str(yr)+", "+str(yawr)+")\n")
		data.write("----------------End test----------------\n")
		data.close()
		robot_home(head_cmd)
		del regions[pose_goal]
	except rospy.ROSInterruptException:
		rospy.loginfo("Interrupt: "+str(rospy.ROSInterruptException))
		pass
	rospy.set_param('robot_status', 'ready')
				
