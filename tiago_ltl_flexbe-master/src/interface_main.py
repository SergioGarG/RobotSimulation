#!/usr/bin/env python

from robot_fts import robot_model
from ltl_planner_motact_aruco import *


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
	while not rospy.is_shutdown():
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