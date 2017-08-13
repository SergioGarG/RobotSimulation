#!/usr/bin/env python

from robot_fts import *
from ltl_planner_motact_aruco import *


if __name__ == '__main__':
		
	robot_name='TIAGo'
	rospy.init_node('ltl_planner_%s' %robot_name)
	init_pose = init_pose()
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
			
		robot_motion = MotionFts(regions, ap, 'office' )
		robot_motion.set_initial(init_pose)
		robot_motion.add_full_edges(unit_cost = 0.1)
		try:
			robot_task = rospy.get_param('plan')
			if str(robot_task) == 'none':
				sys.exit()
			print('Robot task: %s.' %(str(robot_task)))
			planner(robot_motion, init_pose, robot_action, robot_task)
		except rospy.ROSInterruptException:
			pass
