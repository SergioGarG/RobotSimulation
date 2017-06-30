from ltl_tools.ts import MotionFts, ActionModel, MotActModel
from ltl_tools.planner import ltl_planner

# export PYTHONPATH=$PYTHONPATH:/to/your/P_MAS_TG


import time


##############################
# motion FTS
ap = {'r1', 'r2', 'r3', 'r4', 'r5', 'r6', }

regions = {   #Small_office
			  (0.57, -10.01, 0.95): set(['r1',]),
              (-3.25, -0.31, 0.82): set(['r2',]),
              (-2.89, -9.01, 0.52): set(['r3',]), # (0.63, -9.51, 0.52)
              (1.00, -4.50, 3.14): set(['r4',]),
              (2.00, -6.50, 0.52): set(['r5',]),
              (0.65, -7.58, -0.52): set(['r6',]),
              #Tutorial_office
              (-4.15, 9.20, 0.00): set(['charging',]),
              (-2.51, 6.20, 2.50): set(['person',]),
              (1.63, -2.84, 0.00): set(['coke',]),
              #tabletop_cube
              (0.00, 0.00, 0.00): set(['acuro',]),
              (-1.00, -1.00, 0.00): set(['test',]),
              
}

init_pose = (0.57, -10.01, 0.95)
robot_motion = MotionFts(regions, ap, 'office' )
robot_motion.set_initial((0.57, -10.01, 0.95))
robot_motion.add_full_edges(unit_cost = 0.1)


##############################
# action FTS
############# no action model
# action = dict()
############# with action	
# for supported actions in play_motion
# see http://wiki.ros.org/Robots/TIAGo/Tutorials/motions/play_motion

#action = { 'pick': (100, 'r', set(['pick'])),
#           'drop': (50, '1', set(['drop']))
#}

action = {'pick_from_floor': (10, '1', set(['pick_from_floor',])),
          'reach_max': (10, '1', set(['reach_max',])),
          'pick': (10, '1', set(['pick',]))}


robot_action = ActionModel(action)

robot_model = [robot_motion, init_pose, robot_action]
##############################
# complete robot model
# robot_full_model = MotActModel(robot_motion, robot_action)



##############################
# specify tasks
########## only hard
# hard_task = '<>(r1 && <> (r2 && <> r6)) && (<>[] r6)'
#hard_task = '(<>(pick && <> drop)) && ([]<> r3) && ([]<> r6)'
#soft_task = None


########## soft and hard
# hard_task = '(([]<> r3) && ([]<> r4))'
# soft_task = '([]! b)'

