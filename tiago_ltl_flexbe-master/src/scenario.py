#!/usr/bin/env python

import rospy
import roslib

def small_office():
	
	# Displaying the interface in the terminal
	print('------------------------------\nSelect an action :')
	print('0. Type')
	print('1. r1		4. r4\n2. r2		5. r5\n3. r3		6. r6')
	print('7. r4 -> r5  8. tuto\n9. sim')
	print('q. Exit')
	# Asking the scenario to execute
	dest = raw_input()
	# Returning the selected scenario
	return {
		'0': str(raw_input()),
        '1': '<>r1',
        '2': '<>r2',
        '3': '<>r3',
        '4': '<>r4',
        '5': '<>r5',
        '6': '<>r6',
        '7': '<>((r4 && pick_from_floor) && <>(r5 && reach_max))',
        '8': '<>((r2 && pick_from_floor) && <>(r3 && reach_max))',
        '9': '<>(sim && pick)',
        'q': 'none'
	}.get(dest, 0)
    
    
def tutorial_office():

	print('------------------------------\nSelect an action :')
	print('1. Charging 		4. Pick\n2. Pick a coke		5. Table\n3. Request	6. Pick')
	print('q. Exit')
	dest = raw_input()
	return {
		'1': '<>charging',
		'2': '<>(coke && <>(person && <> charging))',
		'3': '<>(person && <>charging)',
		'4': '<>(coke && pick)',
		'5': '<>coke',
		'6': '<>pick',
		'q': 'none'
	}.get(dest, 0)
	
def tabletop_cube():
	
	print('------------------------------\nSelect an action :')
	print('1. Pick')
	print('2. test')
	print('q. Exit')
	dest = raw_input()
	return {
		'1': '<>(aruco && pick)',
		'2': '<>test',
		'q': 'none'
	}.get(dest, 0)


def scenario (world):
	
	dest = {'small_office' : small_office,
			'tutorial_office' : tutorial_office,
			'tabletop_cube' : tabletop_cube,
	}[world]()
	while dest==0:
		print('------------------------------')
		print 'Error : nonexistent input'
		dest = {'small_office' : small_office,
			'tutorial_office' : tutorial_office,
			'tabletop_cube' : tabletop_cube,
		}[world]()
	rospy.set_param('plan', str(dest))
	
	
