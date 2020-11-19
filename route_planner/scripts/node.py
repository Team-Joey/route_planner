#!/usr/bin/python

import 	rospy
import 	route_planner
import 	route_planner.rp


class RoutePlannerNode(object):
    def __init__(self):
	rospy.loginfo("Success! init function in node.py.") # Information is only logged after a node instance has been created
	print("Initialisation successful.")


if __name__ == '__main__':
    rospy.init_node('dummy_node', anonymous = True) 	# (anonymous = True) ensures the name is unique for each node 
    node = RoutePlannerNode() 				# Node initialisation
    rospy.loginfo("Success! Main function in node.py.")
    
    task_incomplete = True
    while task_incomplete:				# Node is active until the task has been completed
	rospy.spin()
    
    print("End.")
