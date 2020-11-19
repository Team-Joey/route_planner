#!/usr/bin/python

import 	rospy
import 	route_planner
import 	route_planner.rp


class RoutePlannerNode(object):
    def __init__(self):
	rospy.loginfo("Success! init function in node.py.")



if __name__ == '__main__':
    rospy.loginfo("Success! Main function in node.py.")
    print("Dummy was successful. Module was connected.")
