#!/usr/bin/env python

# Author: Abdur Rosyid
# Email: abdoorasheed@gmail.com
# Website: https://abdurrosyid.com

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

## PLAN TO JOINT WAYPOINTS

from std_msgs.msg import String

def plan_joint_waypoints():

    ## First initialize moveit_commander and rospy.
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('plan_joint_state', anonymous=True)

    ## Instantiate a RobotCommander object.  This object is an interface to the robot as a whole.
    robot = moveit_commander.RobotCommander()

    ## Instantiate a PlanningSceneInterface object.  This object is an interface to the world surrounding the robot.
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a MoveGroupCommander object.  This object is an interface to one group of joints.  
    group = moveit_commander.MoveGroupCommander("UR5")
    #group = moveit_commander.MoveGroupCommander("ur5_arm")

    ## BLUE LEFT COMPARTMENT

    ## Joint waypoint 1 
    joint_waypoint = group.get_current_joint_values()
    joint_waypoint[0] = 3.12448843
    joint_waypoint[1] = -1.80100526
    joint_waypoint[2] = -0.21485003
    joint_waypoint[3] = -2.68815611
    joint_waypoint[4] = 1.5833627
    joint_waypoint[5] = 0.0
    group.set_joint_value_target(joint_waypoint)
    #plan_joint_waypoint = group.plan()
    group.go(wait=True)

    ## Joint waypoint 2 
    joint_waypoint = group.get_current_joint_values()
    joint_waypoint[0] = 3.1244897
    joint_waypoint[1] = -1.86436071
    joint_waypoint[2] = -0.62395521
    joint_waypoint[3] = -2.1956242
    joint_waypoint[4] = 1.5053465
    joint_waypoint[5] = 0.97406826
    group.set_joint_value_target(joint_waypoint)
    #plan_joint_waypoint = group.plan()
    group.go(wait=True)

    ## Joint waypoint 3 
    joint_waypoint = group.get_current_joint_values()
    joint_waypoint[0] = 5.6173422
    joint_waypoint[1] = -2.30837247
    joint_waypoint[2] = 0.04974188
    joint_waypoint[3] = -2.43421071
    joint_waypoint[4] = 1.5039502
    joint_waypoint[5] = 1.1124729
    group.set_joint_value_target(joint_waypoint)
    #plan_joint_waypoint = group.plan()
    group.go(wait=True)

if __name__=='__main__':
    try:
        plan_joint_waypoints()
    except rospy.ROSInterruptException:
        pass





