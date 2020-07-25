#!/usr/bin/env python

# Author: Abdur Rosyid
# Email: abdoorasheed@gmail.com
# Website: https://abdurrosyid.com

from magnetic_gripper.srv import Attach, AttachResponse
import rospy

def ActuateGripperCallback(req):
    if req.OnOff== True:
        gripperstatus="on"
    else:
        gripperstatus="off"
    print "Gripper is [%s]"%(gripperstatus)
    return AttachResponse(req.OnOff)

def gripper_server():
    rospy.init_node('GripperActuationServer')
    s = rospy.Service('GripperOnOFF', Attach, ActuateGripperCallback)
    print "Ready to Actuate Gripper"
    rospy.spin()

if __name__ == "__main__":
    gripper_server()
