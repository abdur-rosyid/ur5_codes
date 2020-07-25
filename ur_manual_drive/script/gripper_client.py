#! /usr/bin/env python

# Author: Abdur Rosyid
# Email: abdoorasheed@gmail.com
# Website: https://abdurrosyid.com

import sys 
import rospy
from manual_drive.srv import Attach, AttachRequest
 
# init a node as usual
rospy.init_node('GripperActuationClient')
 
# wait for this sevice to be running
rospy.wait_for_service('GripperOnOFF')
 
# Create the connection to the service. Remember it's an Attach service
gripper_on_off = rospy.ServiceProxy('GripperOnOFF', Attach)
 
# Create an object of the type AttachRequest. 
on_off = AttachRequest(True)
 
# Now send the request through the connection
result = gripper_on_off(on_off)
 
# Done
print result


 

