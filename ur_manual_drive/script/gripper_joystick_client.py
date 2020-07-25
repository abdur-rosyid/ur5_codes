#! /usr/bin/env python

# Author: Abdur Rosyid
# Email: abdoorasheed@gmail.com
# Website: https://abdurrosyid.com

import sys 
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from manual_drive.srv import Attach, AttachRequest
 
class gripper_joy_client():
    def __init__(self):
        global gripper_on_off
        # Name this node, it must be unique
	rospy.init_node('gripper_joy_client', anonymous=False)
	
	# Subscribe to /joy and Publish /joy_gripper/state
	rospy.Subscriber("/joy", Joy, self.callback)
	self.gripper_on_off = rospy.Publisher('/joy_gripper/state', Bool, queue_size=5)

        # wait for this sevice to be running
        rospy.wait_for_service('GripperOnOFF')
 
        # Create the connection to the service. Remember it's an Attach service
        gripper_on_off = rospy.ServiceProxy('GripperOnOFF', Attach)

    def callback(self, joy):
        global on_off
        global result
        global gripper_on_off
	gripper_state = Bool()
	gripper_state.data = joy.buttons[3]	# button Y
	if gripper_state.data == 1:
	    on_off = AttachRequest(True)
            result = gripper_on_off(on_off)
	    print result
	else:
	    on_off = AttachRequest(False)
            result = gripper_on_off(on_off)
	    print result
	    
if __name__ == '__main__':
    try:
	gripper_joy_client()
	rospy.spin()
    except KeyboardInterrupt:
	print("Shuting down gripper_joy_client node")



 

