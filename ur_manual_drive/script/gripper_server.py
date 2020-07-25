#!/usr/bin/env python

# Author: Abdur Rosyid
# Email: abdoorasheed@gmail.com
# Website: https://abdurrosyid.com

from manual_drive.srv import Attach, AttachResponse
import rospy
import serial
import serial.tools.list_ports

class testV:
    rr=0

ports = list(serial.tools.list_ports.comports())
for p in ports:
    print p
ser = serial.Serial(p[0], 9600)    

def ActuateGripperCallback(req):
    
    if req.OnOff== True:
        commandtosend="1"
        gripperstatus="on"
        ser.write(str(commandtosend))
        testV.rr = 0
    else:
        commandtosend="2"
        gripperstatus="off"
        testV.rr = testV.rr + 1
        if testV.rr <= 2:
            ser.write(str(commandtosend))

    
	print "Sending to serial port [%s] to turn the gripper [%s]"%(commandtosend, gripperstatus)

    return AttachResponse(req.OnOff)

def gripper_server():
    rospy.init_node('GripperActuationServer')
    s = rospy.Service('GripperOnOFF', Attach, ActuateGripperCallback)
    print "Ready to Actuate Gripper"
    rospy.spin()

if __name__ == "__main__":
    gripper_server()
