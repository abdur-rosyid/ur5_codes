# ur5_codes
ROS packages to control UR5 robotic arm from Universal Robot

- ur_manual_drive: a ROS package which contains 1) teleoperation of UR5 in the joint space using joystick (gamepad) based on moveit and 2) service to activate and de-activate a gripper.
- ur5_preprogram: a ROS package which contains pre-programmed trajectories for storing blocks to and picking them from the storage platform with left, right, and middle compartments. First the joint angles corresponding to the storing/picking postures were recorded. After that, the recorded joint angles were inserted into the code. The trajectories are defined in the joint space considering its unique solution. In this code, a storing/picking trajectory is executed through a keyboard stroke. A long-range, Class 1 Bluetooth keyboard can be used to enable long-range tele-operation.
