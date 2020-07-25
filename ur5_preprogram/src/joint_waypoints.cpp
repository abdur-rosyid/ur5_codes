/* @author Abdur Rosyid and Ardiansyah Al Farouq  */

// CPP headers
#include <stdio.h>
#include <string.h>

#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/stat.h>

#include <iostream>

// ROS headers
#include <ros/ros.h>

// MoveIt! headers
#include <moveit/move_group_interface/move_group.h>

// Std C++ headers
#include <string>
#include <vector>
#include <map>

#define xData 	8888

/////////////////////////////////////////////////////////////
int datX=0;
static int *resultX;
void initData(){
	int KeyX = xData;
	int spaceIdX = shmget(KeyX, sizeof(int), IPC_CREAT | S_IRUSR | S_IWUSR);
	resultX = (int*)shmat(spaceIdX, NULL, 0);

	//while(1){

		datX = *resultX;
		printf("X(%4d)\n", datX);
	//}
}


void handler (int sig)
{
  printf ("nexiting...(%d)\n", sig);
  exit (0);
}

void perror_exit (char *error)
{
  perror (error);
  handler (9);
}

/////////////////////////////////////////////////////////////
void blue_left_compartment_top_pick(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> joints;

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  //moveit::planning_interface::MoveGroup group_arm("ur5_arm");
  //choose your preferred planner
  //group_arm.setPlannerId("SBLkConfigDefault");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  //group_arm.setPlannerId("ESTkConfigDefault");

  //arm_joint_names = group_arm.getJoints();
  group_arm.setMaxVelocityScalingFactor(1.0);

	joints["ur5_arm_shoulder_pan_joint"] = 5.526762;
	joints["ur5_arm_shoulder_lift_joint"] = -1.968783;
	joints["ur5_arm_elbow_joint"] = -1.026079;
	joints["ur5_arm_wrist_1_joint"] = -1.700999;
	joints["ur5_arm_wrist_2_joint"] = 1.5549145;
	joints["ur5_arm_wrist_3_joint"] = 0.956615;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 5.6173422;
        joints["ur5_arm_shoulder_lift_joint"] = -2.30837247;
        joints["ur5_arm_elbow_joint"] = 0.04974188;
        joints["ur5_arm_wrist_1_joint"] = -2.43421071;
        joints["ur5_arm_wrist_2_joint"] = 1.5039502;
        joints["ur5_arm_wrist_3_joint"] = 1.1124729;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 5.473178;
        joints["ur5_arm_shoulder_lift_joint"] = -1.86436071;
        joints["ur5_arm_elbow_joint"] = -0.62395521;
        joints["ur5_arm_wrist_1_joint"] = -2.1956242;
        joints["ur5_arm_wrist_2_joint"] = 1.5053465;
        joints["ur5_arm_wrist_3_joint"] = 0.97406826;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 3.12448843;
        joints["ur5_arm_shoulder_lift_joint"] = -1.80100526;
        joints["ur5_arm_elbow_joint"] = -0.21485003;
        joints["ur5_arm_wrist_1_joint"] = -2.68815611;
        joints["ur5_arm_wrist_2_joint"] = 1.5833627;
        joints["ur5_arm_wrist_3_joint"] = 0.0;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	sleep(2.0);

  spinner.stop();	
}

/////////////////////////////////////////////////////////////
void key_blue_left_compartment_top_store(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> joints;

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  //moveit::planning_interface::MoveGroup group_arm("ur5_arm");
  //choose your preferred planner
  //group_arm.setPlannerId("SBLkConfigDefault");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  //group_arm.setPlannerId("ESTkConfigDefault");

  //arm_joint_names = group_arm.getJoints();
  group_arm.setMaxVelocityScalingFactor(1.0);

        joints["ur5_arm_shoulder_pan_joint"] = 3.12448843;
        joints["ur5_arm_shoulder_lift_joint"] = -1.80100526;
        joints["ur5_arm_elbow_joint"] = -0.21485003;
        joints["ur5_arm_wrist_1_joint"] = -2.68815611;
        joints["ur5_arm_wrist_2_joint"] = 1.5833627;
        joints["ur5_arm_wrist_3_joint"] = 0.0;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 5.473178;
        joints["ur5_arm_shoulder_lift_joint"] = -1.86436071;
        joints["ur5_arm_elbow_joint"] = -0.62395521;
        joints["ur5_arm_wrist_1_joint"] = -2.1956242;
        joints["ur5_arm_wrist_2_joint"] = 1.5053465;
        joints["ur5_arm_wrist_3_joint"] = 0.97406826;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 5.6173422;
        joints["ur5_arm_shoulder_lift_joint"] = -2.30837247;
        joints["ur5_arm_elbow_joint"] = 0.04974188;
        joints["ur5_arm_wrist_1_joint"] = -2.43421071;
        joints["ur5_arm_wrist_2_joint"] = 1.5039502;
        joints["ur5_arm_wrist_3_joint"] = 1.1124729;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 5.526762;
        joints["ur5_arm_shoulder_lift_joint"] = -1.968783;
        joints["ur5_arm_elbow_joint"] = -1.026079;
        joints["ur5_arm_wrist_1_joint"] = -1.700999;
        joints["ur5_arm_wrist_2_joint"] = 1.5549145;
        joints["ur5_arm_wrist_3_joint"] = 0.956615;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	sleep(2.0);

  spinner.stop();	
}

/////////////////////////////////////////////////////////////
void blue_left_compartment_middle_pick(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> joints;

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  //moveit::planning_interface::MoveGroup group_arm("ur5_arm");
  //choose your preferred planner
  //group_arm.setPlannerId("SBLkConfigDefault");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  //group_arm.setPlannerId("ESTkConfigDefault");

  //arm_joint_names = group_arm.getJoints();
  group_arm.setMaxVelocityScalingFactor(1.0);

        joints["ur5_arm_shoulder_pan_joint"] = 5.560621;
        joints["ur5_arm_shoulder_lift_joint"] = -2.003639;
        joints["ur5_arm_elbow_joint"] = -1.4927807;
        joints["ur5_arm_wrist_1_joint"] = -1.196773;
        joints["ur5_arm_wrist_2_joint"] = 1.568703;
        joints["ur5_arm_wrist_3_joint"] = 0.956615;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 5.6173422;
        joints["ur5_arm_shoulder_lift_joint"] = -2.30837247;
        joints["ur5_arm_elbow_joint"] = 0.04974188;
        joints["ur5_arm_wrist_1_joint"] = -2.43421071;
        joints["ur5_arm_wrist_2_joint"] = 1.5039502;
        joints["ur5_arm_wrist_3_joint"] = 1.1124729;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 5.473178;
        joints["ur5_arm_shoulder_lift_joint"] = -1.86436071;
        joints["ur5_arm_elbow_joint"] = -0.62395521;
        joints["ur5_arm_wrist_1_joint"] = -2.1956242;
        joints["ur5_arm_wrist_2_joint"] = 1.5053465;
        joints["ur5_arm_wrist_3_joint"] = 0.97406826;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 3.12448843;
        joints["ur5_arm_shoulder_lift_joint"] = -1.80100526;
        joints["ur5_arm_elbow_joint"] = -0.21485003;
        joints["ur5_arm_wrist_1_joint"] = -2.68815611;
        joints["ur5_arm_wrist_2_joint"] = 1.5833627;
        joints["ur5_arm_wrist_3_joint"] = 0.0;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	sleep(2.0);

  spinner.stop();
}

/////////////////////////////////////////////////////////////
void  blue_left_compartment_middle_store(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> joints;

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  //moveit::planning_interface::MoveGroup group_arm("ur5_arm");
  //choose your preferred planner
  //group_arm.setPlannerId("SBLkConfigDefault");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  //group_arm.setPlannerId("ESTkConfigDefault");

  //arm_joint_names = group_arm.getJoints();
  group_arm.setMaxVelocityScalingFactor(1.0);

        joints["ur5_arm_shoulder_pan_joint"] = 3.12448843;
        joints["ur5_arm_shoulder_lift_joint"] = -1.80100526;
        joints["ur5_arm_elbow_joint"] = -0.21485003;
        joints["ur5_arm_wrist_1_joint"] = -2.68815611;
        joints["ur5_arm_wrist_2_joint"] = 1.5833627;
        joints["ur5_arm_wrist_3_joint"] = 0.0;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 5.473178;
        joints["ur5_arm_shoulder_lift_joint"] = -1.86436071;
        joints["ur5_arm_elbow_joint"] = -0.62395521;
        joints["ur5_arm_wrist_1_joint"] = -2.1956242;
        joints["ur5_arm_wrist_2_joint"] = 1.5053465;
        joints["ur5_arm_wrist_3_joint"] = 0.97406826;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 5.6173422;
        joints["ur5_arm_shoulder_lift_joint"] = -2.30837247;
        joints["ur5_arm_elbow_joint"] = 0.04974188;
        joints["ur5_arm_wrist_1_joint"] = -2.43421071;
        joints["ur5_arm_wrist_2_joint"] = 1.5039502;
        joints["ur5_arm_wrist_3_joint"] = 1.1124729;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 5.560621;
        joints["ur5_arm_shoulder_lift_joint"] = -2.003639;
        joints["ur5_arm_elbow_joint"] = -1.4927807;
        joints["ur5_arm_wrist_1_joint"] = -1.196773;
        joints["ur5_arm_wrist_2_joint"] = 1.568703;
        joints["ur5_arm_wrist_3_joint"] = 0.956615;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	sleep(2.0);

  spinner.stop();
}

/////////////////////////////////////////////////////////////
void blue_left_compartment_bottom_pick(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> joints;

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  //moveit::planning_interface::MoveGroup group_arm("ur5_arm");
  //choose your preferred planner
  //group_arm.setPlannerId("SBLkConfigDefault");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  //group_arm.setPlannerId("ESTkConfigDefault");

  //arm_joint_names = group_arm.getJoints();
  group_arm.setMaxVelocityScalingFactor(1.0);

        joints["ur5_arm_shoulder_pan_joint"] = 5.526762;
	joints["ur5_arm_shoulder_lift_joint"] = -2.237688; //-1.968783;
	joints["ur5_arm_elbow_joint"] = -1.737301;//-1.026079;
	joints["ur5_arm_wrist_1_joint"] = -0.760440;
	joints["ur5_arm_wrist_2_joint"] = 1.558754;
	joints["ur5_arm_wrist_3_joint"] = 0.956615;
	group_arm.setJointValueTarget(joints);
	group_arm.move();

        joints["ur5_arm_shoulder_pan_joint"] = 5.570046;
        joints["ur5_arm_shoulder_lift_joint"] = -2.257196;
        joints["ur5_arm_elbow_joint"] = -1.678658;
        joints["ur5_arm_wrist_1_joint"] = -0.761138;
        joints["ur5_arm_wrist_2_joint"] = 1.568528;
        joints["ur5_arm_wrist_3_joint"] = 0.956615;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 5.6173422;
        joints["ur5_arm_shoulder_lift_joint"] = -2.30837247;
        joints["ur5_arm_elbow_joint"] = 0.04974188;
        joints["ur5_arm_wrist_1_joint"] = -2.43421071;
        joints["ur5_arm_wrist_2_joint"] = 1.5039502;
        joints["ur5_arm_wrist_3_joint"] = 1.1124729;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 5.473178;
        joints["ur5_arm_shoulder_lift_joint"] = -1.86436071;
        joints["ur5_arm_elbow_joint"] = -0.62395521;
        joints["ur5_arm_wrist_1_joint"] = -2.1956242;
        joints["ur5_arm_wrist_2_joint"] = 1.5053465;
        joints["ur5_arm_wrist_3_joint"] = 0.97406826;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 3.12448843;
        joints["ur5_arm_shoulder_lift_joint"] = -1.80100526;
        joints["ur5_arm_elbow_joint"] = -0.21485003;
        joints["ur5_arm_wrist_1_joint"] = -2.68815611;
        joints["ur5_arm_wrist_2_joint"] = 1.5833627;
        joints["ur5_arm_wrist_3_joint"] = 0.0;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	sleep(2.0);

  spinner.stop();
}

/////////////////////////////////////////////////////////////
void blue_left_compartment_bottom_store(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> joints;

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  //moveit::planning_interface::MoveGroup group_arm("ur5_arm");
  //choose your preferred planner
  //group_arm.setPlannerId("SBLkConfigDefault");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  //group_arm.setPlannerId("ESTkConfigDefault");

  //arm_joint_names = group_arm.getJoints();
  group_arm.setMaxVelocityScalingFactor(1.0);

        joints["ur5_arm_shoulder_pan_joint"] = 3.12448843;
        joints["ur5_arm_shoulder_lift_joint"] = -1.80100526;
        joints["ur5_arm_elbow_joint"] = -0.21485003;
        joints["ur5_arm_wrist_1_joint"] = -2.68815611;
        joints["ur5_arm_wrist_2_joint"] = 1.5833627;
        joints["ur5_arm_wrist_3_joint"] = 0.0;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 5.473178;
        joints["ur5_arm_shoulder_lift_joint"] = -1.86436071;
        joints["ur5_arm_elbow_joint"] = -0.62395521;
        joints["ur5_arm_wrist_1_joint"] = -2.1956242;
        joints["ur5_arm_wrist_2_joint"] = 1.5053465;
        joints["ur5_arm_wrist_3_joint"] = 0.97406826;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 5.6173422;
        joints["ur5_arm_shoulder_lift_joint"] = -2.30837247;
        joints["ur5_arm_elbow_joint"] = 0.04974188;
        joints["ur5_arm_wrist_1_joint"] = -2.43421071;
        joints["ur5_arm_wrist_2_joint"] = 1.5039502;
        joints["ur5_arm_wrist_3_joint"] = 1.1124729;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 5.570046;
        joints["ur5_arm_shoulder_lift_joint"] = -2.257196;
        joints["ur5_arm_elbow_joint"] = -1.678658;
        joints["ur5_arm_wrist_1_joint"] = -0.761138;
        joints["ur5_arm_wrist_2_joint"] = 1.568528;
        joints["ur5_arm_wrist_3_joint"] = 0.956615;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 5.526762;
	joints["ur5_arm_shoulder_lift_joint"] = -2.237688; //-1.968783;
	joints["ur5_arm_elbow_joint"] = -1.737301;//-1.026079;
	joints["ur5_arm_wrist_1_joint"] = -0.760440;
	joints["ur5_arm_wrist_2_joint"] = 1.558754;
	joints["ur5_arm_wrist_3_joint"] = 0.956615;
	group_arm.setJointValueTarget(joints);
	group_arm.move();

  spinner.stop();
}

/////////////////////////////////////////////////////////////
void blue_right_compartment_top_pick(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> joints;

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  //moveit::planning_interface::MoveGroup group_arm("ur5_arm");
  //choose your preferred planner
  //group_arm.setPlannerId("SBLkConfigDefault");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  //group_arm.setPlannerId("ESTkConfigDefault");

  //arm_joint_names = group_arm.getJoints();
  group_arm.setMaxVelocityScalingFactor(1.0);

        joints["ur5_arm_shoulder_pan_joint"] = 1.048245;
        joints["ur5_arm_shoulder_lift_joint"] = -1.980251;
        joints["ur5_arm_elbow_joint"] = -1.026254;
        joints["ur5_arm_wrist_1_joint"] = -1.728575;
        joints["ur5_arm_wrist_2_joint"] = 1.558754;
        joints["ur5_arm_wrist_3_joint"] = -0.471065;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
        //sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 1.0337585;
        joints["ur5_arm_shoulder_lift_joint"] = -2.32861829;
        joints["ur5_arm_elbow_joint"] = 0.08115781;
        joints["ur5_arm_wrist_1_joint"] = -2.48604699;
        joints["ur5_arm_wrist_2_joint"] = 1.5557865;
        joints["ur5_arm_wrist_3_joint"] = 0.47054077;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 1.0807079;
        joints["ur5_arm_shoulder_lift_joint"] = -2.2258184;
        joints["ur5_arm_elbow_joint"] = 0.01780236;
        joints["ur5_arm_wrist_1_joint"] = -2.49198111;
        joints["ur5_arm_wrist_2_joint"] = 1.5561356;
        joints["ur5_arm_wrist_3_joint"] = -0.4134685;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 3.12448843;
        joints["ur5_arm_shoulder_lift_joint"] = -1.80100526;
        joints["ur5_arm_elbow_joint"] = -0.21485003;
        joints["ur5_arm_wrist_1_joint"] = -2.68815611;
        joints["ur5_arm_wrist_2_joint"] = 1.5833627;
        joints["ur5_arm_wrist_3_joint"] = 0.0;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	sleep(2.0);

  spinner.stop();
}

/////////////////////////////////////////////////////////////
void blue_right_compartment_top_store(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> joints;

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  //moveit::planning_interface::MoveGroup group_arm("ur5_arm");
  //choose your preferred planner
  //group_arm.setPlannerId("SBLkConfigDefault");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  //group_arm.setPlannerId("ESTkConfigDefault");

  //arm_joint_names = group_arm.getJoints();
  group_arm.setMaxVelocityScalingFactor(1.0);

        joints["ur5_arm_shoulder_pan_joint"] = 3.12448843;
        joints["ur5_arm_shoulder_lift_joint"] = -1.80100526;
        joints["ur5_arm_elbow_joint"] = -0.21485003;
        joints["ur5_arm_wrist_1_joint"] = -2.68815611;
        joints["ur5_arm_wrist_2_joint"] = 1.5833627;
        joints["ur5_arm_wrist_3_joint"] = 0.0;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 1.0807079;
        joints["ur5_arm_shoulder_lift_joint"] = -2.2258184;
        joints["ur5_arm_elbow_joint"] = 0.01780236;
        joints["ur5_arm_wrist_1_joint"] = -2.49198111;
        joints["ur5_arm_wrist_2_joint"] = 1.5561356;
        joints["ur5_arm_wrist_3_joint"] = -0.4134685;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 1.0337585;
        joints["ur5_arm_shoulder_lift_joint"] = -2.32861829;
        joints["ur5_arm_elbow_joint"] = 0.08115781;
        joints["ur5_arm_wrist_1_joint"] = -2.48604699;
        joints["ur5_arm_wrist_2_joint"] = 1.5557865;
        joints["ur5_arm_wrist_3_joint"] = 0.47054077;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 1.048245;
        joints["ur5_arm_shoulder_lift_joint"] = -1.980251;
        joints["ur5_arm_elbow_joint"] = -1.026254;
        joints["ur5_arm_wrist_1_joint"] = -1.728575;
        joints["ur5_arm_wrist_2_joint"] = 1.558754;
        joints["ur5_arm_wrist_3_joint"] = -0.471065;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	sleep(2.0);

  spinner.stop();
}

/////////////////////////////////////////////////////////////
void blue_right_compartment_middle_pick(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> joints;

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  //moveit::planning_interface::MoveGroup group_arm("ur5_arm");
  //choose your preferred planner
  //group_arm.setPlannerId("SBLkConfigDefault");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  //group_arm.setPlannerId("ESTkConfigDefault");

  //arm_joint_names = group_arm.getJoints();
  group_arm.setMaxVelocityScalingFactor(1.0);

        joints["ur5_arm_shoulder_pan_joint"] = 1.071807;
        joints["ur5_arm_shoulder_lift_joint"] = -2.016205;
        joints["ur5_arm_elbow_joint"] = -1.489988;
        joints["ur5_arm_wrist_1_joint"] = -1.240755;
        joints["ur5_arm_wrist_2_joint"] = 1.558754;
        joints["ur5_arm_wrist_3_joint"] = -0.471065;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 1.0337585;
        joints["ur5_arm_shoulder_lift_joint"] = -2.32861829;
        joints["ur5_arm_elbow_joint"] = 0.08115781;
        joints["ur5_arm_wrist_1_joint"] = -2.48604699;
        joints["ur5_arm_wrist_2_joint"] = 1.5557865;
        joints["ur5_arm_wrist_3_joint"] = 0.47054077;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 1.0807079;
        joints["ur5_arm_shoulder_lift_joint"] = -2.2258184;
        joints["ur5_arm_elbow_joint"] = 0.01780236;
        joints["ur5_arm_wrist_1_joint"] = -2.49198111;
        joints["ur5_arm_wrist_2_joint"] = 1.5561356;
        joints["ur5_arm_wrist_3_joint"] = -0.4134685;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 3.12448843;
        joints["ur5_arm_shoulder_lift_joint"] = -1.80100526;
        joints["ur5_arm_elbow_joint"] = -0.21485003;
        joints["ur5_arm_wrist_1_joint"] = -2.68815611;
        joints["ur5_arm_wrist_2_joint"] = 1.5833627;
        joints["ur5_arm_wrist_3_joint"] = 0.0;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	sleep(2.0);

  spinner.stop();
}

/////////////////////////////////////////////////////////////
void blue_right_compartment_middle_store(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> joints;

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  //moveit::planning_interface::MoveGroup group_arm("ur5_arm");
  //choose your preferred planner
  //group_arm.setPlannerId("SBLkConfigDefault");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  //group_arm.setPlannerId("ESTkConfigDefault");

  //arm_joint_names = group_arm.getJoints();
  group_arm.setMaxVelocityScalingFactor(1.0);

        joints["ur5_arm_shoulder_pan_joint"] = 3.12448843;
        joints["ur5_arm_shoulder_lift_joint"] = -1.80100526;
        joints["ur5_arm_elbow_joint"] = -0.21485003;
        joints["ur5_arm_wrist_1_joint"] = -2.68815611;
        joints["ur5_arm_wrist_2_joint"] = 1.5833627;
        joints["ur5_arm_wrist_3_joint"] = 0.0;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 1.0807079;
        joints["ur5_arm_shoulder_lift_joint"] = -2.2258184;
        joints["ur5_arm_elbow_joint"] = 0.01780236;
        joints["ur5_arm_wrist_1_joint"] = -2.49198111;
        joints["ur5_arm_wrist_2_joint"] = 1.5561356;
        joints["ur5_arm_wrist_3_joint"] = -0.4134685;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 1.0337585;
        joints["ur5_arm_shoulder_lift_joint"] = -2.32861829;
        joints["ur5_arm_elbow_joint"] = 0.08115781;
        joints["ur5_arm_wrist_1_joint"] = -2.48604699;
        joints["ur5_arm_wrist_2_joint"] = 1.5557865;
        joints["ur5_arm_wrist_3_joint"] = 0.47054077;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 1.071807;
        joints["ur5_arm_shoulder_lift_joint"] = -2.016205;
        joints["ur5_arm_elbow_joint"] = -1.489988;
        joints["ur5_arm_wrist_1_joint"] = -1.240755;
        joints["ur5_arm_wrist_2_joint"] = 1.558754;
        joints["ur5_arm_wrist_3_joint"] = -0.471065;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	sleep(2.0);

  spinner.stop();
}

/////////////////////////////////////////////////////////////
void blue_right_compartment_bottom_pick(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> joints;

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  //moveit::planning_interface::MoveGroup group_arm("ur5_arm");
  //choose your preferred planner
  //group_arm.setPlannerId("SBLkConfigDefault");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  //group_arm.setPlannerId("ESTkConfigDefault");

  //arm_joint_names = group_arm.getJoints();
  group_arm.setMaxVelocityScalingFactor(1.0);

        joints["ur5_arm_shoulder_pan_joint"] = 1.069189;
        joints["ur5_arm_shoulder_lift_joint"] = -2.237688;
        joints["ur5_arm_elbow_joint"] = -1.737301;
        joints["ur5_arm_wrist_1_joint"] = -0.760440;
        joints["ur5_arm_wrist_2_joint"] = 1.558754;
        joints["ur5_arm_wrist_3_joint"] = -0.471065;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 1.0337585;
        joints["ur5_arm_shoulder_lift_joint"] = -2.32861829;
        joints["ur5_arm_elbow_joint"] = 0.08115781;
        joints["ur5_arm_wrist_1_joint"] = -2.48604699;
        joints["ur5_arm_wrist_2_joint"] = 1.5557865;
        joints["ur5_arm_wrist_3_joint"] = 0.47054077;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
        //sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 1.0807079;
        joints["ur5_arm_shoulder_lift_joint"] = -2.2258184;
        joints["ur5_arm_elbow_joint"] = 0.01780236;
        joints["ur5_arm_wrist_1_joint"] = -2.49198111;
        joints["ur5_arm_wrist_2_joint"] = 1.5561356;
        joints["ur5_arm_wrist_3_joint"] = -0.4134685;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 3.12448843;
        joints["ur5_arm_shoulder_lift_joint"] = -1.80100526;
        joints["ur5_arm_elbow_joint"] = -0.21485003;
        joints["ur5_arm_wrist_1_joint"] = -2.68815611;
        joints["ur5_arm_wrist_2_joint"] = 1.5833627;
        joints["ur5_arm_wrist_3_joint"] = 0.0;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	sleep(2.0);

  spinner.stop();
}

/////////////////////////////////////////////////////////////
void blue_right_compartment_bottom_store(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> joints;

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  //moveit::planning_interface::MoveGroup group_arm("ur5_arm");
  //choose your preferred planner
  //group_arm.setPlannerId("SBLkConfigDefault");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  //group_arm.setPlannerId("ESTkConfigDefault");

  //arm_joint_names = group_arm.getJoints();
  group_arm.setMaxVelocityScalingFactor(1.0);

        joints["ur5_arm_shoulder_pan_joint"] = 3.12448843;
        joints["ur5_arm_shoulder_lift_joint"] = -1.80100526;
        joints["ur5_arm_elbow_joint"] = -0.21485003;
        joints["ur5_arm_wrist_1_joint"] = -2.68815611;
        joints["ur5_arm_wrist_2_joint"] = 1.5833627;
        joints["ur5_arm_wrist_3_joint"] = 0.0;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 1.0807079;
        joints["ur5_arm_shoulder_lift_joint"] = -2.2258184;
        joints["ur5_arm_elbow_joint"] = 0.01780236;
        joints["ur5_arm_wrist_1_joint"] = -2.49198111;
        joints["ur5_arm_wrist_2_joint"] = 1.5561356;
        joints["ur5_arm_wrist_3_joint"] = -0.4134685;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 1.0337585;
        joints["ur5_arm_shoulder_lift_joint"] = -2.32861829;
        joints["ur5_arm_elbow_joint"] = 0.08115781;
        joints["ur5_arm_wrist_1_joint"] = -2.48604699;
        joints["ur5_arm_wrist_2_joint"] = 1.5557865;
        joints["ur5_arm_wrist_3_joint"] = 0.47054077;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 1.069189;
        joints["ur5_arm_shoulder_lift_joint"] = -2.237688;
        joints["ur5_arm_elbow_joint"] = -1.737301;
        joints["ur5_arm_wrist_1_joint"] = -0.760440;
        joints["ur5_arm_wrist_2_joint"] = 1.558754;
        joints["ur5_arm_wrist_3_joint"] = -0.471065;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	sleep(2.0);

  spinner.stop();
}

/////////////////////////////////////////////////////////////
void green_left_compartment_front_pick(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> joints;

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  //moveit::planning_interface::MoveGroup group_arm("ur5_arm");
  //choose your preferred planner
  //group_arm.setPlannerId("SBLkConfigDefault");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  //group_arm.setPlannerId("ESTkConfigDefault");

  //arm_joint_names = group_arm.getJoints();
  group_arm.setMaxVelocityScalingFactor(1.0);

        joints["ur5_arm_shoulder_pan_joint"] = 5.187525;
        joints["ur5_arm_shoulder_lift_joint"] = -1.757175;
        joints["ur5_arm_elbow_joint"] = -0.81235;
        joints["ur5_arm_wrist_1_joint"] = -2.186275;
        joints["ur5_arm_wrist_2_joint"] = 1.550675;
        joints["ur5_arm_wrist_3_joint"] = 0.560875;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 5.39735;
        joints["ur5_arm_shoulder_lift_joint"] = -2.007775;
        joints["ur5_arm_elbow_joint"] = -0.25305;
        joints["ur5_arm_wrist_1_joint"] = -2.4416;
        joints["ur5_arm_wrist_2_joint"] = 1.553475;
        joints["ur5_arm_wrist_3_joint"] = 0.810425;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 3.21965;
        joints["ur5_arm_shoulder_lift_joint"] = -1.599675;
        joints["ur5_arm_elbow_joint"] = -0.551425;
        joints["ur5_arm_wrist_1_joint"] = -2.531375;
        joints["ur5_arm_wrist_2_joint"] = 1.553475;
        joints["ur5_arm_wrist_3_joint"] = 0.217 ;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	sleep(2.0);

  spinner.stop();
}

/////////////////////////////////////////////////////////////
void green_left_compartment_front_store(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> joints;

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  //moveit::planning_interface::MoveGroup group_arm("ur5_arm");
  //choose your preferred planner
  //group_arm.setPlannerId("SBLkConfigDefault");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  //group_arm.setPlannerId("ESTkConfigDefault");

  //arm_joint_names = group_arm.getJoints();
  group_arm.setMaxVelocityScalingFactor(1.0);

        joints["ur5_arm_shoulder_pan_joint"] = 3.21965;
        joints["ur5_arm_shoulder_lift_joint"] = -1.599675;
        joints["ur5_arm_elbow_joint"] = -0.551425;
        joints["ur5_arm_wrist_1_joint"] = -2.531375;
        joints["ur5_arm_wrist_2_joint"] = 1.553475;
        joints["ur5_arm_wrist_3_joint"] = 0.217 ;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 5.39735;
        joints["ur5_arm_shoulder_lift_joint"] = -2.007775;
        joints["ur5_arm_elbow_joint"] = -0.25305;
        joints["ur5_arm_wrist_1_joint"] = -2.4416;
        joints["ur5_arm_wrist_2_joint"] = 1.553475;
        joints["ur5_arm_wrist_3_joint"] = 0.810425;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 5.187525;
        joints["ur5_arm_shoulder_lift_joint"] = -1.757175;
        joints["ur5_arm_elbow_joint"] = -0.81235;
        joints["ur5_arm_wrist_1_joint"] = -2.186275;
        joints["ur5_arm_wrist_2_joint"] = 1.550675;
        joints["ur5_arm_wrist_3_joint"] = 0.560875;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	sleep(2.0);

  spinner.stop();
}

/////////////////////////////////////////////////////////////
void green_right_compartment_front_pick(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> joints;

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  //moveit::planning_interface::MoveGroup group_arm("ur5_arm");
  //choose your preferred planner
  //group_arm.setPlannerId("SBLkConfigDefault");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  //group_arm.setPlannerId("ESTkConfigDefault");

  //arm_joint_names = group_arm.getJoints();
  group_arm.setMaxVelocityScalingFactor(1.0);

        joints["ur5_arm_shoulder_pan_joint"] = 1.557532;
        joints["ur5_arm_shoulder_lift_joint"] = -1.662078;
        joints["ur5_arm_elbow_joint"] = -1.43152;
        joints["ur5_arm_wrist_1_joint"] = -1.43152;
        joints["ur5_arm_wrist_2_joint"] = 1.582316;
        joints["ur5_arm_wrist_3_joint"] = 0.069115;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 1.550725;
        joints["ur5_arm_shoulder_lift_joint"] = -1.84638382;
        joints["ur5_arm_elbow_joint"] = -0.64838982;
        joints["ur5_arm_wrist_1_joint"] = -2.28952291;
        joints["ur5_arm_wrist_2_joint"] = 1.5835372;
        joints["ur5_arm_wrist_3_joint"] = 0.06928957;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 3.12448843;
        joints["ur5_arm_shoulder_lift_joint"] = -1.80100526;
        joints["ur5_arm_elbow_joint"] = -0.21485003;
        joints["ur5_arm_wrist_1_joint"] = -2.68815611;
        joints["ur5_arm_wrist_2_joint"] = 1.5833627;
        joints["ur5_arm_wrist_3_joint"] = 0.0;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	sleep(2.0);

  spinner.stop();
}

/////////////////////////////////////////////////////////////
void green_right_compartment_front_store(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> joints;

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  //moveit::planning_interface::MoveGroup group_arm("ur5_arm");
  //choose your preferred planner
  //group_arm.setPlannerId("SBLkConfigDefault");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  //group_arm.setPlannerId("ESTkConfigDefault");

  //arm_joint_names = group_arm.getJoints();
  group_arm.setMaxVelocityScalingFactor(1.0);

        joints["ur5_arm_shoulder_pan_joint"] = 3.12448843;
        joints["ur5_arm_shoulder_lift_joint"] = -1.80100526;
        joints["ur5_arm_elbow_joint"] = -0.21485003;
        joints["ur5_arm_wrist_1_joint"] = -2.68815611;
        joints["ur5_arm_wrist_2_joint"] = 1.5833627;
        joints["ur5_arm_wrist_3_joint"] = 0.0;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 1.550725;
        joints["ur5_arm_shoulder_lift_joint"] = -1.84638382;
        joints["ur5_arm_elbow_joint"] = -0.64838982;
        joints["ur5_arm_wrist_1_joint"] = -2.28952291;
        joints["ur5_arm_wrist_2_joint"] = 1.5835372;
        joints["ur5_arm_wrist_3_joint"] = 0.06928957;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 1.557532;
        joints["ur5_arm_shoulder_lift_joint"] = -1.662078;
        joints["ur5_arm_elbow_joint"] = -1.43152;
        joints["ur5_arm_wrist_1_joint"] = -1.43152;
        joints["ur5_arm_wrist_2_joint"] = 1.582316;
        joints["ur5_arm_wrist_3_joint"] = 0.069115;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	sleep(2.0);

  spinner.stop();
}

/////////////////////////////////////////////////////////////
void top_compartment_left_pick(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> joints;

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  //moveit::planning_interface::MoveGroup group_arm("ur5_arm");
  //choose your preferred planner
  //group_arm.setPlannerId("SBLkConfigDefault");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  //group_arm.setPlannerId("ESTkConfigDefault");

  //arm_joint_names = group_arm.getJoints();
  group_arm.setMaxVelocityScalingFactor(1.0);

        joints["ur5_arm_shoulder_pan_joint"] = 0.83618734;
        joints["ur5_arm_shoulder_lift_joint"] = -1.5177383;
        joints["ur5_arm_elbow_joint"] = -1.77342905;
        joints["ur5_arm_wrist_1_joint"] = -1.4423401;
        joints["ur5_arm_wrist_2_joint"] = 1.5414748;
        joints["ur5_arm_wrist_3_joint"] = -0.64158303;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 0.81559236;
        joints["ur5_arm_shoulder_lift_joint"] = -1.81950575;
        joints["ur5_arm_elbow_joint"] = -0.40194933;
        joints["ur5_arm_wrist_1_joint"] = -2.48255633;
        joints["ur5_arm_wrist_2_joint"] = 1.5940092;
        joints["ur5_arm_wrist_3_joint"] = -0.5532694;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 3.18295696;
        joints["ur5_arm_shoulder_lift_joint"] = -1.83713357;
        joints["ur5_arm_elbow_joint"] = -0.2994985;
        joints["ur5_arm_wrist_1_joint"] = -2.56860106;
        joints["ur5_arm_wrist_2_joint"] = 1.567306;
        joints["ur5_arm_wrist_3_joint"] = 0.06370452;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	sleep(2.0);

  spinner.stop();
}

/////////////////////////////////////////////////////////////
void top_compartment_left_store(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> joints;

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  //moveit::planning_interface::MoveGroup group_arm("ur5_arm");
  //choose your preferred planner
  //group_arm.setPlannerId("SBLkConfigDefault");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  //group_arm.setPlannerId("ESTkConfigDefault");

  //arm_joint_names = group_arm.getJoints();
  group_arm.setMaxVelocityScalingFactor(1.0);

        joints["ur5_arm_shoulder_pan_joint"] = 3.18295696;
        joints["ur5_arm_shoulder_lift_joint"] = -1.83713357;
        joints["ur5_arm_elbow_joint"] = -0.2994985;
        joints["ur5_arm_wrist_1_joint"] = -2.56860106;
        joints["ur5_arm_wrist_2_joint"] = 1.567306;
        joints["ur5_arm_wrist_3_joint"] = 0.06370452;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 0.81559236;
        joints["ur5_arm_shoulder_lift_joint"] = -1.81950575;
        joints["ur5_arm_elbow_joint"] = -0.40194933;
        joints["ur5_arm_wrist_1_joint"] = -2.48255633;
        joints["ur5_arm_wrist_2_joint"] = 1.5940092;
        joints["ur5_arm_wrist_3_joint"] = -0.5532694;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 0.83618734;
        joints["ur5_arm_shoulder_lift_joint"] = -1.5177383;
        joints["ur5_arm_elbow_joint"] = -1.77342905;
        joints["ur5_arm_wrist_1_joint"] = -1.4423401;
        joints["ur5_arm_wrist_2_joint"] = 1.5414748;
        joints["ur5_arm_wrist_3_joint"] = -0.64158303;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	sleep(2.0);

  spinner.stop();
}

/////////////////////////////////////////////////////////////
void top_compartment_middle_pick(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> joints;

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  //moveit::planning_interface::MoveGroup group_arm("ur5_arm");
  //choose your preferred planner
  //group_arm.setPlannerId("SBLkConfigDefault");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  //group_arm.setPlannerId("ESTkConfigDefault");

  //arm_joint_names = group_arm.getJoints();
  group_arm.setMaxVelocityScalingFactor(1.0);

        joints["ur5_arm_shoulder_pan_joint"] = 0.23125613;
        joints["ur5_arm_shoulder_lift_joint"] = -1.4669492;
        joints["ur5_arm_elbow_joint"] = -1.81880761;
        joints["ur5_arm_wrist_1_joint"] = -1.4512413;
        joints["ur5_arm_wrist_2_joint"] = 1.566433;
        joints["ur5_arm_wrist_3_joint"] = 0.32026792;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        //additional
        //joints["ur5_arm_shoulder_pan_joint"] = 0.2702; 
        //joints["ur5_arm_shoulder_lift_joint"] = -1.132425;
        //joints["ur5_arm_elbow_joint"] = -1.69155;
        //joints["ur5_arm_wrist_1_joint"] = -1.84485;
        //joints["ur5_arm_wrist_2_joint"] = 1.570625;
        //joints["ur5_arm_wrist_3_joint"] = 0.3367;
	//group_arm.setJointValueTarget(joints);
	//group_arm.move();
	////sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 0.2560398;
        joints["ur5_arm_shoulder_lift_joint"] = -1.5522958;
        joints["ur5_arm_elbow_joint"] = -0.92345371;
        joints["ur5_arm_wrist_1_joint"] = -2.2549654;
        joints["ur5_arm_wrist_2_joint"] = 1.566433;
        joints["ur5_arm_wrist_3_joint"] = 0.33562682;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 3.18295696;
        joints["ur5_arm_shoulder_lift_joint"] = -1.83713357;
        joints["ur5_arm_elbow_joint"] = -0.2994985;
        joints["ur5_arm_wrist_1_joint"] = -2.56860106;
        joints["ur5_arm_wrist_2_joint"] = 1.567306;
        joints["ur5_arm_wrist_3_joint"] = 0.06370452;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	sleep(2.0);

  spinner.stop();
}

/////////////////////////////////////////////////////////////
void top_compartment_middle_store(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> joints;

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  //moveit::planning_interface::MoveGroup group_arm("ur5_arm");
  //choose your preferred planner
  //group_arm.setPlannerId("SBLkConfigDefault");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  //group_arm.setPlannerId("ESTkConfigDefault");

  //arm_joint_names = group_arm.getJoints();
  group_arm.setMaxVelocityScalingFactor(1.0);

        joints["ur5_arm_shoulder_pan_joint"] = 3.18295696;
        joints["ur5_arm_shoulder_lift_joint"] = -1.83713357;
        joints["ur5_arm_elbow_joint"] = -0.2994985;
        joints["ur5_arm_wrist_1_joint"] = -2.56860106;
        joints["ur5_arm_wrist_2_joint"] = 1.567306;
        joints["ur5_arm_wrist_3_joint"] = 0.06370452;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 0.2560398;
        joints["ur5_arm_shoulder_lift_joint"] = -1.5522958;
        joints["ur5_arm_elbow_joint"] = -0.92345371;
        joints["ur5_arm_wrist_1_joint"] = -2.2549654;
        joints["ur5_arm_wrist_2_joint"] = 1.566433;
        joints["ur5_arm_wrist_3_joint"] = 0.33562682;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        //additional
        //joints["ur5_arm_shoulder_pan_joint"] = 0.2702; 
        //joints["ur5_arm_shoulder_lift_joint"] = -1.132425;
        //joints["ur5_arm_elbow_joint"] = -1.69155;
        //joints["ur5_arm_wrist_1_joint"] = -1.84485;
        //joints["ur5_arm_wrist_2_joint"] = 1.570625;
        //joints["ur5_arm_wrist_3_joint"] = 0.3367;
	//group_arm.setJointValueTarget(joints);
	//group_arm.move();
	////sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 0.23125613;
        joints["ur5_arm_shoulder_lift_joint"] = -1.4669492;
        joints["ur5_arm_elbow_joint"] = -1.81880761;
        joints["ur5_arm_wrist_1_joint"] = -1.4512413;
        joints["ur5_arm_wrist_2_joint"] = 1.566433;
        joints["ur5_arm_wrist_3_joint"] = 0.32026792;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	sleep(2.0);

  spinner.stop();
}

/////////////////////////////////////////////////////////////
void top_compartment_right_pick(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> joints;

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  //moveit::planning_interface::MoveGroup group_arm("ur5_arm");
  //choose your preferred planner
  //group_arm.setPlannerId("SBLkConfigDefault");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  //group_arm.setPlannerId("ESTkConfigDefault");

  //arm_joint_names = group_arm.getJoints();
  group_arm.setMaxVelocityScalingFactor(1.0);

        joints["ur5_arm_shoulder_pan_joint"] = 5.93464306;
        joints["ur5_arm_shoulder_lift_joint"] = -1.5278612;
        joints["ur5_arm_elbow_joint"] = -1.7697639;
        joints["ur5_arm_wrist_1_joint"] = -1.4060372;
        joints["ur5_arm_wrist_2_joint"] = 1.5653858;
        joints["ur5_arm_wrist_3_joint"] = 1.2725196;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 5.91823696;
        joints["ur5_arm_shoulder_lift_joint"] = -1.7268288;
        joints["ur5_arm_elbow_joint"] = -0.57456239;
        joints["ur5_arm_wrist_1_joint"] = -2.40454011;
        joints["ur5_arm_wrist_2_joint"] = 1.567306;
        joints["ur5_arm_wrist_3_joint"] = 1.27409;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 3.18295696;
        joints["ur5_arm_shoulder_lift_joint"] = -1.83713357;
        joints["ur5_arm_elbow_joint"] = -0.2994985;
        joints["ur5_arm_wrist_1_joint"] = -2.56860106;
        joints["ur5_arm_wrist_2_joint"] = 1.567306;
        joints["ur5_arm_wrist_3_joint"] = 0.06370452;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	sleep(2.0);

  spinner.stop();
}

/////////////////////////////////////////////////////////////
void top_compartment_right_store(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> joints;

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  //moveit::planning_interface::MoveGroup group_arm("ur5_arm");
  //choose your preferred planner
  //group_arm.setPlannerId("SBLkConfigDefault");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  //group_arm.setPlannerId("ESTkConfigDefault");

  //arm_joint_names = group_arm.getJoints();
  group_arm.setMaxVelocityScalingFactor(1.0);

        joints["ur5_arm_shoulder_pan_joint"] = 3.18295696;
        joints["ur5_arm_shoulder_lift_joint"] = -1.83713357;
        joints["ur5_arm_elbow_joint"] = -0.2994985;
        joints["ur5_arm_wrist_1_joint"] = -2.56860106;
        joints["ur5_arm_wrist_2_joint"] = 1.567306;
        joints["ur5_arm_wrist_3_joint"] = 0.06370452;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 5.91823696;
        joints["ur5_arm_shoulder_lift_joint"] = -1.7268288;
        joints["ur5_arm_elbow_joint"] = -0.57456239;
        joints["ur5_arm_wrist_1_joint"] = -2.40454011;
        joints["ur5_arm_wrist_2_joint"] = 1.567306;
        joints["ur5_arm_wrist_3_joint"] = 1.27409;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	//sleep(2.0);

        joints["ur5_arm_shoulder_pan_joint"] = 5.93464306;
        joints["ur5_arm_shoulder_lift_joint"] = -1.5278612;
        joints["ur5_arm_elbow_joint"] = -1.7697639;
        joints["ur5_arm_wrist_1_joint"] = -1.4060372;
        joints["ur5_arm_wrist_2_joint"] = 1.5653858;
        joints["ur5_arm_wrist_3_joint"] = 1.2725196;
	group_arm.setJointValueTarget(joints);
	group_arm.move();
	sleep(2.0);

  spinner.stop();
}

/////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  initData();
  ros::init(argc, argv, "joint_waypoints");
 
  
  int flag = -1;


  while (ros::ok())
  {
	//spinner.start();	
	ros::spinOnce();
	datX = *resultX;

	if(datX != flag){
		ROS_INFO("Key %d", datX);

		if(datX == 16){
			// blue_left_compartment_top_pick
			blue_left_compartment_top_pick();
			ROS_INFO("blue_left_compartment_top_pick");
		}
		else if(datX == 17){
			// blue_left_compartment_middle_pick
			blue_left_compartment_middle_pick();
			ROS_INFO("blue_left_compartment_middle_pick");
		}
		else if(datX == 18){
			// blue_left_compartment_bottom_pick
			blue_left_compartment_bottom_pick();
			ROS_INFO("blue_left_compartment_bottom_pick");
		}
		else if(datX == 19){
			// blue_right_compartment_top_pick
			blue_right_compartment_top_pick();
			ROS_INFO("blue_right_compartment_top_pick");
		}
		else if(datX == 20){
			// blue_right_compartment_middle_pick
			blue_right_compartment_middle_pick();
			ROS_INFO("blue_right_compartment_middle_pick");
		}
		else if(datX == 21){
			// blue_right_compartment_bottom_pick
			blue_right_compartment_bottom_pick();
			ROS_INFO("blue_right_compartment_bottom_pick");
		}
		else if(datX == 22){
			// green_left_compartment_front_pick
			green_left_compartment_front_pick();
			ROS_INFO("green_left_compartment_front_pick");
		}
		else if(datX == 23){
			// green_right_compartment_front_pick
			green_right_compartment_front_pick();
			ROS_INFO("green_right_compartment_front_pick");
		}
		else if(datX == 24){
			// top_compartment_left_pick
			top_compartment_left_pick();
			ROS_INFO("top_compartment_left_pick");
		}
		else if(datX == 25){
			// top_compartment_middle_pick
			top_compartment_middle_pick();
			ROS_INFO("top_compartment_middle_pick");
		}
		else if(datX == 26){
			// top_compartment_right_pick
			top_compartment_right_pick();
			ROS_INFO("top_compartment_right_pick");
		}

		else if(datX == 30){
			// blue_left_compartment_top_store
			key_blue_left_compartment_top_store();
			ROS_INFO("blue_left_compartment_top_store");
		}
		else if(datX == 31){
			// blue_left_compartment_middle_store
			blue_left_compartment_middle_store();
			ROS_INFO("blue_left_compartment_middle_store");
		}
		else if(datX == 32){
			// blue_left_compartment_bottom_store
			blue_left_compartment_bottom_store();
			ROS_INFO("blue_left_compartment_bottom_store");
		}
		else if(datX == 33){
			// blue_right_compartment_top_store
			blue_right_compartment_top_store();
			ROS_INFO("blue_right_compartment_top_store");
		}
		else if(datX == 34){
			// blue_right_compartment_middle_store
			blue_right_compartment_middle_store();
			ROS_INFO("blue_right_compartment_middle_store");
		}
		else if(datX == 35){
			// blue_right_compartment_bottom_store
			blue_right_compartment_bottom_store();
			ROS_INFO("blue_right_compartment_bottom_store");
		}
		else if(datX == 36){
			// green_left_compartment_front_store
			green_left_compartment_front_store();
			ROS_INFO("green_left_compartment_front_store");
		}
		else if(datX == 37){
			// green_right_compartment_front_store
			green_right_compartment_front_store();
			ROS_INFO("green_right_compartment_front_store");
		}
		else if(datX == 38){
			// top_compartment_left_store
			top_compartment_left_store();
			ROS_INFO("top_compartment_left_store");
		}
		else if(datX == 39){
			// top_compartment_middle_store
			top_compartment_middle_store();
			ROS_INFO("top_compartment_middle_store");
		}
		else if(datX == 40){
			// top_compartment_right_store
			top_compartment_right_store();
			ROS_INFO("top_compartment_right_store");
		}
	}

	flag = datX;
  }
  return EXIT_SUCCESS;
}
