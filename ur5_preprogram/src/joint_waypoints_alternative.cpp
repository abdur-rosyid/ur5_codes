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

std::map<std::string, double> initJoints;


////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
float deg2rad(float deg){
return deg*3.14/180.0;
}

std::map<std::string, double> *jointsB1(){
	std::map<std::string, double> *joints = new std::map<std::string, double>[7];


	joints[0]["ur5_arm_shoulder_pan_joint"] = 3.06;
	joints[0]["ur5_arm_shoulder_lift_joint"] = -1.82;
	joints[0]["ur5_arm_elbow_joint"] = -0.04;
	joints[0]["ur5_arm_wrist_1_joint"] = -0.97;
	joints[0]["ur5_arm_wrist_2_joint"] = 1.59;
	joints[0]["ur5_arm_wrist_3_joint"] = 0.11;

	joints[1]["ur5_arm_shoulder_pan_joint"] = 4.64;
	joints[1]["ur5_arm_shoulder_lift_joint"] = -1.85;
	joints[1]["ur5_arm_elbow_joint"] = -0.04;
	joints[1]["ur5_arm_wrist_1_joint"] = -2.42;
	joints[1]["ur5_arm_wrist_2_joint"] = 1.59;
	joints[1]["ur5_arm_wrist_3_joint"] = 0.11;

	joints[2]["ur5_arm_shoulder_pan_joint"] = 5.41;
	joints[2]["ur5_arm_shoulder_lift_joint"] = -1.91;
	joints[2]["ur5_arm_elbow_joint"] = -0.52;
	joints[2]["ur5_arm_wrist_1_joint"] = -2.25;
	joints[2]["ur5_arm_wrist_2_joint"] = 1.57;
	joints[2]["ur5_arm_wrist_3_joint"] = 0.85;

	joints[3]["ur5_arm_shoulder_pan_joint"] = 5.42;
	joints[3]["ur5_arm_shoulder_lift_joint"] = -2.03;
	joints[3]["ur5_arm_elbow_joint"] = -0.52;
	joints[3]["ur5_arm_wrist_1_joint"] = -2.15;
	joints[3]["ur5_arm_wrist_2_joint"] = 1.58;
	joints[3]["ur5_arm_wrist_3_joint"] = 0.85;

	joints[4]["ur5_arm_shoulder_pan_joint"] = 5.42;
	joints[4]["ur5_arm_shoulder_lift_joint"] = -1.93;
	joints[4]["ur5_arm_elbow_joint"] = -0.88;
	joints[4]["ur5_arm_wrist_1_joint"] = -1.92;
	joints[4]["ur5_arm_wrist_2_joint"] = 1.56;
	joints[4]["ur5_arm_wrist_3_joint"] = 0.85;


	joints[5]["ur5_arm_shoulder_pan_joint"] = 5.42;
	joints[5]["ur5_arm_shoulder_lift_joint"] = -1.86;
	joints[5]["ur5_arm_elbow_joint"] = -1.09;
	joints[5]["ur5_arm_wrist_1_joint"] = -1.75;
	joints[5]["ur5_arm_wrist_2_joint"] = 1.56;
	joints[5]["ur5_arm_wrist_3_joint"] =0.85;


	joints[6]["ur5_arm_shoulder_pan_joint"] = deg2rad(310.83);
	joints[6]["ur5_arm_shoulder_lift_joint"] = deg2rad(-106.58);
	joints[6]["ur5_arm_elbow_joint"] = deg2rad(-64.95);
	joints[6]["ur5_arm_wrist_1_joint"] = deg2rad(-98.46);
	joints[6]["ur5_arm_wrist_2_joint"] = deg2rad(89.40);
	joints[6]["ur5_arm_wrist_3_joint"] = deg2rad(48.70);

	return joints;
}

std::map<std::string, double> *jointsB2(){
	std::map<std::string, double> *joints = new std::map<std::string, double>[13];
	joints[0]["ur5_arm_shoulder_pan_joint"] = 3.06;
	joints[0]["ur5_arm_shoulder_lift_joint"] = -1.82;
	joints[0]["ur5_arm_elbow_joint"] = -0.04;
	joints[0]["ur5_arm_wrist_1_joint"] = -2.74;
	joints[0]["ur5_arm_wrist_2_joint"] = 1.59;
	joints[0]["ur5_arm_wrist_3_joint"] = 0.11;

	joints[1]["ur5_arm_shoulder_pan_joint"] = 4.64;
	joints[1]["ur5_arm_shoulder_lift_joint"] = -1.85;
	joints[1]["ur5_arm_elbow_joint"] = -0.04;
	joints[1]["ur5_arm_wrist_1_joint"] = -2.74;
	joints[1]["ur5_arm_wrist_2_joint"] = 1.59;
	joints[1]["ur5_arm_wrist_3_joint"] = 0.11;

	joints[2]["ur5_arm_shoulder_pan_joint"] = 5.41;
	joints[2]["ur5_arm_shoulder_lift_joint"] = -1.91;
	joints[2]["ur5_arm_elbow_joint"] = -0.52;
	joints[2]["ur5_arm_wrist_1_joint"] = -2.25;
	joints[2]["ur5_arm_wrist_2_joint"] = 1.57;
	joints[2]["ur5_arm_wrist_3_joint"] = 0.85;

	joints[3]["ur5_arm_shoulder_pan_joint"] = 5.42;
	joints[3]["ur5_arm_shoulder_lift_joint"] = -2.03;
	joints[3]["ur5_arm_elbow_joint"] = -0.52;
	joints[3]["ur5_arm_wrist_1_joint"] = -2.15;
	joints[3]["ur5_arm_wrist_2_joint"] = 1.58;
	joints[3]["ur5_arm_wrist_3_joint"] = 0.85;

	joints[4]["ur5_arm_shoulder_pan_joint"] = 5.42;
	joints[4]["ur5_arm_shoulder_lift_joint"] = -1.93;
	joints[4]["ur5_arm_elbow_joint"] = -0.88;
	joints[4]["ur5_arm_wrist_1_joint"] = -1.92;
	joints[4]["ur5_arm_wrist_2_joint"] = 1.56;
	joints[4]["ur5_arm_wrist_3_joint"] = 0.85;

	joints[5]["ur5_arm_shoulder_pan_joint"] = 5.42;
	joints[5]["ur5_arm_shoulder_lift_joint"] = -1.86;
	joints[5]["ur5_arm_elbow_joint"] = -1.09;
	joints[5]["ur5_arm_wrist_1_joint"] = -1.75;
	joints[5]["ur5_arm_wrist_2_joint"] = 1.56;
	joints[5]["ur5_arm_wrist_3_joint"] =0.85;

	joints[6]["ur5_arm_shoulder_pan_joint"] = deg2rad(310.83);
	joints[6]["ur5_arm_shoulder_lift_joint"] = deg2rad(-106.58);
	joints[6]["ur5_arm_elbow_joint"] = deg2rad(-64.95);
	joints[6]["ur5_arm_wrist_1_joint"] = deg2rad(-98.46);
	joints[6]["ur5_arm_wrist_2_joint"] = deg2rad(89.40);
	joints[6]["ur5_arm_wrist_3_joint"] = deg2rad(48.70);

	joints[7]["ur5_arm_shoulder_pan_joint"] = 5.43;
	joints[7]["ur5_arm_shoulder_lift_joint"] = -1.87;
	joints[7]["ur5_arm_elbow_joint"] = -1.24;
	joints[7]["ur5_arm_wrist_1_joint"] = -1.63;
	joints[7]["ur5_arm_wrist_2_joint"] = 1.56;
	joints[7]["ur5_arm_wrist_3_joint"] = 0.85;

	joints[8]["ur5_arm_shoulder_pan_joint"] = 5.44;
	joints[8]["ur5_arm_shoulder_lift_joint"] = -1.86;
	joints[8]["ur5_arm_elbow_joint"] = -1.36;
	joints[8]["ur5_arm_wrist_1_joint"] = -1.50;
	joints[8]["ur5_arm_wrist_2_joint"] = 1.56;
	joints[8]["ur5_arm_wrist_3_joint"] = 0.83;

	joints[9]["ur5_arm_shoulder_pan_joint"] = 5.45;
	joints[9]["ur5_arm_shoulder_lift_joint"] = -1.90;
	joints[9]["ur5_arm_elbow_joint"] = -1.38;
	joints[9]["ur5_arm_wrist_1_joint"] = -1.51;
	joints[9]["ur5_arm_wrist_2_joint"] = 1.56;
	joints[9]["ur5_arm_wrist_3_joint"] = 0.84;

	joints[10]["ur5_arm_shoulder_pan_joint"] = 5.43;
	joints[10]["ur5_arm_shoulder_lift_joint"] = -1.87;
	joints[10]["ur5_arm_elbow_joint"] = -1.52;
	joints[10]["ur5_arm_wrist_1_joint"] = -1.32;
	joints[10]["ur5_arm_wrist_2_joint"] = 1.56;
	joints[10]["ur5_arm_wrist_3_joint"] = 0.80;

	joints[11]["ur5_arm_shoulder_pan_joint"] = 5.46;
	joints[11]["ur5_arm_shoulder_lift_joint"] = -1.92;
	joints[11]["ur5_arm_elbow_joint"] = -1.57;
	joints[11]["ur5_arm_wrist_1_joint"] = -1.20;
	joints[11]["ur5_arm_wrist_2_joint"] = 1.55;
	joints[11]["ur5_arm_wrist_3_joint"] = 0.76;

	joints[12]["ur5_arm_shoulder_pan_joint"] = 5.45;
	joints[12]["ur5_arm_shoulder_lift_joint"] = -1.92;
	joints[12]["ur5_arm_elbow_joint"] = -1.58;
	joints[12]["ur5_arm_wrist_1_joint"] = -1.18;
	joints[12]["ur5_arm_wrist_2_joint"] = 1.55;
	joints[12]["ur5_arm_wrist_3_joint"] =0.76;
	return joints;
}

std::map<std::string, double> *jointsG1(){
	std::map<std::string, double> *joints = new std::map<std::string, double>[6];
	joints[0]["ur5_arm_shoulder_pan_joint"] = 1.84;
	joints[0]["ur5_arm_shoulder_lift_joint"] = deg2rad(-106.78);
	joints[0]["ur5_arm_elbow_joint"] = -0.04;
	joints[0]["ur5_arm_wrist_1_joint"] = -0.97;
	joints[0]["ur5_arm_wrist_2_joint"] = 1.59;
	joints[0]["ur5_arm_wrist_3_joint"] = 0.09;

	joints[1]["ur5_arm_shoulder_pan_joint"] = deg2rad(101.22);
	joints[1]["ur5_arm_shoulder_lift_joint"] = deg2rad(-99.03);
	joints[1]["ur5_arm_elbow_joint"] = deg2rad(-41.77);
	joints[1]["ur5_arm_wrist_1_joint"] = deg2rad(-97.90);
	joints[1]["ur5_arm_wrist_2_joint"] = deg2rad(89.56);
	joints[1]["ur5_arm_wrist_3_joint"] = deg2rad(5.28);

	joints[2]["ur5_arm_shoulder_pan_joint"] = 1.75;
	joints[2]["ur5_arm_shoulder_lift_joint"] = -1.77;
	joints[2]["ur5_arm_elbow_joint"] = -0.83;
	joints[2]["ur5_arm_wrist_1_joint"] = -2.13;
	joints[2]["ur5_arm_wrist_2_joint"] = 1.54;
	joints[2]["ur5_arm_wrist_3_joint"] = 0.24;

	joints[3]["ur5_arm_shoulder_pan_joint"] = deg2rad(99.74);
	joints[3]["ur5_arm_shoulder_lift_joint"] = deg2rad(-107.65);
	joints[3]["ur5_arm_elbow_joint"] = deg2rad(-41.59);
	joints[3]["ur5_arm_wrist_1_joint"] = deg2rad(-124.24);
	joints[3]["ur5_arm_wrist_2_joint"] = deg2rad(88.51);
	joints[3]["ur5_arm_wrist_3_joint"] = deg2rad(18.41);

	joints[4]["ur5_arm_shoulder_pan_joint"] = deg2rad(100.29);
	joints[4]["ur5_arm_shoulder_lift_joint"] = deg2rad(-100.63);
	joints[4]["ur5_arm_elbow_joint"] = deg2rad(-60.12);
	joints[4]["ur5_arm_wrist_1_joint"] = deg2rad(-109.81);
	joints[4]["ur5_arm_wrist_2_joint"] = deg2rad(88.52);
	joints[4]["ur5_arm_wrist_3_joint"] = deg2rad(18.43);

	joints[5]["ur5_arm_shoulder_pan_joint"] = deg2rad(100.29);
	joints[5]["ur5_arm_shoulder_lift_joint"] = deg2rad(-95.05);
	joints[5]["ur5_arm_elbow_joint"] = deg2rad(-80.24);
	joints[5]["ur5_arm_wrist_1_joint"] = deg2rad(-93.30);
	joints[5]["ur5_arm_wrist_2_joint"] = deg2rad(88.51);
	joints[5]["ur5_arm_wrist_3_joint"] = deg2rad(18.41);
	return joints;
}

std::map<std::string, double> *jointsG2(){
	std::map<std::string, double> *joints = new std::map<std::string, double>[9];
	joints[0]["ur5_arm_shoulder_pan_joint"] = 1.84;
	joints[0]["ur5_arm_shoulder_lift_joint"] = deg2rad(-106.78);
	joints[0]["ur5_arm_elbow_joint"] = -0.04;
	joints[0]["ur5_arm_wrist_1_joint"] = -0.97;
	joints[0]["ur5_arm_wrist_2_joint"] = 1.59;
	joints[0]["ur5_arm_wrist_3_joint"] = 0.09;

	joints[1]["ur5_arm_shoulder_pan_joint"] = deg2rad(101.22);
	joints[1]["ur5_arm_shoulder_lift_joint"] = deg2rad(-99.03);
	joints[1]["ur5_arm_elbow_joint"] = deg2rad(-41.77);
	joints[1]["ur5_arm_wrist_1_joint"] = deg2rad(-97.90);
	joints[1]["ur5_arm_wrist_2_joint"] = deg2rad(89.56);
	joints[1]["ur5_arm_wrist_3_joint"] = deg2rad(5.28);

	joints[2]["ur5_arm_shoulder_pan_joint"] = 1.75;
	joints[2]["ur5_arm_shoulder_lift_joint"] = -1.77;
	joints[2]["ur5_arm_elbow_joint"] = -0.83;
	joints[2]["ur5_arm_wrist_1_joint"] = -2.13;
	joints[2]["ur5_arm_wrist_2_joint"] = 1.54;
	joints[2]["ur5_arm_wrist_3_joint"] = 0.24;

	joints[3]["ur5_arm_shoulder_pan_joint"] = deg2rad(99.74);
	joints[3]["ur5_arm_shoulder_lift_joint"] = deg2rad(-107.65);
	joints[3]["ur5_arm_elbow_joint"] = deg2rad(-41.59);
	joints[3]["ur5_arm_wrist_1_joint"] = deg2rad(-124.24);
	joints[3]["ur5_arm_wrist_2_joint"] = deg2rad(88.51);
	joints[3]["ur5_arm_wrist_3_joint"] = deg2rad(18.41);

	joints[4]["ur5_arm_shoulder_pan_joint"] = deg2rad(100.29);
	joints[4]["ur5_arm_shoulder_lift_joint"] = deg2rad(-100.63);
	joints[4]["ur5_arm_elbow_joint"] = deg2rad(-60.12);
	joints[4]["ur5_arm_wrist_1_joint"] = deg2rad(-109.81);
	joints[4]["ur5_arm_wrist_2_joint"] = deg2rad(88.52);
	joints[4]["ur5_arm_wrist_3_joint"] = deg2rad(18.43);

	joints[5]["ur5_arm_shoulder_pan_joint"] = deg2rad(100.29);
	joints[5]["ur5_arm_shoulder_lift_joint"] = deg2rad(-95.05);
	joints[5]["ur5_arm_elbow_joint"] = deg2rad(-80.24);
	joints[5]["ur5_arm_wrist_1_joint"] = deg2rad(-93.30);
	joints[5]["ur5_arm_wrist_2_joint"] = deg2rad(88.51);
	joints[5]["ur5_arm_wrist_3_joint"] = deg2rad(18.41);

	joints[6]["ur5_arm_shoulder_pan_joint"] = deg2rad(99.12);
	joints[6]["ur5_arm_shoulder_lift_joint"] = deg2rad(-96.90);
	joints[6]["ur5_arm_elbow_joint"] = deg2rad(-95.64);
	joints[6]["ur5_arm_wrist_1_joint"] = deg2rad(-79.31);
	joints[6]["ur5_arm_wrist_2_joint"] = deg2rad(88.23);
	joints[6]["ur5_arm_wrist_3_joint"] = deg2rad(12.22);

	joints[7]["ur5_arm_shoulder_pan_joint"] = deg2rad(99.12);
	joints[7]["ur5_arm_shoulder_lift_joint"] = deg2rad(-100.36);
	joints[7]["ur5_arm_elbow_joint"] = deg2rad(-97.25);
	joints[7]["ur5_arm_wrist_1_joint"] = deg2rad(-75.74);
	joints[7]["ur5_arm_wrist_2_joint"] = deg2rad(88.43);
	joints[7]["ur5_arm_wrist_3_joint"] = deg2rad(12.22);

	joints[8]["ur5_arm_shoulder_pan_joint"] = deg2rad(97.83);
	joints[8]["ur5_arm_shoulder_lift_joint"] = deg2rad(-101.68);
	joints[8]["ur5_arm_elbow_joint"] = deg2rad(-102.10);
	joints[8]["ur5_arm_wrist_1_joint"] = deg2rad(-68.23);
	joints[8]["ur5_arm_wrist_2_joint"] = deg2rad(88.84);
	joints[8]["ur5_arm_wrist_3_joint"] = deg2rad(12.22);

	return joints;
}

std::map<std::string, double> *jointsG3(){
	std::map<std::string, double> *joints = new std::map<std::string, double>[8];
	joints[0]["ur5_arm_shoulder_pan_joint"] = deg2rad(63.13);
	joints[0]["ur5_arm_shoulder_lift_joint"] = deg2rad(-113.35);
	joints[0]["ur5_arm_elbow_joint"] = deg2rad(-5.19);
	joints[0]["ur5_arm_wrist_1_joint"] = deg2rad(-117.36);
	joints[0]["ur5_arm_wrist_2_joint"] = deg2rad(88.44);
	joints[0]["ur5_arm_wrist_3_joint"] = deg2rad(-20.53);

	joints[1]["ur5_arm_shoulder_pan_joint"] = deg2rad(63.06);
	joints[1]["ur5_arm_shoulder_lift_joint"] = deg2rad(-119.82);
	joints[1]["ur5_arm_elbow_joint"] = deg2rad(-5.21);
	joints[1]["ur5_arm_wrist_1_joint"] = deg2rad(-128.02);
	joints[1]["ur5_arm_wrist_2_joint"] = deg2rad(88.44);
	joints[1]["ur5_arm_wrist_3_joint"] = deg2rad(-20.53);

	joints[2]["ur5_arm_shoulder_pan_joint"] = deg2rad(62.41);
	joints[2]["ur5_arm_shoulder_lift_joint"] = deg2rad(-126.67);
	joints[2]["ur5_arm_elbow_joint"] = deg2rad(-5.16);
	joints[2]["ur5_arm_wrist_1_joint"] = deg2rad(-130.00);
	joints[2]["ur5_arm_wrist_2_joint"] = deg2rad(68.44);
	joints[2]["ur5_arm_wrist_3_joint"] = deg2rad(-24.13);

	joints[3]["ur5_arm_shoulder_pan_joint"] = deg2rad(58.62);
	joints[3]["ur5_arm_shoulder_lift_joint"] = deg2rad(-133.84);
	joints[3]["ur5_arm_elbow_joint"] = deg2rad(-5.18);
	joints[3]["ur5_arm_wrist_1_joint"] = deg2rad(-129.99);
	joints[3]["ur5_arm_wrist_2_joint"] = deg2rad(88.44);
	joints[3]["ur5_arm_wrist_3_joint"] = deg2rad(-27.79);

	joints[4]["ur5_arm_shoulder_pan_joint"] = deg2rad(46.94);
	joints[4]["ur5_arm_shoulder_lift_joint"] =deg2rad(-139.02);
	joints[4]["ur5_arm_elbow_joint"] = deg2rad(-7.12);
	joints[4]["ur5_arm_wrist_1_joint"] = deg2rad(-111.26);
	joints[4]["ur5_arm_wrist_2_joint"] = deg2rad(88.16);
	joints[4]["ur5_arm_wrist_3_joint"] = deg2rad(-38.34);

	joints[5]["ur5_arm_shoulder_pan_joint"] = deg2rad(46.94);
	joints[5]["ur5_arm_shoulder_lift_joint"] =deg2rad(-139.02);
	joints[5]["ur5_arm_elbow_joint"] = deg2rad(-7.12);
	joints[5]["ur5_arm_wrist_1_joint"] = deg2rad(-111.26);
	joints[5]["ur5_arm_wrist_2_joint"] = deg2rad(88.16);
	joints[5]["ur5_arm_wrist_3_joint"] = deg2rad(-38.34);

	joints[6]["ur5_arm_shoulder_pan_joint"] = deg2rad(53.59);
	joints[6]["ur5_arm_shoulder_lift_joint"] =deg2rad(-137.60);
	joints[6]["ur5_arm_elbow_joint"] = deg2rad(-7.17);
	joints[6]["ur5_arm_wrist_1_joint"] = deg2rad(-129.14);
	joints[6]["ur5_arm_wrist_2_joint"] = deg2rad(89.19);
	joints[6]["ur5_arm_wrist_3_joint"] = deg2rad(-38.32);

	joints[7]["ur5_arm_shoulder_pan_joint"] = deg2rad(46.94);
	joints[7]["ur5_arm_shoulder_lift_joint"] =deg2rad(-142.26);
	joints[7]["ur5_arm_elbow_joint"] = deg2rad(-7.18);
	joints[7]["ur5_arm_wrist_1_joint"] = deg2rad(-121.62);
	joints[7]["ur5_arm_wrist_2_joint"] = deg2rad(88.50);
	joints[7]["ur5_arm_wrist_3_joint"] = deg2rad(-36.53);

	return joints;
}

std::map<std::string, double> *jointsG4(){
	std::map<std::string, double> *joints = new std::map<std::string, double>[12];
	joints[0]["ur5_arm_shoulder_pan_joint"] = deg2rad(63.13);
	joints[0]["ur5_arm_shoulder_lift_joint"] = deg2rad(-113.35);
	joints[0]["ur5_arm_elbow_joint"] = deg2rad(-5.19);
	joints[0]["ur5_arm_wrist_1_joint"] = deg2rad(-117.36);
	joints[0]["ur5_arm_wrist_2_joint"] = deg2rad(88.44);
	joints[0]["ur5_arm_wrist_3_joint"] = deg2rad(-20.53);

	joints[1]["ur5_arm_shoulder_pan_joint"] = deg2rad(63.06);
	joints[1]["ur5_arm_shoulder_lift_joint"] = deg2rad(-119.82);
	joints[1]["ur5_arm_elbow_joint"] = deg2rad(-5.21);
	joints[1]["ur5_arm_wrist_1_joint"] = deg2rad(-128.02);
	joints[1]["ur5_arm_wrist_2_joint"] = deg2rad(88.44);
	joints[1]["ur5_arm_wrist_3_joint"] = deg2rad(-20.53);

	joints[2]["ur5_arm_shoulder_pan_joint"] = deg2rad(62.41);
	joints[2]["ur5_arm_shoulder_lift_joint"] = deg2rad(-126.67);
	joints[2]["ur5_arm_elbow_joint"] = deg2rad(-5.16);
	joints[2]["ur5_arm_wrist_1_joint"] = deg2rad(-130.00);
	joints[2]["ur5_arm_wrist_2_joint"] = deg2rad(68.44);
	joints[2]["ur5_arm_wrist_3_joint"] = deg2rad(-24.13);

	joints[3]["ur5_arm_shoulder_pan_joint"] = deg2rad(58.62);
	joints[3]["ur5_arm_shoulder_lift_joint"] = deg2rad(-133.84);
	joints[3]["ur5_arm_elbow_joint"] = deg2rad(-5.18);
	joints[3]["ur5_arm_wrist_1_joint"] = deg2rad(-129.99);
	joints[3]["ur5_arm_wrist_2_joint"] = deg2rad(88.44);
	joints[3]["ur5_arm_wrist_3_joint"] = deg2rad(-27.79);

	joints[4]["ur5_arm_shoulder_pan_joint"] = 0.94;
	joints[4]["ur5_arm_shoulder_lift_joint"] =-2.39 ;
	joints[4]["ur5_arm_elbow_joint"] = -0.24;
	joints[4]["ur5_arm_wrist_1_joint"] = -2.13;
	joints[4]["ur5_arm_wrist_2_joint"] = 1.54;
	joints[4]["ur5_arm_wrist_3_joint"] = -0.57;

	joints[5]["ur5_arm_shoulder_pan_joint"] = 0.86;
	joints[5]["ur5_arm_shoulder_lift_joint"] =-2.54;
	joints[5]["ur5_arm_elbow_joint"] = 0.04;
	joints[5]["ur5_arm_wrist_1_joint"] = -2.18;
	joints[5]["ur5_arm_wrist_2_joint"] = 1.54;
	joints[5]["ur5_arm_wrist_3_joint"] = -0.67;

	joints[6]["ur5_arm_shoulder_pan_joint"] = 0.81;
	joints[6]["ur5_arm_shoulder_lift_joint"] = -2.49;
	joints[6]["ur5_arm_elbow_joint"] = -0.12;
	joints[6]["ur5_arm_wrist_1_joint"] = -2.04;
	joints[6]["ur5_arm_wrist_2_joint"] = 1.59;
	joints[6]["ur5_arm_wrist_3_joint"] = -0.68;

	joints[7]["ur5_arm_shoulder_pan_joint"] = 0.81;
	joints[7]["ur5_arm_shoulder_lift_joint"] = -2.40;
	joints[7]["ur5_arm_elbow_joint"] = -0.46;
	joints[7]["ur5_arm_wrist_1_joint"] = -1.81;
	joints[7]["ur5_arm_wrist_2_joint"] = 1.59;
	joints[7]["ur5_arm_wrist_3_joint"] = -0.68;

	joints[8]["ur5_arm_shoulder_pan_joint"] = 0.81;
	joints[8]["ur5_arm_shoulder_lift_joint"] = -2.50;
	joints[8]["ur5_arm_elbow_joint"] = -0.34;
	joints[8]["ur5_arm_wrist_1_joint"] = -1.85;
	joints[8]["ur5_arm_wrist_2_joint"] = 1.59;
	joints[8]["ur5_arm_wrist_3_joint"] = -0.73;

	joints[9]["ur5_arm_shoulder_pan_joint"] = deg2rad(46.49);
	joints[9]["ur5_arm_shoulder_lift_joint"] = deg2rad(-134.32);
	joints[9]["ur5_arm_elbow_joint"] = deg2rad(-34.32);
	joints[9]["ur5_arm_wrist_1_joint"] = deg2rad(-93.16);
	joints[9]["ur5_arm_wrist_2_joint"] = deg2rad(88.50);
	joints[9]["ur5_arm_wrist_3_joint"] = deg2rad(-41.80);

	joints[10]["ur5_arm_shoulder_pan_joint"] = deg2rad(46.30);
	joints[10]["ur5_arm_shoulder_lift_joint"] = deg2rad(-141.53);
	joints[10]["ur5_arm_elbow_joint"] = deg2rad(-34.38);
	joints[10]["ur5_arm_wrist_1_joint"] = deg2rad(-93.16);
	joints[10]["ur5_arm_wrist_2_joint"] = deg2rad(88.50);
	joints[10]["ur5_arm_wrist_3_joint"] = deg2rad(-41.80);

	joints[11]["ur5_arm_shoulder_pan_joint"] = deg2rad(46.26);
	joints[11]["ur5_arm_shoulder_lift_joint"] = deg2rad(-144.11);
	joints[11]["ur5_arm_elbow_joint"] = deg2rad(-34.42);
	joints[11]["ur5_arm_wrist_1_joint"] = deg2rad(-92.97);
	joints[11]["ur5_arm_wrist_2_joint"] = deg2rad(88.49);
	joints[11]["ur5_arm_wrist_3_joint"] = deg2rad(-41.80);
	return joints;
}

std::map<std::string, double> *jointsR1(){
	std::map<std::string, double> *joints = new std::map<std::string, double>[6];
	joints[0]["ur5_arm_shoulder_pan_joint"] = 3.16;
	joints[0]["ur5_arm_shoulder_lift_joint"] = -1.92;
	joints[0]["ur5_arm_elbow_joint"] = -0.04;
	joints[0]["ur5_arm_wrist_1_joint"] = -2.74;
	joints[0]["ur5_arm_wrist_2_joint"] = 1.59;
	joints[0]["ur5_arm_wrist_3_joint"] = 0.09;

	joints[1]["ur5_arm_shoulder_pan_joint"] = 1.02;
	joints[1]["ur5_arm_shoulder_lift_joint"] = -1.92;
	joints[1]["ur5_arm_elbow_joint"] = -0.04;
	joints[1]["ur5_arm_wrist_1_joint"] = -2.74;
	joints[1]["ur5_arm_wrist_2_joint"] = 1.59;
	joints[1]["ur5_arm_wrist_3_joint"] = 0.09;

	joints[2]["ur5_arm_shoulder_pan_joint"] = 0.79;
	joints[2]["ur5_arm_shoulder_lift_joint"] = -1.92;
	joints[2]["ur5_arm_elbow_joint"] = -0.04;
	joints[2]["ur5_arm_wrist_1_joint"] = -2.74;
	joints[2]["ur5_arm_wrist_2_joint"] = 1.59;
	joints[2]["ur5_arm_wrist_3_joint"] = 0.09;

	joints[3]["ur5_arm_shoulder_pan_joint"] = deg2rad(40.22);
	joints[3]["ur5_arm_shoulder_lift_joint"] = deg2rad(-98.21);
	joints[3]["ur5_arm_elbow_joint"] = deg2rad(-34.44);
	joints[3]["ur5_arm_wrist_1_joint"] = deg2rad(-133.95);
	joints[3]["ur5_arm_wrist_2_joint"] = deg2rad(88.34);
	joints[3]["ur5_arm_wrist_3_joint"] = deg2rad(-39.36);

	joints[4]["ur5_arm_shoulder_pan_joint"] = deg2rad(39.32);
	joints[4]["ur5_arm_shoulder_lift_joint"] = deg2rad(-99.81);
	joints[4]["ur5_arm_elbow_joint"] = deg2rad(-42.78);
	joints[4]["ur5_arm_wrist_1_joint"] = deg2rad(-128.46);
	joints[4]["ur5_arm_wrist_2_joint"] = deg2rad(88.26);
	joints[4]["ur5_arm_wrist_3_joint"] = deg2rad(-39.36);

	joints[5]["ur5_arm_shoulder_pan_joint"] = deg2rad(39.63);
	joints[5]["ur5_arm_shoulder_lift_joint"] = deg2rad(-87.55);
	joints[5]["ur5_arm_elbow_joint"] = deg2rad(-72.03);
	joints[5]["ur5_arm_wrist_1_joint"] = deg2rad(-110.61);
	joints[5]["ur5_arm_wrist_2_joint"] = deg2rad(88.66);
	joints[5]["ur5_arm_wrist_3_joint"] = deg2rad(-44.54);
	return joints;
}

std::map<std::string, double> *jointsR2(){
	std::map<std::string, double> *joints = new std::map<std::string, double>[9];
	joints[0]["ur5_arm_shoulder_pan_joint"] = 3.16;
	joints[0]["ur5_arm_shoulder_lift_joint"] = -1.92;
	joints[0]["ur5_arm_elbow_joint"] = -0.04;
	joints[0]["ur5_arm_wrist_1_joint"] = -2.74;
	joints[0]["ur5_arm_wrist_2_joint"] = 1.59;
	joints[0]["ur5_arm_wrist_3_joint"] = 0.09;

	joints[1]["ur5_arm_shoulder_pan_joint"] = 1.02;
	joints[1]["ur5_arm_shoulder_lift_joint"] = -1.92;
	joints[1]["ur5_arm_elbow_joint"] = -0.04;
	joints[1]["ur5_arm_wrist_1_joint"] = -2.74;
	joints[1]["ur5_arm_wrist_2_joint"] = 1.59;
	joints[1]["ur5_arm_wrist_3_joint"] = 0.09;

	joints[2]["ur5_arm_shoulder_pan_joint"] = 1.02;
	joints[2]["ur5_arm_shoulder_lift_joint"] = -1.92;
	joints[2]["ur5_arm_elbow_joint"] = -0.04;
	joints[2]["ur5_arm_wrist_1_joint"] = -2.74;
	joints[2]["ur5_arm_wrist_2_joint"] = 1.59;
	joints[2]["ur5_arm_wrist_3_joint"] = 0.09;

	joints[3]["ur5_arm_shoulder_pan_joint"] = 0.79;
	joints[3]["ur5_arm_shoulder_lift_joint"] = -1.92;
	joints[3]["ur5_arm_elbow_joint"] = -0.04;
	joints[3]["ur5_arm_wrist_1_joint"] = -2.74;
	joints[3]["ur5_arm_wrist_2_joint"] = 1.59;
	joints[3]["ur5_arm_wrist_3_joint"] = 0.09;

	joints[4]["ur5_arm_shoulder_pan_joint"] = deg2rad(40.22);
	joints[4]["ur5_arm_shoulder_lift_joint"] = deg2rad(-98.21);
	joints[4]["ur5_arm_elbow_joint"] = deg2rad(-34.44);
	joints[4]["ur5_arm_wrist_1_joint"] = deg2rad(-133.95);
	joints[4]["ur5_arm_wrist_2_joint"] = deg2rad(88.34);
	joints[4]["ur5_arm_wrist_3_joint"] = deg2rad(-39.36);

	joints[5]["ur5_arm_shoulder_pan_joint"] = deg2rad(39.32);
	joints[5]["ur5_arm_shoulder_lift_joint"] = deg2rad(-99.81);
	joints[5]["ur5_arm_elbow_joint"] = deg2rad(-42.78);
	joints[5]["ur5_arm_wrist_1_joint"] = deg2rad(-128.46);
	joints[5]["ur5_arm_wrist_2_joint"] = deg2rad(88.26);
	joints[5]["ur5_arm_wrist_3_joint"] = deg2rad(-39.36);

	joints[6]["ur5_arm_shoulder_pan_joint"] = deg2rad(39.63);
	joints[6]["ur5_arm_shoulder_lift_joint"] = deg2rad(-87.55);
	joints[6]["ur5_arm_elbow_joint"] = deg2rad(-72.03);
	joints[6]["ur5_arm_wrist_1_joint"] = deg2rad(-110.61);
	joints[6]["ur5_arm_wrist_2_joint"] = deg2rad(88.66);
	joints[6]["ur5_arm_wrist_3_joint"] = deg2rad(-44.54);

	joints[7]["ur5_arm_shoulder_pan_joint"] = deg2rad(41.45);
	joints[7]["ur5_arm_shoulder_lift_joint"] = deg2rad(-86.02);
	joints[7]["ur5_arm_elbow_joint"] = deg2rad(-80.89);
	joints[7]["ur5_arm_wrist_1_joint"] = deg2rad(-103.04);
	joints[7]["ur5_arm_wrist_2_joint"] = deg2rad(89.09);
	joints[7]["ur5_arm_wrist_3_joint"] = deg2rad(-47.09);

	joints[8]["ur5_arm_shoulder_pan_joint"] = deg2rad(41.4);
	joints[8]["ur5_arm_shoulder_lift_joint"] = deg2rad(-88.12);
	joints[8]["ur5_arm_elbow_joint"] = deg2rad(-100.88);
	joints[8]["ur5_arm_wrist_1_joint"] = deg2rad(-81.62);
	joints[8]["ur5_arm_wrist_2_joint"] = deg2rad(89.08);
	joints[8]["ur5_arm_wrist_3_joint"] = deg2rad(-47.09);
	return joints;
}

std::map<std::string, double> *jointsR3(){
	std::map<std::string, double> *joints = new std::map<std::string, double>[7];

	joints[0]["ur5_arm_shoulder_pan_joint"] = 3.09;
	joints[0]["ur5_arm_shoulder_lift_joint"] = -1.92;
	joints[0]["ur5_arm_elbow_joint"] = -0.04;
	joints[0]["ur5_arm_wrist_1_joint"] = -2.74;
	joints[0]["ur5_arm_wrist_2_joint"] = 1.59;
	joints[0]["ur5_arm_wrist_3_joint"] = 0.09;

	joints[1]["ur5_arm_shoulder_pan_joint"] = 0.42;
	joints[1]["ur5_arm_shoulder_lift_joint"] = -1.92;
	joints[1]["ur5_arm_elbow_joint"] = -0.04;
	joints[1]["ur5_arm_wrist_1_joint"] = -2.74;
	joints[1]["ur5_arm_wrist_2_joint"] = 1.59;
	joints[1]["ur5_arm_wrist_3_joint"] = 0.09;

	joints[2]["ur5_arm_shoulder_pan_joint"] = 0.27;
	joints[2]["ur5_arm_shoulder_lift_joint"] = -1.92;
	joints[2]["ur5_arm_elbow_joint"] = -0.04;
	joints[2]["ur5_arm_wrist_1_joint"] = -2.74;
	joints[2]["ur5_arm_wrist_2_joint"] = 1.59;
	joints[2]["ur5_arm_wrist_3_joint"] = 0.09;

	joints[3]["ur5_arm_shoulder_pan_joint"] = 0.27;
	joints[3]["ur5_arm_shoulder_lift_joint"] = -1.92;
	joints[3]["ur5_arm_elbow_joint"] = -0.04;
	joints[3]["ur5_arm_wrist_1_joint"] = -2.74;
	joints[3]["ur5_arm_wrist_2_joint"] = 1.59;
	joints[3]["ur5_arm_wrist_3_joint"] = 0.09;

	joints[4]["ur5_arm_shoulder_pan_joint"] = deg2rad(14.92);
	joints[4]["ur5_arm_shoulder_lift_joint"] = deg2rad(-89.66);
	joints[4]["ur5_arm_elbow_joint"] = deg2rad(-49.23);
	joints[4]["ur5_arm_wrist_1_joint"] = deg2rad(-130.54);
	joints[4]["ur5_arm_wrist_2_joint"] = deg2rad(86.87);
	joints[4]["ur5_arm_wrist_3_joint"] = deg2rad(-67.44);

	joints[5]["ur5_arm_shoulder_pan_joint"] = deg2rad(14.91);
	joints[5]["ur5_arm_shoulder_lift_joint"] = deg2rad(-81.52);
	joints[5]["ur5_arm_elbow_joint"] = deg2rad(-76.81);
	joints[5]["ur5_arm_wrist_1_joint"] = deg2rad(-112.77);
	joints[5]["ur5_arm_wrist_2_joint"] = deg2rad(86.88);
	joints[5]["ur5_arm_wrist_3_joint"] = deg2rad(-67.43);

	joints[6]["ur5_arm_shoulder_pan_joint"] = deg2rad(14.91);
	joints[6]["ur5_arm_shoulder_lift_joint"] = deg2rad(-81.52);
	joints[6]["ur5_arm_elbow_joint"] = deg2rad(-76.81);
	joints[6]["ur5_arm_wrist_1_joint"] = deg2rad(-112.77);
	joints[6]["ur5_arm_wrist_2_joint"] = deg2rad(86.88);
	joints[6]["ur5_arm_wrist_3_joint"] = deg2rad(-67.43);
	return joints;
}

std::map<std::string, double> *jointsR4(){
	std::map<std::string, double> *joints = new std::map<std::string, double>[9];
	joints[0]["ur5_arm_shoulder_pan_joint"] = 3.09;
	joints[0]["ur5_arm_shoulder_lift_joint"] = -1.92;
	joints[0]["ur5_arm_elbow_joint"] = -0.04;
	joints[0]["ur5_arm_wrist_1_joint"] = -2.74;
	joints[0]["ur5_arm_wrist_2_joint"] = 1.59;
	joints[0]["ur5_arm_wrist_3_joint"] = 0.09;

	joints[1]["ur5_arm_shoulder_pan_joint"] = 0.42;
	joints[1]["ur5_arm_shoulder_lift_joint"] = -1.92;
	joints[1]["ur5_arm_elbow_joint"] = -0.04;
	joints[1]["ur5_arm_wrist_1_joint"] = -2.74;
	joints[1]["ur5_arm_wrist_2_joint"] = 1.59;
	joints[1]["ur5_arm_wrist_3_joint"] = 0.09;

	joints[2]["ur5_arm_shoulder_pan_joint"] = 0.27;
	joints[2]["ur5_arm_shoulder_lift_joint"] = -1.92;
	joints[2]["ur5_arm_elbow_joint"] = -0.04;
	joints[2]["ur5_arm_wrist_1_joint"] = -2.74;
	joints[2]["ur5_arm_wrist_2_joint"] = 1.59;
	joints[2]["ur5_arm_wrist_3_joint"] = 0.09;

	joints[3]["ur5_arm_shoulder_pan_joint"] = 0.27;
	joints[3]["ur5_arm_shoulder_lift_joint"] = -1.92;
	joints[3]["ur5_arm_elbow_joint"] = -0.04;
	joints[3]["ur5_arm_wrist_1_joint"] = -2.74;
	joints[3]["ur5_arm_wrist_2_joint"] = 1.59;
	joints[3]["ur5_arm_wrist_3_joint"] = 0.09;

	joints[4]["ur5_arm_shoulder_pan_joint"] = deg2rad(14.92);
	joints[4]["ur5_arm_shoulder_lift_joint"] = deg2rad(-89.66);
	joints[4]["ur5_arm_elbow_joint"] = deg2rad(-49.23);
	joints[4]["ur5_arm_wrist_1_joint"] = deg2rad(-130.54);
	joints[4]["ur5_arm_wrist_2_joint"] = deg2rad(86.87);
	joints[4]["ur5_arm_wrist_3_joint"] = deg2rad(-67.44);

	joints[5]["ur5_arm_shoulder_pan_joint"] = deg2rad(14.92);
	joints[5]["ur5_arm_shoulder_lift_joint"] = deg2rad(-89.66);
	joints[5]["ur5_arm_elbow_joint"] = deg2rad(-49.23);
	joints[5]["ur5_arm_wrist_1_joint"] = deg2rad(-130.54);
	joints[5]["ur5_arm_wrist_2_joint"] = deg2rad(86.87);
	joints[5]["ur5_arm_wrist_3_joint"] = deg2rad(-67.44);

	joints[6]["ur5_arm_shoulder_pan_joint"] = deg2rad(14.91);
	joints[6]["ur5_arm_shoulder_lift_joint"] = deg2rad(-81.52);
	joints[6]["ur5_arm_elbow_joint"] = deg2rad(-76.81);
	joints[6]["ur5_arm_wrist_1_joint"] = deg2rad(-112.77);
	joints[6]["ur5_arm_wrist_2_joint"] = deg2rad(86.88);
	joints[6]["ur5_arm_wrist_3_joint"] = deg2rad(-67.43);

	joints[7]["ur5_arm_shoulder_pan_joint"] = deg2rad(7.03);
	joints[7]["ur5_arm_shoulder_lift_joint"] = deg2rad(-62.79);
	joints[7]["ur5_arm_elbow_joint"] = deg2rad(-100.97);
	joints[7]["ur5_arm_wrist_1_joint"] = deg2rad(-81.92);
	joints[7]["ur5_arm_wrist_2_joint"] = deg2rad(96.77);
	joints[7]["ur5_arm_wrist_3_joint"] = deg2rad(-64.92);

	joints[8]["ur5_arm_shoulder_pan_joint"] = deg2rad(15.67);
	joints[8]["ur5_arm_shoulder_lift_joint"] = deg2rad(-78.40);
	joints[8]["ur5_arm_elbow_joint"] = deg2rad(-109.78);
	joints[8]["ur5_arm_wrist_1_joint"] = deg2rad(-80.61);
	joints[8]["ur5_arm_wrist_2_joint"] = deg2rad(86.84);
	joints[8]["ur5_arm_wrist_3_joint"] = deg2rad(-64.93);
	return joints;
}

std::map<std::string, double> *jointsR5(){
	std::map<std::string, double> *joints = new std::map<std::string, double>[6];

	joints[0]["ur5_arm_shoulder_pan_joint"] = 3.21;
	joints[0]["ur5_arm_shoulder_lift_joint"] = -1.92;
	joints[0]["ur5_arm_elbow_joint"] = -0.04;
	joints[0]["ur5_arm_wrist_1_joint"] = -2.74;
	joints[0]["ur5_arm_wrist_2_joint"] = 1.59;
	joints[0]["ur5_arm_wrist_3_joint"] = 0.09;

	joints[1]["ur5_arm_shoulder_pan_joint"] = 5.95;
	joints[1]["ur5_arm_shoulder_lift_joint"] = -1.92;
	joints[1]["ur5_arm_elbow_joint"] = -0.04;
	joints[1]["ur5_arm_wrist_1_joint"] = -2.74;
	joints[1]["ur5_arm_wrist_2_joint"] = 1.59;
	joints[1]["ur5_arm_wrist_3_joint"] = 0.09;

	joints[2]["ur5_arm_shoulder_pan_joint"] = 5.95;
	joints[2]["ur5_arm_shoulder_lift_joint"] = -1.92;
	joints[2]["ur5_arm_elbow_joint"] = -0.04;
	joints[2]["ur5_arm_wrist_1_joint"] = -2.74;
	joints[2]["ur5_arm_wrist_2_joint"] = 1.59;
	joints[2]["ur5_arm_wrist_3_joint"] = 0.09;

	joints[3]["ur5_arm_shoulder_pan_joint"] = deg2rad(350.66);
	joints[3]["ur5_arm_shoulder_lift_joint"] = deg2rad(-86.04);
	joints[3]["ur5_arm_elbow_joint"] = deg2rad(-44.39);
	joints[3]["ur5_arm_wrist_1_joint"] = deg2rad(-111.49);
	joints[3]["ur5_arm_wrist_2_joint"] = deg2rad(87.33);
	joints[3]["ur5_arm_wrist_3_joint"] = deg2rad(-96.78);

	joints[4]["ur5_arm_shoulder_pan_joint"] = deg2rad(344.94);
	joints[4]["ur5_arm_shoulder_lift_joint"] = deg2rad(-88.29);
	joints[4]["ur5_arm_elbow_joint"] = deg2rad(-70.69);
	joints[4]["ur5_arm_wrist_1_joint"] = deg2rad(-111.38);
	joints[4]["ur5_arm_wrist_2_joint"] = deg2rad(87.36);
	joints[4]["ur5_arm_wrist_3_joint"] = deg2rad(-96.79);

	joints[5]["ur5_arm_shoulder_pan_joint"] = deg2rad(344.94);
	joints[5]["ur5_arm_shoulder_lift_joint"] = deg2rad(-88.29);
	joints[5]["ur5_arm_elbow_joint"] = deg2rad(-70.69);
	joints[5]["ur5_arm_wrist_1_joint"] = deg2rad(-111.38);
	joints[5]["ur5_arm_wrist_2_joint"] = deg2rad(87.36);
	joints[5]["ur5_arm_wrist_3_joint"] = deg2rad(-96.79);
	return joints;
}

std::map<std::string, double> *jointsR6(){
	std::map<std::string, double> *joints = new std::map<std::string, double>[9];
	joints[0]["ur5_arm_shoulder_pan_joint"] = 3.21;
	joints[0]["ur5_arm_shoulder_lift_joint"] = -1.92;
	joints[0]["ur5_arm_elbow_joint"] = -0.04;
	joints[0]["ur5_arm_wrist_1_joint"] = -2.74;
	joints[0]["ur5_arm_wrist_2_joint"] = 1.59;
	joints[0]["ur5_arm_wrist_3_joint"] = 0.09;

	joints[1]["ur5_arm_shoulder_pan_joint"] = 5.95;
	joints[1]["ur5_arm_shoulder_lift_joint"] = -1.92;
	joints[1]["ur5_arm_elbow_joint"] = -0.04;
	joints[1]["ur5_arm_wrist_1_joint"] = -2.74;
	joints[1]["ur5_arm_wrist_2_joint"] = 1.59;
	joints[1]["ur5_arm_wrist_3_joint"] = 0.09;

	joints[2]["ur5_arm_shoulder_pan_joint"] = 5.95;
	joints[2]["ur5_arm_shoulder_lift_joint"] = -1.92;
	joints[2]["ur5_arm_elbow_joint"] = -0.04;
	joints[2]["ur5_arm_wrist_1_joint"] = -2.74;
	joints[2]["ur5_arm_wrist_2_joint"] = 1.59;
	joints[2]["ur5_arm_wrist_3_joint"] = 0.09;

	joints[3]["ur5_arm_shoulder_pan_joint"] = 5.96;
	joints[3]["ur5_arm_shoulder_lift_joint"] = -1.92;
	joints[3]["ur5_arm_elbow_joint"] = -0.04;
	joints[3]["ur5_arm_wrist_1_joint"] = -2.74;
	joints[3]["ur5_arm_wrist_2_joint"] = 1.59;
	joints[3]["ur5_arm_wrist_3_joint"] = 0.09;

	joints[4]["ur5_arm_shoulder_pan_joint"] = deg2rad(350.66);
	joints[4]["ur5_arm_shoulder_lift_joint"] = deg2rad(-86.04);
	joints[4]["ur5_arm_elbow_joint"] = deg2rad(-44.39);
	joints[4]["ur5_arm_wrist_1_joint"] = deg2rad(-111.49);
	joints[4]["ur5_arm_wrist_2_joint"] = deg2rad(87.33);
	joints[4]["ur5_arm_wrist_3_joint"] = deg2rad(-96.78);

	joints[5]["ur5_arm_shoulder_pan_joint"] = deg2rad(344.94);
	joints[5]["ur5_arm_shoulder_lift_joint"] = deg2rad(-88.29);
	joints[5]["ur5_arm_elbow_joint"] = deg2rad(-70.69);
	joints[5]["ur5_arm_wrist_1_joint"] = deg2rad(-111.38);
	joints[5]["ur5_arm_wrist_2_joint"] = deg2rad(87.36);
	joints[5]["ur5_arm_wrist_3_joint"] = deg2rad(-96.79);

	joints[6]["ur5_arm_shoulder_pan_joint"] = deg2rad(344.94);
	joints[6]["ur5_arm_shoulder_lift_joint"] = deg2rad(-88.29);
	joints[6]["ur5_arm_elbow_joint"] = deg2rad(-70.69);
	joints[6]["ur5_arm_wrist_1_joint"] = deg2rad(-111.38);
	joints[6]["ur5_arm_wrist_2_joint"] = deg2rad(87.36);
	joints[6]["ur5_arm_wrist_3_joint"] = deg2rad(-96.79);

	joints[7]["ur5_arm_shoulder_pan_joint"] = deg2rad(344.61);
	joints[7]["ur5_arm_shoulder_lift_joint"] = deg2rad(-73.51);
	joints[7]["ur5_arm_elbow_joint"] = deg2rad(-95.65);
	joints[7]["ur5_arm_wrist_1_joint"] = deg2rad(-88.61);
	joints[7]["ur5_arm_wrist_2_joint"] = deg2rad(86.84);
	joints[7]["ur5_arm_wrist_3_joint"] = deg2rad(-99.17);

	joints[8]["ur5_arm_shoulder_pan_joint"] = deg2rad(344.01);
	joints[8]["ur5_arm_shoulder_lift_joint"] = deg2rad(-87.92);
	joints[8]["ur5_arm_elbow_joint"] = deg2rad(-100.91);
	joints[8]["ur5_arm_wrist_1_joint"] = deg2rad(-81.76);
	joints[8]["ur5_arm_wrist_2_joint"] = deg2rad(86.87);
	joints[8]["ur5_arm_wrist_3_joint"] = deg2rad(-99.17);
	return joints;
}

std::map<std::string, double> *jointsR7(){
	std::map<std::string, double> *joints = new std::map<std::string, double>[12];
	joints[0]["ur5_arm_shoulder_pan_joint"] = 1.84;
	joints[0]["ur5_arm_shoulder_lift_joint"] = deg2rad(-106.78);
	joints[0]["ur5_arm_elbow_joint"] = -0.04;
	joints[0]["ur5_arm_wrist_1_joint"] = -0.97;
	joints[0]["ur5_arm_wrist_2_joint"] = 1.59;
	joints[0]["ur5_arm_wrist_3_joint"] = 0.09;

	joints[1]["ur5_arm_shoulder_pan_joint"] = 1.75;
	joints[1]["ur5_arm_shoulder_lift_joint"] = -1.95;
	joints[1]["ur5_arm_elbow_joint"] = -0.33;
	joints[1]["ur5_arm_wrist_1_joint"] = -2.42;
	joints[1]["ur5_arm_wrist_2_joint"] = 1.54;
	joints[1]["ur5_arm_wrist_3_joint"] = 0.24;

	joints[2]["ur5_arm_shoulder_pan_joint"] = 1.75;
	joints[2]["ur5_arm_shoulder_lift_joint"] = -1.77;
	joints[2]["ur5_arm_elbow_joint"] = -0.83;
	joints[2]["ur5_arm_wrist_1_joint"] = -2.13;
	joints[2]["ur5_arm_wrist_2_joint"] = 1.54;
	joints[2]["ur5_arm_wrist_3_joint"] = 0.24;

	joints[3]["ur5_arm_shoulder_pan_joint"] = deg2rad(83.81);
	joints[3]["ur5_arm_shoulder_lift_joint"] = deg2rad(-110.73);
	joints[3]["ur5_arm_elbow_joint"] = deg2rad(-36.25);
	joints[3]["ur5_arm_wrist_1_joint"] = deg2rad(-125.63);
	joints[3]["ur5_arm_wrist_2_joint"] = deg2rad(88.59);
	joints[3]["ur5_arm_wrist_3_joint"] = deg2rad(-0.58);

	joints[4]["ur5_arm_shoulder_pan_joint"] = deg2rad(83.92);
	joints[4]["ur5_arm_shoulder_lift_joint"] = deg2rad(-104.51);
	joints[4]["ur5_arm_elbow_joint"] = deg2rad(-52.76);
	joints[4]["ur5_arm_wrist_1_joint"] = deg2rad(-113.73);
	joints[4]["ur5_arm_wrist_2_joint"] = deg2rad(88.60);
	joints[4]["ur5_arm_wrist_3_joint"] = deg2rad(-0.58);

	joints[5]["ur5_arm_shoulder_pan_joint"] = deg2rad(83.90);
	joints[5]["ur5_arm_shoulder_lift_joint"] = deg2rad(-100.65);
	joints[5]["ur5_arm_elbow_joint"] = deg2rad(-71.97);
	joints[5]["ur5_arm_wrist_1_joint"] = deg2rad(-98.32);
	joints[5]["ur5_arm_wrist_2_joint"] = deg2rad(88.60);
	joints[5]["ur5_arm_wrist_3_joint"] = deg2rad(-0.58);

	joints[6]["ur5_arm_shoulder_pan_joint"] = deg2rad(82.16);
	joints[6]["ur5_arm_shoulder_lift_joint"] = deg2rad(-101.79);
	joints[6]["ur5_arm_elbow_joint"] = deg2rad(-82.65);
	joints[6]["ur5_arm_wrist_1_joint"] = deg2rad(-88.38);
	joints[6]["ur5_arm_wrist_2_joint"] = deg2rad(88.60);
	joints[6]["ur5_arm_wrist_3_joint"] = deg2rad(-0.58);

	joints[7]["ur5_arm_shoulder_pan_joint"] = deg2rad(82.23);
	joints[7]["ur5_arm_shoulder_lift_joint"] = deg2rad(-103.39);
	joints[7]["ur5_arm_elbow_joint"] = deg2rad(-87.21);
	joints[7]["ur5_arm_wrist_1_joint"] = deg2rad(-81.64);
	joints[7]["ur5_arm_wrist_2_joint"] = deg2rad(88.60);
	joints[7]["ur5_arm_wrist_3_joint"] = deg2rad(-0.58);

	joints[8]["ur5_arm_shoulder_pan_joint"] = deg2rad(82.08);
	joints[8]["ur5_arm_shoulder_lift_joint"] = deg2rad(-105.61);
	joints[8]["ur5_arm_elbow_joint"] = deg2rad(-92.77);
	joints[8]["ur5_arm_wrist_1_joint"] = deg2rad(-75.75);
	joints[8]["ur5_arm_wrist_2_joint"] = deg2rad(88.60);
	joints[8]["ur5_arm_wrist_3_joint"] = deg2rad(-2.46);

	joints[9]["ur5_arm_shoulder_pan_joint"] = deg2rad(81.66);
	joints[9]["ur5_arm_shoulder_lift_joint"] = deg2rad(-108.72);
	joints[9]["ur5_arm_elbow_joint"] = deg2rad(-102.41);
	joints[9]["ur5_arm_wrist_1_joint"] = deg2rad(-60.59);
	joints[9]["ur5_arm_wrist_2_joint"] = deg2rad(88.60);
	joints[9]["ur5_arm_wrist_3_joint"] = deg2rad(-2.45);

	joints[10]["ur5_arm_shoulder_pan_joint"] = deg2rad(81.69);
	joints[10]["ur5_arm_shoulder_lift_joint"] = deg2rad(-114.81);
	joints[10]["ur5_arm_elbow_joint"] = deg2rad(-104.64);
	joints[10]["ur5_arm_wrist_1_joint"] = deg2rad(-54.22);
	joints[10]["ur5_arm_wrist_2_joint"] = deg2rad(88.60);
	joints[10]["ur5_arm_wrist_3_joint"] = deg2rad(-2.45);

	joints[11]["ur5_arm_shoulder_pan_joint"] = deg2rad(81.54);
	joints[11]["ur5_arm_shoulder_lift_joint"] = deg2rad(-121.68);
	joints[11]["ur5_arm_elbow_joint"] = deg2rad(-109.66);
	joints[11]["ur5_arm_wrist_1_joint"] = deg2rad(-40.28);
	joints[11]["ur5_arm_wrist_2_joint"] = deg2rad(88.60);
	joints[11]["ur5_arm_wrist_3_joint"] = deg2rad(-4.02);
	return joints;
}

std::map<std::string, double> *jointsR8(){
	std::map<std::string, double> *joints = new std::map<std::string, double>[12];
	joints[0]["ur5_arm_shoulder_pan_joint"] = 1.84;
	joints[0]["ur5_arm_shoulder_lift_joint"] = deg2rad(-106.78);
	joints[0]["ur5_arm_elbow_joint"] = -0.04;
	joints[0]["ur5_arm_wrist_1_joint"] = -0.97;
	joints[0]["ur5_arm_wrist_2_joint"] = 1.59;
	joints[0]["ur5_arm_wrist_3_joint"] = 0.09;

	joints[1]["ur5_arm_shoulder_pan_joint"] = 1.75;
	joints[1]["ur5_arm_shoulder_lift_joint"] = -1.95;
	joints[1]["ur5_arm_elbow_joint"] = -0.33;
	joints[1]["ur5_arm_wrist_1_joint"] = -2.42;
	joints[1]["ur5_arm_wrist_2_joint"] = 1.54;
	joints[1]["ur5_arm_wrist_3_joint"] = 0.24;

	joints[2]["ur5_arm_shoulder_pan_joint"] = 1.75;
	joints[2]["ur5_arm_shoulder_lift_joint"] = deg2rad(-94.56);
	joints[2]["ur5_arm_elbow_joint"] = deg2rad(-40.05);
	joints[2]["ur5_arm_wrist_1_joint"] = deg2rad(-110.61);
	joints[2]["ur5_arm_wrist_2_joint"] = deg2rad(88.60);
	joints[2]["ur5_arm_wrist_3_joint"] = deg2rad(-4.01);

	joints[3]["ur5_arm_shoulder_pan_joint"] = deg2rad(117.45);
	joints[3]["ur5_arm_shoulder_lift_joint"] = deg2rad(-94.56);
	joints[3]["ur5_arm_elbow_joint"] = deg2rad(-40.05);
	joints[3]["ur5_arm_wrist_1_joint"] = deg2rad(-110.61);
	joints[3]["ur5_arm_wrist_2_joint"] = deg2rad(88.60);
	joints[3]["ur5_arm_wrist_3_joint"] = deg2rad(-4.01);

	joints[4]["ur5_arm_shoulder_pan_joint"] = deg2rad(112.64);
	joints[4]["ur5_arm_shoulder_lift_joint"] = deg2rad(-98.28);
	joints[4]["ur5_arm_elbow_joint"] = deg2rad(-40.39);
	joints[4]["ur5_arm_wrist_1_joint"] = deg2rad(-115.21);
	joints[4]["ur5_arm_wrist_2_joint"] = deg2rad(88.60);
	joints[4]["ur5_arm_wrist_3_joint"] = deg2rad(19.13);

	joints[5]["ur5_arm_shoulder_pan_joint"] = deg2rad(113.38);
	joints[5]["ur5_arm_shoulder_lift_joint"] = deg2rad(-109.00);
	joints[5]["ur5_arm_elbow_joint"] = deg2rad(-40.39);
	joints[5]["ur5_arm_wrist_1_joint"] = deg2rad(-123.35);
	joints[5]["ur5_arm_wrist_2_joint"] = deg2rad(88.60);
	joints[5]["ur5_arm_wrist_3_joint"] = deg2rad(30.18);

	joints[6]["ur5_arm_shoulder_pan_joint"] = deg2rad(113.37);
	joints[6]["ur5_arm_shoulder_lift_joint"] = deg2rad(-99.55);
	joints[6]["ur5_arm_elbow_joint"] = deg2rad(-72.88);
	joints[6]["ur5_arm_wrist_1_joint"] = deg2rad(-99.77);
	joints[6]["ur5_arm_wrist_2_joint"] = deg2rad(88.14);
	joints[6]["ur5_arm_wrist_3_joint"] = deg2rad(28.94);

	joints[7]["ur5_arm_shoulder_pan_joint"] = deg2rad(113.40);
	joints[7]["ur5_arm_shoulder_lift_joint"] = deg2rad(-99.51);
	joints[7]["ur5_arm_elbow_joint"] = deg2rad(-89.42);
	joints[7]["ur5_arm_wrist_1_joint"] = deg2rad(-81.02);
	joints[7]["ur5_arm_wrist_2_joint"] = deg2rad(88.17);
	joints[7]["ur5_arm_wrist_3_joint"] = deg2rad(28.94);

	joints[8]["ur5_arm_shoulder_pan_joint"] = deg2rad(113.28);
	joints[8]["ur5_arm_shoulder_lift_joint"] = deg2rad(-102.62);
	joints[8]["ur5_arm_elbow_joint"] = deg2rad(-99.67);
	joints[8]["ur5_arm_wrist_1_joint"] = deg2rad(-69.79);
	joints[8]["ur5_arm_wrist_2_joint"] = deg2rad(88.16);
	joints[8]["ur5_arm_wrist_3_joint"] = deg2rad(28.94);

	joints[9]["ur5_arm_shoulder_pan_joint"] = deg2rad(114.78);
	joints[9]["ur5_arm_shoulder_lift_joint"] = deg2rad(-106.77);
	joints[9]["ur5_arm_elbow_joint"] = deg2rad(-102.35);
	joints[9]["ur5_arm_wrist_1_joint"] = deg2rad(-63.46);
	joints[9]["ur5_arm_wrist_2_joint"] = deg2rad(88.16);
	joints[9]["ur5_arm_wrist_3_joint"] = deg2rad(28.94);

	joints[10]["ur5_arm_shoulder_pan_joint"] = deg2rad(115.39);
	joints[10]["ur5_arm_shoulder_lift_joint"] = deg2rad(-113.62);
	joints[10]["ur5_arm_elbow_joint"] = deg2rad(-110.01);
	joints[10]["ur5_arm_wrist_1_joint"] = deg2rad(-47.53);
	joints[10]["ur5_arm_wrist_2_joint"] = deg2rad(88.75);
	joints[10]["ur5_arm_wrist_3_joint"] = deg2rad(28.94);

	joints[11]["ur5_arm_shoulder_pan_joint"] = deg2rad(114.23);
	joints[11]["ur5_arm_shoulder_lift_joint"] = deg2rad(-120.23);
	joints[11]["ur5_arm_elbow_joint"] = deg2rad(-112.72);
	joints[11]["ur5_arm_wrist_1_joint"] = deg2rad(-36.40);
	joints[11]["ur5_arm_wrist_2_joint"] = deg2rad(89.90);
	joints[11]["ur5_arm_wrist_3_joint"] = deg2rad(28.94);
	return joints;
}

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
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

///////////////////////////////////////
//////////////////LEFT/////////////////
///////////////////////////////////////

void blue_left_1_pick(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> *joints = jointsB1();

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  group_arm.setMaxVelocityScalingFactor(1.0);

	for(int r=6;r>=0;r--){
		group_arm.setJointValueTarget(joints[r]);
		group_arm.move();	
	}
	group_arm.setJointValueTarget(initJoints);
	group_arm.move();
	sleep(2.0);

  spinner.stop();
}

void blue_left_1_store(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> *joints = jointsB1();

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  group_arm.setMaxVelocityScalingFactor(1.0);

	group_arm.setJointValueTarget(initJoints);
	group_arm.move();

	for(int r=0;r<=6;r++){
		group_arm.setJointValueTarget(joints[r]);
		group_arm.move();	
	}sleep(2.0);

  spinner.stop();	
}

void blue_left_2_pick(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> *joints = jointsB2();

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  group_arm.setMaxVelocityScalingFactor(1.0);

	for(int r=12;r>=0;r--){
		group_arm.setJointValueTarget(joints[r]);
		group_arm.move();	
	}
	group_arm.setJointValueTarget(initJoints);
	group_arm.move();
	sleep(2.0);

  spinner.stop();
}

void blue_left_2_store(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> *joints = jointsB2();

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  group_arm.setMaxVelocityScalingFactor(1.0);

	group_arm.setJointValueTarget(initJoints);
	group_arm.move();

	for(int r=0;r<=12;r++){
		group_arm.setJointValueTarget(joints[r]);
		group_arm.move();	
	}sleep(2.0);

  spinner.stop();
}

///////////////////////////////////////
/////////////////RIGHT/////////////////
///////////////////////////////////////

void green_right_1_pick(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> *joints = jointsG1();

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  group_arm.setMaxVelocityScalingFactor(1.0);

	for(int r=5;r>=0;r--){
		group_arm.setJointValueTarget(joints[r]);
		group_arm.move();	
	}
	group_arm.setJointValueTarget(initJoints);
	group_arm.move();
	sleep(2.0);

  spinner.stop();
}

void green_right_1_store(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> *joints = jointsG1();

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  group_arm.setMaxVelocityScalingFactor(1.0);

	group_arm.setJointValueTarget(initJoints);
	group_arm.move();

	for(int r=0;r<=5;r++){
		group_arm.setJointValueTarget(joints[r]);
		group_arm.move();	
	}sleep(2.0);

  spinner.stop();
}

void green_right_2_pick(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> *joints = jointsG2();

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  group_arm.setMaxVelocityScalingFactor(1.0);

	for(int r=8;r>=0;r--){
		group_arm.setJointValueTarget(joints[r]);
		group_arm.move();	
	}
	group_arm.setJointValueTarget(initJoints);
	group_arm.move();
	sleep(2.0);

  spinner.stop();
}
void green_right_2_store(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> *joints = jointsG2();

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  group_arm.setMaxVelocityScalingFactor(1.0);

	group_arm.setJointValueTarget(initJoints);
	group_arm.move();

	for(int r=0;r<=8;r++){
		group_arm.setJointValueTarget(joints[r]);
		group_arm.move();	
	}sleep(2.0);

  spinner.stop();
}

void green_right_3_pick(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> *joints = jointsG3();

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  group_arm.setMaxVelocityScalingFactor(1.0);

	for(int r=7;r>=0;r--){
		group_arm.setJointValueTarget(joints[r]);
		group_arm.move();
		if(r==7) sleep(2.0);
	}
	group_arm.setJointValueTarget(initJoints);
	group_arm.move();
	sleep(2.0);

  spinner.stop();
}
void green_right_3_store(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> *joints = jointsG3();

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  group_arm.setMaxVelocityScalingFactor(1.0);

	group_arm.setJointValueTarget(initJoints);
	group_arm.move();

	for(int r=0;r<=5;r++){
		group_arm.setJointValueTarget(joints[r]);
		group_arm.move();	
	}sleep(2.0);

  spinner.stop();
}

void green_right_4_pick(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> *joints = jointsG4();

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  group_arm.setMaxVelocityScalingFactor(1.0);

	for(int r=11;r>=0;r--){
		group_arm.setJointValueTarget(joints[r]);
		group_arm.move();	
	}
	group_arm.setJointValueTarget(initJoints);
	group_arm.move();
	sleep(2.0);

  spinner.stop();
}
void green_right_4_store(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> *joints = jointsG4();

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  group_arm.setMaxVelocityScalingFactor(1.0);

	group_arm.setJointValueTarget(initJoints);
	group_arm.move();

	for(int r=0;r<=11;r++){
		group_arm.setJointValueTarget(joints[r]);
		group_arm.move();	
	}sleep(2.0);

  spinner.stop();
}

void red_right_7_pick(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> *joints = jointsR7();

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  group_arm.setMaxVelocityScalingFactor(1.0);

	for(int r=11;r>=0;r--){
		group_arm.setJointValueTarget(joints[r]);
		group_arm.move();	
	}
	group_arm.setJointValueTarget(initJoints);
	group_arm.move();
	sleep(2.0);

  spinner.stop();
}
void red_right_7_store(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> *joints = jointsR7();

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  group_arm.setMaxVelocityScalingFactor(1.0);

	group_arm.setJointValueTarget(initJoints);
	group_arm.move();

	for(int r=0;r<=11;r++){
		group_arm.setJointValueTarget(joints[r]);
		group_arm.move();	
	}sleep(2.0);

  spinner.stop();
}

void red_right_8_pick(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> *joints = jointsR8();

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  group_arm.setMaxVelocityScalingFactor(1.0);

	for(int r=11;r>=0;r--){
		group_arm.setJointValueTarget(joints[r]);
		group_arm.move();	
	}
	group_arm.setJointValueTarget(initJoints);
	group_arm.move();
	sleep(2.0);

  spinner.stop();
}
void red_right_8_store(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> *joints = jointsR8();

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  group_arm.setMaxVelocityScalingFactor(1.0);

	group_arm.setJointValueTarget(initJoints);
	group_arm.move();

	for(int r=0;r<=11;r++){
		group_arm.setJointValueTarget(joints[r]);
		group_arm.move();	
	}sleep(2.0);

  spinner.stop();
}
///////////////////////////////////////
/////////////////MIDLE/////////////////
///////////////////////////////////////
void red_midle_1_pick(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> *joints = jointsR1();

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  group_arm.setMaxVelocityScalingFactor(1.0);

	for(int r=5;r>=0;r--){
		group_arm.setJointValueTarget(joints[r]);
		group_arm.move();	
	}
	group_arm.setJointValueTarget(initJoints);
	group_arm.move();
	sleep(2.0);

  spinner.stop();
}
void red_midle_1_store(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> *joints = jointsR1();

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  group_arm.setMaxVelocityScalingFactor(1.0);

	group_arm.setJointValueTarget(initJoints);
	group_arm.move();

	for(int r=0;r<=5;r++){
		group_arm.setJointValueTarget(joints[r]);
		group_arm.move();	
	}sleep(2.0);

  spinner.stop();
}

void red_midle_2_pick(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> *joints = jointsR2();

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  group_arm.setMaxVelocityScalingFactor(1.0);

	for(int r=8;r>=0;r--){
		group_arm.setJointValueTarget(joints[r]);
		group_arm.move();	
	}
	group_arm.setJointValueTarget(initJoints);
	group_arm.move();
	sleep(2.0);

  spinner.stop();
}
void red_midle_2_store(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> *joints = jointsR2();

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  group_arm.setMaxVelocityScalingFactor(1.0);

	group_arm.setJointValueTarget(initJoints);
	group_arm.move();

	for(int r=0;r<=8;r++){
		group_arm.setJointValueTarget(joints[r]);
		group_arm.move();	
	}sleep(2.0);

  spinner.stop();
}

void red_midle_3_pick(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> *joints = jointsR3();

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  group_arm.setMaxVelocityScalingFactor(1.0);

	for(int r=6;r>=0;r--){
		group_arm.setJointValueTarget(joints[r]);
		group_arm.move();	
	}
	group_arm.setJointValueTarget(initJoints);
	group_arm.move();
	sleep(2.0);

  spinner.stop();
}
void red_midle_3_store(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> *joints = jointsR3();

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  group_arm.setMaxVelocityScalingFactor(1.0);

	group_arm.setJointValueTarget(initJoints);
	group_arm.move();

	for(int r=0;r<=6;r++){
		group_arm.setJointValueTarget(joints[r]);
		group_arm.move();	
	}sleep(2.0);

  spinner.stop();
}

void red_midle_4_pick(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> *joints = jointsR4();

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  group_arm.setMaxVelocityScalingFactor(1.0);

	for(int r=8;r>=0;r--){
		group_arm.setJointValueTarget(joints[r]);
		group_arm.move();	
	}
	group_arm.setJointValueTarget(initJoints);
	group_arm.move();
	sleep(2.0);

  spinner.stop();
}
void red_midle_4_store(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> *joints = jointsR4();

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  group_arm.setMaxVelocityScalingFactor(1.0);

	group_arm.setJointValueTarget(initJoints);
	group_arm.move();

	for(int r=0;r<=8;r++){
		group_arm.setJointValueTarget(joints[r]);
		group_arm.move();	
	}sleep(2.0);

  spinner.stop();
}

void red_midle_5_pick(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> *joints = jointsR5();

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  group_arm.setMaxVelocityScalingFactor(1.0);

	for(int r=5;r>=0;r--){
		group_arm.setJointValueTarget(joints[r]);
		group_arm.move();	
	}
	group_arm.setJointValueTarget(initJoints);
	group_arm.move();
	sleep(2.0);

  spinner.stop();
}
void red_midle_5_store(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> *joints = jointsR5();

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  group_arm.setMaxVelocityScalingFactor(1.0);

	group_arm.setJointValueTarget(initJoints);
	group_arm.move();

	for(int r=0;r<=5;r++){
		group_arm.setJointValueTarget(joints[r]);
		group_arm.move();	
	}sleep(2.0);

  spinner.stop();
}

void red_midle_6_pick(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> *joints = jointsR6();

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  group_arm.setMaxVelocityScalingFactor(1.0);

	for(int r=8;r>=0;r--){
		group_arm.setJointValueTarget(joints[r]);
		group_arm.move();	
	}
	group_arm.setJointValueTarget(initJoints);
	group_arm.move();
	sleep(2.0);

  spinner.stop();
}
void red_midle_6_store(){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::map<std::string, double> *joints = jointsR6();

  ros::NodeHandle nh;

  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("UR5");
  group_arm.setPlannerId("RRTConnectkConfigDefault");
  group_arm.setMaxVelocityScalingFactor(1.0);

	group_arm.setJointValueTarget(initJoints);
	group_arm.move();

	for(int r=0;r<=8;r++){
		group_arm.setJointValueTarget(joints[r]);
		group_arm.move();	
	}sleep(2.0);

  spinner.stop();
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  initData();
  ros::init(argc, argv, "joint_waypoints");
 	    
	initJoints["ur5_arm_shoulder_pan_joint"] = 3.19;
	initJoints["ur5_arm_shoulder_lift_joint"] = -1.92;
	initJoints["ur5_arm_elbow_joint"] = -0.04;
	initJoints["ur5_arm_wrist_1_joint"] = -2.74;
	initJoints["ur5_arm_wrist_2_joint"] = 1.59;
	initJoints["ur5_arm_wrist_3_joint"] = 0.09;

  int flag = -1;
  *resultX = -1;

  while (ros::ok())
  {
	//spinner.start();	
	ros::spinOnce();
	datX = *resultX;

	if(datX != flag){
		ROS_INFO("Key %d", datX);
		///LEFT///
		if(datX == 2){
			// blue_left_1_pick
			blue_left_1_pick();
			ROS_INFO("blue_left_1_pick");
		}
		else if(datX == 3){
			// blue_left_1_store
			blue_left_1_store();
			ROS_INFO("blue_left_1_store");
		}
		else if(datX == 16){
			// blue_left_2_pick
			blue_left_2_pick();
			ROS_INFO("blue_left_2_pick");
		}
		else if(datX == 17){
			// blue_left_2_store
			blue_left_2_store();
			ROS_INFO("blue_left_2_store");
		}
		else if(datX == 30){
			// blue_left_3_pick
			//blue_left_3_pick();
			ROS_INFO("blue_left_3_pick");
		}
		else if(datX == 31){
			// blue_left_3_store
			//blue_left_3_store();
			ROS_INFO("blue_left_3_store");
		}
		///MIDLE///
		else if(datX == 8){
			// red_midle_1_pick
			red_midle_1_pick();
			ROS_INFO("red_midle_1_pick");
		}
		else if(datX == 9){
			// red_midle_1_store
			red_midle_1_store();
			ROS_INFO("red_midle_1_store");
		}
		else if(datX == 22){
			// red_midle_2_pick
			red_midle_2_pick();
			ROS_INFO("red_midle_2_pick");
		}
		else if(datX == 23){
			// red_midle_2_store
			red_midle_2_store();
			ROS_INFO("red_midle_2_store");
		}
		else if(datX == 6){
			// red_midle_3_pick
			red_midle_3_pick();
			ROS_INFO("red_midle_3_pick");
		}
		else if(datX == 7){
			// red_midle_3_store
			red_midle_3_store();
			ROS_INFO("red_midle_3_store");
		}
		else if(datX == 20){
			// red_midle_4_pick
			red_midle_4_pick();
			ROS_INFO("red_midle_4_pick");
		}
		else if(datX == 21){
			// red_midle_4_store
			red_midle_4_store();
			ROS_INFO("red_midle_4_store");
		}
		else if(datX == 4){
			// red_midle_5_pick
			red_midle_5_pick();
			ROS_INFO("red_midle_5_pick");
		}
		else if(datX == 5){
			// red_midle_5_store
			red_midle_5_store();
			ROS_INFO("red_midle_5_store");
		}
		else if(datX == 18){
			// red_midle_6_pick
			red_midle_6_pick();
			ROS_INFO("red_midle_6_pick");
		}
		else if(datX == 19){
			// red_midle_6_store
			red_midle_6_store();
			ROS_INFO("red_midle_6_store");
		}		
		///RIGHT///
		else if(datX == 12){
			// green_right_1_pick
			green_right_1_pick();
			ROS_INFO("green_right_1_pick");
		}
		else if(datX == 13){
			// green_right_1_store
			green_right_1_store();
			ROS_INFO("green_right_1_store");
		}
		else if(datX == 26){
			// green_right_2_pick
			green_right_2_pick();
			ROS_INFO("green_right_2_pick");
		}
		else if(datX == 27){
			// red_right_2_store
			green_right_2_store();
			ROS_INFO("green_right_2_store");
		}

		else if(datX == 10){
			// green_right_3_pick
			green_right_3_pick();
			ROS_INFO("green_right_3_pick");
		}
		else if(datX == 11){
			// green_right_3_store
			green_right_3_store();
			ROS_INFO("green_right_3_store");
		}
		else if(datX == 24){
			// green_right_4_pick
			green_right_4_pick();
			ROS_INFO("green_right_4_pick");
		}
		else if(datX == 25){
			// green_right_4_store
			green_right_4_store();
			ROS_INFO("green_right_4_store");
		}


		else if(datX == 37){
			// red_right_7_pick
			red_right_7_pick();
			ROS_INFO("red_right_7_pick");
		}
		else if(datX == 38){
			// red_right_7_store
			red_right_7_store();
			ROS_INFO("red_right_7_store");
		}

		else if(datX == 39){
			// red_right_8_pick
			red_right_8_pick();
			ROS_INFO("red_right_8_pick");
		}
		else if(datX == 40){
			// red_right_8_store
			red_right_8_store();
			ROS_INFO("red_right_8_store");
		}
	}

	flag = datX;
  }
  return EXIT_SUCCESS;
}
