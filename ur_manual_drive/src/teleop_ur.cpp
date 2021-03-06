/*
 * Modified from teleop_ur authored by Kevin Watts, Willow Garage, Inc.
 * Modified by @author Abdur Rosyid
 * to be used with Logitech Gamepad
 * The manual commands have two mode: normal and fast.
 */

//Converts joystick commands on /joy to commands to UR arm

#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

//#define ARM_TOPIC "arm_controller/command"
#define ARM_TOPIC "ur_driver/joint_speed"

const int PUBLISH_FREQ = 20;

using namespace std;

class TeleopUR
{
   public:
  //joy::Joy joy;
  double req_pan, req_lift, req_elbow, req_wrist_1, req_wrist_2, req_wrist_3;
  double req_pan_vel, req_lift_vel, req_elbow_vel, req_wrist_1_vel, req_wrist_2_vel, req_wrist_3_vel;
  double max_pan, min_pan, max_lift, min_lift, max_elbow, min_elbow, max_wrist_1, min_wrist_1, max_wrist_2, min_wrist_2, max_wrist_3, min_wrist_3;
  double pan_step, lift_step, elbow_step, wrist_1_step, wrist_2_step, wrist_3_step;
  double pan_step_fast, lift_step_fast, elbow_step_fast, wrist_1_step_fast, wrist_2_step_fast, wrist_3_step_fast;
  int axis_pan, axis_lift, axis_elbow, axis_wrist_1, axis_wrist_2, axis_wrist_3;
  int fast_button;
  bool arm_publish_;
  bool fast_;

  std::string last_selected_topic_;

  sensor_msgs::Joy last_processed_joy_message_;

  ros::Time last_recieved_joy_message_time_;
  ros::Duration joy_msg_timeout_;

  ros::NodeHandle n_, n_private_;
  ros::Publisher arm_pub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber arm_state_sub_;

  TeleopUR() :
    max_pan(3.14), min_pan(-3.14),
    max_lift(3.14), min_lift(-3.14),
    max_elbow(3.14), min_elbow(-3.14),
    max_wrist_1(3.14), min_wrist_1(-3.14),
    max_wrist_2(3.14), min_wrist_2(-3.14),
    max_wrist_3(3.14), min_wrist_3(-3.14),
    pan_step(0.02), lift_step(0.02), elbow_step(0.02), wrist_1_step(0.02), wrist_2_step(0.02), wrist_3_step(0.02),
    pan_step_fast(0.08), lift_step_fast(0.08), elbow_step_fast(0.08), wrist_1_step_fast(0.08), wrist_2_step_fast(0.08), wrist_3_step_fast(0.08),
    arm_publish_(false),
    n_private_("~")
  { }

  void init()
  {
        req_pan = req_lift = req_elbow = req_wrist_1 = req_wrist_2 = req_wrist_3 = 0;

        // Arm parameters
        n_private_.param("max_pan", max_pan, max_pan);
        n_private_.param("min_pan", min_pan, min_pan);
        n_private_.param("max_lift", max_lift, max_lift);
        n_private_.param("min_lift", min_lift, min_lift);
        n_private_.param("max_elbow", max_elbow, max_elbow);
        n_private_.param("min_elbow", min_elbow, min_elbow);
        n_private_.param("max_wrist_1", max_wrist_1, max_wrist_1);
        n_private_.param("min_wrist_1", min_wrist_1, min_wrist_1);
        n_private_.param("max_wrist_2", max_wrist_2, max_wrist_2);
        n_private_.param("min_wrist_2", min_wrist_2, min_wrist_2);
        n_private_.param("max_wrist_3", max_wrist_3, max_wrist_3);
        n_private_.param("min_wrist_3", min_wrist_3, min_wrist_3);

        n_private_.param("pan_step", pan_step, pan_step);
        n_private_.param("lift_step", lift_step, lift_step);
        n_private_.param("elbow_step", elbow_step, elbow_step);
        n_private_.param("wrist_1_step", wrist_1_step, wrist_1_step);
        n_private_.param("wrist_2_step", wrist_2_step, wrist_2_step);
        n_private_.param("wrist_3_step", wrist_3_step, wrist_3_step);

        n_private_.param("pan_step_fast", pan_step_fast, pan_step_fast);
        n_private_.param("lift_step_fast", lift_step_fast, lift_step_fast);
        n_private_.param("elbow_step_fast", elbow_step_fast, elbow_step_fast);
        n_private_.param("wrist_1_step_fast", wrist_1_step_fast, wrist_1_step_fast);
        n_private_.param("wrist_2_step_fast", wrist_2_step_fast, wrist_2_step_fast);
        n_private_.param("wrist_3_step_fast", wrist_3_step_fast, wrist_3_step_fast);

        n_private_.param("axis_pan", axis_pan, 0);
        n_private_.param("axis_lift", axis_lift, 1);
        n_private_.param("axis_elbow", axis_elbow, 3);
        n_private_.param("axis_wrist_1", axis_wrist_1, 5);
        n_private_.param("axis_wrist_2", axis_wrist_2, 2);
        n_private_.param("axis_wrist_3", axis_wrist_3, 4);

        n_private_.param("fast_button", fast_button, 4);

	double joy_msg_timeout;
        n_private_.param("joy_msg_timeout", joy_msg_timeout, 0.5); //default to 0.5 seconds timeout
	if (joy_msg_timeout <= 0)
	  {
	    joy_msg_timeout_ = ros::Duration().fromSec(9999999);//DURATION_MAX;
	    ROS_DEBUG("joy_msg_timeout <= 0 -> no timeout");
	  }
	else
	  {
	    joy_msg_timeout_.fromSec(joy_msg_timeout);
	    ROS_DEBUG("joy_msg_timeout: %.3f", joy_msg_timeout_.toSec());
	  }

        ROS_DEBUG("pan step: %.3f rad\n", pan_step);
        ROS_DEBUG("lift step: %.3f rad\n", lift_step);
        ROS_DEBUG("elbow step: %.3f rad\n", elbow_step);
        ROS_DEBUG("wrist_1 step: %.3f rad\n", wrist_1_step);
        ROS_DEBUG("wrist_2 step: %.3f rad\n", wrist_2_step);
        ROS_DEBUG("wrist_3 step: %.3f rad\n", wrist_3_step);

        ROS_DEBUG("axis_pan: %d\n", axis_pan);
        ROS_DEBUG("axis_lift: %d\n", axis_lift);
        ROS_DEBUG("axis_elbow: %d\n", axis_elbow);
        ROS_DEBUG("axis_wrist_1: %d\n", axis_wrist_1);
        ROS_DEBUG("axis_wrist_2: %d\n", axis_wrist_2);
        ROS_DEBUG("axis_wrist_3: %d\n", axis_wrist_3);

        ROS_DEBUG("fast_button: %d\n", fast_button);
        ROS_DEBUG("joy_msg_timeout: %f\n", joy_msg_timeout);

        arm_pub_ = n_.advertise<trajectory_msgs::JointTrajectory>(ARM_TOPIC, 1);
        arm_publish_ = true;

        joy_sub_ = n_.subscribe("joy", 10, &TeleopUR::joy_cb, this);
        arm_state_sub_ = n_.subscribe("joint_states", 1, &TeleopUR::armCB, this);

      }

  ~TeleopUR() { }

  /** Callback for joy topic **/
  void joy_cb(const sensor_msgs::Joy::ConstPtr& joy_msg)
  {
    // Do not process the same message twice.
    if(joy_msg->header.stamp == last_processed_joy_message_.header.stamp) {
        // notify the user only if the problem persists
        if(ros::Time::now() - joy_msg->header.stamp > ros::Duration(5.0/PUBLISH_FREQ))
            ROS_WARN_THROTTLE(1.0, "Received Joy message with same timestamp multiple times. Ignoring subsequent messages.");
        return;
    }
    last_processed_joy_message_ = *joy_msg;

    //Record this message reciept
    last_recieved_joy_message_time_ = ros::Time::now();

    fast_ = (((unsigned int)fast_button < joy_msg->buttons.size()) && joy_msg->buttons[fast_button]);

    // Arm
    // Update commanded position by how joysticks moving

    if (fast_)
    {
      if (axis_pan >= 0 && axis_pan < (int)joy_msg->axes.size())
      {
        req_pan_vel = joy_msg->axes[axis_pan] * pan_step_fast;
      }

      if (axis_lift >= 0 && axis_lift < (int)joy_msg->axes.size())
      {
        req_lift_vel = joy_msg->axes[axis_lift] * lift_step_fast;
      }

      if (axis_elbow >= 0 && axis_elbow < (int)joy_msg->axes.size())
      {
        req_elbow_vel = joy_msg->axes[axis_elbow] * elbow_step_fast;
      }

      if (axis_wrist_1 >= 0 && axis_wrist_1 < (int)joy_msg->axes.size())
      {
        req_wrist_1_vel = joy_msg->axes[axis_wrist_1] * wrist_1_step_fast;
      }

      if (axis_wrist_2 >= 0 && axis_wrist_2 < (int)joy_msg->axes.size())
      {
        req_wrist_2_vel = joy_msg->axes[axis_wrist_2] * wrist_2_step_fast;
      }

      if (axis_wrist_3 >= 0 && axis_wrist_3 < (int)joy_msg->axes.size())
      {
        req_wrist_3_vel = joy_msg->axes[axis_wrist_3] * wrist_3_step_fast;
      }
    }
    else
    {
      if (axis_pan >= 0 && axis_pan < (int)joy_msg->axes.size())
      {
        req_pan_vel = joy_msg->axes[axis_pan] * pan_step;
      }

      if (axis_lift >= 0 && axis_lift < (int)joy_msg->axes.size())
      {
        req_lift_vel = joy_msg->axes[axis_lift] * lift_step;
      }

      if (axis_elbow >= 0 && axis_elbow < (int)joy_msg->axes.size())
      {
        req_elbow_vel = joy_msg->axes[axis_elbow] * elbow_step;
      }

      if (axis_wrist_1 >= 0 && axis_wrist_1 < (int)joy_msg->axes.size())
      {
        req_wrist_1_vel = joy_msg->axes[axis_wrist_1] * wrist_1_step;
      }

      if (axis_wrist_2 >= 0 && axis_wrist_2 < (int)joy_msg->axes.size())
      {
        req_wrist_2_vel = joy_msg->axes[axis_wrist_2] * wrist_2_step;
      }

      if (axis_wrist_3 >= 0 && axis_wrist_3 < (int)joy_msg->axes.size())
      {
        req_wrist_3_vel = joy_msg->axes[axis_wrist_3] * wrist_3_step;
      }
    }

  }


  void send_cmd_vel()
  {
    if (last_recieved_joy_message_time_ + joy_msg_timeout_ > ros::Time::now())
    {
        double dt = 1.0/double(PUBLISH_FREQ);
        double horizon = 3.0 * dt;

        trajectory_msgs::JointTrajectory traj;
        traj.header.stamp = ros::Time::now() + ros::Duration(0.01);
        traj.joint_names.push_back("ur5_arm_shoulder_pan_joint");
        traj.joint_names.push_back("ur5_arm_shoulder_lift_joint");
        traj.joint_names.push_back("ur5_arm_elbow_joint");
        traj.joint_names.push_back("ur5_arm_wrist_1_joint");
        traj.joint_names.push_back("ur5_arm_wrist_2_joint");
        traj.joint_names.push_back("ur5_arm_wrist_3_joint");
        traj.points.resize(1);
        traj.points[0].positions.push_back(req_pan + req_pan_vel * horizon);
        traj.points[0].velocities.push_back(req_pan_vel);
        traj.points[0].positions.push_back(req_lift + req_lift_vel * horizon);
        traj.points[0].velocities.push_back(req_lift_vel);
        traj.points[0].positions.push_back(req_elbow + req_elbow_vel * horizon);
        traj.points[0].velocities.push_back(req_elbow_vel);
        traj.points[0].positions.push_back(req_wrist_1 + req_wrist_1_vel * horizon);
        traj.points[0].velocities.push_back(req_wrist_1_vel);
        traj.points[0].positions.push_back(req_wrist_2 + req_wrist_2_vel * horizon);
        traj.points[0].velocities.push_back(req_wrist_2_vel);
        traj.points[0].positions.push_back(req_wrist_3 + req_wrist_3_vel * horizon);
        traj.points[0].velocities.push_back(req_wrist_3_vel);
        traj.points[0].time_from_start = ros::Duration(horizon);
        arm_pub_.publish(traj);
    }
  }

  void armCB(const sensor_msgs::JointState::ConstPtr &msg)
  {
    // Updates the current positions
    req_pan = msg->position[0];
    req_pan = max(min(req_pan, max_pan), min_pan);
    req_lift = msg->position[1];
    req_lift = max(min(req_lift, max_lift), min_lift);
    req_elbow = msg->position[2];
    req_elbow = max(min(req_elbow, max_elbow), min_elbow);
    req_wrist_1 = msg->position[3];
    req_wrist_1 = max(min(req_wrist_1, max_wrist_1), min_wrist_1);
    req_wrist_2 = msg->position[4];
    req_wrist_2 = max(min(req_wrist_2, max_wrist_2), min_wrist_2);
    req_wrist_3 = msg->position[5];
    req_wrist_3 = max(min(req_wrist_3, max_wrist_3), min_wrist_3);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "teleop_ur");

  TeleopUR teleop_ur;
  teleop_ur.init();

  ros::Rate pub_rate(PUBLISH_FREQ);

  while (teleop_ur.n_.ok())
  {
    ros::spinOnce();
    teleop_ur.send_cmd_vel();
    pub_rate.sleep();
  }

  exit(0);
  return 0;
}
