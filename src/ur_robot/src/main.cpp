#include "ros/ros.h"
#include "std_msgs/String.h"
#include "robot_msgs/RobotCommand.h"

#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface.h>
#include <iostream>
#include <chrono>
#include <numeric>

using namespace ur_rtde;

RTDEReceiveInterface* rtde_receive;
RTDEControlInterface* rtde_control;

// Data to be sent
double velocity = 0.5;
double acceleration = 0.5;

bool robot_command(robot_msgs::RobotCommand::Request  &req, robot_msgs::RobotCommand::Response &res)
{
  if(req.command == "pick-blue") {
    ROS_INFO("Picking blue brick");

    std::vector<double> joint_q1 = {-1.12738, -1.59118, -2.1059, -0.96981, 1.59176, 0.032187};
    std::vector<double> joint_q2 = {-1.82089, -2.04634, -1.22491, -1.11367, 1.59176, 0.0321989};

    rtde_control->moveJ(joint_q1, velocity, acceleration);
    rtde_control->moveJ(joint_q2, velocity, acceleration);
  }
  else if(req.command == "pick-red") {
    ROS_INFO("Picking red brick");

    std::vector<double> joint_q1 = {-1.12738, -1.59118, -2.1059, -0.96981, 1.59176, 0.032187};
    std::vector<double> joint_q2 = {-1.82089, -2.04634, -1.22491, -1.11367, 1.59176, 0.0321989};

    rtde_control->moveJ(joint_q1, velocity, acceleration);
    rtde_control->moveJ(joint_q2, velocity, acceleration);
  }
  else if(req.command == "pick-yellow") {
    ROS_INFO("Picking yellow brick");

    std::vector<double> joint_q1 = {-1.12738, -1.59118, -2.1059, -0.96981, 1.59176, 0.032187};
    std::vector<double> joint_q2 = {-1.82089, -2.04634, -1.22491, -1.11367, 1.59176, 0.0321989};

    rtde_control->moveJ(joint_q1, velocity, acceleration);
    rtde_control->moveJ(joint_q2, velocity, acceleration);
  }

  res.success = true;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ur_robot");

  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("robot_command", robot_command);

  rtde_control = new RTDEControlInterface("192.168.56.102");

  ros::spin();

  rtde_control->stopRobot();

  return 0;
}