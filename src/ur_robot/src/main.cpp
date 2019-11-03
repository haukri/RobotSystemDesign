#include "ros/ros.h"
#include "std_msgs/String.h"
#include "robot_msgs/RobotCommand.h"
#include "robot_msgs/RobotCmd.h"
#include "robot_msgs/RobotStatus.h"
#include "robot_msgs/RobotIO.h"
#include "packml_msgs/Status.h"

#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <iostream>
#include <chrono>
#include <numeric>

#define STOPPING 7
#define STARTING 3

using namespace ur_rtde;
using namespace std;

RTDEReceiveInterface* rtde_receive;
RTDEControlInterface* rtde_control;
DashboardClient* dashboard_client;
RTDEIOInterface* io_interface; 
ros::Publisher robot_status_pub;

// Data to be sent
double velocity = 0.8;
double acceleration = 0.8;

bool robotStopped = false;

std::string robot_ip = "192.168.56.102";

std::vector<double> jointq_pick_red = {-2.12977, -2.51952, -1.13055, -1.05978, 1.59171, -0.0610016 };
std::vector<double> jointq_pick_blue = {-2.14427, -2.25556, -1.62865, -0.848832, 1.59175, -0.0610016 };
std::vector<double> jointq_pick_yellow = {-2.20997, -2.03419, -2.10585, -0.600345, 1.59149, -0.0609778 };
std::vector<double> jointq_pick_midpoint = {-1.75032, -1.10067, -2.09389, -1.52664, 1.59139, -0.0610135};
std::vector<double> jointq_bin1 = {-0.599419, -1.39831, -2.49499, -0.823781, 1.59121, -0.0609296};
std::vector<double> jointq_bin2 = {-0.530233, -1.63006, -2.36819, -0.733082, 1.5788, -0.0610016};
std::vector<double> jointq_bin3 = {-1.19011, -1.7988, -2.32043, -0.567381, 1.57787, -0.0609534};
std::vector<double> jointq_bin4 = {-0.950713, -2.06645, -2.01177, -0.676582, 1.59532, -0.0611699};

std::vector<double> joint_q1 = {-1.12738, -1.59118, -2.1059, -0.96981, 1.59176, 0.032187};
std::vector<double> joint_q2 = {-1.82089, -2.04634, -1.22491, -1.11367, 1.59176, 0.0321989};

std::vector<double> getBinJointQ(int binNumber) {
  switch(binNumber) {
    case 1:
      return jointq_bin1;
    case 2:
      return jointq_bin2;
    case 3:
      return jointq_bin3;
    case 4:
      return jointq_bin4;
    default:
      return jointq_bin1;
  }
}

bool waitForRobot() {
  while(!rtde_control->commandDoneAsync() && !robotStopped) {

  }
  return robotStopped;
}

vector<vector<double>> generatePath(string brick, int binNumber) {
  if(brick == "pick-blue") {
    return {jointq_pick_midpoint, jointq_pick_blue, jointq_pick_midpoint, getBinJointQ(binNumber)};
  }
  else if(brick == "pick-red") {
    return {jointq_pick_midpoint, jointq_pick_red, jointq_pick_midpoint, getBinJointQ(binNumber)};
  }
  else if(brick == "pick-yellow") {
    return {jointq_pick_midpoint, jointq_pick_yellow, jointq_pick_midpoint, getBinJointQ(binNumber)};
  }
  else {
    return {};
  }
}

void move(vector<double> jointq, double velocity, double acceleration) {
  rtde_control->moveJ(jointq, velocity, acceleration);
  if(waitForRobot()) {
    ROS_INFO("Robot stopped, waiting to start again");
    while(robotStopped){ }
    ROS_INFO("Robot starting");
    move(jointq, velocity, acceleration);
  };
}

void robotCommandCallback(const robot_msgs::RobotCmd::ConstPtr& cmd) {
  if(cmd->command == "pick-blue") {
    ROS_INFO("Picking blue brick");
    for(auto jointq : generatePath(cmd->command, cmd->binNumber)) {
      move(jointq, velocity, acceleration);
    }
    robot_msgs::RobotStatus status;
    status.ready = true;
    robot_status_pub.publish(status);
  }
  else if(cmd->command == "pick-red") {
    ROS_INFO("Picking red brick");
    for(auto jointq : generatePath(cmd->command, cmd->binNumber)) {
      move(jointq, velocity, acceleration);
    }
    robot_msgs::RobotStatus status;
    status.ready = true;
    robot_status_pub.publish(status);
  }
  else if(cmd->command == "pick-yellow") {
    ROS_INFO("Picking yellow brick");
    for(auto jointq : generatePath(cmd->command, cmd->binNumber)) {
      move(jointq, velocity, acceleration);
    }
    robot_msgs::RobotStatus status;
    status.ready = true;
    robot_status_pub.publish(status);
  }
}

bool robot_command(robot_msgs::RobotCommand::Request  &req, robot_msgs::RobotCommand::Response &res)
{
  if(req.command == "pick-blue") {
    ROS_INFO("Picking blue brick");

    rtde_control->moveJ(jointq_pick_midpoint, velocity, acceleration);
    rtde_control->moveJ(jointq_pick_blue, velocity, acceleration);
    rtde_control->moveJ(jointq_pick_midpoint, velocity, acceleration);
    rtde_control->moveJ(getBinJointQ(req.binNumber), velocity, acceleration);
    
  }
  else if(req.command == "pick-red") {
    ROS_INFO("Picking red brick");

    rtde_control->moveJ(jointq_pick_midpoint, velocity, acceleration);
    rtde_control->moveJ(jointq_pick_red, velocity, acceleration);
    rtde_control->moveJ(jointq_pick_midpoint, velocity, acceleration);
    rtde_control->moveJ(getBinJointQ(req.binNumber), velocity, acceleration);
  }
  else if(req.command == "pick-yellow") {
    ROS_INFO("Picking yellow brick");

    rtde_control->moveJ(jointq_pick_midpoint, velocity, acceleration);
    rtde_control->moveJ(jointq_pick_yellow, velocity, acceleration);
    rtde_control->moveJ(jointq_pick_midpoint, velocity, acceleration);
    rtde_control->moveJ(getBinJointQ(req.binNumber), velocity, acceleration);
  }

  res.success = true;
  return true;
}

void stopCallback(const packml_msgs::Status::ConstPtr& msg)
{
  if(msg->state.val == STOPPING) {
    ROS_INFO("Stopping Robot");
    rtde_control->reuploadScript();
    ROS_INFO("Reconnected Robot");
    robotStopped = true;
  }
  if(msg->state.val == STARTING) {
    ROS_INFO("Starting Robot");
    robotStopped = false;
  }
}

void ioCallback(const robot_msgs::RobotIO::ConstPtr& msg)
{
  // ROS_INFO("Got io command");
  io_interface->setStandardDigitalOut(msg->output, msg->level);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ur_robot");

  ros::NodeHandle n;

  // ros::ServiceServer service = n.advertiseService("robot_command", robot_command);

  ros::Subscriber sub = n.subscribe("packml_node/packml/status", 10, stopCallback);
  ros::Subscriber subRobotCmd = n.subscribe("robot_command_new", 10, robotCommandCallback);
  robot_status_pub = n.advertise<robot_msgs::RobotStatus>("robot_command_status", 100);

  ros::Subscriber io_sub = n.subscribe("robot_io", 10, ioCallback);

  rtde_control = new RTDEControlInterface(robot_ip);
  dashboard_client = new DashboardClient(robot_ip);
  dashboard_client->connect();

  io_interface = new RTDEIOInterface(robot_ip);

  ros::MultiThreadedSpinner s(2);

  ros::spin(s);

  rtde_control->stopRobot();

  return 0;
}