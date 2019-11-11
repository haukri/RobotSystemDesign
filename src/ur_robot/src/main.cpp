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
#define HOLDING 10
#define SUSPENDING 101
#define UNSUSPENDING 102
#define UNHOLDING 104

using namespace ur_rtde;
using namespace std;

RTDEReceiveInterface* rtde_receive;
RTDEControlInterface* rtde_control;
DashboardClient* dashboard_client;
RTDEIOInterface* io_interface; 
ros::Publisher robot_status_pub;

// Data to be sent
double velocity = 0.8;
double velocity_high = 2;
double velocity_low = 0.8;
double acceleration = 0.8;

bool robotStopped = false;
bool robotPaused = false;

std::string robot_ip = "192.168.1.10";

std::vector<double> jointq_pick_red = {-1.76932, -2.24, -2.37864, 0.610792, 1.44958, 0.167349};
std::vector<double> jointq_pick_red_over = {-1.72418, -2.1051, -2.53606, 0.642339, 1.39204, 0.167397 };

std::vector<double> jointq_pick_blue = {-1.93548, -2.22668, -2.33629, 0.532674, 1.56827, 0.00825453};
std::vector<double> jointq_pick_blue_over = {-1.92394, -1.99632, -2.45521, 0.401373, 1.54834, 0.00825453};

std::vector<double> jointq_pick_yellow = {-1.59469, -2.24166, -2.39459, 0.608003, 1.32264, 0.291706 };
std::vector<double> jointq_pick_yellow_over = {-1.55849, -2.1221, -2.53371, 0.652527, 1.30927, 0.291958};

std::vector<double> jointq_pick_midpoint = {-1.30838, -1.17342, -2.32572, -1.19639, 1.57305, 0.623872};

std::vector<double> jointq_bin1 = {-1.01448, -1.72352, -2.21327, -0.788078, 1.56507, -0.655195};
std::vector<double> jointq_bin2 = {-0.833121, -1.59163, -2.36416, -0.766303, 1.55147, -0.473551};
std::vector<double> jointq_bin3 = {-1.22998, -1.56485, -2.39417, -0.755475, 1.54941, -0.870467};
std::vector<double> jointq_bin4 = {-1.06105, -1.33726, -2.57864, -0.801904, 1.55033, -0.701143};

std::vector<double> joint_q1 = {-1.12738, -1.59118, -2.1059, -0.96981, 1.59176, 0.032187};
std::vector<double> joint_q2 = {-1.82089, -2.04634, -1.22491, -1.11367, 1.59176, 0.0321989};

std::vector<double> joint_verify_feeder = {-1.78183, -1.62972, -2.13145, -1.78324, 1.72335, 0.0565257};
std::vector<double> jointq_discard_bin = {-2.17897, -1.86781, -2.01717, -0.829591, 1.64373, -0.175439};

std::vector<double> jointq_pick_bins_12 = {-0.959291, -1.84798, -2.24979, -0.642775, 1.56402, -0.565669};
std::vector<double> jointq_pick_bins_12_over = {-0.959411, -1.76864, -2.22276, -0.749148, 1.5644, -0.565538};

std::vector<double> jointq_dropoff_bins_12 = {0.112473, -1.6066, -2.53273, -0.591482, 1.58511, -1.11164};
std::vector<double> jointq_dropoff_bins_12_over = {0.112257, -1.48768, -2.48922, -0.753941, 1.58568, -1.11153};

std::vector<double> jointq_pick_bins_34 = {-1.15152, -1.70928, -2.43062, -0.599233, 1.55877, -0.757671};
std::vector<double> jointq_pick_bins_34_over = {-1.15169, -1.63318, -2.40567, -0.70023, 1.5592, -0.757599};

std::vector<double> jointq_dropoff_bins_34 = {-0.204672, -1.67169, -2.54603, -0.510077, 1.54498, -1.40419};
std::vector<double> jointq_dropoff_bins_34_over = {-0.204995, -1.51274, -2.49631, -0.718729, 1.54574, -1.40397};

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

void waitForRobot() {
  while(!rtde_control->commandDoneAsync() && !robotStopped && !robotPaused) { }
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
  waitForRobot();
  if(robotPaused) {
    ROS_INFO("Robot paused, waiting to start again");
    while(robotPaused){ }
    ROS_INFO("Robot starting");
    move(jointq, velocity, acceleration);
  }
}

void robotCommandCallback(const robot_msgs::RobotCmd::ConstPtr& cmd) {
  // io_interface->reconnect();
  if(cmd->command == "pick-blue") {
    ROS_INFO("Picking blue brick");
    
    io_interface->setStandardDigitalOut(4, 1);
    
    move(jointq_pick_midpoint, velocity_high, acceleration);
    if(robotStopped)
      return;
    move(jointq_pick_blue_over, velocity_high, acceleration);
    if(robotStopped)
      return;
    move(jointq_pick_blue, velocity_low, acceleration);
    if(robotStopped)
      return;
    
    io_interface->setStandardDigitalOut(4, 0);
    
    move(jointq_pick_blue_over, velocity_low, acceleration);
    if(robotStopped)
      return;
    move(jointq_pick_midpoint, velocity_high, acceleration);
    if(robotStopped)
      return;
    move(getBinJointQ(cmd->binNumber), velocity_high, acceleration);
    if(robotStopped)
      return;
    
    io_interface->setStandardDigitalOut(4, 1);
    
    robot_msgs::RobotStatus status;
    status.ready = true;
    robot_status_pub.publish(status);
  }
  else if(cmd->command == "pick-red") {
    ROS_INFO("Picking red brick");
    
    io_interface->setStandardDigitalOut(4, 1);
    
    move(jointq_pick_midpoint, velocity_high, acceleration);
    if(robotStopped)
      return;
    move(jointq_pick_red_over, velocity_high, acceleration);
    if(robotStopped)
      return;
    move(jointq_pick_red, velocity_low, acceleration);
    if(robotStopped)
      return;
    
    io_interface->setStandardDigitalOut(4, 0);
    
    move(jointq_pick_red_over, velocity_low, acceleration);
    if(robotStopped)
      return;
    move(jointq_pick_midpoint, velocity_high, acceleration);
    if(robotStopped)
      return;
    move(getBinJointQ(cmd->binNumber), velocity_high, acceleration);
    if(robotStopped)
      return;
    
    io_interface->setStandardDigitalOut(4, 1);
    
    robot_msgs::RobotStatus status;
    status.ready = true;
    robot_status_pub.publish(status);
  }
  else if(cmd->command == "pick-yellow") {
    ROS_INFO("Picking yellow brick");
    
    io_interface->setStandardDigitalOut(4, 1);
    
    move(jointq_pick_midpoint, velocity_high, acceleration);
    if(robotStopped)
      return;
    move(jointq_pick_yellow_over, velocity_high, acceleration);
    if(robotStopped)
      return;
    move(jointq_pick_yellow, velocity_low, acceleration);
    if(robotStopped)
      return;
    
    io_interface->setStandardDigitalOut(4, 0);
    
    move(jointq_pick_yellow_over, velocity_low, acceleration);
    if(robotStopped)
      return;
    move(jointq_pick_midpoint, velocity_high, acceleration);
    if(robotStopped)
      return;
    move(getBinJointQ(cmd->binNumber), velocity_high, acceleration);
    if(robotStopped)
      return;
    
    io_interface->setStandardDigitalOut(4, 1);
    
    robot_msgs::RobotStatus status;
    status.ready = true;
    robot_status_pub.publish(status);
  }
    else if(cmd->command == "verify-bricks") {
    ROS_INFO("Verifying Bricks");
    move(jointq_pick_midpoint, velocity_high, acceleration);
    if(robotStopped)
      return;
    move(joint_verify_feeder, velocity_high, acceleration);
    if(robotStopped)
      return;
    robot_msgs::RobotStatus status;
    status.ready = true;
    robot_status_pub.publish(status);
  }
  else if(cmd->command == "discard-yellow") {
    ROS_INFO("Discarding yellow brick");
    
    io_interface->setStandardDigitalOut(4, 1);
    
    move(jointq_pick_midpoint, velocity_high, acceleration);
    if(robotStopped)
      return;
    move(jointq_pick_yellow_over, velocity_high, acceleration);
    if(robotStopped)
      return;
    move(jointq_pick_yellow, velocity_low, acceleration);
    if(robotStopped)
      return;
    
    io_interface->setStandardDigitalOut(4, 0);
    
    move(jointq_pick_yellow_over, velocity_low, acceleration);
    if(robotStopped)
      return;
    move(jointq_pick_midpoint, velocity_high, acceleration);
    if(robotStopped)
      return;
    move(jointq_discard_bin, velocity_high, acceleration);
    if(robotStopped)
      return;
    
    io_interface->setStandardDigitalOut(4, 1);
    
    robot_msgs::RobotStatus status;
    status.ready = true;
    robot_status_pub.publish(status);
  }
  else if(cmd->command == "discard-blue") {
    ROS_INFO("Discarding blue brick");
    
    io_interface->setStandardDigitalOut(4, 1);
    
    move(jointq_pick_midpoint, velocity_high, acceleration);
    if(robotStopped)
      return;
    move(jointq_pick_blue_over, velocity_high, acceleration);
    if(robotStopped)
      return;
    move(jointq_pick_blue, velocity_low, acceleration);
    if(robotStopped)
      return;
    
    io_interface->setStandardDigitalOut(4, 0);
    
    move(jointq_pick_blue_over, velocity_low, acceleration);
    if(robotStopped)
      return;
    move(jointq_pick_midpoint, velocity_high, acceleration);
    if(robotStopped)
      return;
    move(jointq_discard_bin, velocity_high, acceleration);
    if(robotStopped)
      return;
    
    io_interface->setStandardDigitalOut(4, 1);
    
    robot_msgs::RobotStatus status;
    status.ready = true;
    robot_status_pub.publish(status);
  }
  else if(cmd->command == "discard-red") {
    ROS_INFO("Discarding red brick");
    
    io_interface->setStandardDigitalOut(4, 1);
    
    move(jointq_pick_midpoint, velocity_high, acceleration);
    if(robotStopped)
      return;
    move(jointq_pick_red_over, velocity_high, acceleration);
    if(robotStopped)
      return;
    move(jointq_pick_red, velocity_low, acceleration);
    if(robotStopped)
      return;
    
    io_interface->setStandardDigitalOut(4, 0);
    
    move(jointq_pick_red_over, velocity_low, acceleration);
    if(robotStopped)
      return;
    move(jointq_pick_midpoint, velocity_high, acceleration);
    if(robotStopped)
      return;
    move(jointq_discard_bin, velocity_high, acceleration);
    if(robotStopped)
      return;
    
    io_interface->setStandardDigitalOut(4, 1);
    
    robot_msgs::RobotStatus status;
    status.ready = true;
    robot_status_pub.publish(status);
  }
  else if(cmd->command == "dropoff-boxes") {
    ROS_INFO("Dropping off boxes");
    
    io_interface->setStandardDigitalOut(4, 1);
    
    move(jointq_pick_midpoint, velocity_high, acceleration);
    if(robotStopped)
      return;
    move(jointq_pick_bins_12_over, velocity_high, acceleration);
    if(robotStopped)
      return;
    move(jointq_pick_bins_12, velocity_low, acceleration);
    if(robotStopped)
      return;
    
    io_interface->setStandardDigitalOut(4, 0);
    
    move(jointq_pick_bins_12_over, velocity_low, acceleration);
    if(robotStopped)
      return;
    move(jointq_dropoff_bins_12_over, velocity_high, acceleration);
    if(robotStopped)
      return;
    move(jointq_dropoff_bins_12, velocity_low, acceleration);
    if(robotStopped)
      return;
    
    io_interface->setStandardDigitalOut(4, 1);

    move(jointq_dropoff_bins_12_over, velocity_low, acceleration);
    if(robotStopped)
      return;
    move(jointq_pick_midpoint, velocity_high, acceleration);
    if(robotStopped)
      return;
    move(jointq_pick_bins_34_over, velocity_high, acceleration);
    if(robotStopped)
      return;
    move(jointq_pick_bins_34, velocity_low, acceleration);
    if(robotStopped)
      return;
    
    io_interface->setStandardDigitalOut(4, 0);
    
    move(jointq_pick_bins_34_over, velocity_low, acceleration);
    if(robotStopped)
      return;
    move(jointq_dropoff_bins_34_over, velocity_high, acceleration);
    if(robotStopped)
      return;
    move(jointq_dropoff_bins_34, velocity_high, acceleration);
    if(robotStopped)
      return;
    
    io_interface->setStandardDigitalOut(4, 1);

    move(jointq_dropoff_bins_34_over, velocity_low, acceleration);
    if(robotStopped)
      return;

    robot_msgs::RobotStatus status;
    status.ready = true;
    robot_status_pub.publish(status);
  }
}

void stopCallback(const packml_msgs::Status::ConstPtr& msg)
{
  if(msg->state.val == STOPPING) {
    ROS_INFO("Stopping Robot");
    rtde_control->reuploadScript();
    ROS_INFO("Reconnected Robot");
    robotStopped = true;
  }
  else if(msg->state.val == HOLDING || msg->state.val == SUSPENDING) {
    ROS_INFO("Pausing Robot");
    rtde_control->reuploadScript();
    ROS_INFO("Reconnected Robot");
    robotPaused = true;
  }
  else if(msg->state.val == STARTING) {
    ROS_INFO("Starting Robot from stopped");
    robotStopped = false;

  }
  else if(msg->state.val == UNHOLDING || msg->state.val == UNSUSPENDING) {
    ROS_INFO("Starting Robot from paused");
    robotPaused = false;
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