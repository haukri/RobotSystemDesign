#include "ros/ros.h"
#include "std_msgs/String.h"
#include "robot_msgs/RobotCommand.h"
#include "robot_msgs/RobotCmd.h"
#include "robot_msgs/RobotStatus.h"
#include "robot_msgs/RobotIO.h"
#include "packml_msgs/Status.h"
#include "packml_msgs/Transition.h"

#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <iostream>
#include <chrono>
#include <numeric>

#define STOPPING 7
#define ABORTING 8
#define STARTING 3
#define HOLDING 10
#define SUSPENDED 5
#define SUSPENDING 101
#define UNSUSPENDING 102
#define UNHOLDING 104

#define START 2
#define STOP 3
#define RESET 6

using namespace ur_rtde;
using namespace std;

RTDEReceiveInterface* rtde_receive;
RTDEControlInterface* rtde_control;
DashboardClient* dashboard_client;
RTDEIOInterface* io_interface; 
ros::Publisher robot_status_pub;

// Data to be sent
double joint_velocity_extreme = 3;
double joint_velocity_high = 2;
double joint_velocity_low = 0.5;
double joint_acceleration_low = 2;
double joint_acceleration_high = 10;
double pose_velocity_low = 0.3;
double pose_velocity_high = 2;
double pose_acceleration_low = 2;
double pose_acceleration_high = 5;

const double gripping_time = 0.2;

bool robotStopped = false;
bool robotPaused = false;

int packMLState = 0;

const double theta = -0.39;

std::string robot_ip = "192.168.1.10";

std::vector<double> jointq_pick_red = {-1.76932, -2.24, -2.37864, 0.610792, 1.44958, 0.167349};
std::vector<double> jointq_pick_red_over = {-1.72418, -2.1051, -2.53606, 0.642339, 1.39204, 0.167397 };

std::vector<double> jointq_pick_blue = {-1.93548, -2.22668, -2.33629, 0.532674, 1.56827, 0.00825453};
std::vector<double> jointq_pick_blue_over = {-1.92394, -1.99632, -2.45521, 0.401373, 1.54834, 0.00825453};

std::vector<double> jointq_pick_yellow = {-1.59469, -2.24166, -2.39459, 0.608003, 1.32264, 0.291706 };
std::vector<double> jointq_pick_yellow_over = {-1.55849, -2.1221, -2.53371, 0.652527, 1.30927, 0.291958};

std::vector<double> jointq_pick_midpoint = {-1.30838, -1.17342, -2.32572, -1.19639, 1.57305, 0.623872};
std::vector<double> jointq_feeder_midpoint = {-1.79356, -1.6487, -2.31727, -0.704819, 1.58955, 0.122001};

// std::vector<double> jointq_bin1 = {-1.07664, -1.76487, -2.18062, -0.736245, 1.59791, -0.683229};
// std::vector<double> jointq_bin2 = {-0.907774, -1.6287, -2.34551, -0.703142, 1.59225, -0.514205};
// std::vector<double> jointq_bin3 = {-1.27674, -1.62441, -2.34612, -0.717447, 1.60367, -0.883123};
// std::vector<double> jointq_bin4 = {-1.14082, -1.44034, -2.52074, -0.72255, 1.6001, -0.746855};

std::vector<double> joint_verify_feeder = {-1.78183, -1.62972, -2.13145, -1.78324, 1.72335, 0.0565257};
std::vector<double> jointq_discard_bin = {-2.17897, -1.86781, -2.01717, -0.829591, 1.64373, -0.175439};

// std::vector<double> jointq_pick_bins_12 = {-0.959291, -1.84798, -2.24979, -0.642775, 1.56402, -0.565669};
// std::vector<double> jointq_pick_bins_12_over = {-0.959411, -1.76864, -2.22276, -0.749148, 1.5644, -0.565538};

// std::vector<double> jointq_dropoff_bins_12 = {0.112281, -1.6495, -2.54432, -0.536036, 1.58595, -1.0534};
// std::vector<double> jointq_dropoff_bins_12_over = {0.112257, -1.48768, -2.48922, -0.753941, 1.58568, -1.11153};

// std::vector<double> jointq_pick_bins_34 = {-1.15152, -1.70928, -2.43062, -0.599233, 1.55877, -0.757671};
// std::vector<double> jointq_pick_bins_34_over = {-1.15169, -1.63318, -2.40567, -0.70023, 1.5592, -0.757599};

// std::vector<double> jointq_dropoff_bins_34 = {-0.204672, -1.67169, -2.54603, -0.510077, 1.54498, -1.40419};
// std::vector<double> jointq_dropoff_bins_34_over = {-0.204995, -1.51274, -2.49631, -0.718729, 1.54574, -1.40397};

std::vector<double> jointq_mir_camera = {1.1203, -1.76657, -1.66757, -2.81709, 1.54344, -0.037145};
std::vector<double> jointq_mir_midpoint = {-0.447611, -1.34799, -2.00223, -1.3087, 1.5398, -0.0375159};

std::vector<double> pose_mir_bins_12 = {0.455386, 0.47967, 0.112601, -3.06558, 0.625002, 0.00759498};
std::vector<double> pose_mir_bins_12_over = {0.455386, 0.47967, 0.368181, -3.06558, 0.625002, 0.00759498};

std::vector<double> pose_mir_bins_34 = {0.414574, 0.381409, 0.112601, -3.06558, 0.625002, 0.00759498};
std::vector<double> pose_mir_bins_34_over = {0.414574, 0.381409, 0.368181, -3.06558, 0.625002, 0.00759498};

std::vector<double> pose_mir_dropoff_12 = {0.668902, 0.408502, 0.112601, -3.06558, 0.625002, 0.00759498};
std::vector<double> pose_mir_dropoff_12_over = {0.668902, 0.408502, 0.368181, -3.06558, 0.625002, 0.00759498};

std::vector<double> pose_mir_dropoff_34 = {0.612559, 0.281815, 0.112601, -3.06558, 0.625002, 0.00759498};
std::vector<double> pose_mir_dropoff_34_over = {0.612559, 0.281815, 0.368181, -3.06558, 0.625002, 0.00759498};

std::vector<double> jointq_dropoff_box_feeder = {-1.33032, -1.90449, -1.90989, -0.302839, 1.16162, 0.511554};
std::vector<double> jointq_dropoff_box_feeder_over = {-1.33049, -1.80391, -1.81834, -0.49512, 1.16212, 0.511996};

std::vector<double> pose_pick_bins_34 = {-0.0265153, -0.592389, 0.24347, -0.53179, -2.76379, 1.0223};
std::vector<double> jointq_pick_bins_34 = {-1.3215, -1.97924, -1.99447, -0.138782, 1.16018, 0.502283};
std::vector<double> pose_pick_bins_34_over = {-0.0265259, -0.592396, 0.363126, -0.531822, -2.76378, 1.02235};
std::vector<double> jointq_pick_bins_34_over = {-1.32189, -1.79068, -1.87183, -0.450243, 1.161, 0.503037};


std::vector<double> pose_pick_bins_12 = {-0.00154898, -0.517094, 0.173199, -0.549038, -2.8043, 1.04563};
std::vector<double> jointq_pick_bins_12 = {-1.23288, -2.01383, -2.24734, 0.137893, 1.14912, 0.595676};
std::vector<double> pose_pick_bins_12_over = {-0.00155777, -0.517087, 0.305297, -0.549087, -2.80429, 1.04566};
std::vector<double> jointq_pick_bins_12_over = {-1.23357, -1.70737, -2.15245, -0.263907, 1.15002, 0.596839};

std::vector<double> jointq_bin1 = {-1.10219, -1.87683, -2.25696, -0.0198606, 1.01892, 0.668068};
std::vector<double> jointq_bin2 = {-1.25404, -1.81634, -2.33334, 0.0766179, 1.12885, 0.561049};
std::vector<double> jointq_bin3 = {-1.23429, -1.87267, -1.9992, -0.199515, 1.10251, 0.554278};
std::vector<double> jointq_bin4 = {-1.35832, -1.82184, -2.03969, -0.235699, 1.17161, 0.485261};



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

vector<double> offsetPose(vector<double> pose, double x_offset, double y_offset) {
  pose[0] += x_offset * cos(theta) - y_offset * sin(theta);
  pose[1] += x_offset * sin(theta) + y_offset * cos(theta);
  return pose;
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

void moveJ(vector<double> jointq, double velocity, double acceleration) {
  rtde_control->moveJ(jointq, velocity, acceleration);
  waitForRobot();
  if(robotPaused) {
    ROS_INFO("Robot paused, waiting to start again");
    while(robotPaused){ }
    ROS_INFO("Robot starting");
    moveJ(jointq, velocity, acceleration);
  }
}

void moveLinear(vector<double> pose, double velocity, double acceleration) {
  rtde_control->moveL(pose, velocity, acceleration);
  waitForRobot();
  if(robotPaused) {
    ROS_INFO("Robot paused, waiting to start again");
    while(robotPaused){ }
    ROS_INFO("Robot starting");
    moveLinear(pose, velocity, acceleration);
  }
}

void robotCommandCallback(const robot_msgs::RobotCmd::ConstPtr& cmd) {
  // io_interface->reconnect();
  if(cmd->command == "pick-blue") {
    ROS_INFO("Picking blue brick");
    
    io_interface->setStandardDigitalOut(4, 1);
    
    moveJ(jointq_feeder_midpoint, joint_velocity_high, joint_acceleration_high);
    if(robotStopped)
      return;
    moveJ(jointq_pick_blue_over, joint_velocity_high, joint_acceleration_high);
    if(robotStopped)
      return;
    moveJ(jointq_pick_blue, joint_velocity_low, joint_acceleration_low);
    if(robotStopped)
      return;
    
    io_interface->setStandardDigitalOut(4, 0);
    ros::Duration(gripping_time).sleep();
    
    moveJ(jointq_pick_blue_over, joint_velocity_low, joint_acceleration_low);
    if(robotStopped)
      return;
    moveJ(jointq_pick_midpoint, joint_velocity_extreme, joint_acceleration_high);
    if(robotStopped)
      return;
    moveJ(getBinJointQ(cmd->binNumber), joint_velocity_high, joint_acceleration_high);
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
    
    moveJ(jointq_feeder_midpoint, joint_velocity_high, joint_acceleration_high);
    if(robotStopped)
      return;
    moveJ(jointq_pick_red_over, joint_velocity_high, joint_acceleration_high);
    if(robotStopped)
      return;
    moveJ(jointq_pick_red, joint_velocity_low, joint_acceleration_low);
    if(robotStopped)
      return;
    
    io_interface->setStandardDigitalOut(4, 0);
    ros::Duration(gripping_time).sleep();
    
    moveJ(jointq_pick_red_over, joint_velocity_low, joint_acceleration_low);
    if(robotStopped)
      return;
    moveJ(jointq_pick_midpoint, joint_velocity_extreme, joint_acceleration_high);
    if(robotStopped)
      return;
    moveJ(getBinJointQ(cmd->binNumber), joint_velocity_high, joint_acceleration_high);
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
    
    moveJ(jointq_feeder_midpoint, joint_velocity_high, joint_acceleration_high);
    if(robotStopped)
      return;
    moveJ(jointq_pick_yellow_over, joint_velocity_high, joint_acceleration_high);
    if(robotStopped)
      return;
    moveJ(jointq_pick_yellow, joint_velocity_low, joint_acceleration_low);
    if(robotStopped)
      return;
    
    io_interface->setStandardDigitalOut(4, 0);
    ros::Duration(gripping_time).sleep();
    
    moveJ(jointq_pick_yellow_over, joint_velocity_low, joint_acceleration_low);
    if(robotStopped)
      return;
    moveJ(jointq_pick_midpoint, joint_velocity_extreme, joint_acceleration_high);
    if(robotStopped)
      return;
    moveJ(getBinJointQ(cmd->binNumber), joint_velocity_high, joint_acceleration_high);
    if(robotStopped)
      return;
    
    io_interface->setStandardDigitalOut(4, 1);
    
    robot_msgs::RobotStatus status;
    status.ready = true;
    robot_status_pub.publish(status);
  }
    else if(cmd->command == "verify-bricks") {
    ROS_INFO("Verifying Bricks");
    moveJ(jointq_pick_midpoint, joint_velocity_extreme, joint_acceleration_high);
    if(robotStopped)
      return;
    moveJ(joint_verify_feeder, joint_velocity_high, joint_acceleration_low);
    if(robotStopped)
      return;
    robot_msgs::RobotStatus status;
    status.ready = true;
    robot_status_pub.publish(status);
  }
  else if(cmd->command == "discard-yellow") {
    ROS_INFO("Discarding yellow brick");
    
    io_interface->setStandardDigitalOut(4, 1);
    
    moveJ(jointq_feeder_midpoint, joint_velocity_high, joint_acceleration_high);
    if(robotStopped)
      return;
    moveJ(jointq_pick_yellow_over, joint_velocity_high, joint_acceleration_high);
    if(robotStopped)
      return;
    moveJ(jointq_pick_yellow, joint_velocity_low, joint_acceleration_low);
    if(robotStopped)
      return;
    
    io_interface->setStandardDigitalOut(4, 0);
    ros::Duration(gripping_time).sleep();
    
    moveJ(jointq_pick_yellow_over, joint_velocity_low, joint_acceleration_low);
    if(robotStopped)
      return;
    moveJ(jointq_pick_midpoint, joint_velocity_extreme, joint_acceleration_high);
    if(robotStopped)
      return;
    moveJ(jointq_discard_bin, joint_velocity_high, joint_acceleration_high);
    if(robotStopped)
      return;
    
    io_interface->setStandardDigitalOut(4, 1);
    ros::Duration(gripping_time).sleep();
    
    robot_msgs::RobotStatus status;
    status.ready = true;
    robot_status_pub.publish(status);
  }
  else if(cmd->command == "discard-blue") {
    ROS_INFO("Discarding blue brick");
    
    io_interface->setStandardDigitalOut(4, 1);
    
    moveJ(jointq_feeder_midpoint, joint_velocity_high, joint_acceleration_high);
    if(robotStopped)
      return;
    moveJ(jointq_pick_blue_over, joint_velocity_high, joint_acceleration_high);
    if(robotStopped)
      return;
    moveJ(jointq_pick_blue, joint_velocity_low, joint_acceleration_low);
    if(robotStopped)
      return;
    
    io_interface->setStandardDigitalOut(4, 0);
    ros::Duration(gripping_time).sleep();
    
    moveJ(jointq_pick_blue_over, joint_velocity_low, joint_acceleration_low);
    if(robotStopped)
      return;
    moveJ(jointq_pick_midpoint, joint_velocity_extreme, joint_acceleration_high);
    if(robotStopped)
      return;
    moveJ(jointq_discard_bin, joint_velocity_high, joint_acceleration_high);
    if(robotStopped)
      return;
    
    io_interface->setStandardDigitalOut(4, 1);
    ros::Duration(gripping_time).sleep();
    
    robot_msgs::RobotStatus status;
    status.ready = true;
    robot_status_pub.publish(status);
  }
  else if(cmd->command == "discard-red") {
    ROS_INFO("Discarding red brick");
    
    io_interface->setStandardDigitalOut(4, 1);
    
    moveJ(jointq_feeder_midpoint, joint_velocity_high, joint_acceleration_high);
    if(robotStopped)
      return;
    moveJ(jointq_pick_red_over, joint_velocity_high, joint_acceleration_high);
    if(robotStopped)
      return;
    moveJ(jointq_pick_red, joint_velocity_low, joint_acceleration_low);
    if(robotStopped)
      return;
    
    io_interface->setStandardDigitalOut(4, 0);
    ros::Duration(gripping_time).sleep();
    
    moveJ(jointq_pick_red_over, joint_velocity_low, joint_acceleration_low);
    if(robotStopped)
      return;
    moveJ(jointq_pick_midpoint, joint_velocity_extreme, joint_acceleration_high);
    if(robotStopped)
      return;
    moveJ(jointq_discard_bin, joint_velocity_high, joint_acceleration_high);
    if(robotStopped)
      return;
    
    io_interface->setStandardDigitalOut(4, 1);
    ros::Duration(gripping_time).sleep();
    
    robot_msgs::RobotStatus status;
    status.ready = true;
    robot_status_pub.publish(status);
  }
  else if(cmd->command == "move-boxes-to-mir") {
    io_interface->setStandardDigitalOut(4, 1);
    
    moveJ(jointq_mir_midpoint, joint_velocity_high, joint_acceleration_high);
    if(robotStopped)
      return;
    moveJ(jointq_pick_bins_34_over, joint_velocity_high, joint_acceleration_high);
    if(robotStopped)
      return;
    moveLinear(pose_pick_bins_34, pose_velocity_low, pose_acceleration_low);
    if(robotStopped)
      return;

    io_interface->setStandardDigitalOut(4, 0);
    ros::Duration(gripping_time).sleep();

    moveLinear(pose_pick_bins_34_over, pose_velocity_low, pose_acceleration_low);
    if(robotStopped)
      return;
    moveJ(jointq_mir_midpoint, joint_velocity_high, joint_acceleration_high);
    if(robotStopped)
      return;
    moveLinear(offsetPose(pose_mir_dropoff_34_over, cmd->x_offset, cmd->y_offset), pose_velocity_high, pose_acceleration_high);
    if(robotStopped)
      return;
    moveLinear(offsetPose(pose_mir_dropoff_34, cmd->x_offset, cmd->y_offset), pose_velocity_low, pose_acceleration_low);
    if(robotStopped)
      return;
    
    io_interface->setStandardDigitalOut(4, 1);

    moveLinear(offsetPose(pose_mir_dropoff_34_over, cmd->x_offset, cmd->y_offset), pose_velocity_low, pose_acceleration_low);
    if(robotStopped)
      return;
    moveJ(jointq_mir_midpoint, joint_velocity_high, joint_acceleration_high);
    if(robotStopped)
      return;    
    moveJ(jointq_pick_bins_12_over, joint_velocity_high, joint_acceleration_high);
    if(robotStopped)
      return;  
    moveLinear(pose_pick_bins_12, pose_velocity_low, pose_acceleration_low);
    if(robotStopped)
      return;

    io_interface->setStandardDigitalOut(4, 0);
    ros::Duration(gripping_time).sleep();

    moveLinear(pose_pick_bins_12_over, pose_velocity_low, pose_acceleration_low);
    if(robotStopped)
      return;
    moveJ(jointq_mir_midpoint, joint_velocity_high, joint_acceleration_high);
    if(robotStopped)
      return;
    moveLinear(offsetPose(pose_mir_dropoff_12_over, cmd->x_offset, cmd->y_offset), pose_velocity_high, pose_acceleration_high);
    if(robotStopped)
      return;
    moveLinear(offsetPose(pose_mir_dropoff_12, cmd->x_offset, cmd->y_offset), pose_velocity_low, pose_acceleration_low);
    if(robotStopped)
      return;
    
    io_interface->setStandardDigitalOut(4, 1);

    moveLinear(offsetPose(pose_mir_dropoff_12_over, cmd->x_offset, cmd->y_offset), pose_velocity_low, pose_acceleration_low);
    if(robotStopped)
      return;

    robot_msgs::RobotStatus status;
    status.ready = true;
    robot_status_pub.publish(status);
  }
  else if(cmd->command == "camera-over-mir") {
    moveJ(jointq_mir_midpoint, joint_velocity_high, joint_acceleration_high);
    if(robotStopped)
      return;
    moveJ(jointq_mir_camera, joint_velocity_high, joint_acceleration_high);
    if(robotStopped)
      return;
    
    robot_msgs::RobotStatus status;
    status.ready = true;
    robot_status_pub.publish(status);
  }
  else if (cmd->command == "move-boxes-from-mir") {
    io_interface->setStandardDigitalOut(4, 1);
    moveLinear(offsetPose(pose_mir_bins_12_over, cmd->x_offset, cmd->y_offset), pose_velocity_high, pose_acceleration_high);
    if(robotStopped)
      return;
    moveLinear(offsetPose(pose_mir_bins_12, cmd->x_offset, cmd->y_offset), pose_velocity_low, pose_acceleration_low);
    if(robotStopped)
      return;

    io_interface->setStandardDigitalOut(4, 0);
    ros::Duration(gripping_time).sleep();

    moveLinear(offsetPose(pose_mir_bins_12_over, cmd->x_offset, cmd->y_offset), pose_velocity_low, pose_acceleration_low);
    if(robotStopped)
      return;
    moveJ(jointq_mir_midpoint, joint_velocity_high, joint_acceleration_high);
    if(robotStopped)
      return;
    moveJ(jointq_dropoff_box_feeder_over, joint_velocity_high, joint_acceleration_high);
    if(robotStopped)
      return;
    moveJ(jointq_dropoff_box_feeder, joint_velocity_low, joint_acceleration_low);
    if(robotStopped)
      return;

    io_interface->setStandardDigitalOut(4, 1);
    
    moveJ(jointq_dropoff_box_feeder_over, joint_velocity_low, joint_acceleration_low);
    if(robotStopped)
      return;
    moveJ(jointq_mir_midpoint, joint_velocity_high, joint_acceleration_high);
    if(robotStopped)
      return;
    moveLinear(offsetPose(pose_mir_bins_34_over, cmd->x_offset, cmd->y_offset), pose_velocity_high, pose_acceleration_high);
    if(robotStopped)
      return;
    moveLinear(offsetPose(pose_mir_bins_34, cmd->x_offset, cmd->y_offset), pose_velocity_low, pose_acceleration_low);
    if(robotStopped)
      return;

    io_interface->setStandardDigitalOut(4, 0);
    ros::Duration(gripping_time).sleep();

    moveLinear(offsetPose(pose_mir_bins_34_over, cmd->x_offset, cmd->y_offset), pose_velocity_low, pose_acceleration_low);
    if(robotStopped)
      return;
    moveJ(jointq_mir_midpoint, joint_velocity_high, joint_acceleration_high);
    if(robotStopped)
      return;
    moveJ(jointq_dropoff_box_feeder_over, joint_velocity_high, joint_acceleration_high);
    if(robotStopped)
      return;
    moveJ(jointq_dropoff_box_feeder, joint_velocity_low, joint_acceleration_low);
    if(robotStopped)
      return;

    io_interface->setStandardDigitalOut(4, 1);
    
    moveJ(jointq_dropoff_box_feeder_over, joint_velocity_low, joint_acceleration_low);
    if(robotStopped)
      return;

    robot_msgs::RobotStatus status;
    status.ready = true;
    robot_status_pub.publish(status);
  }

}

void stopCallback(const packml_msgs::Status::ConstPtr& msg)
{
  packMLState = msg->state.val;
  if(msg->state.val == ABORTING) {
    ROS_INFO("Aborting Robot");
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

  ros::ServiceClient transitionClient = n.serviceClient<packml_msgs::Transition>("packml_node/packml/transition");

  rtde_control = new RTDEControlInterface(robot_ip);
  dashboard_client = new DashboardClient(robot_ip);
  dashboard_client->connect();

  io_interface = new RTDEIOInterface(robot_ip);

  ros::AsyncSpinner s(2);

  s.start();

  bool blue_blink = false;
  string lastSafetyMode = "";

  while(ros::ok()) {
    string safetymode = dashboard_client->safetymode();
    string robotmode = dashboard_client->robotmode();
    if (packMLState != SUSPENDED) {
      if(safetymode == "Safetymode: SAFEGUARD_STOP") {
        if(safetymode != lastSafetyMode) {
          packml_msgs::Transition srv;
          srv.request.command = STOP;
          transitionClient.call(srv);
        }
        blue_blink = !blue_blink;
      }
      else {
        if(lastSafetyMode == "Safetymode: SAFEGUARD_STOP") {
          packml_msgs::Transition srv;
          srv.request.command = RESET;
          transitionClient.call(srv);
          ros::Duration(0.5).sleep();
          srv.request.command = START;
          transitionClient.call(srv);
        }
        blue_blink = false;
      }
    }
    else {
      if(safetymode == "Safetymode: SAFEGUARD_STOP") {
        blue_blink = !blue_blink;
      }
      else {
        blue_blink = false;
      }
    }


    io_interface->setStandardDigitalOut(5, blue_blink ? 1 : 0);

    if (!(safetymode == "Safetymode: NORMAL" || safetymode == "Safetymode: REDUCED" || safetymode == "Safetymode: RECOVERY")) {
      // cout << "resetting safety" << endl;
      // dashboard_client->restartSafety();
      // dashboard_client->closeSafetyPopup();
    }
    // cout << dashboard_client->safetymode() << endl;
    // cout << dashboard_client->robotmode() << endl;
    
    lastSafetyMode = safetymode;
    ros::Duration(0.5).sleep();
  }

  rtde_control->stopRobot();

  return 0;
}