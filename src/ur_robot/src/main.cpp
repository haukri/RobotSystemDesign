#include "ros/ros.h"
#include "std_msgs/String.h"

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

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void robotCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());

  if(msg->data == "pick-red") {
    ROS_INFO("Picking red brick");

    std::vector<double> joint_q1 = {-1.12738, -1.59118, -2.1059, -0.96981, 1.59176, 0.032187};
    std::vector<double> joint_q2 = {-1.82089, -2.04634, -1.22491, -1.11367, 1.59176, 0.0321989};

    rtde_control->moveJ(joint_q1, velocity, acceleration);
    rtde_control->moveJ(joint_q2, velocity, acceleration);
  }
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "ur_robot");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("robot_command", 1000, robotCallback);

  rtde_control = new RTDEControlInterface("192.168.1.10");


  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  rtde_control->stopRobot();

  return 0;
}