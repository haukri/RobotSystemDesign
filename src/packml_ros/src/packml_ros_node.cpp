/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2017 Shaun Edwards
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include "robot_msgs/RobotCommand.h"
#include "order_msgs/NewOrder.h"
#include "order_msgs/CompleteOrder.h"
#include <packml_ros/packml_ros.h>
#include <packml_sm/boost/packml_state_machine_continuous.h>

ros::ServiceClient robot_client;
ros::ServiceClient new_order_client;
ros::ServiceClient complete_order_client;
int binNumber = 0;

int myExecuteMethod()
{
  ROS_INFO_STREAM("This is my execute method(begin)");

  // Get a new order
  order_msgs::NewOrder new_order_srv;
  if (new_order_client.call(new_order_srv))
  {
    ROS_INFO("Got new order");
    // Pick blue bricks
    for(int i=0; i < new_order_srv.response.blue_amount; i++) {
      robot_msgs::RobotCommand srv;
      srv.request.command = "pick-blue";
      ROS_INFO("Packing blue brick number %d", i+1);
      if (robot_client.call(srv))
      {
        ROS_INFO("Done");
      }
    }
    // Pick red bricks
    for(int i=0; i < new_order_srv.response.red_amount; i++) {
      robot_msgs::RobotCommand srv;
      srv.request.command = "pick-red";
      ROS_INFO("Packing red brick number %d", i+1);
      if (robot_client.call(srv))
      {
        ROS_INFO("Done");
      }
    }
    // Pick yellow bricks
    for(int i=0; i < new_order_srv.response.yellow_amount; i++) {
      robot_msgs::RobotCommand srv;
      srv.request.command = "pick-yellow";
      ROS_INFO("Packing yellow brick number %d", i+1);
      if (robot_client.call(srv))
      {
        ROS_INFO("Done");
      }
    }
    // Complete the order
    order_msgs::CompleteOrder complete_order_srv;
    complete_order_srv.request.order_number = new_order_srv.response.order_number;
    if(complete_order_client.call(complete_order_srv)) {
      ROS_INFO("Order completed");
    }
  }

  ROS_INFO_STREAM("This is my execute method(end)");
  return 0;  // returning zero indicates non-failure
}

int myStartingMethod()
{
  ROS_INFO_STREAM("Starting");
  binNumber = 0;
  return 0;  // returning zero indicates non-failure
}

int mySuspendedMethod() {
  ROS_INFO_STREAM("This is my suspended method(begin)");
  ros::Duration(1.0).sleep();
  ROS_INFO_STREAM("This is my suspended method(end)");
  return 0;  // returning zero indicates non-failure
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "packml_node");
  ros::NodeHandle n;

  robot_client = n.serviceClient<robot_msgs::RobotCommand>("robot_command");
  new_order_client = n.serviceClient<order_msgs::NewOrder>("new_order");
  complete_order_client = n.serviceClient<order_msgs::CompleteOrder>("complete_order");

  auto sm = packml_sm::PackmlStateMachineContinuous::spawn();
  sm->setExecute(std::bind(myExecuteMethod));
  sm->setStarting(std::bind(myStartingMethod));
  sm->setSuspended(std::bind(mySuspendedMethod));
  packml_ros::PackmlRos sm_node(ros::NodeHandle(), ros::NodeHandle("~"), sm);
  sm_node.spin();

  return 0;
}
