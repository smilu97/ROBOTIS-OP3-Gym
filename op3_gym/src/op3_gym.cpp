/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Kayman */

/* gazebo */
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <functional>
#include "op3_gym/Step.h"

/* ROS API Header */
#include <ros/ros.h>
#include <std_msgs/String.h>

/* ROBOTIS Controller Header */
#include "robotis_controller/robotis_controller.h"
#include "open_cr_module/open_cr_module.h"

using namespace robotis_framework;
using namespace dynamixel;
using namespace robotis_op;

const int BAUD_RATE = 2000000;
const double PROTOCOL_VERSION = 2.0;
const int SUB_CONTROLLER_ID = 200;
const int DXL_BROADCAST_ID = 254;
const int DEFAULT_DXL_ID = 1;
const std::string SUB_CONTROLLER_DEVICE = "/dev/ttyUSB0";
const int POWER_CTRL_TABLE = 24;
const int RGB_LED_CTRL_TABLE = 26;
const int TORQUE_ON_CTRL_TABLE = 64;

std::string g_robot_file;
std::string g_init_file;
std::string g_device_name;

bool iterateCallback(op3_gym::Step::Request& req, op3_gym::Step::Response& res, const gazebo::transport::PublisherPtr& pub)
{
  // Create Iterate msg
  gazebo::msgs::WorldControl stepper;
  // Set multi-step to requested iterations
  stepper.set_multi_step(req.iterations);
  pub->Publish(stepper);

  res.result = true;

  return true;
}

int main(int argc, char **argv)
{
  gazebo::client::setup(argc,argv);
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  ros::init(argc, argv, "op3_gym");
  ros::NodeHandle nh;

  gazebo::transport::PublisherPtr pub = node->Advertise<gazebo::msgs::WorldControl>("~/world_control");
  //Publish to topic ~/world_control
  pub->WaitForConnection();

  ros::ServiceServer iter_server = nh.advertiseService<op3_gym::Step::Request, op3_gym::Step::Response>(
    "/iterate",
    std::bind(&iterateCallback,
    std::placeholders::_1,
    std::placeholders::_2,
    pub));

  ROS_INFO("gym->init");
  RobotisController *controller = RobotisController::getInstance();

  /* Load ROS Parameter */

  nh.param<std::string>("robot_file_path", g_robot_file, "");
  nh.param<std::string>("init_file_path", g_init_file, "");
  nh.param<std::string>("device_name", g_device_name, SUB_CONTROLLER_DEVICE);

  controller->gazebo_mode_ = true;
  
  std::string robot_name;
  nh.param<std::string>("gazebo_robot_name", robot_name, "");
  if (robot_name != "")
    controller->gazebo_robot_name_ = robot_name;

  if (g_robot_file == "")
  {
    ROS_ERROR("NO robot file path in the ROS parameters.");
    return -1;
  }

  // initialize robot
  if (controller->initialize(g_robot_file, g_init_file) == false)
  {
    ROS_ERROR("ROBOTIS Controller Initialize Fail!");
    return -1;
  }

  usleep(300 * 1000);

  controller->addSensorModule((SensorModule*) OpenCRModule::getInstance());

  // start timer
  controller->startTimer();

  usleep(100 * 1000);

  // {
  //   ros::ServiceClient set_joint_module_client;
  //   set_joint_module_client = nh.serviceClient<robotis_controller_msgs::SetModule>("/robotis/set_present_ctrl_modules");

  //   robotis_controller_msgs::SetModule set_module_srv;
  //   set_module_srv.request.module_name = "direct_control_module";

  //   if (set_joint_module_client.call(set_module_srv) == false)
  //   {
  //     ROS_ERROR("Failed to set module");
  //   }
  // }
  

  while (ros::ok())
  {
    usleep(1 * 1000);
    ros::spin();
  }

  return 0;
}
