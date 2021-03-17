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

/* ROS API Header */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

/* ROBOTIS Controller Header */
#include "robotis_controller/robotis_controller.h"
#include "robotis_controller_msgs/SetModule.h"

/* Sensor Module Header */
#include "open_cr_module/open_cr_module.h"

/* Motion Module Header */
#include "op3_base_module/base_module.h"
#include "op3_direct_control_module/direct_control_module.h"

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

std::string g_offset_file;
std::string g_robot_file;
std::string g_init_file;
std::string g_device_name;

bool resetMotion(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  RobotisController *controller = RobotisController::getInstance();
  DirectControlModule *mod = DirectControlModule::getInstance();
  controller->removeMotionModule(mod);
  DirectControlModule::destroyInstance();
  mod = DirectControlModule::getInstance();
  controller->addMotionModule(mod);
  ROS_INFO("reset motion");
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "op3_manager");
  ros::NodeHandle nh;

  ROS_INFO("manager->init");
  RobotisController *controller = RobotisController::getInstance();

  /* Load ROS Parameter */

  nh.param<std::string>("offset_file_path", g_offset_file, "");
  nh.param<std::string>("robot_file_path", g_robot_file, "");
  nh.param<std::string>("init_file_path", g_init_file, "");
  nh.param<std::string>("device_name", g_device_name, SUB_CONTROLLER_DEVICE);

  // ros::AdvertiseServiceOptions reset_motion_aso =
  //   ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
  //     "/robotis/gym/reset_motion",
  //     resetMotion,
  //     ros::VoidPtr(), &gazebo_queue_);

  // ros::ServiceServer reset_motion_srv = nh.advertiseService(reset_motion_aso);

  ros::ServiceServer reset_motion_srv = nh.advertiseService("/robotis/gym/reset_motion", resetMotion);

  controller->gazebo_mode_ = true;

  {
    ROS_WARN("SET TO GAZEBO MODE!");
    std::string robot_name;
    nh.param<std::string>("gazebo_robot_name", robot_name, "");
    if (robot_name != "")
      controller->gazebo_robot_name_ = robot_name;
  }

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

  // load offset
  if (g_offset_file != "")
    controller->loadOffset(g_offset_file);

  usleep(300 * 1000);

  /* Add Sensor Module */
  controller->addSensorModule((SensorModule*) OpenCRModule::getInstance());

  /* Add Motion Module */
  // controller->addMotionModule((MotionModule*) DirectControlModule::getInstance());

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
