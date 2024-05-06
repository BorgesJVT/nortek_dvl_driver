/*
 * Copyright (c) 2023, Instituto de Pesquisas da Marinha do Brasil
 * All rights reserved.
 * Written by João Victor Torres Borges <borgesjvt@gmail.com>
*/

#ifndef PRIMUS_IMU_DRIVER__PRIMUS_HPP_
#define PRIMUS_IMU_DRIVER__PRIMUS_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "dvl_msgs/msg/dvl.hpp"

#define BUFSIZE \
  78  // MAX buffer size for serial reading. The same size of PRIMUS data package (78 bytes)

class NortekDVLDriver : public rclcpp::Node
{
public:
  NortekDVLDriver();
  ~NortekDVLDriver();
  void run();
  // void setup();

private:
  rclcpp::Publisher<dvl_msgs::msg::Dvl>::SharedPtr dvl_pub_;      // DVL ROS Publisher
  rclcpp::TimerBase::SharedPtr timer_;  // Timer to define callback frequency
  dvl_msgs::msg::Dvl dvl_data_;      // DVL ROS data
  std::string serial_port_name_;        // ROS parameter to get serial port name

};
#endif  // PRIMUS_IMU_DRIVER__PRIMUS_HPP_
