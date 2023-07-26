/*
 * Copyright (c) 2023, Instituto de Pesquisas da Marinha do Brasil
 * All rights reserved.
 * Written by João Victor Torres Borges <borgesjvt@gmail.com>
*/

#ifndef PRIMUS_IMU_DRIVER__PRIMUS_HPP_
#define PRIMUS_IMU_DRIVER__PRIMUS_HPP_

// TODO(BORGES): remove unecessary libraries
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ipqm_msgs/msg/dvl.hpp"

#define BUFSIZE \
  78  // MAX buffer size for serial reading. The same size of PRIMUS data package (78 bytes)

class NortekDVLDriver : public rclcpp::Node
{
public:
  NortekDVLDriver();
  // ~NortekDVLDriver();
  void run();
  // void setup();

private:
  rclcpp::Publisher<ipqm_msgs::msg::Dvl>::SharedPtr dvl_pub_;      // DVL ROS Publisher
  rclcpp::TimerBase::SharedPtr timer_;  // Timer to define callback frequency
  ipqm_msgs::msg::Dvl dvl_data_;      // DVL ROS data
  // std::string serial_port_name_;        // ROS parameter to get serial port name

  // uint8_t buffer_[BUFSIZE] = {};   // Stores data read directly from the serial
  // uint8_t package_[BUFSIZE] = {};  // Shift the buffer data to match nortek driver description
  // uint8_t total_time_counter_[8] =
  //   "";  // Internal nortek dvl timestamp (the last digit is incremented by one each 10ms)
  // uint8_t linear_accel_x_[4] = "";  // Linear acceleration in x (m/s^2)
  // uint8_t linear_accel_y_[4] = "";  // Linear acceleration in y (m/s^2)
  // uint8_t linear_accel_z_[4] = "";  // Linear acceleration in z (m/s^2)
  // uint8_t angular_vel_x_[4] = "";   // Angular velocity in x (rad/s)
  // uint8_t angular_vel_y_[4] = "";   // Angular velocity in y (rad/s)
  // uint8_t angular_vel_z_[4] = "";   // Angular velocity in z (rad/s)
  // int sockfd_ = -1;                       // Serial port file descriptor
  // uint32_t execution_time_;  // For measuring the execution time of reading the data

  // void open_serial();
  // void read_serial();
  // void close_serial();
  // int set_interface_attribs(int speed);
  // bool get_check_sum_status();
  // void decode_package();
  // bool find_header(int & index);
  // void closing_last_package(int index);
  // void starting_next_package(int index);
};
#endif  // PRIMUS_IMU_DRIVER__PRIMUS_HPP_
