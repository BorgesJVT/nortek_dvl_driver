/*
 * Copyright (c) 2023, Instituto de Pesquisas da Marinha do Brasil
 * All rights reserved.
 * Written by João Victor Torres Borges <borgesjvt@gmail.com>
 */
#include <string>
#include "nortek_dvl_driver/nortek_dvl_driver.hpp"

/* NortekDVLDriver Constructor */
NortekDVLDriver::NortekDVLDriver()
: Node("nortek_dvl_driver_node")
{
  declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
  dvl_pub_ = create_publisher<dvl_msgs::msg::Dvl>("/nortek/dvl", 10);
  timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&NortekDVLDriver::run, this));
  dvl_data_.header.frame_id = "dvl_link";
}

/* NortekDVLDriver Destructor */
NortekDVLDriver::~NortekDVLDriver() {}


void NortekDVLDriver::run() {
  dvl_data_.velocity.x = 1.0;
  dvl_data_.velocity.y = 2.0;
  dvl_data_.velocity.z = 3.0;
  dvl_pub_->publish(dvl_data_);
}
