/*
 * Copyright (c) 2023, Instituto de Pesquisas da Marinha do Brasil
 * All rights reserved.
 * Written by João Victor Torres Borges <borgesjvt@gmail.com>
*/
#include <memory>
#include "nortek_dvl_driver/nortek_dvl_driver.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto nortek_dvl_driver_node = std::make_shared<NortekDVLDriver>();
  rclcpp::spin(std::make_shared<NortekDVLDriver>());
  rclcpp::shutdown();

  return 0;
}
