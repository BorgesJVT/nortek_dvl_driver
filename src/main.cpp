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
  // nortek_dvl_driver_node->setup();
  rclcpp::spin(nortek_dvl_driver_node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
