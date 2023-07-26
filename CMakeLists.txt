cmake_minimum_required(VERSION 3.5)
project(nortek_dvl_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ipqm_msgs REQUIRED)

include_directories(include)

set(ament_dependencies
  rclcpp
  std_msgs
  geometry_msgs
  ipqm_msgs
)

add_library(nortek_dvl_driver src/nortek_dvl_driver.cpp)
ament_target_dependencies(nortek_dvl_driver ${ament_dependencies})

add_executable(nortek_dvl_driver_node src/main.cpp)
ament_target_dependencies(nortek_dvl_driver_node ${ament_dependencies})
target_link_libraries(nortek_dvl_driver_node nortek_dvl_driver)

install(TARGETS nortek_dvl_driver_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY 
  launch 
  # test
  DESTINATION share/${PROJECT_NAME}
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()

#   find_package(ament_cmake_gtest REQUIRED)
#   add_subdirectory(test)
# endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
