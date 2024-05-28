cmake_minimum_required(VERSION 3.0)
project(ClientApp)

add_executable(client main.cpp)

# Find and link against the required libraries (sockets)
target_link_libraries(client PRIVATE "-pthread")
