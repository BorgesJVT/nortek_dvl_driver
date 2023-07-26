/*
 * Copyright (c) 2023, Instituto de Pesquisas da Marinha do Brasil
 * All rights reserved.
 * Written by João Victor Torres Borges <borgesjvt@gmail.com>
 */
#include "nortek_dvl_driver/nortek_dvl_driver.hpp"
#include <string>

/* NortekDVLDriver Constructor */
NortekDVLDriver::NortekDVLDriver()
: Node("nortek_dvl_driver_node")
{
  // declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
  dvl_pub_ = create_publisher<ipqm_msgs::msg::Dvl>("/dvl_data", 10);
  timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&NortekDVLDriver::run, this));
  dvl_data_.header.frame_id = "dvl_link";
}

/* NortekDVLDriver Destructor */
// NortekDVLDriver::~NortekDVLDriver() {close_serial();}

// /* Initial configurations */
// void NortekDVLDriver::setup() {open_serial();}

// /* Configure serial interface */
// int NortekDVLDriver::set_interface_attribs(int speed)
// {
//   struct termios tty;

//   if (tcgetattr(sockfd_, &tty) < 0) {
//     RCLCPP_ERROR(get_logger(), "Error from tcgetattr: %s", strerror(errno));
//     exit(EXIT_FAILURE);
//   }

//   cfsetospeed(&tty, (speed_t)speed);
//   cfsetispeed(&tty, (speed_t)speed);

//   tty.c_cflag |= (CLOCAL | CREAD); /* ignore modem controls */
//   tty.c_cflag &= ~CSIZE;
//   tty.c_cflag |= CS8;      /* 8-bit characters */
//   tty.c_cflag &= ~PARENB;  /* no parity bit */
//   tty.c_cflag &= ~CSTOPB;  /* only need 1 stop bit */
//   tty.c_cflag &= ~CRTSCTS; /* no hardware flowcontrol */
//   /* setup for non-canonical mode */
//   tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
//   /* Disable any special handling of input chars/bytes */
//   tty.c_iflag &=
//     ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
//   /* Disable any special handling of output chars/bytes */
//   tty.c_oflag &= ~OPOST;

//   /* ????? */
//   tty.c_cc[VMIN] = 0;
//   tty.c_cc[VTIME] = 0;

//   if (tcsetattr(sockfd_, TCSANOW, &tty) != 0) {
//     RCLCPP_ERROR(get_logger(), "Error from tcsetattr: %s", strerror(errno));
//     exit(EXIT_FAILURE);
//   }
//   return 0;
// }

// TODO(BORGES): verify checksum algorithm
// bool NortekDVLDriver::get_check_sum_status()
// {
//   uint16_t sum = 0, temp = 0;
//   for (int count = 0; count < 76; count = count + 2) {
//     temp = package_[count] + 256 * (package_[count + 1]);
//     sum = sum + temp;
//   }
//   temp = package_[76] + 256 * (package_[77]);

//   return sum == temp;
// }

// /* Decode PRIMUS package data */
// void NortekDVLDriver::decode_package()
// {
//   if (get_check_sum_status()) {
//     // Total Time Counter [2-9]
//     total_time_counter_[0] = package_[2];
//     total_time_counter_[1] = package_[3];
//     total_time_counter_[2] = package_[4];
//     total_time_counter_[3] = package_[5];
//     total_time_counter_[4] = package_[6];
//     total_time_counter_[5] = package_[7];
//     total_time_counter_[6] = package_[8];
//     total_time_counter_[7] = package_[9];

//     // linear accel x (m/s^2) [48-51]
//     linear_accel_x_[0] = package_[48];
//     linear_accel_x_[1] = package_[49];
//     linear_accel_x_[2] = package_[50];
//     linear_accel_x_[3] = package_[51];

//     // linear accel y (m/s^2) [52-55]
//     linear_accel_y_[0] = package_[52];
//     linear_accel_y_[1] = package_[53];
//     linear_accel_y_[2] = package_[54];
//     linear_accel_y_[3] = package_[55];

//     // linear accel z (m/s^2) [56-59]
//     linear_accel_z_[0] = package_[56];
//     linear_accel_z_[1] = package_[57];
//     linear_accel_z_[2] = package_[58];
//     linear_accel_z_[3] = package_[59];

//     // angular vel x (rad/s) [56-59]
//     angular_vel_x_[0] = package_[60];
//     angular_vel_x_[1] = package_[61];
//     angular_vel_x_[2] = package_[62];
//     angular_vel_x_[3] = package_[63];

//     // angular vel y (rad/s) [64-67]
//     angular_vel_y_[0] = package_[64];
//     angular_vel_y_[1] = package_[65];
//     angular_vel_y_[2] = package_[66];
//     angular_vel_y_[3] = package_[67];

//     // angular vel z (rad/s) [68-71]
//     angular_vel_z_[0] = package_[68];
//     angular_vel_z_[1] = package_[69];
//     angular_vel_z_[2] = package_[70];
//     angular_vel_z_[3] = package_[71];

//     // TODO(BORGES): Consider changing to RCLCPP_DEBUG
//     RCLCPP_INFO(
//       get_logger(), "Total time counter: %lu",
//       *(reinterpret_cast<uint64_t *>(total_time_counter_)));
//     RCLCPP_INFO(
//       get_logger(), "Gyroscope: {X: %f, Y: %f, Z: %f}",
//       *(reinterpret_cast<float *>(angular_vel_x_)),
//       *(reinterpret_cast<float *>(angular_vel_y_)),
//       *(reinterpret_cast<float *>(angular_vel_z_)));
//     RCLCPP_INFO(
//       get_logger(), "Accelerometer: {X: %f, Y: %f, Z: %f}",
//       *(reinterpret_cast<float *>(linear_accel_x_)),
//       *(reinterpret_cast<float *>(linear_accel_y_)),
//       *(reinterpret_cast<float *>(linear_accel_z_)));
//   }
// }

// void NortekDVLDriver::closing_last_package(int index)
// {
//   // If the package has not been started yet (by starting_next_package),
//   // we have nothing to do here.
//   if (package_[0] == '\0') {
//     return;
//   }

//   // If already started, fill the remain positions until close it.
//   for (int i = 0; i < index; ++i) {
//     package_[BUFSIZE - index + i] = buffer_[i];
//   }
// }

// void NortekDVLDriver::starting_next_package(int index)
// {
//   // Clean the package array.
//   package_[0] = '\0';
//   // Starting fill it.
//   for (int j = 0; j < BUFSIZE; ++j) {
//     package_[j] = buffer_[index + j];
//   }
// }

// const unsigned char header = 0x55;
// bool NortekDVLDriver::find_header(int & index)
// {
//   if (header == buffer_[index] && header == buffer_[index + 1]) {
//     // TODO(BORGES): Consider changing the log level to RCLCPP_DEBUG
//     // RCLCPP_INFO(get_logger(), "Index of Header inside the buffer: %d.",
//     // index);
//     return true;
//   }
//   return false;
// }

void NortekDVLDriver::run() {
//   read_serial();
//   imu_data_.angular_velocity.x = *(reinterpret_cast<float *>(angular_vel_x_));
//   imu_data_.angular_velocity.y = *(reinterpret_cast<float *>(angular_vel_y_));
//   imu_data_.angular_velocity.z = *(reinterpret_cast<float *>(angular_vel_z_));
//   imu_data_.linear_acceleration.x =
//     *(reinterpret_cast<float *>(linear_accel_x_));
//   imu_data_.linear_acceleration.y =
//     *(reinterpret_cast<float *>(linear_accel_y_));
//   imu_data_.linear_acceleration.z =
//     *(reinterpret_cast<float *>(linear_accel_z_));

//   imu_data_.header.stamp = rclcpp::Clock().now();
//   imu_pub_->publish(imu_data_);
// }

// void NortekDVLDriver::open_serial()
// {
//   get_parameter("serial_port", serial_port_name_);
//   RCLCPP_INFO(
//     get_logger(), "Connecting to serial port %s",
//     serial_port_name_.c_str());

//   sockfd_ = open(serial_port_name_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
//   if (sockfd_ < 0) {
//     // TODO(BORGES): Consider changing to RCLCPP_FATAL
//     RCLCPP_ERROR(
//       get_logger(), "Could not open serial port: %s. sockfd_ = %d",
//       serial_port_name_.c_str(), sockfd_);
//     exit(EXIT_FAILURE);
//   }

//   // We want to check if it is a serial port or a binary file
//   if (serial_port_name_.substr(0, 8) == "/dev/tty") {
//     // Set serial port baudrate 115200, 8 bits, no parity, 1 stop bit
//     set_interface_attribs(B115200);
//   }
}

// void NortekDVLDriver::read_serial()
// {
//   // DEBUG: starting count time
//   auto last_update = std::chrono::system_clock::now();

//   // Sleep some time to avoid high processing
//   std::this_thread::sleep_for(std::chrono::milliseconds(5));
//   int rdlen = read(sockfd_, buffer_, BUFSIZE);

//   if (rdlen > 0) {
//     // rdlen must be the same size of BUFSIZE
//     if (rdlen < BUFSIZE) {
//       RCLCPP_ERROR(
//         get_logger(),
//         "The size of rdlen=%d and BUFSIZE=%d are different.", rdlen,
//         BUFSIZE);
//     }

//     for (int idx = 0; idx < rdlen; ++idx) {
//       if (find_header(idx)) {
//         closing_last_package(idx);
//         decode_package();
//         starting_next_package(idx);
//       }
//     }

//   } else {
//     // RCLCPP_ERROR(get_logger(),
//     //              "Could not read serial port: %s. sockfd_ = %d and rdlen =
//     //              %d", serial_port_name_.c_str(), sockfd_, rdlen);
//   }

//   // DEBUG: Calculate elapsed time
//   execution_time_ = std::chrono::duration_cast<std::chrono::milliseconds>(
//     std::chrono::system_clock::now() - last_update)
//     .count();

//   RCLCPP_DEBUG(
//     get_logger(), "Elapsed time for reading the package: %ldms.",
//     execution_time_);
// }

// void NortekDVLDriver::close_serial()
// {
//   RCLCPP_INFO(
//     get_logger(), "Closing serial port: %s. sockfd_ = %d.",
//     serial_port_name_.c_str(), sockfd_);
//   close(sockfd_);
// }
