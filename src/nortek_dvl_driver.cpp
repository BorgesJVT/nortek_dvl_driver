/*
 * Copyright (c) 2024, Instituto de Pesquisas da Marinha do Brasil
 * All rights reserved.
 * Written by João Victor Torres Borges <borgesjvt@gmail.com>
 */
#include <string>
#include "nortek_dvl_driver/nortek_dvl_driver.hpp"

#include <unistd.h>
#include <arpa/inet.h>

unsigned short calculate_checksum(unsigned short *pData, unsigned short size) {
  unsigned short checksum = 0xB58C;
  unsigned short nbshorts = (size >> 1);
  
  for (int i = 0; i < nbshorts; ++i) {
      checksum += *pData;
      size -= 2;
      pData++;
  }
  
  if (size > 0) {
      checksum += ((unsigned short) (*pData)) << 8;
  }
  
  return checksum;
}

DVL decode_data(char *buffer) {
  DVL dvl;
  dvl.header.sync = static_cast<uint8_t>(buffer[0]); // (unsigned) buffer[0];
  dvl.header.hdrSize = static_cast<uint8_t>(buffer[1]); // (unsigned) buffer[1];
  dvl.header.ID = static_cast<uint8_t>(buffer[2]); // (unsigned) buffer[2];
  dvl.header.family = static_cast<uint8_t>(buffer[3]); // (unsigned) buffer[3];
  dvl.header.dataSize = *(reinterpret_cast<uint16_t *>(&buffer[4])); // *((unsigned short*)&buffer[4]);
  dvl.header.dataChecksum = *(reinterpret_cast<uint16_t *>(&buffer[6])); // *((unsigned short*)&buffer[6]);
  dvl.header.hdrChecksum = *(reinterpret_cast<uint16_t *>(&buffer[8])); // // *((unsigned short*)&buffer[8]);

  dvl.data.version = static_cast<uint8_t>(buffer[10]); // (unsigned) buffer[10];
  dvl.data.offsetOfData = static_cast<uint8_t>(buffer[11]); // (unsigned) buffer[11];
  dvl.data.serialNumber = *(reinterpret_cast<uint32_t *>(&buffer[12])); // *((unsigned long*)&buffer[12]);
  dvl.data.year = static_cast<uint8_t>(buffer[16]); // (unsigned) buffer[16];
  dvl.data.month = static_cast<uint8_t>(buffer[17]); // (unsigned) buffer[17];
  dvl.data.day = static_cast<uint8_t>(buffer[18]); // (unsigned) buffer[18]; 
  dvl.data.hour = static_cast<uint8_t>(buffer[19]); // (unsigned) buffer[19];
  dvl.data.minute = static_cast<uint8_t>(buffer[20]); // (unsigned) buffer[20];
  dvl.data.second = static_cast<uint8_t>(buffer[21]); // (unsigned) buffer[21];
  dvl.data.microSecond100 =  *(reinterpret_cast<uint16_t *>(&buffer[22])); // *((unsigned short*)&buffer[22]);
  dvl.data.nBeams = *(reinterpret_cast<uint16_t *>(&buffer[24])); // *((unsigned short*)&buffer[24]);
  dvl.data.error = *(reinterpret_cast<uint32_t *>(&buffer[26])); // *((unsigned long*)&buffer[26]);
  dvl.data.status = *(reinterpret_cast<DVLStatus_t *>(&buffer[30])); // *((t_DVLstatus*)&buffer[30]);
  dvl.data.soundSpeed = *(reinterpret_cast<float *>(&buffer[34])); // *((float*)&buffer[34]);
  dvl.data.temperature = *(reinterpret_cast<float *>(&buffer[38])); // *((float*)&buffer[38]);
  dvl.data.pressure = *(reinterpret_cast<float *>(&buffer[42])); // *((float*)&buffer[42]);

  /* Beam data */
  for (int i = 0; i < 4; i++) {
      dvl.data.velBeam[i] = *(reinterpret_cast<float *>(&buffer[46 + i*4])); // *((float*)&buffer[46 + i*4]);
      dvl.data.distBeam[i] = *(reinterpret_cast<float *>(&buffer[62 + i*4])); // *((float*)&buffer[62 + i*4]);
      dvl.data.fomBeam[i] = *(reinterpret_cast<float *>(&buffer[78 + i*4])); // *((float*)&buffer[78 + i*4]);
      dvl.data.timeDiff1Beam[i] = *(reinterpret_cast<float *>(&buffer[94 + i*4])); // *((float*)&buffer[94 + i*4]);
      dvl.data.timeDiff2Beam[i] = *(reinterpret_cast<float *>(&buffer[110 + i*4])); // *((float*)&buffer[110 + i*4]);
      dvl.data.timeVelEstBeam[i] = *(reinterpret_cast<float *>(&buffer[126 + i*4])); // *((float*)&buffer[126 + i*4]);
  }

  /* XYZ data */
  dvl.data.velX = *(reinterpret_cast<float *>(&buffer[142])); // *((float*)&buffer[142]);
  dvl.data.velY = *(reinterpret_cast<float *>(&buffer[146])); // *((float*)&buffer[146]);
  dvl.data.velZ1 = *(reinterpret_cast<float *>(&buffer[150])); // *((float*)&buffer[150]);
  dvl.data.velZ2 = *(reinterpret_cast<float *>(&buffer[154])); // *((float*)&buffer[154]);
  dvl.data.fomX = *(reinterpret_cast<float *>(&buffer[158])); // *((float*)&buffer[158]);
  dvl.data.fomY = *(reinterpret_cast<float *>(&buffer[162])); // *((float*)&buffer[162]);
  dvl.data.fomZ1 = *(reinterpret_cast<float *>(&buffer[166])); // *((float*)&buffer[166]);
  dvl.data.fomZ2 = *(reinterpret_cast<float *>(&buffer[170])); // *((float*)&buffer[170]);
  dvl.data.timeDiff1X = *(reinterpret_cast<float *>(&buffer[174])); // *((float*)&buffer[174]);
  dvl.data.timeDiff1Y = *(reinterpret_cast<float *>(&buffer[178])); // *((float*)&buffer[178]);
  dvl.data.timeDiff1Z2 = *(reinterpret_cast<float *>(&buffer[182])); // *((float*)&buffer[182]);
  dvl.data.timeDiff1Z1 = *(reinterpret_cast<float *>(&buffer[186])); // *((float*)&buffer[186]);
  dvl.data.timeDiff2X = *(reinterpret_cast<float *>(&buffer[190])); // *((float*)&buffer[190]);
  dvl.data.timeDiff2Y = *(reinterpret_cast<float *>(&buffer[194])); // *((float*)&buffer[194]);
  dvl.data.timeDiff2Z1 = *(reinterpret_cast<float *>(&buffer[198])); // *((float*)&buffer[198]);
  dvl.data.timeDiff2Z2 = *(reinterpret_cast<float *>(&buffer[202])); // *((float*)&buffer[202]);
  dvl.data.timeVelEstX = *(reinterpret_cast<float *>(&buffer[206])); // *((float*)&buffer[206]);
  dvl.data.timeVelEstY = *(reinterpret_cast<float *>(&buffer[210])); // *((float*)&buffer[210]);
  dvl.data.timeVelEstZ1 = *(reinterpret_cast<float *>(&buffer[214])); // *((float*)&buffer[214]);
  dvl.data.timeVelEstZ2 = *(reinterpret_cast<float *>(&buffer[218])); // *((float*)&buffer[218]);

  return dvl;
}

/* NortekDVLDriver Constructor */
NortekDVLDriver::NortekDVLDriver()
: Node("nortek_dvl_driver_node")
{
  this->declare_parameter<std::string>("ip_address", "192.168.113.50");
  dvl_pub_ = this->create_publisher<dvl_msgs::msg::Dvl>("/nortek/dvl", 10);
  dvl_data_.header.frame_id = "dvl_link";

  // Access the parameter value and Run
  std::string ip_address = this->get_parameter("ip_address").as_string();;
  this->run(ip_address.c_str());
}

/* NortekDVLDriver Destructor */
NortekDVLDriver::~NortekDVLDriver() {
  // delete[] buffer;
  // close(sock);
}


void NortekDVLDriver::run(const char* ip_addr) {
  int sock = 0;
  struct sockaddr_in serv_addr;
  char* buffer = new char[BUFSIZE];

  // Create socket
  if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    throw std::runtime_error("Socket creation error");
  }

  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(PORT);

  // Convert IPv4 and IPv6 addresses from text to binary form
  if (inet_pton(AF_INET, ip_addr, &serv_addr.sin_addr) <= 0) {
    throw std::runtime_error("Invalid address/ Address not supported");
  }

  // Connect to the server
  if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
    //throw std::runtime_error("Connection Failed");
    RCLCPP_ERROR(this->get_logger(), "Connection Failed");
  }

  RCLCPP_INFO(this->get_logger(), "Connected to: %s", ip_addr);

  // Continuously read data from the connection
  while (rclcpp::ok()) {
    ssize_t valread = read(sock, buffer, BUFSIZE);
    if (valread > 0) {
      RCLCPP_DEBUG(this->get_logger(), "Received %d bytes of data.", valread);
      if (valread < BUFSIZE) continue;

      DVL dvl = decode_data(buffer);

      // std::cout << "Time: " << (dvl.data.year+1900) << "-" << (dvl.data.month+1) << "-" <<
      //     dvl.data.day << " " << dvl.data.hour << ":" << dvl.data.minute << ":" << dvl.data.second << std::endl;
      RCLCPP_INFO(this->get_logger(), "Time: %u:%u:%u", dvl.data.hour, dvl.data.minute, dvl.data.second);

      dvl_data_.velocity.x = dvl.data.velX;
      dvl_data_.velocity.y = dvl.data.velY;
      dvl_data_.velocity.z = dvl.data.velZ1;
      dvl_pub_->publish(dvl_data_);

    } else if (valread == 0) {
      RCLCPP_INFO(this->get_logger(), "No more data.");
      break;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Read error");
    }

    memset(buffer, 0, BUFSIZE); // Fill the buffer with zeros
  } // end while
  delete[] buffer;
  close(sock);
} // end run
