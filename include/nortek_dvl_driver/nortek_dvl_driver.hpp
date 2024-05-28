/*
 * Copyright (c) 2024, Instituto de Pesquisas da Marinha do Brasil
 * All rights reserved.
 * Written by João Victor Torres Borges <borgesjvt@gmail.com>
*/

#ifndef PRIMUS_IMU_DRIVER__PRIMUS_HPP_
#define PRIMUS_IMU_DRIVER__PRIMUS_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "dvl_msgs/msg/dvl.hpp"

#define PORT 9002
#define BUFSIZE 222

typedef struct {
    unsigned long beam1VelValid : 1; // BIT (0)
    unsigned long beam2VelValid : 1; // BIT (1)
    unsigned long beam3VelValid : 1; // BIT (2)
    unsigned long beam4VelValid : 1; // BIT (3)
    unsigned long beam1DistValid : 1; // BIT (4)
    unsigned long beam2DistValid : 1; // BIT (5)
    unsigned long beam3DistValid : 1; // BIT (6)
    unsigned long beam4DistValid : 1; // BIT (7)
    unsigned long beam1FOMValid : 1; // BIT (8)
    unsigned long beam2FOMValid : 1; // BIT (9)
    unsigned long beam3FOMValid : 1; // BIT (10)
    unsigned long beam4FOMValid : 1; // BIT (11)
    unsigned long xVelValid : 1; // BIT (12)
    unsigned long yVelValid : 1; // BIT (13)
    unsigned long z1VelValid : 1; // BIT (14)
    unsigned long z2VelValid : 1; // BIT (15)
    unsigned long xFOMValid : 1; // BIT (16)
    unsigned long yFOMValid : 1; // BIT (17)
    unsigned long z1FOMValid : 1; // BIT (18)
    unsigned long z2FOMValid : 1; // BIT (19)
    unsigned long procIdle3 : 1; // BIT (20)
    unsigned long procIdle6 : 1; // BIT (21)
    unsigned long procIdle12 : 1; // BIT (22)
    unsigned long _empty1 : 5; // BIT (23-27)
    unsigned long wakeupstate : 4; // BIT (28-31)
} DVLStatus_t;

typedef struct {
    uint16_t sync;              ///< 0
    uint16_t hdrSize;           ///< 1
    uint16_t ID;                ///< 2
    uint16_t family;            ///< 3
    uint16_t dataSize;         ///< 4-5
    uint16_t dataChecksum;     ///< 6-7
    uint16_t hdrChecksum;      ///< 8-9
} DVLHeader_t;                   ///< 10 bytes

typedef struct {
    uint16_t version;           ///< 10
    uint16_t offsetOfData;      ///< 11
    uint32_t serialNumber;      ///< 12-15
    uint16_t year;              ///< 16
    uint16_t month;             ///< 17
    uint16_t day;               ///< 18
    uint16_t hour;              ///< 19
    uint16_t minute;            ///< 20
    uint16_t second;           ///< 21
    uint16_t microSecond100;  ///< 22-23
    uint16_t nBeams;           ///< 24-25
    uint32_t error;             ///< 26-29
    DVLStatus_t status;            ///< 30-33
    float soundSpeed;                ///< 34-37 < [m/s]
    float temperature;               ///< 38-41 < [Celsius] 
    float pressure;                  ///< 42-45 < [Bar]
    
    /* Beam data */
    float velBeam[4];                ///< 46-61 < Velocities for each beam. [m/s]
    float distBeam[4];               ///< 62-77 < Distance for each beam. [m]
    float fomBeam[4];                ///< 78-93 < FOM for each beam. [m/s]
    float timeDiff1Beam[4];          ///< 94-109 < DT1 for each beam. [s]
    float timeDiff2Beam[4];          ///< 110-125 < DT2 for each beam. [s]
    float timeVelEstBeam[4];         ///< 126-141 < Duration of velocity estimate for each beam. [s]
    
    /* XYZ data */
    float velX;                       ///< 142-145 < Velocity X. [m/s]
    float velY;                       ///< 146-149 < Velocity Y. [m/s]
    float velZ1;                      ///< 150-153 < Velocity Z1. [m/s]
    float velZ2;                      ///< 154-157 < Velocity Z2. [m/s]
    float fomX;                       ///< 158-161 < FOM X. [m/s]
    float fomY;                       ///< 162-165 < FOM Y. [m/s]
    float fomZ1;                      ///< 166-169 < FOM Z1. [m/s]
    float fomZ2;                      ///< 170-173 < FOM Z2. [m/s]
    float timeDiff1X;                 ///< 174-177 < DT1 X. [s]
    float timeDiff1Y;                 ///< 178-181 < DT1 Y. [s]
    float timeDiff1Z1;                ///< 182-185 < DT1 Z1. [s]
    float timeDiff1Z2;                ///< 186-189 < DT1 Z2. [s]
    float timeDiff2X;                 ///< 190-193 < DT2 X. [s]
    float timeDiff2Y;                 ///< 194-197 < DT2 Y. [s]
    float timeDiff2Z1;                ///< 198-201 < DT2 Z1. [s]
    float timeDiff2Z2;                ///< 202-205 < DT2 Z2. [s]
    float timeVelEstX;                ///< 206-209 < Duration of velocity estimate for each component. [s]
    float timeVelEstY;                ///< 210-213 < Duration of velocity estimate for each component. [s]
    float timeVelEstZ1;               ///< 214-217 < Duration of velocity estimate for each component. [s]
    float timeVelEstZ2;               ///< 218-221 < Duration of velocity estimate for each component. [s]
} DVLDataFormat21_t;                      ///< 210 bytes

typedef struct {
    DVLHeader_t header;
    DVLDataFormat21_t data;
} DVL;

class NortekDVLDriver : public rclcpp::Node
{
public:
  NortekDVLDriver();
  ~NortekDVLDriver();
  void run(const char* host);

private:
  rclcpp::Publisher<dvl_msgs::msg::Dvl>::SharedPtr dvl_pub_;      // DVL ROS Publisher
  rclcpp::TimerBase::SharedPtr timer_;  // Timer to define callback frequency
  dvl_msgs::msg::Dvl dvl_data_;      // DVL ROS data
};
#endif  // PRIMUS_IMU_DRIVER__PRIMUS_HPP_
