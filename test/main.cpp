#include <iostream>
#include <vector>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>

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

// Function to find the index of the byte 0xA5 in the data
// int find_index_of_byte(const std::vector<uint8_t>& data, uint8_t byte_to_find) {
//     for (size_t i = 0; i < data.size(); ++i) {
//         if (data[i] == byte_to_find) {
//             return i;
//         }
//     }
//     return -1; // Return -1 if the byte is not found
// }

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

void read_binary_data(const char* host, int port, size_t buffer_size = 222) {
    int sock = 0;
    struct sockaddr_in serv_addr;
    char* buffer = new char[buffer_size];

    try {
        // Create socket
        if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
            throw std::runtime_error("Socket creation error");
        }

        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(port);

        // Convert IPv4 and IPv6 addresses from text to binary form
        if (inet_pton(AF_INET, host, &serv_addr.sin_addr) <= 0) {
            throw std::runtime_error("Invalid address/ Address not supported");
        }

        // Connect to the server
        if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
            throw std::runtime_error("Connection Failed");
        }

        std::cout << "Connected to " << host << ":" << port << std::endl;


        uint8_t sync = 0xA5;
        // Continuously read data from the connection
        while (true) {
            ssize_t valread = read(sock, buffer, buffer_size);
            if (valread > 0) {
                //data.insert(data.end(), buffer.begin(), buffer.begin() + valread);
                std::cout << "Received " << valread << " bytes of data." << std::endl;
                if (valread < buffer_size) continue;

                DVL dvl = decode_data(buffer);

                std::cout << "Time: " << (dvl.data.year+1900) << "-" << (dvl.data.month+1) << "-" <<
                    dvl.data.day << " " << dvl.data.hour << ":" << dvl.data.minute << ":" << dvl.data.second << std::endl;


            } else if (valread == 0) {
                std::cout << "No more data." << std::endl;
                break;
            } else {
                throw std::runtime_error("Read error");
            }

            memset(buffer, 0, buffer_size); // Fill the buffer with zeros
        }
    } catch (const std::exception &e) {
        std::cerr << "An error occurred: " << e.what() << std::endl;
    }

    delete[] buffer;
    close(sock);
}

int main() {
    const char* host = "10.7.113.50";
    int port = 9002;
    read_binary_data(host, port);
    return 0;
}

