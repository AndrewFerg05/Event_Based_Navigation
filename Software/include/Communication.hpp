/*
Filename    : Software/include/Communication.hpp
Author      : Samuel Kliskey
Project     : Event Based Navigation
Date        : 11/1/25
Description : Header file for the communication (C) thread
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
11-JAN-2025 SARK created to design code structure
--------------------------------------------------------------------------------
*/

#ifndef COMMUNICATION_HPP
#define COMMUNICATION_HPP

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib") // Link with the winsock library
    #define little_endian(x) (x)  // Convert byte order to little endian on Windows
#elif __APPLE__
    #include <arpa/inet.h>
    #include <libkern/OSByteOrder.h>
    #define little_endian(x) OSSwapHostToLittleInt32(x) // Convert byte order to little endian on macOS
#else
    #include <arpa/inet.h>
    #include <endian.h>
    #define little_endian(x) htole32(x) // Convert byte order to little endian on Linux
#endif

//==============================================================================
// External Files
//------------------------------------------------------------------------------
//#include <arpa/inet.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

#include <cstring>
#include <cstdlib>
#include <unistd.h>

#include <libserialport.h>

#include "DataAcquisition.hpp"
#include "FrontEnd.hpp"
#include "DavisDriver.hpp"
#include "ThreadInterface.hpp"
#include "TypeAliases.hpp"
#include "Types.hpp"
#include "Logging.hpp"

//==============================================================================
//      Classes
//------------------------------------------------------------------------------
class CM_serialInterface{
    private:
        struct sp_port *ESPPort = NULL;
        int timeout = 50;
        bool open = 0;
    public:
        bool ESPCheckOpen() {return this->open;}
    CM_serialInterface() = default;
    ~CM_serialInterface() = default;
        bool ESPOpen();
        void ESPClose();
        int ESPCheckBuffer() {return sp_input_waiting(this->ESPPort);}
        bool ESPWrite(char* message);
        char* ESPRead();

};

//==============================================================================
//      Function Prototypes
//------------------------------------------------------------------------------
cv::Mat CM_formatCameraFrame(ImageData image);

cv::Mat CM_formatEventFrame(TrackedFrames image);

int CM_initNet();

void CM_cleanupNet();

std::uint8_t CM_serialReceive(CM_serialInterface* serial);

void CM_serialSendState(CM_serialInterface* serial, int32_t state);

void CM_serialSendStatus(CM_serialInterface* serial, int32_t x, int32_t y);

void CM_transmitStatus(int32_t x, int32_t y, int32_t z, int32_t yaw, int32_t pitch, int32_t roll);

void CM_loop(
    std::shared_ptr<DavisDriver> driver_,
    std::shared_ptr<DataAcquisition> dataAcquistion_,
    std::shared_ptr<FrontEnd> frontEnd_,
    std::shared_ptr<CommunicationManager> comms,
    CM_serialInterface* serial);

void CM_transmitFrame(cv::Mat frame, int frame_id);


#endif  // COMMUNICATION_HPP
//==============================================================================
// End of File :  Software/include/Communication.hpp