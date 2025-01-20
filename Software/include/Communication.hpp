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

#include "ThreadInterface.hpp"
#include "TypeAliases.hpp"
//==============================================================================
//      Classes
//------------------------------------------------------------------------------


//==============================================================================
//      Function Prototypes
//------------------------------------------------------------------------------
int CM_initNet();

void CM_cleanupNet();


void CM_loop(
    std::atomic<ThreadState>& data_sync_state,
    std::atomic<ThreadState>& frontend_state,
    std::atomic<ThreadState>& backend_state,
    ThreadSafeFIFO<InputDataSync>* data_DA,
    CommunicationManager* comms);

void CM_transmitFrame(cv::Mat frame, int frame_id);

#endif  // COMMUNICATION_HPP
//==============================================================================
// End of File :  Software/include/Communication.hpp