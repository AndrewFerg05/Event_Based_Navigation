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
    #define little_endian(x) htonl(x)  // Convert byte order to little endian on Windows
#else
    #include <arpa/inet.h>
    #include <endian.h>
    #define little_endian(x) htole32(x)  // Convert byte order to little endian on Mac / Linux
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

#include "threads.hpp"

//==============================================================================
//      Classes
//------------------------------------------------------------------------------


//==============================================================================
//      Function Prototypes
//------------------------------------------------------------------------------
void thread_Communication(
    std::atomic<ThreadState>& data_sync_state,
    std::atomic<ThreadState>& frontend_state,
    std::atomic<ThreadState>& backend_state,
    interface_DA_to_FE* data_DA,
    interface_FE_to_BE* data_FE);

void C_transmit_frame(cv::Mat frame, int frame_id);

#endif  // COMMUNICATION_HPP
//==============================================================================
// End of File :  Software/include/Communication.hpp