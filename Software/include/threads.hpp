/*
Filename    : Software/include/threads.hpp
Author      : Samuel Kliskey
Project     : Event Based Navigation
Date        : 10/1/25
Description : Header file for thread functions and shared data structures
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
10-JAN-2025 SARK created to design code structure
11-JAN-2025 SARK added thread interfaces
--------------------------------------------------------------------------------
*/

#ifndef THREADS_HPP
#define THREADS_HPP

//==============================================================================
// Preprocessor Directives
//------------------------------------------------------------------------------
#ifdef _WIN32
    #include <windows.h>
    #define sleep_ms(x) Sleep(x)  // Sleep a thread in milliseconds on Windows
#else
    #include <chrono>
    #define sleep_ms(x) std::this_thread::sleep_for(std::chrono::milliseconds(x))  // Sleep a thread in other systems
#endif

//==============================================================================
// External Files
//------------------------------------------------------------------------------
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>
#include <shared_mutex>
#include <time.h>
#include <atomic>
//==============================================================================
//      Classes
// Classes defined here are the shared data structures between threads to ensure
// race condition issues.
//------------------------------------------------------------------------------

enum class ThreadState {
    Running,
    Paused,
    Stopped,
    Reset,
    Test
};

class interface_DA_to_FE
{
    private:
        std::shared_mutex mtx;
        std::vector<int> buffer;
        int indexFE = 0;
        int indexC = 0;
        //SYNCHRONISED DATA TO SEND (FRAMES / EVENTS / IMU)

    public:
        void addToBuffer(int);
        int checkBuffer();
        int checkIndex(char);
        int readBuffer(char);
        void removeFirstFromBuffer();
};

//==============================================================================
//      Function Prototypes
//------------------------------------------------------------------------------


#endif  // THREADS_HPP
//==============================================================================
// End of File :  Software/include/threads.hpp