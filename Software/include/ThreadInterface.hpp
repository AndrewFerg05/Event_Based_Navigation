/*
Filename    : Software/include/ThreadInterface.hpp
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

#ifndef THREAD_INTERFACE_HPP
#define THREAD_INTERFACE_HPP

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
#include <queue>
#include <time.h>
#include <atomic>

#include "TypeAliases.hpp"
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

class interface_DA_to_FE {
private:
    std::queue<std::pair<InputDataSync, bool>> queue; // Queue of items and their peek status
    std::mutex queue_mutex;                      // Mutex for thread safety
    std::condition_variable data_ready;          // Condition variable for synchronization
    bool stop = false;                           // Stop flag
    int frames_dropped = 0;                        // Counter for missed peeks

public:
    interface_DA_to_FE();  // Constructor
    ~interface_DA_to_FE(); // Destructor

    void push(const InputDataSync& value);
    std::optional<InputDataSync> pop();
    std::optional<InputDataSync> peek();
    void stop_queue();
    int get_frame_drop_count();
};

class interface_FE_to_BE
{
    private:
        std::shared_mutex mtx;
        std::vector<int> buffer;
        int indexBE = 0;
        int indexC = 0;
        //SYNCHRONISED DATA TO SEND (EVENTS / IMU)

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


#endif  // THREAD_INTERFACE_HPP
//==============================================================================
// End of File :  Software/include/ThreadInterface.hpp