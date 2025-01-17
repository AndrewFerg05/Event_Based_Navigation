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
#include <condition_variable>
#include <optional>

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

template<typename T>
class ThreadSafeFIFO {
private:
    std::queue<T> queue; // Queue of items and their peek status
    std::mutex queue_mutex;                      // Mutex for thread safety
    std::condition_variable data_ready;          // Condition variable for synchronization
    std::condition_variable space_available;
    bool stop = false;                           // Stop flag
    size_t max_size;

public:
    explicit ThreadSafeFIFO(size_t capacity) : max_size(capacity) {};  // Constructor
    ~ThreadSafeFIFO() = default; // Destructor

    void push(const T& value){
        std::unique_lock<std::mutex> lock(queue_mutex);
        space_available.wait(lock, [this]() { return queue.size() < max_size || stop; });
        if (stop) return;
        queue.push(value); 
        data_ready.notify_one(); 
    }
    
    std::optional<T> pop(){
        std::unique_lock<std::mutex> lock(queue_mutex);
        data_ready.wait(lock, [this]() { return !queue.empty() || stop; });
        if (queue.empty()) return std::nullopt;
        T data = queue.front(); 
        queue.pop();
        space_available.notify_one();
        return data;
    }

    std::optional<T> peek(){
        std::lock_guard<std::mutex> lock(queue_mutex);
        if (queue.empty()) return std::nullopt;
        return queue.front(); 
    }

    void stop_queue(){
        {
            std::lock_guard<std::mutex> lock(queue_mutex);
            stop = true;
        }
        data_ready.notify_all();
        space_available.notify_all();
    }

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