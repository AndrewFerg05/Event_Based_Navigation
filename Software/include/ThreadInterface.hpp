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
    std::queue<T> queue;                         // Queue of items
    std::mutex queue_mutex;                      // Mutex for thread safety
    std::condition_variable data_ready;          // Condition variable when queue is populated
    std::condition_variable space_available;     // Condition variable space to add to queue
    bool stop = false;                           // Stop flag
    size_t max_size;                             //Max queue length

public:
    explicit ThreadSafeFIFO(size_t capacity) : max_size(capacity) {}
    ~ThreadSafeFIFO() = default;

    // Add to queue
    void push(const T& value){
        std::unique_lock<std::mutex> lock(queue_mutex);
        space_available.wait(lock, [this]() { return queue.size() < max_size || stop; });   //If full wait untill space available
        if (stop) return;
        queue.push(value); 
        data_ready.notify_one();    //Notify that there is data to process
    }
    
    // Remove from queue
    std::optional<T> pop(){
        std::unique_lock<std::mutex> lock(queue_mutex);
        data_ready.wait(lock, [this]() { return !queue.empty() || stop; });     //If empty wait untill there is data added
        if (queue.empty()) return std::nullopt;
        T data = queue.front(); 
        queue.pop();
        space_available.notify_one();   //Notify there is room to add to queue
        return data;
    }

    // Read first element but don't remove
    std::optional<T> peek(){
        std::lock_guard<std::mutex> lock(queue_mutex);
        if (queue.empty()) return std::nullopt;
        return queue.front(); 
    }

    //Wake all waiting threads so they can exit gracefully
    void stop_queue(){
        {
            std::lock_guard<std::mutex> lock(queue_mutex);
            stop = true;
        }
        data_ready.notify_all();
        space_available.notify_all();
    }

};


// Draft class to be editted
// Need to address queue capacities and blocking - currently if front end cannot send data externally will block the front end
// Blocking the producer thread kinda works for processing queues but not for comms queues
class CommunicationManager {
private:
    // Queues for different data types
    ThreadSafeFIFO<InputDataSync> from_camera;
    ThreadSafeFIFO<TrackedFrames> from_frontend;
    ThreadSafeFIFO<OtherData> from_backend;
public:
    //Constructor sets size of each input queue
    CommunicationManager(size_t queue_size_1, size_t queue_size_2, size_t queue_size_3)
        : from_camera(queue_size_1),
          from_frontend(queue_size_2),
          from_backend(queue_size_3)
    {}
    ~CommunicationManager() = default;

    //Check queues and send data
    bool processQueues();

    // Add data to send queue
    void queueInputData(InputDataSync data);
    void queueTrackedFrameData(TrackedFrames data);
    void queueOther(OtherData data);

    template<typename T>
    void sendToExternal(const T& data) {
        //TODO send them externally - generic with template - templates function definitions have to go in hpp
        //std::cout << "Sending data externally: " << typeid(T).name() << std::endl;
    }
};
//==============================================================================
//      Function Prototypes
//------------------------------------------------------------------------------


#endif  // THREAD_INTERFACE_HPP
//==============================================================================
// End of File :  Software/include/ThreadInterface.hpp