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
    #define sleep_us(x) Sleep(1)
#else
    #include <chrono>
    #define sleep_ms(x) std::this_thread::sleep_for(std::chrono::milliseconds(x))  // Sleep a thread in other systems
    #define sleep_us(x) std::this_thread::sleep_for(std::chrono::microseconds(x))  // Sleep a thread in other systems
#endif

//==============================================================================
// External Files
//------------------------------------------------------------------------------
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>
#include <shared_mutex>
#include <deque>
#include <time.h>
#include <atomic>
#include <condition_variable>
#include <optional>

#include "TypeAliases.hpp"
#include "Types.hpp"
#include "Flags.hpp"
#include "Logging.hpp"


//==============================================================================
//      Development Global variables
//------------------------------------------------------------------------------
volatile bool FLAG_CONNECTED_ESP    = 0;
volatile bool FLAG_CONNECTED_DAVIS  = 0;


//==============================================================================
//      Classes
//------------------------------------------------------------------------------

enum class ThreadState {
    Run,
    Stop,
    Idle
};

template<typename T>
class ThreadSafeFIFO {
private:
    std::deque<T> queue;                // Using deque for efficient removal from front
    std::mutex queue_mutex;             // Mutex for thread safety
    std::condition_variable data_ready; // Condition variable to notify data availability
    bool stop = false;                  // Stop flag
    size_t max_size;                    // Max queue length
    std::string queue_name;             //Queue name for debug
    bool sleep_if_empty;                // Determines if consumers should sleep if empty

public:
    explicit ThreadSafeFIFO(size_t capacity, std::string name = "UnnamedQueue", bool sleep = false) 
        : max_size(capacity),
        sleep_if_empty(sleep),
        queue_name(std::move(name)) {}

    // Add to queue (replace oldest if full)
    void push(const T& value) {
        std::lock_guard<std::mutex> lock(queue_mutex);
        if (queue.size() >= max_size) {
            LOG(WARNING) << "Queue overflow: " << queue_name;
            queue.pop_front();  // Remove oldest element
        }
        queue.push_back(value);
        data_ready.notify_one(); // Notify that data is available
    }
    
    // Remove from queue
    std::optional<T> pop() {
        std::unique_lock<std::mutex> lock(queue_mutex);
        if (sleep_if_empty) {
            data_ready.wait(lock, [this]() { return !queue.empty() || stop; });  // Blocking wait
        }
        if (queue.empty()) return std::nullopt;  // If stopped, return nothing
        T data = queue.front();
        queue.pop_front();
        return data;
    }

    // Read first element but don't remove
    std::optional<T> peek() {
        std::lock_guard<std::mutex> lock(queue_mutex);
        if (queue.empty()) return std::nullopt;
        return queue.front();
    }

    // Stop the queue gracefully
    void stop_queue() {
        {
            std::lock_guard<std::mutex> lock(queue_mutex);
            stop = true;
        }
        data_ready.notify_all();
    }

    void clear() {
    std::lock_guard<std::mutex> lock(queue_mutex);
    queue.clear();  // Clear the deque
    }

    // Check if queue is empty
    bool empty() {
        std::lock_guard<std::mutex> lock(queue_mutex);
        return queue.empty();
    }

    // Get the current queue size
    size_t size() {
        std::lock_guard<std::mutex> lock(queue_mutex);
        return queue.size();
    }
};

// Data Queus Class to mimic ROS Topics
class DataQueues {
public:
    // Queues for different data types
    std::shared_ptr<ThreadSafeFIFO<EventData>> event_queue;
    std::shared_ptr<ThreadSafeFIFO<CameraInfoData>> camera_info_queue;
    std::shared_ptr<ThreadSafeFIFO<IMUData>> imu_queue;
    std::shared_ptr<ThreadSafeFIFO<ImageData>> image_queue;
    std::shared_ptr<ThreadSafeFIFO<ExposureData>> exposure_queue;

    // Constructor
    explicit DataQueues(size_t queue_size) {
        event_queue = std::make_shared<ThreadSafeFIFO<EventData>>(1000, "Input_DVS", false);          // Events buffer (10)
        camera_info_queue = std::make_shared<ThreadSafeFIFO<CameraInfoData>>(queue_size, "Input_CamInfo", false);        // Camera info buffer (1)
        imu_queue = std::make_shared<ThreadSafeFIFO<IMUData>>(1000, "Input_IMU", false);              // IMU buffer (10)
        image_queue = std::make_shared<ThreadSafeFIFO<ImageData>>(30, "Input_APS", false);                   // Images buffer (1)
        exposure_queue = std::make_shared<ThreadSafeFIFO<ExposureData>>(1, "Input_Expo", false);    // Exposure buffer (10)
    }

};

// Draft class to be editted
class CommunicationManager 
{
    private:
        // Queues for different data types
        ThreadSafeFIFO<cv::Mat> framesCamera;
        ThreadSafeFIFO<cv::Mat> framesEvents;
        ThreadSafeFIFO<cv::Mat> framesAugmented;
        ThreadSafeFIFO<OtherData> pose;
    public:
        //Constructor sets size of each input queue
        CommunicationManager(size_t queue_size_1, size_t queue_size_2, size_t queue_size_3, size_t queue_size_4)
            : framesCamera(queue_size_1, "Comms_1", false),
            framesEvents(queue_size_2, "Comms_2", false),
            framesAugmented(queue_size_3, "Comms_3", false),
            framesAugmented(queue_size_4, "Comms_4", false)
        {}
        ~CommunicationManager() = default;

        //Check queues and send data
        cv::Mat getFrameCamera();
        cv::Mat getFrameEvents();
        cv::Mat getFrameAugmented();
        OtherData getPose();

        // Add data to send queue
        void queueFrameCamera(cv::Mat data);
        void queueFrameEvents(cv::Mat data);
        void queueFrameAugmented(cv::Mat data);
        void queuePose(OtherData data);
    };
//==============================================================================
//      Function Prototypes
//------------------------------------------------------------------------------


#endif  // THREAD_INTERFACE_HPP
//==============================================================================
// End of File :  Software/include/ThreadInterface.hpp