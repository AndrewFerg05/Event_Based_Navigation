/*
Filename    : Software/src/DataAcquisition.cpp
Author      : Samuel Kliskey
Project     : Event Based Navigation
Date        : 11/1/25
Description : Data Acquisition code for getting DAVIS346 camera data
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
11-JAN-2025 SARK created to design code structure
--------------------------------------------------------------------------------
*/

//==============================================================================
// External Files
//------------------------------------------------------------------------------

// Local
#include "DataAcquisition.hpp"

//==============================================================================
// Function Prototypes
//------------------------------------------------------------------------------

//==============================================================================
// MACROs
//------------------------------------------------------------------------------

//==============================================================================
// Global Variable Initialisation
//------------------------------------------------------------------------------


//==============================================================================
// Functions
//------------------------------------------------------------------------------


DataAcquisition::DataAcquisition(std::shared_ptr<DataQueues> data_queues, std::atomic<ThreadState>& state, std::shared_ptr<CommunicationManager> comms)
    : input_data_queues_(data_queues), state_(state), comms_interface_(comms) 
    {
        initBuffers();
    }

DataAcquisition::~DataAcquisition() {
    stop();
}

void DataAcquisition::start() {
    if (acquisition_thread_.joinable()) return;  // Prevent multiple starts
    running_ = true;
    state_ = ThreadState::Run;
    acquisition_thread_ = std::thread(&DataAcquisition::run, this);
}

void DataAcquisition::idle() {
    std::cout << "[DataAcquisition] Resetting queues and entering idle mode..." << std::endl;
    resetQueues();
    imu_buffer_.clear();
    state_ = ThreadState::Idle;
}

void DataAcquisition::stop() {
    running_ = false;
    state_ = ThreadState::Stop;
    if (acquisition_thread_.joinable()) {
        acquisition_thread_.join();
    }
    std::cout << "[DataAcquisition] Stopped." << std::endl;
}

void DataAcquisition::initBuffers() 
{
    imu_buffer_.clear(); 
}


void DataAcquisition::addImageData()
{

}


void DataAcquisition::addEventsData()
{

}

void DataAcquisition::addImuData(const IMUData& imu_data)
{
    const Vector3 gyr(
    imu_data.angular_velocity.x,
    imu_data.angular_velocity.y,
    imu_data.angular_velocity.z);
    const Vector3 acc(
    imu_data.linear_acceleration.x,
    imu_data.linear_acceleration.y,
    imu_data.linear_acceleration.z);
    int64_t stamp = imu_data.header.stamp;

    Vector6 acc_gyr;
    acc_gyr.head<3>() = acc;
    acc_gyr.tail<3>() = gyr;
    //stamp -= timeshift_cam_imu_;
    imu_buffer_.insert(stamp, acc_gyr);

    if (imu_callback_)
    {
        imu_callback_(stamp, acc, gyr);
    }

  checkImuDataAndImageAndEventsCallback();
}

bool DataAcquisition::processDataQueues()
{
        bool processed = false;

        auto imu_data = input_data_queues_->imu_queue->pop();
        if (imu_data.has_value()) {
            addImuData(imu_data.value());
            processed = true;
        }

        auto event_data = input_data_queues_->event_queue->pop();
        if (event_data) {
            addEventsData();
            processed = true;
        }

        auto image_data = input_data_queues_->image_queue->pop();
        if (image_data) {
            addImageData();
            processed = true;
        }

        return processed;

}

void DataAcquisition::resetQueues()
{
    if (!input_data_queues_) {
        std::cerr << "[ERROR] Data queues not initialized!" << std::endl;
        return;
    }

    std::cout << "[INFO] Resetting all data queues..." << std::endl;
    
    input_data_queues_->event_queue->clear();
    input_data_queues_->camera_info_queue->clear();
    input_data_queues_->imu_queue->clear();
    input_data_queues_->image_queue->clear();
    input_data_queues_->exposure_queue->clear();

    std::cout << "[INFO] All queues cleared!" << std::endl;
}


void DataAcquisition::extractAndEraseEvents()
{

}

void DataAcquisition::checkImuDataAndImageAndEventsCallback()
{

}


void DataAcquisition::run() 
{
    while (running_) 
    {
        if (state_ == ThreadState::Idle) 
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));  // Avoid busy waiting
            continue;
        }

        if (state_ == ThreadState::Stop) 
        {
            running_ = false;
            break;
        }

        if (state_ ==ThreadState::Run)
        {
            if (!processDataQueues()) 
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));  // No data, sleep briefly
            }

        }
    }
}






void DA_loop(std::atomic<ThreadState>& state,
                            ThreadSafeFIFO<InputDataSync>* data_DA,
                            CommunicationManager* comms) {
    
    InputDataSync valueToAdd = 0;

    // while (true) {
    //     if (state == ThreadState::Stopped) {
    //         break;
    //     }

    //     if (state == ThreadState::Paused) {
    //         //TODO - Wait while some condition
    //         sleep_ms(100);
    //         continue;
    //     }

    //     if (state == ThreadState::Reset) {
    //         //TODO call reset function then set running again
    //         state = ThreadState::Running; 
    //     }

    //     if (state == ThreadState::Running) {
    //         //TODO - Get data from camera

    //         //Get data
    //         valueToAdd++;                       //For Sam architecture testing (replace with actual frames)

    //         //Synchronise data

    //         //Put in buffer
    //         data_DA->push(valueToAdd);   //For Sam architecture testing (replace with actual frames)
    //         comms->queueInputData(valueToAdd); 
    //         sleep_ms(25);
    //     }

    //     if (state == ThreadState::Test) {
    //         sleep_ms(100);
    //         std::cout << "Data Aquisition Testing" << std::endl; 
    //     }
    // }
}
//==============================================================================
// End of File : Software/src/DataAcquisition.cpp
