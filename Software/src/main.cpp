/*
Filename    : Software/src/main.cpp
Author      : Samuel Kliskey
Project     : Event Based Navigation
Date        : 10/1/25
Description : Main file of the project
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
10-JAN-2025     SARK    created to design code structure
10-JAN-2025     AF      Added Libcaer test
12-Jan-2025     AF      Changed to Atomics
--------------------------------------------------------------------------------
Prefix Conventions
DA - Data Aquisition
FE - Front End
BE - Back end
CM - Communication
*/

//==============================================================================
// External Files
//------------------------------------------------------------------------------
// External
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>
#include <iostream>
#include <Eigen/Dense>

// Local
#include  "ThreadInterface.hpp"
#include  "BackEnd.hpp"
#include  "Communication.hpp"
#include  "DataAcquisition.hpp"
#include  "FrontEnd.hpp"
#include "DavisDriver.hpp"
#include "Logging.hpp"


//==============================================================================
// Function Prototypes
//------------------------------------------------------------------------------

//==============================================================================
// MACROs
//------------------------------------------------------------------------------
#define THIS_IS_MACRO_EXAMPLE   15

//==============================================================================
// Global Variable Initialisation
//------------------------------------------------------------------------------

//==============================================================================
// Functions
//------------------------------------------------------------------------------
int main(int argc, char* argv[]) 
{
    static AlignedLogSink sink;
    initLogging(argv[0], &sink);


    size_t input_queue_size = 100; //Actually manually set in constructor
    auto data_queues = std::make_shared<DataQueues>(input_queue_size);
    std::string config_path = "../config/camera_configuration.yaml";
    
    std::shared_ptr<DavisDriver> driver = std::make_shared<DavisDriver>(config_path, data_queues);   //Starts driver to add data to input queues

    
    // Create atomic control flags
    // std::atomic<ThreadState> frontend_state(ThreadState::Idle);
    // std::atomic<ThreadState> backend_state(ThreadState::Idle);

    // //Create data interfaces
    size_t test_queue_capacity = 100;
    // ThreadSafeFIFO<InputDataSync> data_DA_to_FE(test_queue_capacity, "Sync_data", true);
    
    std::shared_ptr<CommunicationManager> comms_interface = std::make_shared<CommunicationManager>(test_queue_capacity, test_queue_capacity, test_queue_capacity);

    // Perform initial setup
    // LOG(INFO) << "MAIN: Setting up...";

    // Initialise camera

    // Initialise WiFi
    // if (CM_initNet() != 0) 
    // {
    //     LOG(ERROR) << "MAIN: WiFi Setup Failed";
    //     return 0;
    // }
    // LOG(INFO) << "MAIN: WiFi Initialised";
    

    // // Initialise serial
    // CM_serialInterface serial;
    // Prepare ESP for connection
    // if (serial.ESPOpen() != 0){
    //     LOG(ERROR) << "MAIN: Failed to open ESP32";
    // }
    // else{
    //     LOG(INFO) << "MAIN: ESP32 Serial Initialised";
    // }
 
    // Start threads
    std::shared_ptr<DataAcquisition> DataAquistion_ = std::make_shared<DataAcquisition>(data_queues, comms_interface);

    std::shared_ptr<FrontEnd> FrontEnd_ = std::make_shared<FrontEnd>(comms_interface);

    DataAquistion_->registerCameraImuCallback(
        std::bind(
          static_cast<void(FrontEnd::*)(
            const StampedImage&    /*image*/,
            const std::pair<int64_t, EventArrayPtr>& stamped_events,
            const ImuStamps& imu_stamps,
            const ImuAccGyrContainer& imu_accgyr,
            const bool& no_motion_prior
            )>(&FrontEnd::addData),
            FrontEnd_.get(),
          std::placeholders::_1,
          std::placeholders::_2,
          std::placeholders::_3,
          std::placeholders::_4,
          std::placeholders::_5));

    DataAquistion_->registerImuCallback(
        std::bind(&FrontEnd::addImuData, FrontEnd_.get(),
                std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3));


    driver->start();
    DataAquistion_->start();

    auto start_time = std::chrono::steady_clock::now();  // Capture the start time
    bool triggered_3s = false, triggered_6s = false, triggered_9s = false, triggered_12s = false;

    DataAquistion_->start();
    
    while (true) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
    
        if (elapsed_time >= 3 && !triggered_3s) {
            std::cout << "3 seconds elapsed! Switching to idle..." << std::endl;
            DataAquistion_->idle();
            triggered_3s = true;
        }
    
        if (elapsed_time >= 6 && !triggered_6s) {
            std::cout << "6 seconds elapsed! Resuming data acquisition..." << std::endl;
            DataAquistion_->start();
            triggered_6s = true;
        }
    
        if (elapsed_time >= 9 && !triggered_9s) {
            std::cout << "9 seconds elapsed! Switching to idle..." << std::endl;
            DataAquistion_->idle();
            triggered_9s = true;
        }
    
        if (elapsed_time >= 12 && !triggered_12s) {
            std::cout << "12 seconds elapsed! Resuming data acquisition..." << std::endl;
            DataAquistion_->start();
            triggered_12s = true;
        }
    
        if (elapsed_time >= 15) {
            std::cout << "Stopping timer loop." << std::endl;
            DataAquistion_->stop();
            break;
        }
    }
    
  
    
    // Start this thread
    // CM_loop(std::ref(data_aquire_state), 
    //         std::ref(frontend_state), 
    //         std::ref(backend_state),
    //         &data_DA_to_FE,
    //         &DataAquistion_,
    //         comms_interface,
    //         &serial);


    // LOG(INFO) << "MAIN: CM Thread Ended";

    // // Wait for other threads to exit
    // DataAquistion_.stop();
    // LOG(INFO) << "MAIN: DA Thread Ended";

    // // // Close WiFi
    // CM_cleanupNet();
    // LOG(INFO) << "MAIN: WiFi closed";

    // // Close ESP connection
    // serial.ESPClose();
    // LOG(INFO) << "MAIN: Serial closed";

    LOG(INFO) << "MAIN: Program ended";
    endLogging(&sink);

    return 0;
}

//==============================================================================
// End of File : Software/src/main.cpp