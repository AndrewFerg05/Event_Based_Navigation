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
    initLogging(argv[0]);

    std::atomic<ThreadState> DA_state(ThreadState::Idle);
    std::atomic<ThreadState> FE_state(ThreadState::Idle);
    std::atomic<ThreadState> BE_state(ThreadState::Idle);

    size_t input_queue_size = 10; //Actually manually set in constructor
    auto data_queues = std::make_shared<DataQueues>(input_queue_size);

    std::string config_path = "../config/blank_config.yaml";

    DavisDriver driver(config_path, data_queues);   //Starts driver to add data to input queues
    LOG(INFO) << "MAIN: Driver setup...";

    //Create data interfaces
    size_t test_queue_capacity = 10;
    ThreadSafeFIFO<InputDataSync> data_DA_to_FE(test_queue_capacity, "Sync_data", true);
    
    std::shared_ptr<CommunicationManager> comms_interface = std::make_shared<CommunicationManager>(test_queue_capacity, test_queue_capacity, test_queue_capacity, test_queue_capacity);

    // Init Main Classes
    DataAcquisition DataAquisition_(data_queues, std::ref(DA_state), comms_interface);
    FrontEnd Frontend_;

    // Bind Callbacks
    DataAquisition_.registerCameraImuCallback(
        std::bind(
          static_cast<void(FrontEnd::*)(
            const std::pair<int64_t, EventArrayPtr>& stamped_events,
            const std::vector<ImuStamps>& imu_stamps_vec,
            const std::vector<ImuAccGyrContainer>& imu_accgyr_vec
            )>(&FrontEnd::processData),
          &Frontend_,
          std::placeholders::_1,
          std::placeholders::_2,
          std::placeholders::_3));

    DataAquisition_.registerImuCallback(
    std::bind(&FrontEnd::addImuData, &Frontend_,
                std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3, std::placeholders::_4));


 
    // Perform initial setup
    LOG(INFO) << "MAIN: Setting up...";

    // Initialise camera

    // Initialise WiFi
    if (CM_initNet() != 0) 
    {
        LOG(ERROR) << "MAIN: WiFi Setup Failed";
        return 0;
    }
    LOG(INFO) << "MAIN: WiFi Initialised";
    

    // // Initialise serial
    CM_serialInterface serial;
    // Prepare ESP for connection
    if (serial.ESPOpen() != 0){
        LOG(ERROR) << "MAIN: Failed to open ESP32";
    }
    else{
        LOG(INFO) << "MAIN: ESP32 Serial Initialised";
    }
 
    // Start threads
    // std::thread data_aquire_thread(DA_loop, std::ref(data_aquire_state), &data_DA_to_FE, &comms_interface);
    // std::thread frontend_thread(FE_loop, std::ref(frontend_state), &data_DA_to_FE, &comms_interface);
    // std::thread backend_thread(BE_loop, std::ref(backend_state), &comms_interface);
    
    // Start this thread
    CM_loop(std::ref(DA_state), 
            std::ref(FE_state), 
            std::ref(BE_state),
            &data_DA_to_FE,
            comms_interface,
            &serial);

    LOG(INFO) << "MAIN: CM Thread Ended";

    // Wait for other threads to exit
    // data_aquire_thread.join();
    // std::cout << "DA Thread Ended" << std::endl;
    // frontend_thread.join();
    // std::cout << "FE Thread Ended" << std::endl;
    // backend_thread.join();
    // std::cout << "BE Thread Ended" << std::endl;

    // // Perform cleanup
    LOG(INFO) << "MAIN: Cleaning up...";
    
    // // Close WiFi
    CM_cleanupNet();
    LOG(INFO) << "MAIN: WiFi closed";

    // Close ESP connection
    serial.ESPClose();
    LOG(INFO) << "MAIN: Serial closed";

    LOG(INFO) << "MAIN: Program ended";

    return 0;
}

//==============================================================================
// End of File : Software/src/main.cpp