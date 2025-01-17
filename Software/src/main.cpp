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
#include <libcaer/libcaer.h>
#include <libcaer/devices/device_discover.h>

// Local
#include  "ThreadInterface.hpp"
#include  "BackEnd.hpp"
#include  "Communication.hpp"
#include  "DataAcquisition.hpp"
#include  "FrontEnd.hpp"


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
int main() 
{

    // Create atomic control flags
    std::atomic<ThreadState> data_aquire_state(ThreadState::Paused);
    std::atomic<ThreadState> frontend_state(ThreadState::Paused);
    std::atomic<ThreadState> backend_state(ThreadState::Paused);

    //Create data interfaces
    size_t input_queue_capacity = 20;
    ThreadSafeFIFO<InputDataSync> data_DA_to_FE(input_queue_capacity);

 
    // Start threads
    std::thread data_aquire_thread(DA_loop, std::ref(data_aquire_state), &data_DA_to_FE);
    std::thread frontend_thread(FE_loop, std::ref(frontend_state), &data_DA_to_FE);
    std::thread backend_thread(BE_loop, std::ref(backend_state));
    
    // Start this thread
    CM_loop(std::ref(data_aquire_state), 
            std::ref(frontend_state), 
            std::ref(backend_state),
            &data_DA_to_FE);

    // Wait for other threads to exit
    data_aquire_thread.join();
    frontend_thread.join();
    backend_thread.join();

    std::cout << "Test Ended" << std::endl;

    return 0;
}

//==============================================================================
// End of File : Software/src/main.cpp