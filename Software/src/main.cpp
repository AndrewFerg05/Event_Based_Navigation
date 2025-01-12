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
--------------------------------------------------------------------------------
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
#include  "threads.hpp"
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
    run_control go;
    interface_DA_to_FE_and_C sharedData_DA_FE_C;

    //Check Device Connected
    caerDeviceDiscoveryResult discovered;
    ssize_t result = caerDeviceDiscover(CAER_DEVICE_DISCOVER_ALL, &discovered);
    if (result < 1) 
    {
        std::cerr << "No device found" << std::endl;
    }
    else
    {
        std::cout << "Device found" << std::endl;
    }
    free(discovered);


    std::thread DA(thread_DataAcquistion, &go, &sharedData_DA_FE_C);
    std::thread C(thread_Communication, &go, &sharedData_DA_FE_C);
    std::thread FE(thread_FrontEnd, &go, &sharedData_DA_FE_C);
    std::thread BE(thread_BackEnd, &go);

    C.join();   // Communications thread will finish first
    DA.join();
    FE.join();
    BE.join();

    std::cout << "Test Output!" << std::endl;
    

    return 0;
}

//==============================================================================
// End of File : Software/src/main.cpp