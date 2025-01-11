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
protectedData myData;


//==============================================================================
// Functions
//------------------------------------------------------------------------------
int main() {

    std::thread incrementer(&protectedData::incrementCounter, &myData);
    std::thread reader(&protectedData::readCounter, &myData);

    reader.join();
    incrementer.join();
    std::cout << "Test Output!" << std::endl;
    

    caerDeviceDiscoveryResult discovered;
    ssize_t result = caerDeviceDiscover(CAER_DEVICE_DISCOVER_ALL, &discovered);

    if (result < 1) {
        std::cerr << "No device found" << std::endl;
    }
    else{
        std::cout << "Device found" << std::endl;
    }

    free(discovered);

    return 0;
}

//==============================================================================
// End of File : Software/src/main.cpp
