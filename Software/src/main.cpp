/*
Filename    : Software/src/main.cpp
Author      : Samuel Kliskey
Project     : Event Based Navigation
Date        : 10/1/25
Description : Main file of the project
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
10-JAN-2025 SARK created to design code structure
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
    return 0;
}

//==============================================================================
// End of File : Software/src/main.cpp
