/*
Filename    : Software/src/threads.cpp
Author      : Samuel Kliskey
Project     : Event Based Navigation
Date        : 10/1/25
Description : Multithreading in the project
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
10-JAN-2025 SARK created to design code structure
--------------------------------------------------------------------------------
*/

//==============================================================================
// External Files
//------------------------------------------------------------------------------

// Local
#include "../include/threads.hpp"

//==============================================================================
// Function Prototypes
//------------------------------------------------------------------------------


//==============================================================================
// MACROs
//------------------------------------------------------------------------------
#define INCREMENT_AMOUNT   15

//==============================================================================
// Global Variable Initialisation
//------------------------------------------------------------------------------

using namespace std;

//==============================================================================
// Functions
//------------------------------------------------------------------------------
void protectedData::incrementCounter()
{
    for (int i = 0; i < 100; i++) 
    {
        std::unique_lock<std::shared_mutex> ul(mtx);
        counter += INCREMENT_AMOUNT;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    return;
}

void protectedData::readCounter()
{
    for (int i = 0; i < 100; i++) 
    {
        std::shared_lock<std::shared_mutex> sl(mtx);
        std::cout << "Counter value: " << counter << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    return;
}

//==============================================================================
// End of File : Software/src/main.cpp
