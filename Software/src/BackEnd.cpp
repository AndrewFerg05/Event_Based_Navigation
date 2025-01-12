/*
Filename    : Software/src/BackEnd.cpp
Author      : Samuel Kliskey
Project     : Event Based Navigation
Date        : 11/1/25
Description : Back end code for optimsation of pose estimation
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
#include "BackEnd.hpp"

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

void thread_BackEnd(std::atomic<ThreadState>& state) {
    while (true) {
        if (state == ThreadState::Stopped) {
            std::cout << "Backend Stopping" << std::endl;
            break;
        }

        if (state == ThreadState::Paused) {
            //TODO - Wait while some condition
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        if (state == ThreadState::Reset) {
            //TODO call reset function then set running again
            state = ThreadState::Running; 
        }

        if (state == ThreadState::Running) {
            //TODO - Get data from frontend
        }

        if (state == ThreadState::Test) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            std::cout << "Backend Testing" << std::endl; 
        }
    }
}

//==============================================================================
// End of File : Software/src/BackEnd.cpp
