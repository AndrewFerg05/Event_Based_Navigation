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

void thread_BackEnd(std::atomic<ThreadState>& state,
                    interface_FE_to_BE* data_FE) {
    while (true) {
        if (state == ThreadState::Stopped) {
            std::cout << "Backend Stopping" << std::endl;
            break;
        }

        if (state == ThreadState::Paused) {
            //TODO - Wait while some condition
            sleep_ms(100);
            continue;
        }

        if (state == ThreadState::Reset) {
            //TODO call reset function then set running again
            state = ThreadState::Running; 
        }

        if (state == ThreadState::Running) {
            //TODO - Get data from frontend
            sleep_ms(100);
        }

        if (state == ThreadState::Test) {
            sleep_ms(100);
            std::cout << "Backend Testing" << std::endl; 
        }
    }
}

//==============================================================================
// End of File : Software/src/BackEnd.cpp
