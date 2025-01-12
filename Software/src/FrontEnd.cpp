/*
Filename    : Software/src/FrontEnd.cpp
Author      : Samuel Kliskey
Project     : Event Based Navigation
Date        : 11/1/25
Description : Front End code for determining camera pose
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
#include "FrontEnd.hpp"

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
void thread_FrontEnd(std::atomic<ThreadState>& state) {
    while (true) {
        if (state == ThreadState::Stopped) {
            break;
        }

        if (state == ThreadState::Paused) {
            //TODO - Wait while some condition
            continue;
        }

        if (state == ThreadState::Reset) {
            //TODO call reset function then set running again
            state = ThreadState::Running; 
        }

        if (state == ThreadState::Running) {
            //TODO - Get data from frontend
        }
    }
}


//==============================================================================
// End of File : Software/src/DataAcquisition.cpp
