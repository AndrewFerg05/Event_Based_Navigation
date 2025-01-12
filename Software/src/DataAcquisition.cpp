/*
Filename    : Software/src/DataAcquisition.cpp
Author      : Samuel Kliskey
Project     : Event Based Navigation
Date        : 11/1/25
Description : Data Acquisition code for getting DAVIS346 camera data
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
#include "DataAcquisition.hpp"

//==============================================================================
// Function Prototypes
//------------------------------------------------------------------------------

//==============================================================================
// MACROs
//------------------------------------------------------------------------------

//==============================================================================
// Global Variable Initialisation
//------------------------------------------------------------------------------
int valueToAdd = 0;


//==============================================================================
// Functions
//------------------------------------------------------------------------------
void thread_DataAcquistion(std::atomic<ThreadState>& state) {
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
            //TODO - Get data from camera
        }
    }
}
//==============================================================================
// End of File : Software/src/DataAcquisition.cpp
