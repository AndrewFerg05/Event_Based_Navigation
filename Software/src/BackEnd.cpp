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

void BE_loop(std::atomic<ThreadState>& state, CommunicationManager* comms) {
    // while (true) {
    //     if (state == ThreadState::Stopped) {
    //         break;
    //     }

    //     if (state == ThreadState::Paused) {
    //         //TODO - Wait while some condition
    //         sleep_ms(100);
    //         continue;
    //     }

    //     if (state == ThreadState::Reset) {
    //         //TODO call reset function then set running again
    //         state = ThreadState::Running; 
    //     }

    //     if (state == ThreadState::Running) {
    //         comms->queueOther(69);
    //         sleep_ms(30);
    //     }

    //     if (state == ThreadState::Test) {
    //         sleep_ms(100);
    //         std::cout << "Backend Testing" << std::endl; 
    //     }
    // }
}

//==============================================================================
// End of File : Software/src/BackEnd.cpp
