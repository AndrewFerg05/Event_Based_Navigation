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
void FE_loop(std::atomic<ThreadState>& state,
                    ThreadSafeFIFO<InputDataSync>* data_DA,
                    CommunicationManager* comms) {
    
    int bufferSize = 0;
    int readData = 0;
    int processedData = 0;

    while (true) {
        if (state == ThreadState::Stopped) {
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
            
            //TODO
            // Check buffer can be read
            auto item_DA = data_DA->pop(); // Get data from queue
            if (item_DA.has_value()) {
                comms->queueTrackedFrameData(item_DA.value());
                sleep_ms(30);
            } else {
                std::cout << "Queue returned no value (stopped or empty)." << std::endl;
                sleep_ms(10);
            }
        }

        if (state == ThreadState::Test) {
            sleep_ms(100);
            std::cout << "Frontend Testing" << std::endl; 
        }
    }
}


//==============================================================================
// End of File : Software/src/DataAcquisition.cpp
