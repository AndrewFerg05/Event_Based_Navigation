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
                    interface_DA_to_FE* data_DA,
                    interface_FE_to_BE* data_FE) {
    
    int bufferSize = 0;
    int readData = 0;
    int processedData = 0;

    while (true) {
        if (state == ThreadState::Stopped) {
            std::cout << "Frontend Stopping" << std::endl;
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
            bufferSize = data_DA->checkBuffer();
            if (bufferSize > 0 && data_DA->checkIndex('F') < bufferSize)
            {
                // Read from buffer
                readData = data_DA->readBuffer('F');
                
                // Data processing
                processedData = readData << 1;

                // Load into BE
                data_FE->addToBuffer(processedData);
            }
            else
            {
                //Short wait until data ready
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
