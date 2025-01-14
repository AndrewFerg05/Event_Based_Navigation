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


//==============================================================================
// Functions
//------------------------------------------------------------------------------
void thread_DataAcquisition(std::atomic<ThreadState>& state,
                            interface_DA_to_FE* data_DA) {
    
    int valueToAdd = 0;

    while (true) {
        if (state == ThreadState::Stopped) {
            std::cout << "Data Aquisition Stopping" << std::endl;
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
            //TODO - Get data from camera

            //Get data
            valueToAdd++;                       //For Sam architecture testing (replace with actual frames)

            //Synchronise data

            //Put in buffer
            data_DA->addToBuffer(valueToAdd);   //For Sam architecture testing (replace with actual frames)

            sleep_ms(20);
        }

        if (state == ThreadState::Test) {
            sleep_ms(100);
            std::cout << "Data Aquisition Testing" << std::endl; 
        }
    }
}
//==============================================================================
// End of File : Software/src/DataAcquisition.cpp
