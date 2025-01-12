/*
Filename    : Software/src/Communication.cpp
Author      : Samuel Kliskey
Project     : Event Based Navigation
Date        : 11/1/25
Description : Communication code for communicating with base station and Arduino
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
#include "Communication.hpp"

//==============================================================================
// Function Prototypes
//------------------------------------------------------------------------------

//==============================================================================
// MACROs
//------------------------------------------------------------------------------

//==============================================================================
// Global Variable Initialisation
//------------------------------------------------------------------------------
int iterations = 0;


//==============================================================================
// Functions
//------------------------------------------------------------------------------
void thread_Communication(run_control* run, interface_DA_to_FE_and_C* data_DA)
{
    int bufferSize = 0;

    while(run->run_check() == true)
    {
        bufferSize = data_DA->checkBuffer();

        if (bufferSize > 0 && data_DA->checkIndex('C') < bufferSize)
        {
            std::cout << "Data Acquired: " << data_DA->readBuffer('C') << std::endl;
            iterations++;
        }
        else
        {
            std::cout << "Buffer Empty" << std::endl;
        }

        if (iterations >= 20)
        {
            run->run_end();
        }

        sleep_ms(5);
    }
}

//==============================================================================
// End of File : Software/src/Communication.cpp
