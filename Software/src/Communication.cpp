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
void thread_Communication(
    std::atomic<ThreadState>& data_sync_state,
    std::atomic<ThreadState>& frontend_state,
    std::atomic<ThreadState>& backend_state) {

    auto start_time = std::chrono::steady_clock::now();   

    std::uint8_t command = 100; //Get this from external source

    while (true) {

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();

        // AF - Test Schedule
        if (elapsed > 1)
        {
            if (elapsed > 2)
            {
                command = 1;
            }
            else 
            {
                command = 4;
            }
        }


        if (command == 0) {
            data_sync_state = ThreadState::Running;
            frontend_state = ThreadState::Running;
            backend_state = ThreadState::Running;
        } else if (command == 1) {
            std::cout << "Comms Stopping" << std::endl;
            data_sync_state = ThreadState::Stopped;
            frontend_state = ThreadState::Stopped;
            backend_state = ThreadState::Stopped;
            break;
        } else if (command == 2) {
            data_sync_state = ThreadState::Paused;
            frontend_state = ThreadState::Paused;
            backend_state = ThreadState::Paused;
        } else if (command == 3) {
            data_sync_state = ThreadState::Reset;
            frontend_state = ThreadState::Reset;
            backend_state = ThreadState::Reset;
        } else if (command == 4) {
            std::cout << "Comms Testing" << std::endl;
            data_sync_state = ThreadState::Test;
            frontend_state = ThreadState::Test;
            backend_state = ThreadState::Test;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        else if (command == 100){
            std::cout << "Unknown state" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            

    }
}








//     int bufferSize = 0;

//     while(run->run_check() == true)
//     {
//         bufferSize = data_DA->checkBuffer();

//         if (bufferSize > 0 && data_DA->checkIndex('C') < bufferSize)
//         {
//             std::cout << "Data Acquired: " << data_DA->readBuffer('C') << std::endl;
//             iterations++;
//         }
//         else
//         {
//             std::cout << "Buffer Empty" << std::endl;
//         }

//         if (iterations >= 20)
//         {
//             run->run_end();
//         }

//         sleep_ms(5);
//     }
// }

//==============================================================================
// End of File : Software/src/Communication.cpp
