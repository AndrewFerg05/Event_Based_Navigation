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
void CM_loop(
    std::atomic<ThreadState>& data_sync_state,
    std::atomic<ThreadState>& frontend_state,
    std::atomic<ThreadState>& backend_state,
    ThreadSafeFIFO<InputDataSync>* data_DA,
    CommunicationManager* comms) {

    auto start_time = std::chrono::steady_clock::now();   

    std::uint8_t command = 100; //Get this from external source

    bool state_change_called = false; //Used to only set the atomics once
    int bufferSize = 0;
    std::optional<int> last_output;
    while (true) {

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();

        // AF - Test Thread Control
        if (elapsed > 1)
        {
            if (elapsed > 2)
            {

                command = 1;
                state_change_called = true;
            }
            else 
            {
                command = 0;
                state_change_called = true;
            }
        }

        // Thread control
        if (command == 0) {

            if(state_change_called){
            data_sync_state = ThreadState::Running;
            frontend_state = ThreadState::Running;
            backend_state = ThreadState::Running;
            state_change_called = false;
            }

            // Arduino Communication
            //      Receive from arduino control instructions
                

            //      Transmit to arduino displacement estimates
            //          Get pose / displacement from BE



            // Base Station Communication
            //      Get frames from DA and transmit on UDP

            if(!comms->processQueues())
            {
                std::cout << "No data To Send" << std::endl;
            }
            sleep_ms(10);
            
        } else if (command == 1) {
            // Stop Condition
            if(state_change_called){
            data_sync_state = ThreadState::Stopped;
            frontend_state = ThreadState::Stopped;
            backend_state = ThreadState::Stopped;
            state_change_called = false;
            }

            data_DA->stop_queue();  //Wake FE if waiting on data
            break;

        } else if (command == 2) {
            // Pause Condition
           if(state_change_called){
            data_sync_state = ThreadState::Paused;
            frontend_state = ThreadState::Paused;
            backend_state = ThreadState::Paused;
            state_change_called = false;
            }

        } else if (command == 3) {
            // Reset Condition
           if(state_change_called){
            data_sync_state = ThreadState::Reset;
            frontend_state = ThreadState::Reset;
            backend_state = ThreadState::Reset;
            state_change_called = false;
            }

        } else if (command == 4) {
            // Testing Conditionn
          if(state_change_called){
            data_sync_state = ThreadState::Test;
            frontend_state = ThreadState::Test;
            backend_state = ThreadState::Test;
            state_change_called = false;
            }
            std::cout << "Comms Testing" << std::endl;
            sleep_ms(100);
        }
        else if (command == 100){
            std::cout << "Unknown state" << std::endl;
            sleep_ms(100);
        }
    }
}

//==============================================================================
// End of File : Software/src/Communication.cpp
