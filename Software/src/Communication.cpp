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
    ThreadSafeFIFO<InputDataSync>* data_DA) {

    auto start_time = std::chrono::steady_clock::now();   

    std::uint8_t command = 100; //Get this from external source


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
            }
            else 
            {
                command = 0;
            }
        }

        // Thread control
        if (command == 0) {
            data_sync_state = ThreadState::Running;
            frontend_state = ThreadState::Running;
            backend_state = ThreadState::Running;

            // Arduino Communication
            //      Receive from arduino control instructions
                

            //      Transmit to arduino displacement estimates
            //          Get pose / displacement from BE



            // Base Station Communication
            //      Get frames from DA and transmit on UDP

            auto item_DA = data_DA->peek();
            if (item_DA.has_value()) {
                if (item_DA != last_output) { // Compare the current value with the last
                    std::cout << "Read Data: " << static_cast<int>(item_DA.value()) << std::endl;
                    last_output = item_DA; // Update the last value
                    sleep_ms(10);
                }
            } 
            else {
            // std::cout << "CM - DA Buffer Empty!" << std::endl;
            }




            
        } else if (command == 1) {
            // Stop Condition
            data_DA->stop_queue();  //Wake FE if waiting on data
            data_sync_state = ThreadState::Stopped;
            frontend_state = ThreadState::Stopped;
            backend_state = ThreadState::Stopped;
            break;
        } else if (command == 2) {
            // Pause Condition
            data_sync_state = ThreadState::Paused;
            frontend_state = ThreadState::Paused;
            backend_state = ThreadState::Paused;
        } else if (command == 3) {
            // Reset Condition
            data_sync_state = ThreadState::Reset;
            frontend_state = ThreadState::Reset;
            backend_state = ThreadState::Reset;
        } else if (command == 4) {
            // Testing Conditionn
            std::cout << "Comms Testing" << std::endl;
            data_sync_state = ThreadState::Test;
            frontend_state = ThreadState::Test;
            backend_state = ThreadState::Test;
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
