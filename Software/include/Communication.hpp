/*
Filename    : Software/include/Communication.hpp
Author      : Samuel Kliskey
Project     : Event Based Navigation
Date        : 11/1/25
Description : Header file for the communication (C) thread
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
11-JAN-2025 SARK created to design code structure
--------------------------------------------------------------------------------
*/

#ifndef COMMUNICATION_HPP
#define COMMUNICATION_HPP

//==============================================================================
// External Files
//------------------------------------------------------------------------------
#include "threads.hpp"

//==============================================================================
//      Classes
//------------------------------------------------------------------------------


//==============================================================================
//      Function Prototypes
//------------------------------------------------------------------------------
void thread_Communication(
    std::atomic<ThreadState>& data_sync_state,
    std::atomic<ThreadState>& frontend_state,
    std::atomic<ThreadState>& backend_state);


#endif  // COMMUNICATION_HPP
//==============================================================================
// End of File :  Software/include/Communication.hpp