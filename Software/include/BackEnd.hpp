/*
Filename    : Software/include/BackEnd.hpp
Author      : Samuel Kliskey
Project     : Event Based Navigation
Date        : 11/1/25
Description : Header file for the BackEnd (BE) thread
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
11-JAN-2025 SARK created to design code structure
--------------------------------------------------------------------------------
*/

#ifndef BACKEND_HPP
#define BACKEND_HPP

//==============================================================================
// External Files
//------------------------------------------------------------------------------
#include "ThreadInterface.hpp"



//==============================================================================
//      Classes
//------------------------------------------------------------------------------



//==============================================================================
//      Function Prototypes
//------------------------------------------------------------------------------
void thread_BackEnd(std::atomic<ThreadState>& state,
                    interface_FE_to_BE* data_FE);





#endif  // BACKEND_HPP
//==============================================================================
// End of File :  Software/include/Communication.hpp