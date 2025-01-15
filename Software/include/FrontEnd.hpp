/*
Filename    : Software/include/FrontEnd.hpp
Author      : Samuel Kliskey
Project     : Event Based Navigation
Date        : 11/1/25
Description : Header file for the FrontEnd (FE) thread
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
11-JAN-2025 SARK created to design code structure
--------------------------------------------------------------------------------
*/

#ifndef FRONTEND_HPP
#define FRONTEND_HPP

//==============================================================================
// External Files
//------------------------------------------------------------------------------
#include "ThreadInterface.hpp"
#include "TypeAliases.hpp"


//==============================================================================
//      Classes
//------------------------------------------------------------------------------


//==============================================================================
//      Function Prototypes
//------------------------------------------------------------------------------
void FE_loop(std::atomic<ThreadState>& state,
                    interface_DA_to_FE* data_DA,
                    interface_FE_to_BE* data_FE);





#endif  // FRONTEND_HPP
//==============================================================================
// End of File :  Software/include/Communication.hpp