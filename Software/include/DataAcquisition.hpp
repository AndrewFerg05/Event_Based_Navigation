/*
Filename    : Software/include/DataAcquisition.hpp
Author      : Samuel Kliskey
Project     : Event Based Navigation
Date        : 11/1/25
Description : Header file for the DataAcquisition (DA) thread
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
11-JAN-2025 SARK created to design code structure
--------------------------------------------------------------------------------
*/

#ifndef DATAACQUISITION_HPP
#define DATAACQUISITION_HPP

//==============================================================================
// External Files
//------------------------------------------------------------------------------
#include "threads.hpp"
#include "threads.hpp"



//==============================================================================
//      Classes
//------------------------------------------------------------------------------


//==============================================================================
//      Function Prototypes
//------------------------------------------------------------------------------
void thread_DataAcquistion(run_control*, interface_DA_to_FE_and_C*);




#endif  // DATAACQUISITION_HPP
//==============================================================================
// End of File :  Software/include/Communication.hpp