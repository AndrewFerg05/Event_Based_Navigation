/*
Filename    : Software/include/threads.hpp
Author      : Samuel Kliskey
Project     : Event Based Navigation
Date        : 10/1/25
Description : Header file for thread functions and shared data structures
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
10-JAN-2025 SARK created to design code structure
--------------------------------------------------------------------------------
*/

#ifndef THREADS_HPP
#define THREADS_HPP

//==============================================================================
// External Files
//------------------------------------------------------------------------------
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>
#include <shared_mutex>

//==============================================================================
//      Classes
// Classes defined here are the shared data structures between threads to ensure
// race condition issues.
//------------------------------------------------------------------------------

class protectedData {
    public:
        std::shared_mutex mtx;
        int counter;

        void readCounter();
        void incrementCounter();
};

//==============================================================================
//      Function Prototypes
//------------------------------------------------------------------------------








#endif  // THREADS_HPP
//==============================================================================
// End of File :  Software/include/threads.hpp