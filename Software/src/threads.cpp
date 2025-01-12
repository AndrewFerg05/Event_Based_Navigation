/*
Filename    : Software/src/threads.cpp
Author      : Samuel Kliskey
Project     : Event Based Navigation
Date        : 10/1/25
Description : Multithreading in the project
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
10-JAN-2025 SARK created to design code structure
11-JAN-2025 SARK added thread interfaces
--------------------------------------------------------------------------------
*/

//==============================================================================
// External Files
//------------------------------------------------------------------------------

// Local
#include "threads.hpp"

//==============================================================================
// Function Prototypes
//------------------------------------------------------------------------------


//==============================================================================
// MACROs
//------------------------------------------------------------------------------
#define INCREMENT_AMOUNT   15

//==============================================================================
// Global Variable Initialisation
//------------------------------------------------------------------------------

//==============================================================================
// Functions
//------------------------------------------------------------------------------
void interface_DA_to_FE_and_C::addToBuffer(int x)
{
    std::unique_lock<std::shared_mutex> ul(mtx);
    buffer.push_back(x);
}

int interface_DA_to_FE_and_C::checkBuffer()
{
    std::shared_lock<std::shared_mutex> sl(mtx);
    return buffer.size();
}

int interface_DA_to_FE_and_C::checkIndex(char threadID)
{
    std::shared_lock<std::shared_mutex> sl(mtx);
    if (threadID == 'C')
    {
        return indexC;
    }
    else if (threadID == 'F')
    {
        return indexFE;
    }
    return -1;
}

int interface_DA_to_FE_and_C::readBuffer(char threadID)
{
    
    std::unique_lock<std::shared_mutex> ul(mtx);

    int pos = 0;

    // Find value at the oldest unread index of the thread
    if (threadID == 'C')
    {
        pos = indexC;
    }
    else if (threadID == 'F')
    {
        pos = indexFE;
    }

    if (pos < 0 || pos >= buffer.size()) 
    {
        throw std::out_of_range("Position out of range");
    }
    int value = buffer.at(pos);

    // Add to threads read index
    if (threadID == 'C')
    {
        indexC++;

        //Remove all read parts of buffer (done only on communications thread)
        for (int i = 0; i < std::min(indexC, indexFE); i++)
        {
            removeFirstFromBuffer();
        }
    }
    else if (threadID == 'F')
    {
        indexFE++;
    }

    return value;
}

void interface_DA_to_FE_and_C::removeFirstFromBuffer()
{
    if (!buffer.empty()) 
    {
        buffer.erase(buffer.begin());
    }
    indexC--;
    indexFE--;
}

bool run_control::run_check()
{
    std::shared_lock<std::shared_mutex> sl(mtx);

    return run;
}

void run_control::run_end()
{
    std::unique_lock<std::shared_mutex> ul(mtx);

    run = false;
}

//==============================================================================
// End of File : Software/src/threads.cpp
