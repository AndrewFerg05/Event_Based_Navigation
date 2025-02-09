/*
Filename    : Software/src/ThreadInterface.cpp
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
#include "ThreadInterface.hpp"

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

ImageData CommunicationManager::getFrameData()
{
    // Return latest from FIFO
    if (auto data = from_camera.pop()) {
        return data.value();
    }
    else {
        ImageData nothing;
        return nothing;
    }
}

TrackedFrames CommunicationManager::getTrackedFrameData()
{
    // Return latest from FIFO
    if (auto data = from_frontend.pop()) {
        return data.value();
    }
    else {
        TrackedFrames nothing;
        return nothing;
    }
}

OtherData CommunicationManager::getOtherData()
{
    // Return latest from FIFO
    if (auto data = from_backend.pop()) {
        return data.value();
    }
    else {
        OtherData nothing;
        return 0;
    }
}


void CommunicationManager::queueFrameData(ImageData data)
{
    from_camera.push(data);
}


void CommunicationManager::queueTrackedFrameData(TrackedFrames data){
    from_frontend.push(data);
}


void CommunicationManager::queueOther(OtherData data){
    from_backend.push(data);
}

//==============================================================================
// End of File : Software/src/threads.cpp
