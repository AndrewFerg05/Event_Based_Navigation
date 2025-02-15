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
    LOG(INFO) << "DA: Frame pushed to CM";

}


void CommunicationManager::queueTrackedFrameData(TrackedFrames data){
    from_frontend.push(data);
}


void CommunicationManager::queueOther(OtherData data){
    from_backend.push(data);
}


bool CommunicationManager::processQueues()
{
    
    bool processed = false;

    // Send queue 1 data
    if (auto data_1 = from_camera.pop()) {
        sendToExternal(*data_1);
        processed = true;
    }

    // Send queue 2 data
    if (auto data_2 = from_frontend.pop()) {
        sendToExternal(*data_2);
        processed = true;
    }

    // Send queue 3 data
    if (auto data_3 = from_backend.pop()) {
        sendToExternal(*data_3);
        processed = true;
    }
    return processed;   //return false if there was no data to send
}

//==============================================================================
// End of File : Software/src/threads.cpp
