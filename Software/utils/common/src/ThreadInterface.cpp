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


cv::Mat CommunicationManager::getFrameCamera()
{
    // Return latest from FIFO
    if (auto data = framesCamera.pop()) {
        return data.value();
    }
    else {
        cv::Mat nothing;
        return nothing;
    }
}

cv::Mat CommunicationManager::getFrameEvents()
{
    // Return latest from FIFO
    if (auto data = framesEvents.pop()) {
        return data.value();
    }
    else {
        cv::Mat nothing;
        return nothing;
    }
}

cv::Mat CommunicationManager::getFrameAugmented()
{
    // Return latest from FIFO
    if (auto data = framesAugmented.pop()) {
        return data.value();
    }
    else {
        cv::Mat nothing;
        return nothing;
    }
}

OtherData CommunicationManager::getPose()
{
    // Return latest from FIFO
    if (auto data = pose.pop()) {
        return data.value();
    }
    else {
        OtherData nothing;
        return 0;
    }
}


void CommunicationManager::queueFrameCamera(cv::Mat data)
{
    framesCamera.push(data);
    LOG(INFO) << "TI: Camera Frame pushed to CM";
}


void CommunicationManager::queueFrameEvents(cv::Mat data){
    framesEvents.push(data);
    LOG(INFO) << "TI: Event Frame pushed to CM";
}

void CommunicationManager::queueFrameAugmented(cv::Mat data)
{
    framesAugmented.push(data);
    LOG(INFO) << "TI: Augmented Frame pushed to CM";
}

void CommunicationManager::queuePose(OtherData data){
    pose.push(data);
    LOG(INFO) << "TI: Pose pushed to CM";
}

//==============================================================================
// End of File : Software/src/threads.cpp
