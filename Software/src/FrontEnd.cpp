/*
Filename    : Software/src/FrontEnd.cpp
Author      : Samuel Kliskey
Project     : Event Based Navigation
Date        : 11/1/25
Description : Front End code for determining camera pose
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
#include "FrontEnd.hpp"

//==============================================================================
// Function Prototypes
//------------------------------------------------------------------------------

//==============================================================================
// MACROs
//------------------------------------------------------------------------------

//==============================================================================
// Global Variable Initialisation
//------------------------------------------------------------------------------

//==============================================================================
// Functions
//------------------------------------------------------------------------------
FrontEnd::FrontEnd(std::shared_ptr<CommunicationManager> comms)
    : comms_interface_(comms)
    {}


void FrontEnd::start()
{

}

void FrontEnd::idle()
{
    
}

void FrontEnd::stop()
{
    
}

void FrontEnd::addData(
    const StampedImage&    /*image*/,
    const StampedEventArray& /*event_arrays*/,
    const ImuStamps& /*imu_timestamps*/,
    const ImuAccGyrContainer& /*imu_measurements*/,
    const bool& no_motion_prior)
{
    std::cout << "Add data" << std::endl; 
}

void FrontEnd::addImuData(
    int64_t stamp, const Vector3& acc, const Vector3& gyr)
{
    std::cout << "Add IMU" << std::endl;
}
//==============================================================================
// End of File : Software/src/DataAcquisition.cpp
