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
FrontEnd::FrontEnd()
//   : rig_(cameraRigFromGflags())
//   , imu_integrator_(std::make_shared<ImuIntegrator>())
//   , thread_pool_(rig_->size())
//   , T_C_B_(rig_->T_C_B_vec())
{
//   initModules();
//   initDvs();
//   VLOG(1) << "Initialized frontend with camera:\n" << *rig_;
}

FrontEnd::~FrontEnd(){}


void FrontEnd::processData(
    const std::pair<int64_t, EventArrayPtr>& stamped_events,
    const std::vector<ImuStamps>& imu_stamps_vec,
    const std::vector<ImuAccGyrContainer>& imu_accgyr_vec)
{

}

void FrontEnd::addImuData(
    int64_t stamp, const Vector3& acc, const Vector3& gyr, const uint32_t imu_idx)
{
    
}


//==============================================================================
// End of File : Software/src/DataAcquisition.cpp
