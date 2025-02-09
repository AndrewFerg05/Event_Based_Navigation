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

//Replace with actual classes]
using CameraRig = uint16_t;
using ImuIntegrator = uint16_t;
using FeatureTracker = uint16_t;
using FeatureInitializer = uint16_t;


enum class FrontendStage : std::int8_t
{
  Paused,
  AttitudeEstimation,
  Initializing,
  Running
};


class FrontEnd
{


public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW

FrontEnd();
~FrontEnd();

 // Modules
 std::shared_ptr<const CameraRig> rig_;
 std::shared_ptr<ImuIntegrator> imu_integrator_;
 std::shared_ptr<FeatureTracker> feature_tracker_;
 std::shared_ptr<FeatureInitializer> feature_initializer_;

 void processData(
    const std::pair<int64_t, EventArrayPtr>& stamped_events,
    const std::vector<ImuStamps>& imu_timestamps,
    const std::vector<ImuAccGyrContainer>& imu_measurements);

void addImuData(
        int64_t stamp, 
        const Vector3& acc, 
        const Vector3& gyr, 
        const uint32_t imu_idx);


};

//==============================================================================
//      Function Prototypes
//------------------------------------------------------------------------------


#endif  // FRONTEND_HPP
//==============================================================================
// End of File :  Software/include/Communication.hpp