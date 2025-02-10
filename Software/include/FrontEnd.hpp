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
#include "Flags.hpp"
#include "landmark_table.hpp"
#include "nframe_table.hpp"
#include "RingBuffer.hpp"
#include "imu_integrator.hpp"
#include "landmark_triangulation.hpp"


//==============================================================================
//      Classes
//------------------------------------------------------------------------------

//Replace with actual classes]
// using CameraRig = uint16_t;
// using ImuIntegrator = uint16_t;
using FeatureTracker = uint16_t;
using FeatureInitializer = uint16_t;

using TrackedNFrameCallback =
  std::function<void(const std::shared_ptr<NFrame>&,
                     const ImuStamps& imu_stamps,
                     const ImuAccGyrContainer& imu_accgyr,
                     const VioMotionType,
                     const std::vector<LandmarkHandle>& lm_opportunistic,
                     const std::vector<LandmarkHandle>& lm_persistent_new,
                     const std::vector<LandmarkHandle>& lm_persistent_continued)>;
using UpdateStatesCallback = std::function<bool(const bool wait_for_backend)>;


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

 //System state
 ImuStamps imu_stamps_since_lkf_;
 ImuAccGyrContainer imu_accgyr_since_lkf_;
 LandmarkTable landmarks_;
 const TransformationVector T_C_B_;
 NFrameTable states_;
 FrontendStage stage_ = FrontendStage::AttitudeEstimation;


 void processData(
    const std::pair<int64_t, EventArrayPtr>& stamped_events,
    const std::vector<ImuStamps>& imu_timestamps,
    const std::vector<ImuAccGyrContainer>& imu_measurements);

void addImuData(
        int64_t stamp, 
        const Vector3& acc, 
        const Vector3& gyr, 
        const uint32_t imu_idx);
bool addImuMeasurementsBetweenKeyframes(
    const ImuStamps& imu_stamps,
    const ImuAccGyrContainer& imu_accgyr);

void cleanupInactiveLandmarksFromLastIteration();
bool pollBackend(bool block);
  // Temporaries
  int attitude_init_count_ = 0;       //!< Number of frames used for attitude estimation.
  VioMotionType motion_type_ = VioMotionType::GeneralMotion;

  protected:
  UpdateStatesCallback update_states_cb_;
  Odometry odom_;
  Ringbuffer<real_t, 6, 1000> imu_buffer_;
};

//==============================================================================
//      Function Prototypes
//------------------------------------------------------------------------------


#endif  // FRONTEND_HPP
//==============================================================================
// End of File :  Software/include/Communication.hpp