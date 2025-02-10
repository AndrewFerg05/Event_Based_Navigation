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
    motion_type_ = VioMotionType::NotComputed;
    CHECK(!imu_stamps_vec.empty()) << "FE: There are no IMU stamps";
    CHECK(!imu_accgyr_vec.empty()) << "FE: There are no IMU measurements";

    if (!addImuMeasurementsBetweenKeyframes(imu_stamps_vec.at(0),
                                            imu_accgyr_vec.at(0))
                                            && FLAGS_num_imus > 0)
    {
        LOG(ERROR) << "FE: No IMU messages provided.";
        return;
    }

    cleanupInactiveLandmarksFromLastIteration();

    if (states_.nframeKm1() &&
    FLAGS_vio_activate_backend && update_states_cb_)
    {
        pollBackend(true); // If optimizer has new result, this callback will copy them in our states.
        retriangulateAllLandmarks(states_, T_C_B_, landmarks_);
    }


}

void FrontEnd::addImuData(
    int64_t stamp, const Vector3& acc, const Vector3& gyr, const uint32_t imu_idx)
{
    
}

bool FrontEnd::addImuMeasurementsBetweenKeyframes(
    const ImuStamps& imu_stamps,
    const ImuAccGyrContainer& imu_accgyr)
{
    if (imu_stamps.size() < 3) 
    {
        LOG(ERROR) << "Less than 3 IMU measurements in vector.";
        return false;
    }
    
    if (imu_stamps_since_lkf_.size() > 0)
    {
        // Check that last imu stamp since last key frame is equal to
        // the first imu stamp passed to this function.
        DEBUG_CHECK_EQ(
            imu_stamps_since_lkf_[imu_stamps_since_lkf_.size() - 1],
            imu_stamps[0]
        );

        uint64_t n = imu_stamps.size();
        uint64_t size_new = imu_stamps_since_lkf_.size() + n - 1;
        uint64_t current_size = imu_stamps_since_lkf_.size();

        if (imu_stamps.cols() > 1) 
        {
            int64_t delta_t_actual = imu_stamps_since_lkf_[current_size - 2] -
            imu_stamps_since_lkf_[current_size - 3];
            int64_t delta_t_start = imu_stamps[1] - imu_stamps[0];
            LOG(WARNING) << "FE: Number of IMU messages:" << imu_stamps.cols();

            DEBUG_CHECK_LE(delta_t_start, delta_t_actual);
            static constexpr int64_t kSamplePeriodTolerance = 100;
            if ((delta_t_actual - delta_t_start) > kSamplePeriodTolerance)
            {
                // This should ensure we remove the interpolated IMU measurement.
                // Since this interpolated value is present as the last element of
                // imu_stamps_since_lkf_ and first measurement of imu_stamps, we have
                // to reduce n and size_new accordingly.
                --n;
                --size_new;
            }
          
        }

        imu_stamps_since_lkf_.conservativeResize(size_new);
        imu_accgyr_since_lkf_.conservativeResize(6, size_new);
        imu_stamps_since_lkf_.tail(n) = imu_stamps.tail(n);
        imu_accgyr_since_lkf_.rightCols(n) = imu_accgyr.rightCols(n);
    }
    else
    {
        imu_stamps_since_lkf_ = imu_stamps;
        imu_accgyr_since_lkf_ = imu_accgyr;
    }
    return true;

}

void FrontEnd::cleanupInactiveLandmarksFromLastIteration()
{
    landmarks_.cleanupInactiveLandmarks();
    VLOG(40) << landmarks_.typesFormattedString();
}

bool FrontEnd::pollBackend(bool block)
{
  // Return false if no updates from backend are available.
  if (!update_states_cb_(block))
    return false;

  // Backend states somehow are not normalized.
  states_.T_Bk_W().getRotation().normalize();

  // Reset integration point using newest result from backend.
  odom_.v_W = states_.v_W(states_.nframeHandleK());

  ImuStamps imu_timestamps;
  ImuAccGyrContainer imu_measurements;

  std::tie(imu_timestamps, imu_measurements) =
    imu_buffer_.getBetweenValuesInterpolated(
      states_.nframeK()->timestamp(),
      odom_.stamp);

  Transformation T_Bkm1_Bk =
    imu_integrator_->integrateImu(
      imu_timestamps,
      imu_measurements,
      states_.T_Bk_W(),
      odom_.v_W);

  odom_.T_W_B = states_.T_Bk_W().inverse() * T_Bkm1_Bk;

  // Attitude is estimated from the IMU in the backend.
  // Hence, continue to Initializing stage when first update has arrived.
  if (stage_ == FrontendStage::AttitudeEstimation)
  {
      stage_ = FrontendStage::Initializing;
  }

  return true;
}
//==============================================================================
// End of File : Software/src/DataAcquisition.cpp
