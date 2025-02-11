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
#include "image_cv.hpp"
#include "feature_tracker.hpp"
#include "landmark_utils.hpp"
#include "statistics.hpp"
#include <opencv2/core/core.hpp>
#include "nframe_utils.hpp"
#include "keyframe_selection.hpp"
#include "landmarks_reprojector.hpp"
#include "feature_initializer.hpp"
#include "stereo_matcher.hpp"
#include "refinement.hpp"
#include "track_extractor.hpp"
//==============================================================================
//      Classes
//------------------------------------------------------------------------------
enum class FrontendStage : std::int8_t
{
  Paused,
  AttitudeEstimation,
  Initializing,
  Running
};


using TrackedNFrameCallback =
  std::function<void(const std::shared_ptr<NFrame>&,
                     const ImuStamps& imu_stamps,
                     const ImuAccGyrContainer& imu_accgyr,
                     const VioMotionType,
                     const std::vector<LandmarkHandle>& lm_opportunistic,
                     const std::vector<LandmarkHandle>& lm_persistent_new,
                     const std::vector<LandmarkHandle>& lm_persistent_continued)>;

using UpdateStatesCallback = std::function<bool(const bool wait_for_backend)>;

using VisualOdometryCallback =
  std::function<void(const int64_t timestamp,
                     const Eigen::Quaterniond& orientation,
                     const Eigen::Vector3d& position,
                     const FrontendStage stage,
                     const uint32_t num_tracked_features)>;



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
  std::shared_ptr<LandmarksReprojector> reprojector_;
  std::shared_ptr<StereoMatcher> stereo_matcher_;
  ThreadPool thread_pool_;

  //System state
  int imu_meas_count_ = -1;
  int frame_count_ = -1;                //!< Frame counter.
  ImuStamps imu_stamps_since_lkf_;
  ImuAccGyrContainer imu_accgyr_since_lkf_;
  LandmarkTable landmarks_;
  const TransformationVector T_C_B_;
  NFrameTable states_;
  real_t scene_depth_;
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

  bool pollBackend(bool block=false);

  void drawEvents(
    const EventArray::iterator& first,
    const EventArray::iterator& last,
    const int64_t& t0,
    const int64_t& t1,
    const Transformation& T_1_0,
    cv::Mat &out);

  std::pair<std::vector<real_t>, uint32_t> trackFrameKLT();

  VioMotionType classifyMotion(
    std::vector<real_t>& disparities_sq,
    const uint32_t num_outliers);

  // Prieviously Derived Class Functions
  void vio_processData(const Transformation& T_Bkm1_Bk);
  void vio_makeKeyframeIfNecessary(const uint32_t num_tracked);

  std::shared_ptr<NFrame> createNFrame(
      const std::vector<std::pair<int64_t, ImageBase::Ptr>>& stamped_images);

  // Temporaries
  int attitude_init_count_ = 0;       //!< Number of frames used for attitude estimation.
  VioMotionType motion_type_ = VioMotionType::GeneralMotion;

  private:
    cv::Mat dvs_img_;
    void initModules();
    void initDvs();

  protected:
    UpdateStatesCallback update_states_cb_;
    TrackedNFrameCallback tracked_nframe_cb_;
    VisualOdometryCallback result_cb_;
    Odometry odom_;
    Ringbuffer<real_t, 6, 1000> imu_buffer_;
};

//==============================================================================
//      Function Prototypes
//------------------------------------------------------------------------------


#endif  // FRONTEND_HPP
//==============================================================================
// End of File :  Software/include/Communication.hpp