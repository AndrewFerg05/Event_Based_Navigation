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
   : rig_(cameraRigFromGflags())
   , imu_integrator_(std::make_shared<ImuIntegrator>())
   , thread_pool_(rig_->size())
   , T_C_B_(rig_->T_C_B_vec())
{
  VLOG(1) << "Initialized frontend with camera:\n" << *rig_;
}

FrontEnd::~FrontEnd(){}

void FrontEnd::initDvs() {
  Camera::Ptr dvs_cam;
  if (rig_->getDvsCamera(&dvs_cam)) {
    // Initialize dvs_img_ to black.
    dvs_img_ = cv::Mat::zeros(dvs_cam->height(), dvs_cam->width(), CV_8U);
  }
}

void FrontEnd::initModules()
{
  EpipolarMatcherOptions options;
  stereo_matcher_ =
  std::make_shared<StereoMatcher>(
    *rig_, FLAGS_imp_detector_border_margin, FLAGS_vio_ransac_relpose_thresh_px,
    landmarks_);

  feature_tracker_ =
    std::make_shared<FeatureTracker>(
      *rig_, FLAGS_imp_detector_border_margin, FLAGS_vio_ransac_relpose_thresh_px,
      FLAGS_vio_use_5pt_ransac, *stereo_matcher_, landmarks_);
  feature_initializer_ =
    std::make_shared<FeatureInitializer>(
      *rig_, FLAGS_imp_detector_border_margin, landmarks_);
  reprojector_ = std::make_shared<LandmarksReprojector>(
    *rig_, FLAGS_imp_detector_border_margin,
    FLAGS_imp_detector_grid_size, landmarks_, states_);
}


void FrontEnd::start()
{
  stage_ = FrontendStage::AttitudeEstimation;
}

void FrontEnd::idle()
{
  stage_ = FrontendStage::Paused;
  initModules();
  initDvs();
  //Clear queues
}
void FrontEnd::stop()
{
  stage_ = FrontendStage::Paused;
}


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
        retriangulateAllLandmarks(states_, T_C_B_, landmarks_); // Retriangulate all landmarks if we have new states.
        scene_depth_ = states_.nframeKm1()->at(0).median_depth_; // Update scene depth
    }
    
    // Predict pose of new frame using IMU:
    Transformation T_Bkm1_Bk;
    if (states_.nframeKm1())
    {
        if (!imu_stamps_vec.empty() && imu_stamps_vec.at(0).size() > 0)
        {
        Vector3 v_W = states_.v_W(states_.nframeHandleK());

        T_Bkm1_Bk = imu_integrator_->integrateImu(
                imu_stamps_vec.at(0),
                imu_accgyr_vec.at(0),
                states_.T_Bk_W(),
                v_W);
        }
    }
    //---------------------------------------------------------------------//
    // Events related things
    CHECK(rig_->dvs_bearing_lut_.size() > 0) << "Bearing lookup table is empty";

    int64_t t0 = imu_stamps_vec.at(0).head(1)[0];
    int64_t t1 = imu_stamps_vec.at(0).tail(1)[0];

    // Create new stamped_images from stamped_events
    cv::Mat event_img = cv::Mat::zeros(rig_->at(0).height(), rig_->at(0).width(), CV_32F);
    const EventArrayPtr events_ptr = stamped_events.second;
    VLOG(20) << "Number of events: " << events_ptr->size();

    size_t n_events_for_noise_detection = std::min(events_ptr->size(), size_t(4000));
    
    // Event_rate = (# of events) / (time difference between last and first of these events)
    real_t event_rate = n_events_for_noise_detection /
            (static_cast<real_t>(events_ptr->back().timestamp_ns) * 1e-9 -
            static_cast<real_t>(events_ptr->at(events_ptr->size()-n_events_for_noise_detection).timestamp_ns) * 1e-9);
    
    if (event_rate < FLAGS_noise_event_rate)
    {
        motion_type_ = VioMotionType::NoMotion;
    }
    else
    {
        // Build event frame with fixed number of events
        const size_t winsize_events = FLAGS_vio_frame_size;
        VLOG(10) << "Window size: " << winsize_events << " events";

        int first_idx = std::max((int)events_ptr->size() - (int) winsize_events, 0);

        uint64_t frame_length = events_ptr->back().timestamp_ns - events_ptr->at(first_idx).timestamp_ns;

        if(events_ptr->size() < winsize_events)
        {
            VLOG(1) << "Requested frame size of length " << winsize_events
                        << " events, but I only have "
                        << events_ptr->size()
                        << " events in the last event array";
        }

        Transformation T_1_0 = T_C_B_[0] * T_Bkm1_Bk.inverse() * T_C_B_[0].inverse();

        drawEvents(
            events_ptr->begin()+first_idx,
            events_ptr->end(),
            t0, t1,
            T_1_0,
            event_img);

        const float bmax = FLAGS_vio_frame_norm_factor;
        event_img.convertTo(event_img, CV_8U, 255./bmax);
        //cv::normalize(event_img, event_img, 0, 255, cv::NORM_MINMAX, CV_8U);
        dvs_img_ = event_img;   //Can send motion corrected image externally
    }

    std::shared_ptr<ImageCv8uC1> img_cv_ptr
      = std::make_shared<ImageCv8uC1>(dvs_img_, PixelOrder::gray);

    StampedImages stamped_images;
    const int64_t stamp = stamped_events.first;
    stamped_images.push_back(std::pair<int64_t, ImageBase::Ptr>(stamp, img_cv_ptr));
    //---------------------------------------------------------------------//

    // Create new frame:
    NFrame::Ptr nframe_k = createNFrame(stamped_images);
    VLOG(3) << " =============== Frame " << nframe_k->seq() << " - "
    << nframe_k->handle() << " ===============";

    // Predict pose of current frame by integrating the gyroscope:
    if (states_.nframeKm1())
    {
      states_.T_Bk_W() = T_Bkm1_Bk.inverse() * states_.T_Bkm1_W();
    }
    else
    {
      LOG(WARNING) << "Initialize T_Bk_W to identity.";
      states_.T_Bk_W() = Transformation();
    }

    // Run tracking in derived class.
    vio_processData(T_Bkm1_Bk); //LKT/RANSAC/FAST

    //ProcessCallbacks

    if (tracked_nframe_cb_
      && (nframe_k->isKeyframe()
          || (FLAGS_vio_add_every_nth_frame_to_backend > 0
              && nframe_k->seq() % FLAGS_vio_add_every_nth_frame_to_backend == 0)))
  {
    std::vector<LandmarkHandle> lm_opportunistic;
    std::vector<LandmarkHandle> lm_persistent_new;
    std::vector<LandmarkHandle> lm_persistent_continued;

    if (stage_ == FrontendStage::Running && nframe_k->isKeyframe())
    {
      // Extract feature tracks to be processed by backend.
      DEBUG_CHECK(motion_type_ != VioMotionType::NotComputed);
      selectLandmarksForBackend(
            motion_type_, T_C_B_, states_, *nframe_k, landmarks_,
            lm_opportunistic, lm_persistent_new, lm_persistent_continued);
    }
    feature_initializer_->extractFeatureDescriptors(*nframe_k);

    if (FLAGS_vio_activate_backend)
    {
      // Run optimization.
      tracked_nframe_cb_(
            nframe_k, imu_stamps_since_lkf_, imu_accgyr_since_lkf_, motion_type_,
            lm_opportunistic, lm_persistent_new, lm_persistent_continued);
    }
  }

  if (nframe_k->isKeyframe())
  {
    states_.setKeyframe(nframe_k->handle());
    imu_stamps_since_lkf_.resize(0);
    imu_accgyr_since_lkf_.resize(6,0);
  }

  if (result_cb_)
  {
    const Transformation T_W_B = states_.T_Bk_W().inverse();
    result_cb_(
          nframe_k->timestamp(),
          T_W_B.getEigenQuaternion().cast<double>(),
          T_W_B.getPosition().cast<double>(),
          stage_,
          0u //! @todo: return num tracked keypoints!
          );
  }

  // if (FLAGS_vio_log_performance)
  // {
  //   logNumTrackedFeatures(*nframe_k, landmarks_);
  // }

}

void FrontEnd::addImuData(
    int64_t stamp, const Vector3& acc, const Vector3& gyr, const uint32_t imu_idx)
{
    if (imu_idx != 0)
    return;

  // Add measurement to buffer
  Vector6 acc_gyr;
  acc_gyr.head<3>() = acc;
  acc_gyr.tail<3>() = gyr;
  imu_buffer_.insert(stamp, acc_gyr);
  ++imu_meas_count_;

  real_t dt = nanosecToSecTrunc(stamp - odom_.stamp);
  odom_.stamp = stamp;

  if (stage_ == FrontendStage::Running)
  {
    if (!pollBackend())
    {
      imu_integrator_->propagate(
            odom_.T_W_B.getRotation(), odom_.T_W_B.getPosition(), odom_.v_W,
            acc, gyr, dt);
    }

    odom_.omega_B = gyr - imu_integrator_->getGyrBias();
    // visualizer_->publishOdometry(odom_);
  }
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

void FrontEnd::drawEvents(
    const EventArray::iterator& first,
    const EventArray::iterator& last,
    const int64_t& t0,
    const int64_t& t1,
    const Transformation& T_1_0,
    cv::Mat &out)
{
  size_t n_events = 0;

  Eigen::Matrix<float, 2, Eigen::Dynamic> events;
  events.resize(2, last - first);

  Camera::ConstPtr dvs_cam;
  if (!rig_->getDvsCamera(&dvs_cam)) {
    LOG(WARNING) << "Draw events is called but there is no Dvs camera in rig";
  }

  const int height = dvs_cam->height();
  const int width = dvs_cam->width();
  CHECK_EQ(out.rows, height);
  CHECK_EQ(out.cols, width);

  const VectorX& params = dvs_cam->projectionParameters();
  const float fx = params[0];
  const float fy = params[1];
  const float cx = params[2];
  const float cy = params[3];
  Eigen::Matrix4f K;
  K << fx, 0., cx, 0.,
       0., fy, cy, 0.,
       0., 0., 1., 0.,
       0., 0., 0., 1.;

  Eigen::Matrix4f T =
      K * T_1_0.getTransformationMatrix().cast<float>() * K.inverse();

  float depth = scene_depth_;

  bool do_motion_correction = FLAGS_vio_do_motion_correction;

  float dt = 0;
  for(auto e = first; e != last; ++e)
  {
    if (n_events % 10 == 0)
    {
      dt = static_cast<float>(t1 - e->timestamp_ns) / (t1 - t0);
    }

    Eigen::Vector4f f;
    f.head<2>() = rig_->dvs_keypoint_lut_.col(e->x + e->y * width);
    f[2] = 1.;
    f[3] = 1./depth;

    if (do_motion_correction)
    {
      f = (1.f - dt) * f + dt * (T * f);
    }

    events.col(n_events++) = f.head<2>();
  }

  for (size_t i=0; i != n_events; ++i)
  {
    const Eigen::Vector2f& f = events.col(i);

    int x0 = std::floor(f[0]);
    int y0 = std::floor(f[1]);

    if(x0 >= 0 && x0 < width-1 && y0 >= 0 && y0 < height-1)
    {
      const float fx = f[0] - x0,
                  fy = f[1] - y0;
      Eigen::Vector4f w((1.f-fx)*(1.f-fy),
                        (fx)*(1.f-fy),
                        (1.f-fx)*(fy),
                        (fx)*(fy));

      out.at<float>(y0,   x0)   += w[0];
      out.at<float>(y0,   x0+1) += w[1];
      out.at<float>(y0+1, x0)   += w[2];
      out.at<float>(y0+1, x0+1) += w[3];
    }
  }
}

std::shared_ptr<NFrame> FrontEnd::createNFrame(
  const std::vector<std::pair<int64_t, ImageBase::Ptr>>& stamped_images)
{
++frame_count_;
NFrame::Ptr nframe =
    states_.makeAndStoreNewNFrame(stamped_images,
                                  FLAGS_imp_detector_max_features_per_frame);
nframe->setSeq(frame_count_);
return nframe;
}


std::pair<std::vector<real_t>, uint32_t> FrontEnd::trackFrameKLT()
{
  NFrame::Ptr nframe_k   = states_.nframeK();
  NFrame::Ptr nframe_km1 = states_.nframeKm1();
  NFrame::Ptr nframe_lkf = states_.nframeLkf();

  // Project landmarks in Nframe using klt.
  {
    feature_tracker_->trackFeaturesInNFrame(
          states_.T_Bkm1_W(), states_.T_Bk_W(), *nframe_km1, *nframe_k);
  }

  // Outlier rejection using RANSAC.
  std::vector<real_t> disparities_sq;
  uint32_t num_outliers = 0u;
  {
    const Transformation T_Bk_Blkf = states_.T_Bk_W() * states_.T_Blkf_W().inverse();

    //! @todo: should we compute the disparity w.r.t the last frame or keyframe?
    std::tie(disparities_sq, num_outliers) =
        feature_tracker_->outlierRemoval(*nframe_lkf, *nframe_k, T_Bk_Blkf);
  }

  // Set last observation in landmarks:
  setLandmarksLastObservationInNFrame(*nframe_k, landmarks_);

  return std::make_pair(disparities_sq, num_outliers);
}

VioMotionType FrontEnd::classifyMotion(
  std::vector<real_t>& disparities_sq,
  const uint32_t num_outliers)
{
if (motion_type_ == VioMotionType::NoMotion)
{
  return VioMotionType::NoMotion;
}

const uint32_t num_inliers = disparities_sq.size();

// Check if we have enough features:
if (num_inliers < 20u)
{
  LOG(WARNING) << "Motion Type: INVALID (tracking " << num_inliers
               << " features, < 20)";
  return VioMotionType::Invalid;
}

// Check that we have X% inliers:
const real_t inlier_ratio =
    static_cast<real_t>(num_inliers) / (num_inliers + num_outliers);
if (inlier_ratio < 0.6)
{
  LOG(WARNING) << "Motion Type: INVALID (inlier ratio = "
               << inlier_ratio * 100.0 << "%)";
  return VioMotionType::Invalid;
}

// Check disparity:
real_t median_disparity = std::sqrt(median(disparities_sq).first);
VLOG(10) << "Median Disparity = " << median_disparity;
if (median_disparity < FLAGS_vio_disparity_median_for_static_motion_classification)
{
  VLOG(10) << "Motion Type: ROTATION ONLY";
  return VioMotionType::RotationOnly;
}
else
{
  VLOG(10) << "Motion Type: GENERAL";
  return VioMotionType::GeneralMotion;
}
}

//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
// Prieviously Derived Class Functions

void FrontEnd::vio_processData(const Transformation& T_Bkm1_Bk)
{
  NFrame::Ptr nframe_k   = states_.nframeK();
  NFrame::Ptr nframe_lkf = states_.nframeLkf();
  bool add_frame_to_backend = (frame_count_ % FLAGS_vio_add_every_nth_frame_to_backend == 0);

  switch(stage_)
  {
    //--------------------------------------------------------------------------
    case FrontendStage::Running:
    {
      CHECK_NOTNULL(nframe_lkf.get());

      // KLT-Tracking of features and RANSAC:
      std::vector<real_t> disparities_sq;
      uint32_t num_outliers;
      std::tie(disparities_sq, num_outliers) = trackFrameKLT();
      uint32_t num_tracked = disparities_sq.size();

      motion_type_ = classifyMotion(disparities_sq, num_outliers);

      vio_makeKeyframeIfNecessary(num_tracked);
      break;
    }
    //--------------------------------------------------------------------------
    case FrontendStage::AttitudeEstimation:
    {
      LOG(WARNING) << "Stage = AttitudeEstimation";
      motion_type_ = VioMotionType::GeneralMotion;
      if (add_frame_to_backend)
      {
        states_.setKeyframe(nframe_k->handle());
      }
      break;
    }
    //--------------------------------------------------------------------------
    case FrontendStage::Initializing:
    {
      LOG(WARNING) << "Stage = Initializing";
      motion_type_ = VioMotionType::GeneralMotion;
      CHECK(add_frame_to_backend);
      for (size_t i = 0u; i < nframe_k->size(); ++i)
      {
        Frame& frame = nframe_k->at(i);
        frame.min_depth_ = FLAGS_vio_min_depth;
        frame.max_depth_ = FLAGS_vio_max_depth;
        frame.median_depth_ = FLAGS_vio_median_depth;
      }
      feature_initializer_->detectAndInitializeNewFeatures(
            *nframe_k, range(rig_->size()));

      //if (rig_->size() > 1u)
        //stereo_matcher_->matchStereoAndRejectOutliers(*nframe_k, states_.T_Bk_W());

      // Add observations to landmarks.
      addLandmarkObservations(*nframe_k, landmarks_);

      // Switch state:
      stage_ = FrontendStage::Running;
      break;
    }
    default:
      LOG(FATAL) << "Case not implemented.";
      break;
  }

}

void FrontEnd::vio_makeKeyframeIfNecessary(const uint32_t num_tracked)
{
  NFrame::Ptr nframe_k   = states_.nframeK();
  NFrame::Ptr nframe_lkf = states_.nframeLkf();

  // Set last observation in landmarks:
  setLandmarksLastObservationInNFrame(*nframe_k, landmarks_);

  // Remove old landmarks.
    removeOldLandmarks(FLAGS_vio_max_landmarks, nframe_k->seq(), landmarks_);

  // Compute scene depth statistics.
    setSceneDepthInNFrame(*nframe_k, landmarks_, states_.T_Bk_W(), T_C_B_,
                          FLAGS_vio_min_depth, FLAGS_vio_max_depth,
                          FLAGS_vio_median_depth);

  // Select new keyframe.
  if (!needNewKeyframe(
        *nframe_k, *nframe_lkf, states_, states_.T_Bk_W(), T_C_B_, num_tracked))
  {
    return; // No new keyframe.
  }

  // Don't detect features close to existing features.
  feature_initializer_->setOccupancyGrid(reprojector_->gridVec());

  // // Mark all features in keyframe as opportunistic.
  setTypeOfConvergedSeedsInNFrame(
        *nframe_k, LandmarkType::Opportunistic, landmarks_);

  // // Detect new features.
  if (num_tracked < FLAGS_vio_min_tracked_features_total)
  {
   

    // If critical, we extract in all frames features and match stereo.
    // otherwise, just detect features in camera with least features.
    if (num_tracked < FLAGS_vio_kfselect_numfts_lower_thresh
        && rig_->stereoPairs().size() > 0u)
    {
      LOG(WARNING) << "Critical: Force stereo triangulation";
      std::vector<uint32_t> frame_idx_vec = range(rig_->size());
      feature_initializer_->detectAndInitializeNewFeatures(*nframe_k, frame_idx_vec);

      std::vector<std::pair<uint32_t, std::vector<uint32_t>>> stereo_matches =
          stereo_matcher_->matchStereoAndRejectOutliers(*nframe_k, states_.T_Bk_W());

      // Set all stereo observations to opportunistic.
      for (const std::pair<uint32_t, std::vector<uint32_t>>& it : stereo_matches)
      {
        const Frame& ref_frame = nframe_k->at(it.first);
        for (const uint32_t i : it.second)
        {
          DEBUG_CHECK_LT(i, ref_frame.landmark_handles_.size());
          const LandmarkHandle lm_h = ref_frame.landmark_handles_[i];
          if (isValidLandmarkHandle(lm_h)
              && landmarks_.type(lm_h) == LandmarkType::Seed)
          {
            landmarks_.type(lm_h) = LandmarkType::Opportunistic;
          }
        }
      }

      uint32_t num_inliers = 0u;
      for (auto it : stereo_matches)
      {
        num_inliers += it.second.size();
      }
      VLOG(3) << "Upgraded " << num_inliers << " stereo landmarks to opportunistic.";
    }
    else
    {
      for (size_t i = 0u; i < nframe_k->size(); ++i)
      {
        Frame& frame = nframe_k->at(i);
        frame.min_depth_ = FLAGS_vio_min_depth;
        frame.max_depth_ = FLAGS_vio_max_depth;
        frame.median_depth_ = scene_depth_ > FLAGS_vio_min_depth ?
                              scene_depth_ : FLAGS_vio_median_depth;
      }
      feature_initializer_->detectAndInitializeNewFeatures(
            *nframe_k, range(rig_->size()));

      //if (rig_->size() > 1u)
        //stereo_matcher_->matchStereoAndRejectOutliers(*nframe_k, states_.T_Bk_W());
    }
  }

  // Store frame as keyframe. Must be before seed update to do inter-frame updates.
  states_.setKeyframe(states_.nframeHandleK());
  addLandmarkObservations(*nframe_k, landmarks_);

  // Optimize landmarks.
    optimizeLandmarks(*rig_, states_, *nframe_k, landmarks_);
}

//==============================================================================
// End of File : Software/src/DataAcquisition.cpp
