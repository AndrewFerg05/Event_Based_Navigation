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


//==============================================================================
// End of File : Software/src/DataAcquisition.cpp
