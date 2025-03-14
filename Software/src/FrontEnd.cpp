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
// Test Functions
//------------------------------------------------------------------------------

// Can Only Run Test Functions In Main Thread

// Define the event frame size for the DAVIS 346 camera
const int EVENT_FRAME_WIDTH = 346;
const int EVENT_FRAME_HEIGHT = 260;

// Frames for event data and APS data
cv::Mat event_frame = cv::Mat::zeros(EVENT_FRAME_HEIGHT, EVENT_FRAME_WIDTH, CV_32FC3);
cv::Mat aps_frame;  // Will store the latest APS frame

//==============================================================================
// Functions
//------------------------------------------------------------------------------


FrontEnd::FrontEnd(std::shared_ptr<CommunicationManager> comms, const std::string& config_path)
    : comms_interface_(comms), config_path_(config_path)
    {
        // setupVIO();
    }

void FrontEnd::start()
{
    setupVIO();
}

void FrontEnd::idle()
{
    vioReady_ = false;
    vio_manager_.reset(); 
}

void FrontEnd::stop()
{
    vioReady_ = false;
    vio_manager_.reset(); 
}

void FrontEnd::setupVIO()
{
    // Load configuration file
     auto parser = std::make_shared<ov_core::YamlParser>(config_path_);

    // Initialize VIO options
    ov_msckf::VioManagerOptions params;
    params.print_and_load(parser);

    params.num_opencv_threads = 0; // for repeatability

    vio_manager_ = std::make_shared<ov_msckf::VioManager>(params);

    // Ensure we read in all parameters required
    if (!parser->successful()) {
      LOG(FATAL) << "FE: Unable to parse all parameters";
    }

    vioReady_ = true;
}

void FrontEnd::initState(int64_t stamp, const Vector3& acc, const Vector3& gyr)
{
    Eigen::Matrix<double, 17, 1> imustate;
    imustate.setZero();  // Initialize to zero

    imustate(0, 0) = stamp / 1e9;  // Set initial timestamp

    // **Orientation (q_GtoI)**: Assume identity rotation (modify if needed)
    imustate(1, 0) = 1.0;  // Quaternion (w)
    imustate(2, 0) = 0.0;  // Quaternion (x)
    imustate(3, 0) = 0.0;  // Quaternion (y)
    imustate(4, 0) = 0.0;  // Quaternion (z)

    // **Position (p_IinG)**: Start at the origin
    imustate(5, 0) = 0.0;  // x
    imustate(6, 0) = 0.0;  // y
    imustate(7, 0) = 0.0;  // z

    // **Velocity (v_IinG)**: Assume zero initial velocity
    imustate(8, 0) = 0.0;  // x
    imustate(9, 0) = 0.0;  // y
    imustate(10, 0) = 0.0; // z
    
    // **IMU Biases (b_gyro, b_accel)**: Assume initial biases from factory calibration
    imustate(11, 0) = -0.0356587;  // Gyro bias x
    imustate(12, 0) = -0.00267168;  // Gyro bias y
    imustate(13, 0) = 0.0295803;  // Gyro bias z
    imustate(14, 0) = 0.181629;  // Accel bias x
    imustate(15, 0) = -9.94785;  // Accel bias y
    imustate(16, 0) = 9.25731;  // Accel bias z


    vio_manager_->initialize_with_gt(imustate);

    stateInitialised_ = true;

    if (vio_manager_->initialized()) {
        LOG(INFO) << "VIO Manager successfully initialized!";
    } else {
        LOG(ERROR) << "ERROR: VIO Manager failed to initialize!";
    }
}

bool FrontEnd::buildImage(ov_core::CameraData& camera_data, 
    const StampedImage& stamped_image,
    const StampedEventArray& stamped_events,
    const ImuStamps& imu_stamps,
    const ImuAccGyrContainer& imu_accgyr,
    FrameType frame_type)
    {
        double timestamp = stamped_image.first / 1e9;  // Convert nanoseconds to seconds
        ImagePtr imagePtr = stamped_image.second;
    
        if (!imagePtr)
        {
            LOG(ERROR) << "FE: Error: Null ImagePtr!";
            return false;
        }
    
        int width = imagePtr->width;
        int height = imagePtr->height;
        std::string encoding = imagePtr->encoding;
    
        if (imagePtr->data.empty())
        {
            LOG(ERROR) << "FE: Error: Image data is empty!";
            return false;
        }
    
        // Convert APS image (ensure grayscale)
        cv::Mat frame;
        if (encoding == "mono8") {
            frame = cv::Mat(height, width, CV_8UC1, imagePtr->data.data()).clone();
            comms_interface_->queueFrameCamera(frame);
        }
        else 
        {
            LOG(ERROR) << "FE: Unsupported encoding format: " << encoding;
            return false;
        }
    
        cv::Mat processed_frame = frame.clone();
    
        // Process event data if needed
        if (frame_type == EVENT_FRAME || frame_type == COMBINED_FRAME)
        {
            cv::Mat event_frame = cv::Mat::zeros(height, width, CV_8UC1); // Blank event frame
    
            for (const auto& event : *stamped_events.second)
            {
                int x = event.x;
                int y = event.y;
                bool polarity = event.polarity;
    
                if (x >= 0 && x < width && y >= 0 && y < height)
                {
                    event_frame.at<uint8_t>(y, x) = polarity ? 255 : 128; // White for ON, Gray for OFF
                }
            }
            comms_interface_->queueFrameEvents(event_frame);
    
            if (frame_type == EVENT_FRAME)
            {
                processed_frame = event_frame.clone();
            }
            else if (frame_type == COMBINED_FRAME)
            {
                // Blend APS frame and event frame (keeping grayscale)
                cv::addWeighted(frame, 0.5, event_frame, 0.5, 0, processed_frame);
                comms_interface_->queueFrameAugmented(processed_frame);
            }
        }
    
        // Store results in camera data
        camera_data.timestamp = timestamp;
        camera_data.sensor_ids.push_back(0);  // Assuming single-camera setup (ID=0)
        camera_data.images.push_back(processed_frame);
        camera_data.masks.push_back(cv::Mat::zeros(height, width, CV_8UC1)); // Adding blank mask

        // cv::imshow("Processed Frame", processed_frame);
        // cv::waitKey(1);
        return true;
    }

void FrontEnd::addData(
    const StampedImage& stamped_image,
    const StampedEventArray& stamped_events,
    const ImuStamps& imu_stamps,
    const ImuAccGyrContainer& imu_accgyr,
    const bool& no_motion_prior)
{

    //Build Image frame to input to VIO frontend
    ov_core::CameraData camera_data;
    if(!buildImage(camera_data, stamped_image, stamped_events, imu_stamps, imu_accgyr, COMBINED_FRAME))
    {
        LOG(ERROR) << "FE: Error building frame";
        return;
    }

    if(!vioReady_) return;

    vio_manager_->feed_measurement_camera(camera_data);

    if (!vio_manager_->initialized())
    {
        return;
    }
    
    // Log global position - Check
    auto state = vio_manager_->get_state();

    // Check if state is valid before accessing
    if (state && state->_imu)
    {
        Eigen::Vector3d position = state->_imu->pos();  // Global position (x, y, z)
        Eigen::Quaterniond orientation(
            state->_imu->quat()(3),  // w
            state->_imu->quat()(0),  // x
            state->_imu->quat()(1),  // y
            state->_imu->quat()(2)   // z
        );
        
        // Log global pose
        LOG(INFO) << "Global Position: ["
                    << state->_imu->pos()(0) << ", " 
                    << state->_imu->pos()(1) << ", " 
                    << state->_imu->pos()(2) << "]";

        LOG(INFO) << "Global Orientation (Quaternion): ["
                    << orientation.w() << ", "
                    << orientation.x() << ", "
                    << orientation.y() << ", "
                    << orientation.z() << "]";
    }
    else
    {
        LOG(WARNING) << "VIO state is not initialized yet.";
    }


}

void FrontEnd::addImuData(
    int64_t stamp, const Vector3& acc, const Vector3& gyr)
{

    ov_core::ImuData imu_data;
    imu_data.timestamp = static_cast<double>(stamp) / 1e9;  // Convert nanoseconds to seconds

    imu_data.wm << gyr.x(), gyr.y(), gyr.z();  // Angular velocity (rad/s)
    imu_data.am << acc.x(), acc.y(), acc.z();  // Linear acceleration (m/s²)

    if(!vioReady_) return;
    
    vio_manager_->feed_measurement_imu(imu_data);
}
//==============================================================================
// End of File : Software/src/DataAcquisition.cpp
