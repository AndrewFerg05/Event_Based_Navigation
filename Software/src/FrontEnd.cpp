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

#include <opencv2/opencv.hpp>
#include <vector>

cv::Mat filterIsolatedEvents(cv::Mat frame, int max_distance, int min_neighbors)
{
    cv::Mat binary_mask = (frame != 128);  // Convert to binary (1 = event, 0 = background)

    cv::Mat neighbor_count;
    int kernel_size = 2 * max_distance + 1;
    cv::boxFilter(binary_mask, neighbor_count, CV_32F, cv::Size(kernel_size, kernel_size));

    // Remove pixels that don't meet the min_neighbors requirement
    frame.setTo(128, (neighbor_count < min_neighbors) & binary_mask);

    return frame;
}

cv::Mat motionComp_byAvg(cv::Mat aps, cv::Mat events) {
    int m = aps.rows;
    int n = aps.cols;
    
    for (int i = 0; i < m; i++)
    {
        events.at<uchar>(i,0) = 255;
        events.at<uchar>(i,n-1) = 255;
    }

    for (int i = 0; i < n; i++)
    {
        events.at<uchar>(0,i) = 255;
        events.at<uchar>(m-1,i) = 255;
    }

    cv::Mat hoizontalComp = cv::Mat::zeros(m, n, CV_8UC1);
    cv::Mat verticalComp = cv::Mat::zeros(m, n, CV_8UC1);
    cv::Mat diagRLComp = cv::Mat::zeros(m, n, CV_8UC1);
    cv::Mat diagLRComp = cv::Mat::zeros(m, n, CV_8UC1);

    // Determine average horizontally
    for (int x = 0; x < m; x++) {
        int sum = 0;
        int start = 0;
        for (int y = 0; y < n; y++) {
            sum += aps.at<uchar>(x, y);
            if (events.at<uchar>(x, y) == 255) {
                int avg = sum / (y - start + 1);
                for (int k = start; k <= y; k++) {
                    hoizontalComp.at<uchar>(x, k) = static_cast<uchar>(avg);
                }
                start = y + 1;
                sum = 0;
            }
        }
    }

    // Determine average vertically
    for (int y = 0; y < n; y++) {
        int sum = 0;
        int start = 0;
        for (int x = 0; x < m; x++) {
            sum += aps.at<uchar>(x, y);
            if (events.at<uchar>(x, y) == 255) {
                int avg = sum / (x - start + 1);
                for (int k = start; k <= x; k++) {
                    verticalComp.at<uchar>(k, y) = static_cast<uchar>(avg);
                }
                start = x + 1;
                sum = 0;
            }
        }
    }

    // Diagonal average LR (\)
    for (int d = 0; d < (m + n - 1); ++d) {
        int sum = 0;
        std::vector<cv::Point> indices;
        for (int x = std::max(0, d - n + 1); x < std::min(m, d + 1); ++x) {
            int y = d - x;
            sum += aps.at<uchar>(x, y);
            indices.emplace_back(x, y);
            if (events.at<uchar>(x, y) == 255) {
                int avg = sum / indices.size();
                for (const auto& idx : indices) {
                    diagLRComp.at<uchar>(idx.x, idx.y) = static_cast<uchar>(avg);
                }
                sum = 0;
                indices.clear();
            }
        }
    }

    // Diagonal average RL (/)
    for (int d = 0; d < (m + n - 1); ++d) {
        int sum = 0;
        std::vector<cv::Point> indices;
        for (int x = std::max(0, d - n + 1); x < std::min(m, d + 1); ++x) {
            int y = n - 1 - (d - x);
            sum += aps.at<uchar>(x, y);
            indices.emplace_back(x, y);
            if (events.at<uchar>(x, y) == 255) {
                int avg = sum / indices.size();
                for (const auto& idx : indices) {
                    diagRLComp.at<uchar>(idx.x, idx.y) = static_cast<uchar>(avg);
                }
                sum = 0;
                indices.clear();
            }
        }
    }

    // Combine the four processed images
    cv::Mat motionComp = (hoizontalComp + verticalComp + diagLRComp + diagRLComp) / 4;
    return motionComp;
}


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
            cv::Mat event_frame = cv::Mat::ones(height, width, CV_8UC1) * 128; // gray event frame
    
            for (const auto& event : *stamped_events.second)
            {
                int x = event.x;
                int y = event.y;
                bool polarity = event.polarity;
    
                if (x >= 0 && x < width && y >= 0 && y < height)
                {
                    event_frame.at<uint8_t>(y, x) = polarity ? 255 : 0; // White for ON, black for OFF
                }
            }
            cv::Mat newFrame = filterIsolatedEvents(event_frame.clone(), 1, 80);
            comms_interface_->queueFrameEvents(newFrame.clone());
    
            if (frame_type == EVENT_FRAME)
            {
                processed_frame = newFrame.clone();
            }
            else if (frame_type == COMBINED_FRAME)
            {
                // Blend APS frame and event frame (keeping grayscale)
                newFrame.setTo(255, newFrame == 0);
                
                // newFrame.setTo(0, newFrame == 128);
                // cv::addWeighted(frame, 0.75, newFrame, 0.25, 0, processed_frame);

                processed_frame = motionComp_byAvg(frame, event_frame);

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

        Pose pose;
        pose.x = state->_imu->pos()(0);
        pose.y = state->_imu->pos()(1);
        pose.z = state->_imu->pos()(2);
        pose.yaw = orientation.x();
        pose.pitch = orientation.y();
        pose.roll = orientation.z();
        comms_interface_->queuePose(pose);
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
