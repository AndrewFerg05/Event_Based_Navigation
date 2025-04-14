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
cv::Mat previous_event_frame = cv::Mat::zeros(EVENT_FRAME_HEIGHT, EVENT_FRAME_WIDTH, CV_8UC1);
//==============================================================================
// Functions
//------------------------------------------------------------------------------

#include <opencv2/opencv.hpp>
#include <vector>

cv::Mat closeEvents(cv::Mat frame, int dilation_size, int event_type)
{
    cv::Mat binary_mask = cv::Mat::zeros(EVENT_FRAME_HEIGHT, EVENT_FRAME_WIDTH, CV_8UC1);
    binary_mask.setTo(1, frame == event_type);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,  
                        cv::Size(dilation_size, dilation_size));    

    cv::Mat closed;
    cv::morphologyEx(binary_mask, closed, cv::MORPH_CLOSE, kernel);

    frame.setTo(event_type, closed);

    return frame;
}

cv::Mat filterIsolatedEvents(cv::Mat frame, int max_distance, int min_neighbours)
{
    // All Event Mask
    cv::Mat binary_mask = cv::Mat::zeros(EVENT_FRAME_HEIGHT, EVENT_FRAME_WIDTH, CV_32F);

    // ON Event Mask (255)
    cv::Mat on_mask = (frame == 255) / 255; // Convert to binary: 1 = ON, 0 otherwise

    cv::Mat neighbour_count;
    int kernel_size = 2 * max_distance + 1;
    cv::boxFilter(on_mask, neighbour_count, CV_32F, cv::Size(kernel_size, kernel_size), cv::Point(-1, -1), false);
    neighbour_count.setTo(0, on_mask == 0);

    binary_mask += neighbour_count;

    // OFF Event Mask (128)
    cv::Mat off_mask = (frame == 128) / 128; // Convert to binary: 1 = OFF, 0 otherwise

    cv::boxFilter(off_mask, neighbour_count, CV_32F, cv::Size(kernel_size, kernel_size), cv::Point(-1, -1), false);
    neighbour_count.setTo(0, off_mask == 0);

    binary_mask += neighbour_count;

    // Set isolated ON events to background (0)
    cv::Mat mask_on_isolated;
    cv::compare(binary_mask, min_neighbours, mask_on_isolated, cv::CMP_LT);
    frame.setTo(0, mask_on_isolated & (frame == 255));

    // Set isolated OFF events to background (0)
    frame.setTo(0, mask_on_isolated & (frame == 128));
    // frame.setTo(255, frame == 128);
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
        loadFrameType();
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

void FrontEnd::loadFrameType()
    {
        std::string frame_config_file = "../config/frame_type_configuration.yaml";
        int frame_type = 3;
        try {
            YAML::Node frame_config = YAML::LoadFile(frame_config_file);

            // Extract frame Type
            if (frame_config["frame_type"]) {
                frame_type = frame_config["frame_type"].as<int>();
            }
    
            LOG(INFO) << "FE: frane data loaded successfully";
    
        } catch (const std::exception &e) {
            LOG(ERROR) << "FE: Error loading frame file: " << e.what() ;
        }

        switch (frame_type) 
        {
            case 0:  
                frame_config_ = REGULAR_FRAME;
                break;
            case 1:  
                frame_config_ = EVENT_FRAME;
                break;
            case 2:  
                frame_config_ = COMBINED_FRAME;
                break;
            default: 
                frame_config_ = COMBINED_FRAME;
                break;
        }
        
        LOG(INFO) << "FE: Loaded frame Format: " << frame_config_;
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
            // cv::Mat event_frame = cv::Mat::zeros(height, width, CV_8UC1);
            real_t blend_factor = 0.0;
            static real_t smoothed_blend_factor = 0.1;
            const real_t smoothing_alpha = 0.2;  // adjust for responsiveness vs smoothness (1 = no smoothing)
            real_t event_rate; 
    
            if (!stamped_events.second->empty()) 
            {
                const EventArrayPtr& events_ptr = stamped_events.second;
                size_t n_events_for_noise_detection = std::min(events_ptr->size(), size_t(2000));

                event_rate = n_events_for_noise_detection /
                ((events_ptr->back().timestamp_ns -
                  events_ptr->at(events_ptr->size()-n_events_for_noise_detection).timestamp_ns) *1e-9); //Calculate Event/s
                
                 float blend_scale_factor = 1 / FLAGS_max_event_blend;
                  blend_factor = (((event_rate - FLAGS_noise_event_rate) / FLAGS_max_event_rate)) / blend_scale_factor;
                  smoothed_blend_factor = smoothing_alpha * blend_factor + (1.0 - smoothing_alpha) * smoothed_blend_factor;
                  smoothed_blend_factor = std::max(FLAGS_min_event_blend, std::min(FLAGS_max_event_blend, smoothed_blend_factor));
            }

            cv::Mat decay_sum = cv::Mat::zeros(height, width, CV_32FC1);
            cv::Mat decay_count = cv::Mat::zeros(height, width, CV_32FC1);
            
            // Parameters
            const double tau = 5e7; // Decay constant in nanoseconds
            int64_t now_ts = stamped_image.first; // Current image timestamp
            
            for (const auto& event : *stamped_events.second) {
                int x = event.x;
                int y = event.y;
                if (x >= 0 && x < width && y >= 0 && y < height) {
                    double delta_t = static_cast<double>(now_ts - event.timestamp_ns);
                    float decay_value = std::exp(-delta_t / tau);
            
                    decay_sum.at<float>(y, x) += decay_value;
                    decay_count.at<float>(y, x) += 1.0f;
                }
            }

            // Avoid division by zero
            cv::Mat safe_count;
            cv::max(decay_count, 1.0f, safe_count);
            
            // Average decay values per pixel
            cv::Mat time_surface;
            cv::divide(decay_sum, safe_count, time_surface);
            
            // Normalize and convert to 8-bit image
            cv::Mat event_frame;
            cv::normalize(time_surface, event_frame, 0.0, 255.0, cv::NORM_MINMAX);
            event_frame.convertTo(event_frame, CV_8UC1);
            
            // Post Frame Build Processing
            // event_frame = filterIsolatedEvents(event_frame, 3, 10);  //Not applicable with events in TS

            // comms_interface_->queueFrameEvents(event_frame.clone());


            LOG(INFO) << "Rate: " << event_rate;
            // Use previous frame if static or too many events
            if (!event_frame.empty() &&
            (event_rate > FLAGS_noise_event_rate) &&
            (event_rate < FLAGS_event_ignore_threshold))
            {
                previous_event_frame = event_frame.clone();
            }
            else if (!previous_event_frame.empty())
            {
                event_frame = previous_event_frame.clone();
            }
            else
            {
                // As a fallback, initialize with black image
                event_frame = cv::Mat::zeros(height, width, CV_8UC1);
            }
            comms_interface_->queueFrameEvents(event_frame.clone());
            // Decide processed frame to pass in
            if (frame_type == EVENT_FRAME)
            {
                if (!vio_manager_->initialized())
                {
                    //If uninitialised used a combined frame
                    cv::addWeighted(frame, 0.80, event_frame, 0.20, 0, processed_frame);
                }
                else
                {
                    // Use the previous or current event frame
                    processed_frame = event_frame.clone();
                }
            }
            else if (frame_type == COMBINED_FRAME)
            {
                if(!vio_manager_->initialized())
                {
                    // If uninitialised use fixed blended frame
                    cv::addWeighted(frame, 1 - FLAGS_min_event_blend, event_frame, FLAGS_min_event_blend, 0, processed_frame);
                }
                else
                {
                    //Use dynamically blended frame
                    cv::addWeighted(frame, 1-smoothed_blend_factor, event_frame, smoothed_blend_factor, 0, processed_frame);
                }
                comms_interface_->queueFrameAugmented(processed_frame);
            }
        }

        
        // Store results in camera data
        camera_data.timestamp = timestamp;
        camera_data.sensor_ids.push_back(0);  // Assuming single-camera setup (ID=0)
        camera_data.images.push_back(processed_frame);
        camera_data.masks.push_back(cv::Mat::zeros(height, width, CV_8UC1)); // Adding blank mask

        //Tests to display processed frame - Have to run DA in main thread to do this
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

    if(!vioReady_) return;

    //Build Image frame to input to VIO frontend
    ov_core::CameraData camera_data;
    if(!buildImage(camera_data, stamped_image, stamped_events, imu_stamps, imu_accgyr, frame_config_))
    {
        LOG(ERROR) << "FE: Error building frame";
        return;
    }

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
        Eigen::Vector3d velocity = state->_imu->vel();  // Global velocity
        std::vector<Eigen::Vector3d> slam_landmarks = vio_manager_->get_features_SLAM(); //Current SLAM features

        Eigen::Quaterniond orientation(
            state->_imu->quat()(3),  // w
            state->_imu->quat()(0),  // x
            state->_imu->quat()(1),  // y
            state->_imu->quat()(2)   // z
        );
        
        // Log global pose
        // LOG(INFO) << "Global Position: ["
        //             << state->_imu->pos()(0) << ", " 
        //             << state->_imu->pos()(1) << ", " 
        //             << state->_imu->pos()(2) << "]";

        LOG(INFO) << "Global Position: ["
                    << -state->_imu->pos()(1) << ", "   // Correct X to Y
                    << state->_imu->pos()(0) << ", "    // Correct Y to X
                    << state->_imu->pos()(2) << "]";    // Z-axis remains the same


        LOG(INFO) << "Global Orientation (Quaternion): ["
                    << orientation.w() << ", "
                    << orientation.x() << ", "
                    << orientation.y() << ", "
                    << orientation.z() << "]";

        // Pose pose;
        // pose.x = state->_imu->pos()(0);
        // pose.y = state->_imu->pos()(1);
        // pose.z = state->_imu->pos()(2);
        // pose.yaw = orientation.x();
        // pose.pitch = orientation.y();
        // pose.roll = orientation.z();
        // comms_interface_->queuePose(pose);

        Pose pose;
        pose.x = -state->_imu->pos()(1);  // Correct X to Y
        pose.y = state->_imu->pos()(0);   // Correct Y to X
        pose.z = state->_imu->pos()(2);   // Z remains the same
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
