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

// Function to display events from each incoming packet separately
void displayStampedEvents(const StampedEventArray& stamped_events) {
    // Clear the event frame at the start of each function call
    event_frame = cv::Mat::zeros(EVENT_FRAME_HEIGHT, EVENT_FRAME_WIDTH, CV_32FC3);

    // Extract timestamp and event array pointer
    int64_t timestamp = stamped_events.first;
    EventArrayPtr eventArrayPtr = stamped_events.second;

    if (!eventArrayPtr || eventArrayPtr->empty()) {
        std::cerr << "Warning: No events received at timestamp " << timestamp << std::endl;
        return;
    }

    // Iterate through all events and plot them on the event_frame
    for (const Event& e : *eventArrayPtr) {
        if (e.x >= EVENT_FRAME_WIDTH || e.y >= EVENT_FRAME_HEIGHT) {
            continue;  // Ignore out-of-bounds events
        }

        // Set pixel intensity based on event polarity
        if (e.polarity) {
            event_frame.at<cv::Vec3f>(e.y, e.x) = cv::Vec3f(1.0f, 1.0f, 1.0f); // ON events → White
        } else {
            event_frame.at<cv::Vec3f>(e.y, e.x) = cv::Vec3f(0.0f, 0.0f, 1.0f); // OFF events → Blue
        }
    }

    // Convert event frame to 8-bit (CV_8UC3)
    cv::Mat event_frame_8bit;
    event_frame.convertTo(event_frame_8bit, CV_8UC3, 255.0); // Scale float to 8-bit

    // Display the cleaned event frame
    cv::imshow("Processed Event Frame", event_frame_8bit);

    // Short delay for smooth video playback
    cv::waitKey(1);
}

// Function to display images in a continuous video stream
void displayStampedImage(const StampedImage& stamped_image) {
    // Extract timestamp and image pointer
    int64_t timestamp = stamped_image.first;
    ImagePtr imagePtr = stamped_image.second;

    if (!imagePtr) {
        std::cerr << "Error: Image pointer is null!" << std::endl;
        return;
    }

    // Extract image properties
    int width = imagePtr->width;
    int height = imagePtr->height;
    std::string encoding = imagePtr->encoding;
    std::vector<uint8_t> data = imagePtr->data;

    if (data.empty()) {
        std::cerr << "Error: Image data is empty!" << std::endl;
        return;
    }

    // Convert the data vector to cv::Mat
    if (encoding == "mono8") {
        aps_frame = cv::Mat(height, width, CV_8UC1, data.data());
        cv::cvtColor(aps_frame, aps_frame, cv::COLOR_GRAY2BGR);  // Convert grayscale to 3-channel BGR
    } else if (encoding == "rgb8") {
        cv::Mat temp = cv::Mat(height, width, CV_8UC3, data.data());
        cv::cvtColor(temp, aps_frame, cv::COLOR_RGB2BGR); // Convert RGB to OpenCV's BGR
    } else {
        std::cerr << "Error: Unsupported encoding format: " << encoding << std::endl;
        return;
    }

    // Display the APS image
    cv::imshow("Stamped Image Stream", aps_frame);
    
    // Short delay to allow OpenCV to refresh the display
    cv::waitKey(1);  // 1ms delay, ensures smooth video playback
}

void displayCombinedFrame() {
    if (aps_frame.empty() || event_frame.empty()) {
        std::cerr << "Warning: One or both frames are empty!" << std::endl;
        return;
    }

    // Ensure both images are the same size
    if (aps_frame.size() != event_frame.size()) {
        cv::resize(aps_frame, aps_frame, event_frame.size());
    }

    // Convert event_frame to 8-bit (CV_8UC3) for blending
    cv::Mat event_frame_8bit;
    event_frame.convertTo(event_frame_8bit, CV_8UC3, 255.0); // Scale float to 8-bit

    // Blend APS frame with event frame
    cv::Mat combined_frame;
    cv::addWeighted(aps_frame, 0.7, event_frame_8bit, 0.5, 0, combined_frame);
    return;
    // Display the combined frame
    cv::imshow("Combined Event + APS Frame", combined_frame);

    // Short delay for smooth video playback
    cv::waitKey(1);
}

//==============================================================================
// Functions
//------------------------------------------------------------------------------


FrontEnd::FrontEnd(std::shared_ptr<CommunicationManager> comms, const std::string& config_path)
    : comms_interface_(comms), config_path_(config_path)
    {
        setupVIO();
        loadCalibrationData();
    }

void FrontEnd::start()
{

}

void FrontEnd::idle()
{
    
}

void FrontEnd::stop()
{
    
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
}

void FrontEnd::loadCalibrationData()
{
    std::string imu_cam_file = "../config/kalibr_imucam_chain.yaml";
    std::string imu_file = "../config/kalibr_imu_chain.yaml";

    try {
        YAML::Node imu_cam_config = YAML::LoadFile(imu_cam_file);
        YAML::Node imu_config = YAML::LoadFile(imu_file);

        // Extract T_cam_imu (Camera-to-IMU transformation)
        if (imu_cam_config["cam0"]["T_cam_imu"]) {
            auto t = imu_cam_config["cam0"]["T_cam_imu"];
            for (int i = 0; i < 4; ++i)
                for (int j = 0; j < 4; ++j)
                    calib_.T_cam_imu(i, j) = t[i][j].as<float>();
        }

        // Extract T_imu_body (IMU-to-body transformation)
        if (imu_config["imu0"]["T_i_b"]) {
            auto t = imu_config["imu0"]["T_i_b"];
            for (int i = 0; i < 4; ++i)
                for (int j = 0; j < 4; ++j)
                    calib_.T_imu_body(i, j) = t[i][j].as<float>();
        } else {
            calib_.T_imu_body = Eigen::Matrix4f::Identity(); // Default to Identity
        }

        // Extract distortion coefficients
        if (imu_cam_config["cam0"]["distortion_coeffs"]) {
            auto d = imu_cam_config["cam0"]["distortion_coeffs"];
            for (int i = 0; i < 4; ++i)
                calib_.distortion_coeffs(i) = d[i].as<float>();
        }

        // Extract camera intrinsics
        if (imu_cam_config["cam0"]["intrinsics"]) {
            auto intrinsics = imu_cam_config["cam0"]["intrinsics"];
            calib_.K << intrinsics[0].as<float>(), 0.0, intrinsics[2].as<float>(),
                        0.0, intrinsics[1].as<float>(), intrinsics[3].as<float>(),
                        0.0, 0.0, 1.0;
        }

        // Extract camera resolution
        if (imu_cam_config["cam0"]["resolution"]) {
            calib_.width = imu_cam_config["cam0"]["resolution"][0].as<int>();
            calib_.height = imu_cam_config["cam0"]["resolution"][1].as<int>();
        }

        // Extract time shift between camera and IMU
        if (imu_cam_config["cam0"]["timeshift_cam_imu"]) {
            calib_.timeshift_cam_imu = imu_cam_config["cam0"]["timeshift_cam_imu"].as<float>();
        }

        LOG(INFO) << "FE: Calibration data loaded successfully";

    } catch (const std::exception &e) {
        LOG(ERROR) << "FE: Error loading calibration file: " << e.what() ;
    }
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
            //==========================================================================================
            // New event processing
            Eigen::Isometry3d T_Bkm1_Bk = Eigen::Isometry3d::Identity();

            // Get projected translation bewteen state n-1 and state n
            if(prevState_.stateInit)
            {
                if(imu_stamps.size() > 0)
                {
                    Eigen::Isometry3d T_W_B = prevState_.T_Bk_W.inverse(); // Invert previous pose
                    Eigen::Quaterniond q(T_W_B.rotation()); // Extract rotation as a quaternion
                    Eigen::Vector3d p = T_W_B.translation(); // Extract translation
                    Eigen::Vector3d v = prevState_.velocity;
                    Vector3 g_ = Vector3(0., 0., 9.81);
                    Vector3 acc_bias_ = Vector3::Zero();
                    Vector3 gyr_bias_ = Vector3::Zero();

                    // Vector3 acc_bias_ = Vector3(0.181629, -9.94785, 9.25731);
                    // Vector3 gyr_bias_ = Vector3( -0.0356587, -0.00267168, 0.0295803);

                                    
                    for(int i = 0; i < (imu_stamps.size() - 1); ++i)
                    {
                        real_t dt = static_cast<real_t>(imu_stamps(i+1) - imu_stamps(i)) / 1e9;

                        // Convert angular velocity to quaternion update
                        Eigen::Vector3d omega = imu_accgyr.block<3,1>(3,i) - gyr_bias_;  // Gyroscope measurement corrected for bias
                        Eigen::AngleAxisd delta_rotation(omega.norm() * dt, omega.normalized()); // Convert to small rotation update
                        q = q * Eigen::Quaterniond(delta_rotation);  // Apply rotation update

                        // Position update using velocity
                        p = p + v * dt;

                        // Velocity update using acceleration
                        Eigen::Vector3d acc_corrected = imu_accgyr.block<3,1>(0,i) - acc_bias_;  // Correct acceleration
                        v = v + (q * acc_corrected - g_) * dt;  // Apply rotation and subtract gravity
                    }
                    
                    T_W_B.linear() = q.toRotationMatrix();  // Assign updated rotation
                    T_W_B.translation() = p;  // Assign updated position
                    T_Bkm1_Bk = prevState_.T_Bk_W * T_W_B;
                }
            }

            const int64_t& event_frame_timestamp = stamped_image.first;

            const int64_t& t0 = imu_stamps(0);  // First element
            const int64_t& t1 = imu_stamps(imu_stamps.rows() - 1);  // Last element

            cv::Mat event_frame = cv::Mat::zeros(height, width, CV_32F); // Blank event frame

            if (!stamped_events.second->empty()) 
            {
                const EventArrayPtr& events_ptr = stamped_events.second;
                size_t n_events_for_noise_detection = std::min(events_ptr->size(), size_t(2000));

                real_t event_rate = n_events_for_noise_detection /
                ((events_ptr->back().timestamp_ns -
                  events_ptr->at(events_ptr->size()-n_events_for_noise_detection).timestamp_ns) *1e-9); //Calculate Event/s

                  if (event_rate >= FLAGS_noise_event_rate)
                  {
                          // Build event frame with fixed number of events
                        const size_t winsize_events = FLAGS_vio_frame_size;
                        int first_idx = std::max((int)events_ptr->size() - (int) winsize_events, 0);

                        uint64_t frame_length =
                        events_ptr->back().timestamp_ns - events_ptr->at(first_idx).timestamp_ns;

                        if(events_ptr->size() < winsize_events)
                        {
                          LOG(WARNING) << "Requested frame size of length " << winsize_events
                                        << " events, but I only have "
                                        << events_ptr->size()
                                        << " events in the last event array";
                        }

                        Eigen::Isometry3d T_C_B = Eigen::Isometry3d::Identity();
                        T_C_B.matrix() = calib_.T_cam_imu;

                        // Compute the transformation
                        Eigen::Isometry3d T_1_0 = T_C_B * T_Bkm1_Bk.inverse() * T_C_B.inverse();

                        drawEvents(
                            events_ptr->begin()+first_idx,
                            events_ptr->end(),
                            t0, t1,
                            T_1_0,
                            event_frame);
                  }


            }
            else
            {
                LOG(WARNING) << "FE: No events in packet";
            }
            //==========================================================================================
            // Old event processing

            // cv::Mat event_frame = cv::Mat::zeros(height, width, CV_8UC1); // Blank event frame
    
            // // Dereference shared pointer before iteration
            // for (const auto& event : *stamped_events.second)
            // {
            //     int x = event.x;
            //     int y = event.y;
            //     bool polarity = event.polarity;
    
            //     if (x >= 0 && x < width && y >= 0 && y < height)
            //     {
            //         event_frame.at<uint8_t>(y, x) = polarity ? 255 : 128; // White for ON, Gray for OFF
            //     }
            // }
            //==========================================================================================
            if (frame_type == EVENT_FRAME)
            {
                processed_frame = event_frame.clone();
            }
            else if (frame_type == COMBINED_FRAME)
            {
                // Blend APS frame and event frame (keeping grayscale)
                cv::addWeighted(frame, 0.5, event_frame, 0.5, 0, processed_frame);
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


void FrontEnd::drawEvents(
    const EventArray::iterator& first,
    const EventArray::iterator& last,
    const int64_t& t0,
    const int64_t& t1,
    const Eigen::Isometry3d& T_1_0,
    cv::Mat &out)
{

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
        Eigen::Vector3d velocity = state->_imu->vel();


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
                            // Update previous state

        prevState_.velocity = velocity;
        prevState_.T_Bk_W.linear() = orientation.toRotationMatrix();  // Set rotation
        prevState_.T_Bk_W.translation() = position;  // Set translation
        prevState_.stateInit = true;

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

    vio_manager_->feed_measurement_imu(imu_data);
}
//==============================================================================
// End of File : Software/src/DataAcquisition.cpp
