/*
Filename    : Software/src/DataAcquisition.cpp
Author      : Samuel Kliskey
Project     : Event Based Navigation
Date        : 11/1/25
Description : Data Acquisition code for getting DAVIS346 camera data
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
#include "DataAcquisition.hpp"

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
void showImage(const ImageData& imgData) {
    static cv::Mat lastFrame;  // Store last valid frame
    static bool windowCreated = false;

    // Ensure the window is created only once
    if (!windowCreated) {
        cv::namedWindow("Camera Stream", cv::WINDOW_AUTOSIZE);
        windowCreated = true;
    }

    cv::Mat frame;
    
    // Convert image based on encoding
    if (imgData.encoding == "mono8") {
        frame = cv::Mat(imgData.height, imgData.width, CV_8UC1, (void*)imgData.data.data()).clone();
    } else if (imgData.encoding == "rgb8") {
        frame = cv::Mat(imgData.height, imgData.width, CV_8UC3, (void*)imgData.data.data()).clone();
        cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR); // OpenCV uses BGR
    } else {
        std::cerr << "Unsupported encoding format: " << imgData.encoding << std::endl;
        return;
    }

    // Update last valid frame
    lastFrame = frame;

    // Display the last valid frame
    if (!lastFrame.empty()) {
        cv::imshow("Camera Stream", lastFrame);
    }

    cv::waitKey(1); // Needed to update window
}


DataAcquisition::DataAcquisition(std::shared_ptr<DataQueues> data_queues, std::atomic<ThreadState>& state, std::shared_ptr<CommunicationManager> comms)
    : input_data_queues_(data_queues), state_(state), comms_interface_(comms) 
    {
        cur_ev_ = 0;
        last_package_stamp_ = -1;
        last_package_ev_ = 0;
        CHECK_GE(FLAGS_data_size_augmented_event_packet, 0) <<
        "DA: data_size_augmented_event_packet should be positive.";
        initBuffers();
    }

DataAcquisition::~DataAcquisition() {
    stop();
}

void DataAcquisition::start() {
    LOG(INFO) << "DA: Thread started";
    if (acquisition_thread_.joinable()) return;  // Prevent multiple starts
    running_ = true;
    state_ = ThreadState::Run;
    // acquisition_thread_ = std::thread(&DataAcquisition::run, this);
    run();
}

void DataAcquisition::idle() {
    LOG(INFO) << "DA: Resetting queues and entering idle mode...";
    resetQueues();
    imu_buffer_.clear();
    event_buffer_.clear();
    event_packages_ready_to_process_.clear();
    cur_ev_ = 0;
    last_package_stamp_ = -1;
    last_package_ev_ = 0;
    state_ = ThreadState::Idle;
}

void DataAcquisition::stop() {
    running_ = false;
    state_ = ThreadState::Stop;
    if (acquisition_thread_.joinable()) {
        acquisition_thread_.join();
    }
    LOG(INFO) << "DA: Thread stopped";
}

void DataAcquisition::initBuffers() 
{
    imu_buffer_.clear();
    event_buffer_.clear(); 
}


void DataAcquisition::addImageData()
{
    // No image processing done here
}

void DataAcquisition::addEventsData(const EventData& event_data)
{
 
}

void DataAcquisition::addImuData(const IMUData& imu_data)
{
    // const Vector3 gyr(
    // imu_data.angular_velocity.x,
    // imu_data.angular_velocity.y,
    // imu_data.angular_velocity.z);
    // const Vector3 acc(
    // imu_data.linear_acceleration.x,
    // imu_data.linear_acceleration.y,
    // imu_data.linear_acceleration.z);
    // int64_t stamp = imu_data.header.stamp;

    // Vector6 acc_gyr;
    // acc_gyr.head<3>() = acc;
    // acc_gyr.tail<3>() = gyr;
    //stamp -= timeshift_cam_imu_;
    // imu_buffer_[0].insert(stamp, acc_gyr);

    // if (imu_callback_)
    // {
    //     imu_callback_(stamp, acc, gyr);
    // }

    // checkImuDataAndCallback();
}

bool DataAcquisition::processDataQueues()
{
        bool processed = false;

        auto imu_data = input_data_queues_->imu_queue->pop();
        if (imu_data.has_value()) {
            addImuData(imu_data.value());
            processed = true;
        }

        auto event_data = input_data_queues_->event_queue->pop();
        if (event_data.has_value()) {
            addEventsData(event_data.value());
            processed = true;
        }

        auto image_data = input_data_queues_->image_queue->pop();
        if (image_data) {
            showImage(image_data.value());
            addImageData();
            processed = true;
        }

        return processed;

}

void DataAcquisition::resetQueues()
{
    if (!input_data_queues_) {
        LOG(ERROR) << "DA: Data queues not initialized!";
        return;
    }

    LOG(INFO) << "DA: Resetting all data queues...";
    
    input_data_queues_->event_queue->clear();
    input_data_queues_->camera_info_queue->clear();
    input_data_queues_->imu_queue->clear();
    input_data_queues_->image_queue->clear();
    input_data_queues_->exposure_queue->clear();

    LOG(INFO) << "DA: All queues cleared";
}

void DataAcquisition::extractAndEraseEvents()
{

}

void DataAcquisition::checkImuDataAndCallback()
{
  
}


bool DataAcquisition::validateImuBuffers(
    const int64_t& min_stamp,
    const int64_t& max_stamp,
    const std::vector<std::tuple<int64_t, int64_t, bool> >&
    oldest_newest_stamp_vector)
{
    // Check if we have received some IMU measurements for at least one of the imu's.
    if (std::none_of(oldest_newest_stamp_vector.begin(),
                    oldest_newest_stamp_vector.end(),
                    [](const std::tuple<int64_t, int64_t, bool>& oldest_newest_stamp)
                    {
                        if (std::get<2>(oldest_newest_stamp))
                        {
                            return true;
                        }
                            return false;
                    }))        
    {
    return false;
    }

    // At least one IMU measurements before image
    if (std::none_of(oldest_newest_stamp_vector.begin(),
                    oldest_newest_stamp_vector.end(),
                    [&min_stamp](const std::tuple<int64_t, int64_t, bool>& oldest_newest_stamp)
                    {
                        if (std::get<0>(oldest_newest_stamp) < min_stamp) {
                            return true;
                        }
                    return false;
                    }))
    {
    return false;
    }

    // At least one IMU measurements after image
    if (std::none_of(oldest_newest_stamp_vector.begin(),
                    oldest_newest_stamp_vector.end(),
                    [&max_stamp](const std::tuple<int64_t, int64_t, bool>& oldest_newest_stamp) 
                    {
                        if (std::get<1>(oldest_newest_stamp) > max_stamp)
                        {
                            return true;
                        }
                    return false;
                    }))
    {
        return false;
    }
    
    return true;
}

void DataAcquisition::run() 
{
    while (running_) 
    {
        if (state_ == ThreadState::Idle) 
        {
            sleep_ms(10);  // Avoid busy waiting
            continue;
        }

        if (state_ == ThreadState::Stop) 
        {
            break;
        }

        if (state_ == ThreadState::Run)
        {
            if (!processDataQueues()) 
            {
                LOG(INFO) << "DA: No data in queues";
                sleep_ms(100);
            }
        }
    }
}



//==============================================================================
// End of File : Software/src/DataAcquisition.cpp
