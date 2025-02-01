/*
Filename    : Software/src/DavisDriver.hpp
Author      : Andrew Ferguson
Project     : Event Based Navigation
Date        : 28/01/25
Description : Source file for the Davis Camera Driver
--------------------------------------------------------------------------------
--------------------------------------------------------------------------------
*/

//==============================================================================
// External Files
//------------------------------------------------------------------------------

// Local
#include "DavisDriver.hpp"


//==============================================================================
// MACROS
//------------------------------------------------------------------------------
#define CF_N_TYPE(COARSE, FINE) (struct caer_bias_coarsefine) \
{ .coarseValue = (uint8_t)(COARSE), .fineValue = (uint8_t)(FINE), \
    .enabled = true, .sexN = true, \
    .typeNormal = true, .currentLevelNormal = true }

#define CF_P_TYPE(COARSE, FINE) (struct caer_bias_coarsefine) \
{ .coarseValue = (uint8_t)(COARSE), .fineValue = (uint8_t)(FINE), \
    .enabled = true, .sexN = false, \
    .typeNormal = true, .currentLevelNormal = true }

#define SHIFTSOURCE(REF, REG, OPMODE) (struct caer_bias_shiftedsource) \
{ .refValue = (uint8_t)(REF), .regValue = (uint8_t)(REG), \
    .operatingMode = (caer_bias_shiftedsource_operating_mode)(OPMODE), .voltageLevel = (caer_bias_shiftedsource_voltage_level)(SPLIT_GATE) }

#define VDAC(VOLT, CURR) (struct caer_bias_vdac) \
{ .voltageValue = (uint8_t)(VOLT), .currentValue = (uint8_t)(CURR) }
//==============================================================================
// Functions
//------------------------------------------------------------------------------

ConfigManager::ConfigManager(const std::string& config_path) 
    : config_file_path(config_path) {}

void ConfigManager::loadConfig(const std::string& config_path){  
    try {
        YAML::Node config = YAML::LoadFile(config_path);

        // Device ID
        device_id_ = config["serial_number"] ? config["serial_number"].as<std::string>() : "UNKNOWN";
        reset_timestamps_delay_ = config["reset_timestamps_delay"] ? config["reset_timestamps_delay"].as<double>() : -1.0;
        imu_calibration_sample_size_ = config["imu_calibration_sample_size"] ? config["imu_calibration_sample_size"].as<int>() : 1000;

        // IMU Biases
        bias_.linear_acceleration.x = config["imu_bias"]["ax"] ? config["imu_bias"]["ax"].as<double>() : 0.0;
        bias_.linear_acceleration.y = config["imu_bias"]["ay"] ? config["imu_bias"]["ay"].as<double>() : 0.0;
        bias_.linear_acceleration.z = config["imu_bias"]["az"] ? config["imu_bias"]["az"].as<double>() : 0.0;
        bias_.angular_velocity.x = config["imu_bias"]["wx"] ? config["imu_bias"]["wx"].as<double>() : 0.0;
        bias_.angular_velocity.y = config["imu_bias"]["wy"] ? config["imu_bias"]["wy"].as<double>() : 0.0;
        bias_.angular_velocity.z = config["imu_bias"]["wz"] ? config["imu_bias"]["wz"].as<double>() : 0.0;

        // Camera
        aps_enabled = config["aps_enabled"] ? config["aps_enabled"].as<bool>() : true;
        dvs_enabled = config["dvs_enabled"] ? config["dvs_enabled"].as<bool>() : true;
        imu_enabled = config["imu_enabled"] ? config["imu_enabled"].as<bool>() : true;

        // Auto-exposure parameters
        autoexposure_desired_intensity = config["autoexposure_desired_intensity"] ? config["autoexposure_desired_intensity"].as<float>() : 128.0f;
        autoexposure_gain = config["autoexposure_gain"] ? config["autoexposure_gain"].as<float>() : 100.0f;


        frame_mode = config["frame_mode"] ? config["frame_mode"].as<int>() : 0;
        frame_interval = config["frame_interval"] ? config["frame_interval"].as<int>() : 0;

        exposure = config["exposure"] ? config["exposure"].as<int>() : 4000;
        frame_delay = config["frame_delay"] ? config["frame_delay"].as<int>() : 0;
        max_events = config["max_events"] ? config["max_events"].as<int>() : 0;
        streaming_rate = config["streaming_rate"] ? config["streaming_rate"].as<int>() : 30;

        imu_acc_scale = config["imu_acc_scale"] ? config["imu_acc_scale"].as<int>() : 3;
        imu_gyro_scale = config["imu_gyro_scale"] ? config["imu_gyro_scale"].as<int>() : 3;
        imu_low_pass_filter = config["imu_low_pass_filter"] ? config["imu_low_pass_filter"].as<int>() : 0;
        imu_sample_rate_divider = (imu_low_pass_filter == 0) ? 7 : 0;



        // ADC Parameters
        ADC_RefHigh_volt = config["ADC_RefHigh_volt"] ? config["ADC_RefHigh_volt"].as<int>() : 24;
        ADC_RefHigh_curr = config["ADC_RefHigh_curr"] ? config["ADC_RefHigh_curr"].as<int>() : 7;
        ADC_RefLow_volt = config["ADC_RefLow_volt"] ? config["ADC_RefLow_volt"].as<int>() : 1;
        ADC_RefLow_curr = config["ADC_RefLow_curr"] ? config["ADC_RefLow_curr"].as<int>() : 7;

        // Bias Parameters
        DiffBn_coarse = config["DiffBn_coarse"] ? config["DiffBn_coarse"].as<int>() : 4;
        DiffBn_fine = config["DiffBn_fine"] ? config["DiffBn_fine"].as<int>() : 39;

        OFFBn_coarse = config["OFFBn_coarse"] ? config["OFFBn_coarse"].as<int>() : 4;
        OFFBn_fine = config["OFFBn_fine"] ? config["OFFBn_fine"].as<int>() : 0;

        ONBn_coarse = config["ONBn_coarse"] ? config["ONBn_coarse"].as<int>() : 6;
        ONBn_fine = config["ONBn_fine"] ? config["ONBn_fine"].as<int>() : 255;

        PrBp_coarse = config["PrBp_coarse"] ? config["PrBp_coarse"].as<int>() : 2;
        PrBp_fine = config["PrBp_fine"] ? config["PrBp_fine"].as<int>() : 255;

        PrSFBp_coarse = config["PrSFBp_coarse"] ? config["PrSFBp_coarse"].as<int>() : 1;
        PrSFBp_fine = config["PrSFBp_fine"] ? config["PrSFBp_fine"].as<int>() : 199;

        RefrBp_coarse = config["RefrBp_coarse"] ? config["RefrBp_coarse"].as<int>() : 3;
        RefrBp_fine = config["RefrBp_fine"] ? config["RefrBp_fine"].as<int>() : 7;

         // Hardware Filters (DVS)
        pixel_auto_train = config["pixel_auto_train"] ? config["pixel_auto_train"].as<bool>() : false;
        pixel_0_row = config["pixel_0_row"] ? config["pixel_0_row"].as<int>() : 0;
        pixel_0_column = config["pixel_0_column"] ? config["pixel_0_column"].as<int>() : 0;
        pixel_1_row = config["pixel_1_row"] ? config["pixel_1_row"].as<int>() : 0;
        pixel_1_column = config["pixel_1_column"] ? config["pixel_1_column"].as<int>() : 0;
        pixel_2_row = config["pixel_2_row"] ? config["pixel_2_row"].as<int>() : 0;
        pixel_2_column = config["pixel_2_column"] ? config["pixel_2_column"].as<int>() : 0;
        pixel_3_row = config["pixel_3_row"] ? config["pixel_3_row"].as<int>() : 0;
        pixel_3_column = config["pixel_3_column"] ? config["pixel_3_column"].as<int>() : 0;

        // Background Activity Filter (OFF)
        background_activity_filter_enabled = config["background_activity_filter_enabled"] ? config["background_activity_filter_enabled"].as<bool>() : false;
        background_activity_filter_time = config["background_activity_filter_time"] ? config["background_activity_filter_time"].as<int>() : 0;

        // Refractory Period Filter (OFF)
        refractory_period_enabled = config["refractory_period_enabled"] ? config["refractory_period_enabled"].as<bool>() : false;
        refractory_period_time = config["refractory_period_time"] ? config["refractory_period_time"].as<int>() : 0;

        // ROI Filter (Defaults to full frame)
        roi_start_column = config["roi_start_column"] ? config["roi_start_column"].as<int>() : 0;
        roi_start_row = config["roi_start_row"] ? config["roi_start_row"].as<int>() : 0;
        roi_end_column = config["roi_end_column"] ? config["roi_end_column"].as<int>() : 0;
        roi_end_row = config["roi_end_row"] ? config["roi_end_row"].as<int>() : 0;

        // Skip Events Filter (OFF)
        skip_enabled = config["skip_enabled"] ? config["skip_enabled"].as<bool>() : false;
        skip_every = config["skip_every"] ? config["skip_every"].as<int>() : 1;

        // Polarity Filter (OFF)
        polarity_flatten = config["polarity_flatten"] ? config["polarity_flatten"].as<bool>() : false;
        polarity_suppress = config["polarity_suppress"] ? config["polarity_suppress"].as<bool>() : false;
        polarity_suppress_type = config["polarity_suppress_type"] ? config["polarity_suppress_type"].as<int>() : 0;

        // APS Region of Interest
        aps_roi_start_column = config["aps_roi_start_column"] ? config["aps_roi_start_column"].as<int>() : 0;
        aps_roi_start_row = config["aps_roi_start_row"] ? config["aps_roi_start_row"].as<int>() : 0;
        aps_roi_end_column = config["aps_roi_end_column"] ? config["aps_roi_end_column"].as<int>() : 0;
        aps_roi_end_row = config["aps_roi_end_row"] ? config["aps_roi_end_row"].as<int>() : 0;

        std::cout << "Loaded parameters successfully from " << config_file_path << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error loading YAML config: " << e.what() << std::endl;
    }
}

DavisDriver::DavisDriver(const std::string& config_path, std::shared_ptr<DataQueues> data_queues) :
    parameter_update_required_(false),
    parameter_bias_update_required_(false),
    imu_calibration_running_(false),
    data_queues_(data_queues),
     config_manager_(config_path)
{
    config_manager_.loadConfig(config_path);
    caerConnect();
    config_manager_.streaming_rate = 30;
    bias = config_manager_.getBias();
    delta_ = std::chrono::microseconds(static_cast<long>(1e6 / config_manager_.streaming_rate));
}

DavisDriver::~DavisDriver()
{
    running_ = false; // Stop the threads gracefully
    if (parameter_thread_.joinable()) {
        parameter_thread_.join();
    }
    if (readout_thread_.joinable()) {
        readout_thread_.join();
    }
}

void DavisDriver::caerConnect()
{
    bool device_is_running = false;

    while (!device_is_running) {
        const char* serial_number_restrict = (config_manager_.device_id_.empty()) ? nullptr : config_manager_.device_id_.c_str();

        if (serial_number_restrict) {
            std::cerr << "Requested serial number: " << config_manager_.device_id_ << std::endl;
        }

        // Attempt to open the DAVIS camera
        davis_handle_ = caerDeviceOpen(1, CAER_DEVICE_DAVIS, 0, 0, serial_number_restrict);

        // Check if device was successfully opened
        device_is_running = (davis_handle_ != nullptr);

        if (!device_is_running) {
            std::cerr << "Could not find DAVIS. Retrying every second..." << std::endl;
            std::this_thread::sleep_for(std::chrono::microseconds(500));  // Wait before retrying
        }

        // Exit if the driver is manually stopped (replace with actual stopping condition)
        if (!running_) {
            std::cerr << "Driver stopped. Exiting startup loop." << std::endl;
            return;
        }
    }
    // Retrieve device information
    davis_info_ = caerDavisInfoGet(davis_handle_);
    config_manager_.device_id_ = "DAVIS-" + std::string(davis_info_.deviceString).substr(14, 8);

    // Print device info
    std::cout << davis_info_.deviceString << " --- ID: " << davis_info_.deviceID
            << ", Master: " << davis_info_.deviceIsMaster
            << ", DVS X: " << davis_info_.dvsSizeX
            << ", DVS Y: " << davis_info_.dvsSizeY
            << ", Logic: " << davis_info_.firmwareVersion << std::endl;

    // Send the default configuration before using the device.
    caerDeviceSendDefaultConfig(davis_handle_);;
    // Set default exposure
    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_EXPOSURE, config_manager_.exposure);

    parameter_bias_update_required_ = true;
    parameter_update_required_ = true;

    running_ = true; // Check this Runs

    //parameter_thread_ = std::thread(&DavisDriver::changeDvsParameters, this); //Add while loop back into function if calling in other thread
    changeDvsParameters(); // Might not need to run in other thread if not dynamicall configuring 
    readout_thread_ = std::thread(&DavisDriver::readout, this);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

}

void DavisDriver::updateImuBias()
{
    bias.linear_acceleration.x = 0.0;
    bias.linear_acceleration.y = 0.0;
    bias.linear_acceleration.z = 0.0;
    bias.angular_velocity.x = 0.0;
    bias.angular_velocity.y = 0.0;
    bias.angular_velocity.z = 0.0;

    for (auto m : imu_calibration_samples_)
    {
        bias.linear_acceleration.x += m.linear_acceleration.x;
        bias.linear_acceleration.y += m.linear_acceleration.y;
        bias.linear_acceleration.z += m.linear_acceleration.z;
        bias.angular_velocity.x += m.angular_velocity.x;
        bias.angular_velocity.y += m.angular_velocity.y;
        bias.angular_velocity.z += m.angular_velocity.z;
    }

    bias.linear_acceleration.x /= (double) imu_calibration_samples_.size();
    bias.linear_acceleration.y /= (double) imu_calibration_samples_.size();
    bias.linear_acceleration.z /= (double) imu_calibration_samples_.size();
    bias.linear_acceleration.z -= STANDARD_GRAVITY * ((bias.linear_acceleration.z > 0) ? 1 : -1);

    bias.angular_velocity.x /= (double) imu_calibration_samples_.size();
    bias.angular_velocity.y /= (double) imu_calibration_samples_.size();
    bias.angular_velocity.z /= (double) imu_calibration_samples_.size();

    std::cout << "IMU calibration done.\n";
    std::cout << "Acceleration biases: " 
              << bias.linear_acceleration.x << " "
              << bias.linear_acceleration.y << " "
              << bias.linear_acceleration.z << " [m/s^2]" << std::endl;

    std::cout << "Gyroscope biases: "
              << bias.angular_velocity.x << " "
              << bias.angular_velocity.y << " "
              << bias.angular_velocity.z << " [rad/s]" << std::endl;
}

void DavisDriver::triggerImuCalibration()
{
    std::cout << "Starting IMU calibration with " << config_manager_.imu_calibration_sample_size_ << " samples..." << std::endl;
    imu_calibration_running_ = true;
    imu_calibration_samples_.clear();
}

int DavisDriver::computeNewExposure(const std::vector<uint8_t>& img_data, const uint32_t current_exposure) const
{
    const float desired_intensity = static_cast<float>(config_manager_.autoexposure_desired_intensity);
    static constexpr int min_exposure = 10;
    static constexpr int max_exposure = 25000;
    static constexpr float proportion_to_cut = 0.25f;

    const float current_intensity = trim_mean(img_data, proportion_to_cut);

    const float err = desired_intensity - current_intensity;
    const float delta_exposure = static_cast<float>(current_exposure) * static_cast<float>(config_manager_.autoexposure_gain) / 1000.f * err;

    const int new_exposure = static_cast<int> (static_cast<float>(current_exposure) + delta_exposure + 0.5f);

    return clip(new_exposure, min_exposure, max_exposure);
}

void DavisDriver::readout()
{
    caerDeviceConfigSet(davis_handle_, CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, true);
    caerDeviceDataStart(davis_handle_, NULL, NULL, NULL, &DavisDriver::onDisconnectUSB, this);
    
    std::chrono::steady_clock::time_point next_send_time = std::chrono::steady_clock::now();

    std::shared_ptr<EventArray> event_array_msg;

    while(running_)
    {
        try
        {
            caerEventPacketContainer packetContainer = caerDeviceDataGet(davis_handle_);
            if (packetContainer == NULL)
            {
                continue; // Skip if nothing there.
            }
            int32_t packetNum = caerEventPacketContainerGetEventPacketsNumber(packetContainer);

            for (int32_t i = 0; i < packetNum; i++)
            {
                caerEventPacketHeader packetHeader = caerEventPacketContainerGetEventPacket(packetContainer, i);
                if (packetHeader == NULL)
                {
                    continue; // Skip if nothing there.
                }

                const int type = caerEventPacketHeaderGetEventType(packetHeader);

                if (type == POLARITY_EVENT)
                {
                    if (!event_array_msg) 
                    {
                        event_array_msg = std::make_shared<EventArray>(davis_info_.dvsSizeX, davis_info_.dvsSizeY);
                    }
                    

                    caerPolarityEventPacket polarity = (caerPolarityEventPacket) packetHeader;
                    const int numEvents = caerEventPacketHeaderGetEventNumber(packetHeader);
                
                    for (int j = 0; j < numEvents; j++)
                    {
                        // Get full timestamp and addresses of first event.
                        caerPolarityEvent event = caerPolarityEventPacketGetEvent(polarity, j);
                        Event e(
                        caerPolarityEventGetX(event),
                        caerPolarityEventGetY(event),
                        (uint64_t(caerPolarityEventGetTimestamp64(event, polarity)) * 1000), // removed reset_time_
                        caerPolarityEventGetPolarity(event));

                        if (j == 0) {
                            event_array_msg->header.stamp = e.timestamp_ns;  // Assign first event timestamp to the header
                        }

                        event_array_msg->events.push_back(e);
                    }

                    if (std::chrono::steady_clock::now() > next_send_time ||
                    config_manager_.streaming_rate == 0 ||
                    (config_manager_.max_events != 0 && event_array_msg->events.size() > config_manager_.max_events))
                    {
                        data_queues_->event_queue->push(*event_array_msg);
                            
                        if (config_manager_.streaming_rate > 0)
                        {
                            next_send_time += delta_;
                        }

                        if (config_manager_.max_events != 0 && event_array_msg->events.size() > config_manager_.max_events)
                        {
                            next_send_time = std::chrono::steady_clock::now() + delta_;
                        }

                        event_array_msg.reset();
                    }

                    //Camera Info Published Here in ROS Driver
                }              
                else if (type == IMU6_EVENT)
                {
                    caerIMU6EventPacket imu = (caerIMU6EventPacket) packetHeader;

                    const int numEvents = caerEventPacketHeaderGetEventNumber(packetHeader);

                    for (int j = 0; j < numEvents; j++)
                    {
                        caerIMU6Event event = caerIMU6EventPacketGetEvent(imu, j);
                        IMUData imu_data;
                        
                        // convert from g's to m/s^2 and align axes with camera frame
                        imu_data.linear_acceleration.x = -caerIMU6EventGetAccelX(event) * STANDARD_GRAVITY;
                        imu_data.linear_acceleration.y = caerIMU6EventGetAccelY(event) * STANDARD_GRAVITY;
                        imu_data.linear_acceleration.z = -caerIMU6EventGetAccelZ(event) * STANDARD_GRAVITY;
                        // convert from deg/s to rad/s and align axes with camera frame
                        imu_data.angular_velocity.x = -caerIMU6EventGetGyroX(event) / 180.0 * M_PI;
                        imu_data.angular_velocity.y = caerIMU6EventGetGyroY(event) / 180.0 * M_PI;
                        imu_data.angular_velocity.z = -caerIMU6EventGetGyroZ(event) / 180.0 * M_PI;

                        // no orientation estimate: http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
                        imu_data.orientation_covariance[0] = -1.0;

                        imu_data.header.stamp = caerIMU6EventGetTimestamp64(event, imu) * 1000;
                        imu_data.header.frame_id = "base_link";     //might be redundant

                        if (imu_calibration_running_)
                        {
                            if (imu_calibration_samples_.size() < config_manager_.imu_calibration_sample_size_)
                            {
                                imu_calibration_samples_.push_back(imu_data);
                            }
                            else
                            {
                                imu_calibration_running_ = false;
                                updateImuBias();
                            }
                        }

                        // bias correction
                        imu_data.linear_acceleration.x -= bias.linear_acceleration.x;
                        imu_data.linear_acceleration.y -= bias.linear_acceleration.y;
                        imu_data.linear_acceleration.z -= bias.linear_acceleration.z;
                        imu_data.angular_velocity.x -= bias.angular_velocity.x;
                        imu_data.angular_velocity.y -= bias.angular_velocity.y;
                        imu_data.angular_velocity.z -= bias.angular_velocity.z;

                        // Push to the IMU queue
                        data_queues_->imu_queue->push(imu_data);
                    }
                }
                else if (type == FRAME_EVENT)
                {
                    caerFrameEventPacket frame = (caerFrameEventPacket) packetHeader;
                    caerFrameEvent event = caerFrameEventPacketGetEvent(frame, 0);

                    uint16_t* image = caerFrameEventGetPixelArrayUnsafe(event);

                    ImageData image_data_;

                    // get image metadata
                    caer_frame_event_color_channels frame_channels = caerFrameEventGetChannelNumber(event);
                    const int32_t frame_width = caerFrameEventGetLengthX(event);
                    const int32_t frame_height = caerFrameEventGetLengthY(event);
                    
                    // set message metadata
                    image_data_.width = frame_width;
                    image_data_.height = frame_height;
                    image_data_.step = frame_width * frame_channels;

                    if (frame_channels==1)
                    {
                      image_data_.encoding = "mono8";
                    }
                    else if (frame_channels==3)
                    {
                      image_data_.encoding = "rgb8";
                    }
                    
                    // set message data
                    for (int img_y=0; img_y<frame_height*frame_channels; img_y++)
                    {
                        for (int img_x=0; img_x<frame_width; img_x++)
                        {
                            const uint16_t value = image[img_y*frame_width + img_x];
                            image_data_.data.push_back(value >> 8); //DVS outputs 16 bit - libraries like open CV often expect 8-bit hence conversion
                        }
                    }

                    // time
                    image_data_.header.stamp = caerFrameEventGetTimestamp64(event, frame) * 1000;

                    data_queues_->image_queue->push(image_data_);

                    const int32_t exposure_time_microseconds = caerFrameEventGetExposureLength(event);
                    // Removed exposure publishing

                    //Auto-exposure algorithm
                    // using the requested exposure instead of the actual, measured one gives more stable results
                    uint32_t current_exposure;
                    caerDeviceConfigGet(davis_handle_, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_EXPOSURE, &current_exposure);
                    const int new_exposure = computeNewExposure(image_data_.data, current_exposure);
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_EXPOSURE, new_exposure);
                }

            }
        
        caerEventPacketContainerFree(packetContainer);
        }
        catch (const std::exception& e) 
        {
            std::cerr << "Exception caught in readout thread: " << e.what() << std::endl;
        }
    }
    caerDeviceDataStop(davis_handle_);
}

void DavisDriver::changeDvsParameters()
{
    // while(running_)
    // {
        try
        {
            if (parameter_update_required_)
            {
                parameter_update_required_ = false;

                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_EXPOSURE, config_manager_.exposure);
                
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_FRAME_MODE, config_manager_.frame_mode);
                //caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_FRAME_INTERVAL, config_manager_.frame_interval);

                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RUN, config_manager_.aps_enabled);
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_RUN, config_manager_.dvs_enabled);
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN_ACCELEROMETER, config_manager_.imu_enabled);
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN_GYROSCOPE, config_manager_.imu_enabled);
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN_TEMPERATURE, config_manager_.imu_enabled);

                if (config_manager_.imu_gyro_scale >= 0 && config_manager_.imu_gyro_scale <= 3)
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_GYRO_FULL_SCALE, config_manager_.imu_gyro_scale);

                if (config_manager_.imu_acc_scale >= 0 && config_manager_.imu_acc_scale <= 3)
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_ACCEL_FULL_SCALE, config_manager_.imu_acc_scale);

                if (config_manager_.imu_low_pass_filter >= 0 && config_manager_.imu_low_pass_filter <= 6)
                {
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_DIGITAL_LOW_PASS_FILTER, config_manager_.imu_low_pass_filter);

                    if(config_manager_.imu_low_pass_filter == 0)
                    {
                      // When the low pass filter is disabled, the output frequency of IMU events
                      // is raised to 8KHz. To keep it to 1 kHz, we use the sample rate divider
                      // (setting its value to 7 divides the frequency by 8).
                      caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_SAMPLE_RATE_DIVIDER, 7);
                    }
                    else
                    {
                      // When the low pass filter is enabled, the gyroscope output rate is set to 1 kHz,
                      // so we should not use the sample rate divider, in order to keep the IMU output rate to 1 kHz.
                      caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_SAMPLE_RATE_DIVIDER, 0);
                    }
                }
            
                // Params for DAVIS346B
                // VDAC
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_APSOVERFLOWLEVEL,
                                    caerBiasVDACGenerate(VDAC(27,6)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_APSCAS,
                                    caerBiasVDACGenerate(VDAC(21,6)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_ADCREFHIGH,
                                    caerBiasVDACGenerate(VDAC(config_manager_.ADC_RefHigh_volt, config_manager_.ADC_RefHigh_curr)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_ADCREFLOW,
                                    caerBiasVDACGenerate(VDAC(config_manager_.ADC_RefLow_volt, config_manager_.ADC_RefLow_curr)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_ADCTESTVOLTAGE,
                                    caerBiasVDACGenerate(VDAC(21,7)));
                // CF Biases
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_LOCALBUFBN,
                                    caerBiasCoarseFineGenerate(CF_N_TYPE(5, 164)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_PADFOLLBN,
                                    caerBiasCoarseFineGenerate(CF_N_TYPE(7, 215)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_DIFFBN,
                                    caerBiasCoarseFineGenerate(CF_N_TYPE(config_manager_.DiffBn_coarse, config_manager_.DiffBn_fine)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_ONBN,
                                    caerBiasCoarseFineGenerate(CF_N_TYPE(config_manager_.ONBn_coarse, config_manager_.ONBn_fine)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_OFFBN,
                                    caerBiasCoarseFineGenerate(CF_N_TYPE(config_manager_.OFFBn_coarse, config_manager_.OFFBn_fine)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_PIXINVBN,
                                    caerBiasCoarseFineGenerate(CF_N_TYPE(5, 129)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_PRBP,
                                    caerBiasCoarseFineGenerate(CF_P_TYPE(config_manager_.PrBp_coarse, config_manager_.PrBp_fine)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_PRSFBP,
                                    caerBiasCoarseFineGenerate(CF_P_TYPE(config_manager_.PrSFBp_coarse, config_manager_.PrSFBp_fine)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_REFRBP,
                                    caerBiasCoarseFineGenerate(CF_P_TYPE(config_manager_.RefrBp_coarse, config_manager_.RefrBp_fine)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_READOUTBUFBP,
                                    caerBiasCoarseFineGenerate(CF_P_TYPE(6, 20)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_APSROSFBN,
                                    caerBiasCoarseFineGenerate(CF_N_TYPE(6, 219)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_ADCCOMPBP,
                                    caerBiasCoarseFineGenerate(CF_P_TYPE(5, 20)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_COLSELLOWBN,
                                    caerBiasCoarseFineGenerate(CF_N_TYPE(0, 1)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_DACBUFBP,
                                    caerBiasCoarseFineGenerate(CF_P_TYPE(6, 60)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_LCOLTIMEOUTBN,
                                    caerBiasCoarseFineGenerate(CF_N_TYPE(5, 49)));;
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_AEPDBN,
                                    caerBiasCoarseFineGenerate(CF_N_TYPE(6, 91)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_AEPUXBP,
                                    caerBiasCoarseFineGenerate(CF_P_TYPE(4, 80)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_AEPUYBP,
                                    caerBiasCoarseFineGenerate(CF_P_TYPE(7, 152)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_IFREFRBN,
                                    caerBiasCoarseFineGenerate(CF_N_TYPE(5, 255)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_IFTHRBN,
                                    caerBiasCoarseFineGenerate(CF_N_TYPE(5, 255)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_BIASBUFFER,
                                    caerBiasCoarseFineGenerate(CF_N_TYPE(5, 254)));
                // Special Biases
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_SSP,
                                    caerBiasShiftedSourceGenerate(SHIFTSOURCE(1,33,SHIFTED_SOURCE)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_SSN,
                                    caerBiasShiftedSourceGenerate(SHIFTSOURCE(1,33,SHIFTED_SOURCE)));

                // Hardware filters
                if (davis_info_.dvsHasPixelFilter)
                {
                  caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_AUTO_TRAIN, config_manager_.pixel_auto_train);
                  
                  // if using auto train, update the configuration with hardware values
                  if (config_manager_.pixel_auto_train)
                  {
                    std::cout << "Auto-training hot-pixel filter..." << std::endl;
                    while(config_manager_.pixel_auto_train)
                    {
                      std::this_thread::sleep_for(std::chrono::milliseconds(100));
                      caerDeviceConfigGet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_AUTO_TRAIN, (uint32_t*)&config_manager_.pixel_auto_train);
                    }
                  
                    caerDeviceConfigGet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_0_ROW, (uint32_t*)&config_manager_.pixel_0_row);
                    caerDeviceConfigGet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_0_COLUMN, (uint32_t*)&config_manager_.pixel_0_column);
                    caerDeviceConfigGet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_1_ROW, (uint32_t*)&config_manager_.pixel_1_row);
                    caerDeviceConfigGet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_1_COLUMN, (uint32_t*)&config_manager_.pixel_1_column);
                    caerDeviceConfigGet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_2_ROW, (uint32_t*)&config_manager_.pixel_2_row);
                    caerDeviceConfigGet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_2_COLUMN, (uint32_t*)&config_manager_.pixel_2_column);
                    caerDeviceConfigGet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_3_ROW, (uint32_t*)&config_manager_.pixel_3_row);
                    caerDeviceConfigGet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_3_COLUMN, (uint32_t*)&config_manager_.pixel_3_column);
                    caerDeviceConfigGet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_4_ROW, (uint32_t*)&config_manager_.pixel_4_row);
                    caerDeviceConfigGet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_4_COLUMN, (uint32_t*)&config_manager_.pixel_4_column);
                    caerDeviceConfigGet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_5_ROW, (uint32_t*)&config_manager_.pixel_5_row);
                    caerDeviceConfigGet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_5_COLUMN, (uint32_t*)&config_manager_.pixel_5_column);
                    caerDeviceConfigGet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_6_ROW, (uint32_t*)&config_manager_.pixel_6_row);
                    caerDeviceConfigGet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_6_COLUMN, (uint32_t*)&config_manager_.pixel_6_column);
                    caerDeviceConfigGet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_7_ROW, (uint32_t*)&config_manager_.pixel_7_row);
                    caerDeviceConfigGet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_7_COLUMN, (uint32_t*)&config_manager_.pixel_7_column);
                    
                    
                    std::cout << "Done auto-training hot-pixel filter." << std::endl;
                  }
                  else // apply current configuration to hardware
                  {
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_0_ROW, config_manager_.pixel_0_row);
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_0_COLUMN, config_manager_.pixel_0_column);
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_1_ROW, config_manager_.pixel_1_row);
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_1_COLUMN, config_manager_.pixel_1_column);
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_2_ROW, config_manager_.pixel_2_row);
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_2_COLUMN, config_manager_.pixel_2_column);
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_3_ROW, config_manager_.pixel_3_row);
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_3_COLUMN, config_manager_.pixel_3_column);
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_4_ROW, config_manager_.pixel_4_row);
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_4_COLUMN, config_manager_.pixel_4_column);
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_5_ROW, config_manager_.pixel_5_row);
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_5_COLUMN, config_manager_.pixel_5_column);
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_6_ROW, config_manager_.pixel_6_row);
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_6_COLUMN, config_manager_.pixel_6_column);
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_7_ROW, config_manager_.pixel_7_row);
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_7_COLUMN, config_manager_.pixel_7_column);
                  }
                }
                
                if (davis_info_.dvsHasBackgroundActivityFilter)
                {
                  caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_BACKGROUND_ACTIVITY, config_manager_.background_activity_filter_enabled);
                  caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_BACKGROUND_ACTIVITY_TIME, config_manager_.background_activity_filter_time);

                  caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_REFRACTORY_PERIOD, config_manager_.refractory_period_enabled);
                  caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_REFRACTORY_PERIOD_TIME, config_manager_.refractory_period_time);
                }
                
                if (davis_info_.dvsHasROIFilter)
                {
                  caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_ROI_START_COLUMN, config_manager_.roi_start_column);
                  caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_ROI_START_ROW, config_manager_.roi_start_row);
                  caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_ROI_END_COLUMN, config_manager_.roi_end_column);
                  caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_ROI_END_ROW, config_manager_.roi_end_row);
                }
                
                if (davis_info_.dvsHasSkipFilter)
                {
                  caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_SKIP_EVENTS, config_manager_.skip_enabled);
                  caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_SKIP_EVENTS_EVERY, config_manager_.skip_every);
                }
                
                if (davis_info_.dvsHasPolarityFilter)
                {
                  caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_POLARITY_FLATTEN, config_manager_.polarity_flatten);
                  caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_POLARITY_SUPPRESS, config_manager_.polarity_suppress);
                  caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_POLARITY_SUPPRESS_TYPE, config_manager_.polarity_suppress_type);
                }
                
                // APS region of interest
                // caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_COLUMN_0, config_manager_.aps_roi_start_column);
                // caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_ROW_0, config_manager_.aps_roi_start_row);
                // caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_COLUMN_0, config_manager_.aps_roi_end_column);
                // caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_ROW_0, config_manager_.aps_roi_end_row);
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << "Exception in changeDvsParameters: " << e.what() << std::endl;
        }
    // }
}

void DavisDriver::onDisconnectUSB(void* driver)
{
    std::cerr << "[ERROR] USB connection lost with DVS!" << std::endl;
    static_cast<DavisDriver*>(driver)->caerConnect(); 
}
//==============================================================================
// End of File : Software/src/DavisDriver.cpp
