/*
Filename    : Software/src/DavisDriver.hpp
Author      : Andrew Ferguson
Project     : Event Based Navigation
Date        : 28/01/25
Description : Header file for the Davis Camera Driver
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
        imu_bias_.ax = config["imu_bias"]["ax"] ? config["imu_bias"]["ax"].as<double>() : 0.0;
        imu_bias_.ay = config["imu_bias"]["ay"] ? config["imu_bias"]["ay"].as<double>() : 0.0;
        imu_bias_.az = config["imu_bias"]["az"] ? config["imu_bias"]["az"].as<double>() : 0.0;
        imu_bias_.wx = config["imu_bias"]["wx"] ? config["imu_bias"]["wx"].as<double>() : 0.0;
        imu_bias_.wy = config["imu_bias"]["wy"] ? config["imu_bias"]["wy"].as<double>() : 0.0;
        imu_bias_.wz = config["imu_bias"]["wz"] ? config["imu_bias"]["wz"].as<double>() : 0.0;

        // DAVIS346 Specific Settings
        aps_enabled = config["aps_enabled"] ? config["aps_enabled"].as<bool>() : true;
        dvs_enabled = config["dvs_enabled"] ? config["dvs_enabled"].as<bool>() : true;
        imu_enabled = config["imu_enabled"] ? config["imu_enabled"].as<bool>() : true;

        imu_acc_scale = config["imu_acc_scale"] ? config["imu_acc_scale"].as<int>() : 3;
        imu_gyro_scale = config["imu_gyro_scale"] ? config["imu_gyro_scale"].as<int>() : 3;

        exposure = config["exposure"] ? config["exposure"].as<int>() : 4000;
        frame_delay = config["frame_delay"] ? config["frame_delay"].as<int>() : 0;
        max_events = config["max_events"] ? config["max_events"].as<int>() : 0;
        streaming_rate = config["streaming_rate"] ? config["streaming_rate"].as<int>() : 30;

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

    running_ = true;
    parameter_thread_ = std::thread(&DavisDriver::changeDvsParameters, this);
    readout_thread_ = std::thread(&DavisDriver::readout, this);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

}

void DavisDriver::readout()
{

}

void DavisDriver::changeDvsParameters()
{

}

//==============================================================================
// End of File : Software/src/DavisDriver.cpp
