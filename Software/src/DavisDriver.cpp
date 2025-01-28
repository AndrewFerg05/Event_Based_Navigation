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
DavisDriver::DavisDriver(const std::string& config_path, std::shared_ptr<DataQueues> data_queues) :
    parameter_update_required_(false),
    parameter_bias_update_required_(false),
    imu_calibration_running_(false),
    data_queues_(data_queues)
{
    loadParameters(config_path);
 
}

void DavisDriver::loadParameters(const std::string& config_path) 
{
    // Set up to read from YAML File - Unsure where these come from in ROS driver Driver.cpp - ROS Params Server?
    try {
        YAML::Node config = YAML::LoadFile(config_path);

        //Load Parameters - TODO
        device_id_ = config["serial_number"] ? config["serial_number"].as<std::string>() : "UNKNOWN";
        master_ = config["master"] ? config["master"].as<bool>() : true; //Not needed
        reset_timestamps_delay_ = config["reset_timestamps_delay"] ? config["reset_timestamps_delay"].as<double>() : -1.0;
        imu_calibration_sample_size_ = config["imu_calibration_sample_size"] ? config["imu_calibration_sample_size"].as<int>() : 1000;

        // IMU Biases - TODO
        bias.linear_acceleration.x = config["imu_bias/ax"] ? config["imu_bias/ax"].as<double>() : 0.0;
        bias.linear_acceleration.y = config["imu_bias/ay"] ? config["imu_bias/ay"].as<double>() : 0.0;
        bias.linear_acceleration.z = config["imu_bias/az"] ? config["imu_bias/az"].as<double>() : 0.0;
        bias.angular_velocity.x = config["imu_bias/wx"] ? config["imu_bias/wx"].as<double>() : 0.0;
        bias.angular_velocity.y = config["imu_bias/wy"] ? config["imu_bias/wy"].as<double>() : 0.0;
        bias.angular_velocity.z = config["imu_bias/wz"] ? config["imu_bias/wz"].as<double>() : 0.0;

        std::cout << "Loaded parameters successfully from " << config_path << std::endl;
    } 
    catch (const std::exception& e) {
        std::cerr << "Error loading YAML config: " << e.what() << std::endl;
    }
}

//==============================================================================
// End of File : Software/src/DavisDriver.cpp
