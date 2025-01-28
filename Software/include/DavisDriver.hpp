/*
Filename    : Software/include/DataAcquisition.hpp
Author      : Andrew Ferguson
Project     : Event Based Navigation
Date        : 28/01/25
Description : Header file for the Davis Camera Driver
--------------------------------------------------------------------------------

--------------------------------------------------------------------------------
*/

#ifndef DAVISDRIVER_HPP
#define DAVISDRIVER_HPP

//==============================================================================
// External Files
//------------------------------------------------------------------------------
#include "ThreadInterface.hpp"
#include "TypeAliases.hpp"

#include <libcaer/libcaer.h>
#include <libcaer/devices/davis.h>
#include <yaml-cpp/yaml.h>


//==============================================================================
//      Classes
//------------------------------------------------------------------------------

  struct ImuBias {
        double ax, ay, az;
        double wx, wy, wz;
    };

class ConfigManager {
public:
    explicit ConfigManager(const std::string& config_path);
    
    void loadConfig(const std::string& config_path);      // Load parameters from YAML

    //TODO - Check Params
    std::string device_id_;
    double reset_timestamps_delay_;
    int imu_calibration_sample_size_;
    ImuBias imu_bias_;

    //Camera Parameters
    bool aps_enabled, dvs_enabled, imu_enabled;
    int imu_acc_scale, imu_gyro_scale;
    int exposure, max_events, streaming_rate;
    int frame_delay;
    int frame_mode, frame_interval;

    // IMU Filter Parameters
    int imu_low_pass_filter;
    int imu_sample_rate_divider;

    // Hardware Filtering (DVS)
    bool pixel_auto_train;
    int pixel_0_row, pixel_0_column;
    int pixel_1_row, pixel_1_column;
    int pixel_2_row, pixel_2_column;
    int pixel_3_row, pixel_3_column;
    int pixel_4_row, pixel_4_column;
    int pixel_5_row, pixel_5_column;
    int pixel_6_row, pixel_6_column;
    int pixel_7_row, pixel_7_column;

    // Background Activity Filter
    bool background_activity_filter_enabled;
    int background_activity_filter_time;

    // Refractory Period Filter
    bool refractory_period_enabled;
    int refractory_period_time;

    // ROI Filter
    int roi_start_column, roi_start_row;
    int roi_end_column, roi_end_row;

    // Skip Events Filter
    bool skip_enabled;
    int skip_every;

    // Polarity Filter
    bool polarity_flatten;
    bool polarity_suppress;
    int polarity_suppress_type;

    // ADC Parameters
    int ADC_RefHigh_volt, ADC_RefHigh_curr;
    int ADC_RefLow_volt, ADC_RefLow_curr;

    // Bias Parameters
    int DiffBn_coarse, DiffBn_fine;
    int OFFBn_coarse, OFFBn_fine;
    int ONBn_coarse, ONBn_fine;
    int PrBp_coarse, PrBp_fine;
    int PrSFBp_coarse, PrSFBp_fine;
    int RefrBp_coarse, RefrBp_fine;

        // APS Region of Interest
    int aps_roi_start_column;
    int aps_roi_start_row;
    int aps_roi_end_column;
    int aps_roi_end_row;

private:
    std::string config_file_path; // Path to YAML file
    std::mutex config_mutex;      // Thread safety for updates
};


class DavisDriver {
public:
  DavisDriver(const std::string& config_path, std::shared_ptr<DataQueues> data_queues);
  ~DavisDriver();
  void caerConnect();
  void changeDvsParameters();
  void readout();

private:
  std::shared_ptr<DataQueues> data_queues_;
  ConfigManager config_manager_;
  std::chrono::microseconds delta_;

  std::thread parameter_thread_;
  std::thread readout_thread_;    

  caerDeviceHandle davis_handle_;
  struct caer_davis_info davis_info_;
  volatile bool running_;

  bool parameter_update_required_;
  bool parameter_bias_update_required_;
  bool imu_calibration_running_;
  
};

//==============================================================================
//      Function Prototypes
//------------------------------------------------------------------------------


#endif  // DAVISDRIVER_HPP
//==============================================================================
// End of File :  Software/include/Communication.hpp