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
#include "Types.hpp"
#include "Logging.hpp"

#include <libcaer/libcaer.h>
#include <libcaer/devices/davis.h>
#include <yaml-cpp/yaml.h>
#include <numeric>


//==============================================================================
//      Classes
//------------------------------------------------------------------------------



class ConfigManager {
public:
    explicit ConfigManager(const std::string& config_path);
    
    void loadConfig(const std::string& config_path);      // Load parameters from YAML

    //TODO - Check Params
    std::string device_id_;
    double reset_timestamps_delay_;
    int imu_calibration_sample_size_;
    IMUData bias_;

    //Camera Parameters
    bool aps_enabled, dvs_enabled, imu_enabled;
    int imu_acc_scale, imu_gyro_scale;
    int exposure, max_events, streaming_rate;
    int frame_delay;
    int frame_mode, frame_interval;

    // Auto-exposure parameters
    float autoexposure_desired_intensity;
    float autoexposure_gain;


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

    IMUData getBias() const { return bias_; }

private:
    std::string config_file_path; // Path to YAML file
    std::mutex config_mutex;      // Thread safety for updates
};


class DavisDriver {
public:
  DavisDriver(const std::string& config_path, std::shared_ptr<DataQueues> data_queues);
  ~DavisDriver();
  static void onDisconnectUSB(void*);
private:

  void caerConnect();
  void changeDvsParameters();
  void readout();
  void updateImuBias();
  void triggerImuCalibration();
  int computeNewExposure(const std::vector<uint8_t>& img_data,
                          const uint32_t current_exposure) const;

  IMUData bias;
  std::shared_ptr<DataQueues> data_queues_;
  ConfigManager config_manager_;
  std::chrono::microseconds delta_;

  std::thread parameter_thread_;
  std::thread readout_thread_;    

  caerDeviceHandle davis_handle_;
  struct caer_davis_info davis_info_;
  volatile bool running_ = true;

  bool parameter_update_required_;
  bool parameter_bias_update_required_;

  bool imu_calibration_running_;
  std::vector<IMUData> imu_calibration_samples_;

  static constexpr double STANDARD_GRAVITY = 9.81;
  
};

//==============================================================================
//      Function Prototypes
//------------------------------------------------------------------------------
template<typename T>
T clip(T n, T lower, T upper) {
    return std::max(lower, std::min(n, upper));
}

template<typename T>
float mean(const std::vector<T>& v)
{
    if(v.empty())
    {
        return 0.f;
    }

    float sum = static_cast<float>(std::accumulate(v.begin(), v.end(), 0.0));
    float mean = sum / v.size();
    return mean;
}

/* Trimmed mean: removes the first and last proportion_to_cur percentiles of the data
 *  before computing the mean, e.g.:
 *     proportion_to_cut = 0 -> normal mean
 *      proportion_to_cur = 0.5 -> median
 */
template<typename T>
float trim_mean(const std::vector<T>& v_original, const float proportion_to_cut = 0)
{
    if(v_original.empty())
    {
        return 0.f;
    }

    std::vector<T> v(v_original);
    std::sort(v.begin(), v.end());

    const size_t size = v.size();
    const size_t num_values_to_cut = static_cast<int>(size * proportion_to_cut);
    const size_t start_index = num_values_to_cut - 1;
    const size_t end_index = size - num_values_to_cut - 1;

    std::vector<T> trimmed_vec(v.begin() + start_index, v.begin() + end_index);
    return mean(trimmed_vec);
}


#endif  // DAVISDRIVER_HPP
//==============================================================================
// End of File :  Software/include/Communication.hpp