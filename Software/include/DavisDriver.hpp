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

struct IMU_Bias {
    struct {
        double x, y, z;
    } linear_acceleration, angular_velocity;
};

class DavisDriver {
public:
  DavisDriver(const std::string& config_path);
  ~DavisDriver() = default;
  void loadParameters(const std::string& config_path);

private:

std::string device_id_;
  bool master_;
  double reset_timestamps_delay_;
  int imu_calibration_sample_size_;
  IMU_Bias bias;

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