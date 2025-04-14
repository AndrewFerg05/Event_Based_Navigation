/*
Filename    : Software/include/FrontEnd.hpp
Author      : Samuel Kliskey
Project     : Event Based Navigation
Date        : 11/1/25
Description : Header file for the FrontEnd (FE) thread
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
11-JAN-2025 SARK created to design code structure
--------------------------------------------------------------------------------
*/

#ifndef FRONTEND_HPP
#define FRONTEND_HPP

//==============================================================================
// External Files
//------------------------------------------------------------------------------
#include "ThreadInterface.hpp"
#include "TypeAliases.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <open_vins/core/VioManagerOptions.h>
#include <open_vins/core/VioManager.h>
#include <open_vins/state/State.h>
#include <yaml-cpp/yaml.h>



//==============================================================================
//      Classes
//------------------------------------------------------------------------------
struct FrameProcessingParams {
    // Frame Building flags to ignore stationary or extreme event rates
    int32_t noise_event_rate = static_cast<int32_t>(0.11e+06);         // Minimum event rate below which ignored
    int32_t event_ignore_threshold = static_cast<int32_t>(6e+06);      // Maximum event rate above which ignored

    // Frame Blending flags
    int32_t max_event_rate = static_cast<int32_t>(0.18e+06);           // Level at which Maximum blend is used
    double max_event_blend = 0.4;                                      // Maximum blend amount
    double min_event_blend = 0.2;                                      // Minimum blend amount
};


class FrontEnd
{
    public:
    explicit FrontEnd(
        std::shared_ptr<CommunicationManager> comms,
        const std::string& config_path);

    ~FrontEnd() = default;
    
    void start();
    void stop();
    void idle();
    void addData(
        const StampedImage& ,
        const StampedEventArray&,
        const ImuStamps&,
        const ImuAccGyrContainer&,
        const bool& no_motion_prior);
    void addImuData(
        int64_t stamp,
        const Vector3& acc, 
        const Vector3& gyr);

    void initState(int64_t stamp, const Vector3& acc, const Vector3& gyr);
    bool buildImage(ov_core::CameraData& camera_data, 
        const StampedImage& stamped_image,
        const StampedEventArray& stamped_events,
        const ImuStamps& imu_stamps,
        const ImuAccGyrContainer& imu_accgyr,
        FrameType frame_type);

    void loadFrameType();

    private:
    FrameType frame_config_ = COMBINED_FRAME;
    FrameProcessingParams params;
    std::atomic<bool> stateInitialised_{false};
    std::atomic<bool> vioReady_{false};
    std::shared_ptr<CommunicationManager> comms_interface_;
    std::shared_ptr<ov_msckf::VioManager> vio_manager_; // OpenVINS VIO manager
    std::string config_path_; // Configuration file path

    void setupVIO(); // Function to initialize OpenVINS
    
};

//==============================================================================
//      Function Prototypes
//------------------------------------------------------------------------------




#endif  // FRONTEND_HPP
//==============================================================================
// End of File :  Software/include/Communication.hpp