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



//==============================================================================
//      Classes
//------------------------------------------------------------------------------
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

    private:
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