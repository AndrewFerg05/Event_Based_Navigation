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
struct CalibrationData {
    Eigen::Matrix4d T_cam_imu; // Camera-to-IMU transformation
    Eigen::Matrix4f T_imu_body; // IMU-to-body transformation (Identity in this case)
    Eigen::Vector4f distortion_coeffs; // Radial-Tangential distortion coefficients
    Eigen::Matrix4f K; // Intrinsic matrix
    int width, height; // Camera resolution
    float timeshift_cam_imu; // Time shift between IMU and Camera
    Eigen::Matrix<float, 4, Eigen::Dynamic> dvs_bearing_lut_; // 3D bearing vectors
    Eigen::Matrix<float, 2, Eigen::Dynamic> dvs_keypoint_lut_; // 2D keypoints
    float median_depth;
};

struct State
{
    bool stateInit;
    Eigen::Matrix<double, 3, 1> velocity;
    Eigen::Isometry3d T_Bk_W;
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

    void drawEvents(
        const EventArray::iterator& first,
        const EventArray::iterator& last,
        const int64_t& t0,
        const int64_t& t1,
        const Eigen::Isometry3d& T_1_0,
        cv::Mat &out);

    void loadCalibrationData();

    CalibrationData calib_;
    State prevState_;  
    private:
    std::atomic<bool> stateInitialised_{false};
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