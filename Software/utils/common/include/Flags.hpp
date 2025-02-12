/*
Filename    : Software/include/Flags.hpp
Author      : Andrew Ferguson
Project     : Event Based Navigation
Date        : 14/1/25
Description : 
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
11-JAN-2025 AF created
--------------------------------------------------------------------------------
*/

#ifndef FLAGS_HPP
#define FLAGS_HPP

//==============================================================================
// External Files
//------------------------------------------------------------------------------
#include <cstdint>
#include <string>
//==============================================================================
//      Classes
//------------------------------------------------------------------------------

// GFlag replacements
inline bool VIO_CERES_VERBOSE = false;
inline bool VIO_CERES_MARGINALIZE = true;
inline int VIO_CERES_ITERATIONS = 3;
inline int VIO_CERES_SLIDING_WINDOW_SIZE = 3;
inline int VIO_CERES_NUM_KEYFRAMES = 5;
inline int VIO_CERES_NUM_THREADS = 1;

// Depth thresholds
inline double VIO_MIN_DEPTH = 0.1;  // Minimum depth for VIO
inline double VIO_MAX_DEPTH = 10.0; // Maximum depth for VIO
inline double VIO_MEDIAN_DEPTH = 1.0; // Median depth for VIO

// Feature tracking limits
inline uint64_t VIO_MIN_TRACKED_FEATURES_TOTAL = 50;  // Min tracked features globally
inline uint64_t VIO_MAX_TRACKED_FEATURES_PER_FRAME = 150; // Max features per frame

// Processing mode
inline bool VIO_DELAYED_NFRAME_PROCESSING = false; // Enable or disable delayed processing

inline bool FLAGS_data_use_time_interval = 0;
inline uint64_t FLAGS_data_interval_between_event_packets = 15000;
inline size_t FLAGS_data_size_augmented_event_packet = 15000;

inline int FLAGS_num_imus = 1;

inline bool vio_use_events = true; 
// Use events instead of images.

inline bool vio_use_events_and_images = false; 
// Use events and images. Ignores vio_use_events flag.

inline bool vio_rescale_intrinsics_for_distortion = true; 
// Whether to rescale the camera intrinsics to mitigate the loss of FoV introduced by image undistortion.

extern const std::string FLAGS_calib_filename;
extern const std::string FLAGS_mask_cam0;
extern const std::string FLAGS_mask_cam1;
extern const std::string FLAGS_mask_cam2;
extern const std::string FLAGS_mask_cam3;


inline uint64_t FLAGS_vio_frame_pyramid_levels = 3;

inline bool FLAGS_vio_use_events_and_images = 0;
inline bool FLAGS_vio_use_events = 1;
inline bool FLAGS_vio_rescale_intrinsics_for_distortion = 1;
inline bool FLAGS_vio_activate_backend = 1;


inline double FLAGS_vio_landmark_triangulate_purge_floor = -1;
inline double FLAGS_noise_event_rate = 20000;

inline uint64_t FLAGS_vio_frame_size = 15000;

inline bool FLAGS_vio_do_motion_correction = 1;
inline double FLAGS_vio_frame_norm_factor = 3.0;


inline uint32_t FLAGS_vio_add_every_nth_frame_to_backend = -1; 

extern const std::string FLAGS_vio_feature_tracker_patch_size_by8;
inline double FLAGS_vio_feature_tracker_termcrit_min_update_squared = 0.03;

inline double FLAGS_vio_disparity_median_for_static_motion_classification = 1.75;

inline uint64_t FLAGS_vio_max_landmarks = 1000;

inline double FLAGS_vio_min_depth = 0.1;
inline uint32_t FLAGS_vio_max_depth = 10;
inline uint32_t FLAGS_vio_median_depth = 1;

// Keyframe Selection
inline uint32_t FLAGS_vio_first_n_frames_as_keyframes = 0;
inline uint32_t FLAGS_vio_kfselect_min_every_nth_frame = -1;
inline uint32_t FLAGS_vio_kfselect_every_nth_frame = -1;
inline uint32_t FLAGS_vio_kfselect_criterion = 1;
inline uint64_t FLAGS_vio_kfselect_numfts_upper_thresh = 180;
inline uint64_t FLAGS_vio_kfselect_numfts_lower_thresh = 90;
inline uint32_t FLAGS_vio_kfselect_min_num_frames_between_kfs = 0;
inline double FLAGS_vio_kfselect_min_disparity = 30.0;
inline double FLAGS_vio_kfselect_min_dist = 0.12;


// Landmark Reprojector
inline bool FLAGS_vio_reprojector_pyr_alignment = true;
inline bool FLAGS_vio_reprojector_limit_num_features = true;
inline uint32_t FLAGS_vio_reprojector_min_quality_to_project = 2;

// Feature detector
inline bool FLAGS_vio_descriptor_use_dummy = false;
extern const std::string FLAGS_imp_detector_name;
inline uint32_t FLAGS_imp_detector_grid_size = 32;
inline uint32_t FLAGS_imp_detector_border_margin = 8;
inline uint32_t FLAGS_imp_detector_num_octaves = 3;
inline uint32_t FLAGS_imp_detector_max_features_per_frame = 300;
inline uint32_t FLAGS_vio_max_tracked_features_per_frame = 300;
inline uint32_t FLAGS_imp_brisk_uniformity_radius = 0;
inline uint32_t FLAGS_imp_brisk_absolute_threshold = 45;
inline uint32_t FLAGS_imp_detector_threshold = 10;

inline uint32_t FLAGS_vio_min_tracked_features_total = 50; // Guess

// Track Extractor
// Max number of landmarks to add to back-end optimization per keyframe.
inline uint32_t FLAGS_vio_max_num_tracks_per_update = 50;
// Number of observations in feature-track to classify landmark as persistent
inline uint32_t FLAGS_vio_num_obs_for_persistent = 10;
// Minimum parallax threshold to use a landmark in the back-end optimization
inline double FLAGS_vio_min_parallax_deg = 0.5;
// Favor an equal number of new persistent tracks in every frame.
inline bool FLAGS_vio_favor_equal_number_of_tracks_per_frame = false;
inline bool FLAGS_vio_delayed_nframe_processing = 0; // Guess and might change

// Relative pose RANSAC reprojection error threshold [px].
inline double FLAGS_vio_ransac_relpose_thresh_px = 2;
inline bool FLAGS_vio_use_5pt_ransac = false;
//==============================================================================
// Backend
// Main
inline bool FLAGS_vio_ceres_verbose = false; // Output ceres optimization progress.
inline bool FLAGS_vio_ceres_marginalize = true; // Apply marginalization?
inline int FLAGS_vio_ceres_iterations = 3; // Maximum number of iterations.
inline int FLAGS_vio_ceres_sliding_window_size = 3; // Sliding window size of ceres backend.
inline int FLAGS_vio_ceres_numkeyframes = 5; // Number of keyframes of ceres backend.
inline int FLAGS_vio_ceres_num_threads = 1; // Number of threads in ceres backend.

// Backend
inline double FLAGS_swe_imu_rate = 200.0; // IMU Rate [Hz]
inline double FLAGS_vio_ceres_accel_saturation = 176.0; // Acceleration saturation [m/s^2]
inline double FLAGS_vio_ceres_gyro_saturation = 7.8; // Gyro saturation [rad/s]
inline double FLAGS_gyro_noise_density = 12.0e-4; // Gyro noise density [rad/s/sqrt(Hz)]
inline double FLAGS_gyro_bias_random_walk = 0.03; // Gyro bias prior noise density [rad/s]
inline double FLAGS_acc_noise_density = 8.0e-3; // Accelerometer noise density [m/s^2/sqrt(Hz)]
inline double FLAGS_acc_bias_random_walk = 0.1; // Accelerometer bias prior noise density [m/s^2]
inline double FLAGS_vio_ceres_sigma_gyro_drift = 4.0e-6; // Gyro drift noise density [rad/s^2/sqrt(Hz)]
inline double FLAGS_vio_ceres_sigma_accel_drift = 4.0e-5; // Accelerometer drift noise density [m/s^3/sqrt(Hz)]
inline double FLAGS_vio_ceres_gravity = 9.81; // Earth gravitational acceleration [m/s^2]
inline bool FLAGS_vio_ceres_add_velocity_prior = false; // Add velocity priors. [Experimental]
inline double FLAGS_extrinsics_sigma_abs_translation = 0.0; // Absolute translation sigma of camera extrinsics w.r.t IMU Frame
inline double FLAGS_extrinsics_sigma_abs_orientation = 0.0; // Absolute orientation sigma of camera extrinsics w.r.t IMU Frame
inline double FLAGS_extrinsics_sigma_rel_translation = 0.0; // Relative translation sigma (temporal) of camera extrinsics
inline double FLAGS_extrinsics_sigma_rel_orientation = 0.0; // Relative orientation sigma (temporal) of camera extrinsics
inline double FLAGS_vio_ceres_max_optimization_time = -1.0; // Maximum optimization time [s]. Negative value means use max iterations.



inline double FLAGS_vio_acc_bias_init_x = 0;
inline double FLAGS_vio_acc_bias_init_y = 0;
inline double FLAGS_vio_acc_bias_init_z= 0;
inline double FLAGS_vio_gyr_bias_init_x = 0;
inline double FLAGS_vio_gyr_bias_init_y = 0;
inline double FLAGS_vio_gyr_bias_init_z = 0;







#endif  // TYPE_ALIASES_HPP
//==============================================================================
// End of File :  Software/include/TYPE_ALIASES_HPP.hpp