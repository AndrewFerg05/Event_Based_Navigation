/*
Filename    : Software/include/TypeAliases.hpp
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
constexpr bool VIO_CERES_VERBOSE = false;
constexpr bool VIO_CERES_MARGINALIZE = true;
constexpr int VIO_CERES_ITERATIONS = 3;
constexpr int VIO_CERES_SLIDING_WINDOW_SIZE = 3;
constexpr int VIO_CERES_NUM_KEYFRAMES = 5;
constexpr int VIO_CERES_NUM_THREADS = 1;

// Depth thresholds
constexpr double VIO_MIN_DEPTH = 0.1;  // Minimum depth for VIO
constexpr double VIO_MAX_DEPTH = 10.0; // Maximum depth for VIO
constexpr double VIO_MEDIAN_DEPTH = 1.0; // Median depth for VIO

// Feature tracking limits
constexpr uint64_t VIO_MIN_TRACKED_FEATURES_TOTAL = 50;  // Min tracked features globally
constexpr uint64_t VIO_MAX_TRACKED_FEATURES_PER_FRAME = 150; // Max features per frame

// Processing mode
constexpr bool VIO_DELAYED_NFRAME_PROCESSING = false; // Enable or disable delayed processing

inline bool FLAGS_data_use_time_interval = 0;
inline uint64_t FLAGS_data_interval_between_event_packets = 15000; //number of frames in event packet/ ms between packets set which by above flag
inline size_t FLAGS_data_size_augmented_event_packet = 15000;

constexpr int FLAGS_num_imus = 1;

constexpr bool vio_use_events = true; 
// Use events instead of images.

constexpr bool vio_use_events_and_images = false; 
// Use events and images. Ignores vio_use_events flag.

constexpr bool vio_rescale_intrinsics_for_distortion = true; 
// Whether to rescale the camera intrinsics to mitigate the loss of FoV introduced by image undistortion.

extern const std::string FLAGS_calib_filename;
extern const std::string FLAGS_mask_cam0;
extern const std::string FLAGS_mask_cam1;
extern const std::string FLAGS_mask_cam2;
extern const std::string FLAGS_mask_cam3;


constexpr uint64_t FLAGS_vio_frame_pyramid_levels = 3;

constexpr bool FLAGS_vio_use_events_and_images = 0;
constexpr bool FLAGS_vio_use_events = 1;
constexpr bool FLAGS_vio_rescale_intrinsics_for_distortion = 1;
constexpr bool FLAGS_vio_activate_backend = 1; //Feel like this might also change

constexpr double FLAGS_vio_landmark_triangulate_purge_floor = -1;
constexpr double FLAGS_noise_event_rate = 20000;

constexpr uint64_t FLAGS_vio_frame_size = 15000;

constexpr bool FLAGS_vio_do_motion_correction = 1;
constexpr double FLAGS_vio_frame_norm_factor = 3.0;

constexpr uint32_t FLAGS_vio_add_every_nth_frame_to_backend = -1; 

extern const std::string FLAGS_vio_feature_tracker_patch_size_by8;
constexpr double FLAGS_vio_feature_tracker_termcrit_min_update_squared = 0.03;

constexpr double FLAGS_vio_disparity_median_for_static_motion_classification = 1.75;

constexpr uint64_t FLAGS_vio_max_landmarks = 1000;

constexpr double FLAGS_vio_min_depth = 0.1;
constexpr uint32_t FLAGS_vio_max_depth= 10;
constexpr uint32_t FLAGS_vio_median_depth = 1;

// Keyframe Selection
constexpr uint32_t FLAGS_vio_first_n_frames_as_keyframes = 0;
constexpr uint32_t FLAGS_vio_kfselect_min_every_nth_frame = -1;
constexpr uint32_t FLAGS_vio_kfselect_every_nth_frame = -1;
constexpr uint32_t FLAGS_vio_kfselect_criterion = 1;
constexpr uint64_t FLAGS_vio_kfselect_numfts_upper_thresh = 180;
constexpr uint64_t FLAGS_vio_kfselect_numfts_lower_thresh = 90;
constexpr uint32_t FLAGS_vio_kfselect_min_num_frames_between_kfs = 0;
constexpr double FLAGS_vio_kfselect_min_disparity = 30.0;
constexpr double FLAGS_vio_kfselect_min_dist = 0.12;


// Landmark Reprojector
constexpr bool FLAGS_vio_reprojector_pyr_alignment = true;
constexpr bool FLAGS_vio_reprojector_limit_num_features = true;
constexpr uint32_t FLAGS_vio_reprojector_min_quality_to_project = 2;

// Feature detector
constexpr bool FLAGS_vio_descriptor_use_dummy = false;
extern const std::string FLAGS_imp_detector_name;
constexpr uint32_t FLAGS_imp_detector_grid_size = 32;
constexpr uint32_t FLAGS_imp_detector_border_margin = 8;
constexpr uint32_t FLAGS_imp_detector_num_octaves = 3;
constexpr uint32_t FLAGS_imp_detector_max_features_per_frame = 300;
constexpr uint32_t FLAGS_vio_max_tracked_features_per_frame = 300;
constexpr uint32_t FLAGS_imp_brisk_uniformity_radius = 0;
constexpr uint32_t FLAGS_imp_brisk_absolute_threshold = 45;
constexpr uint32_t FLAGS_imp_detector_threshold = 10;

constexpr uint32_t FLAGS_vio_min_tracked_features_total = 20; //Guess

// Track Extracter
// Max number of landmarks to add to back-end optimization per keyframe.
constexpr uint32_t FLAGS_vio_max_num_tracks_per_update = 50;
// Number of observations in feature-track to classify landmark as persistent
constexpr uint32_t FLAGS_vio_num_obs_for_persistent = 10;
// Minimum parallax threshold to use a landmark in the back-end optimization
constexpr double FLAGS_vio_min_parallax_deg = 0.5;
// Favor a n equal number of new persistent tracks in every frame.
constexpr bool FLAGS_vio_favor_equal_number_of_tracks_per_frame = false;
constexpr bool FLAGS_vio_delayed_nframe_processing = 0; //Guess and feel like might be changed somewhere

 




//==============================================================================
//      Function Prototypes
//------------------------------------------------------------------------------



#endif  // TYPE_ALIASES_HPP
//==============================================================================
// End of File :  Software/include/TYPE_ALIASES_HPP.hpp